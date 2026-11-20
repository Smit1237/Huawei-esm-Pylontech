#include "huawei_esm.h"

Print *g_esmLog = &Serial;   // You can redirect this. Refer to huawei_esm.h.

static const uint32_t ESM_BAUD = 9600;

#ifdef ESM_DEBUG_FRAMES
// Build the whole line in one buffer. Send it with one write.
// This keeps the ESM task dumps separate from the Deye task
// dumps on the shared USB console, not mixed byte by byte.
static void dumpFrame(const char *dir, const uint8_t *buf, size_t len) {
    char line[8 + 3 * 64];
    int n = snprintf(line, sizeof(line), "[ESM %s %2u]", dir, (unsigned)len);
    for (size_t i = 0; i < len && n < (int)sizeof(line) - 4; i++)
        n += snprintf(line + n, sizeof(line) - n, " %02X", buf[i]);
    g_esmLog->println(line);
}
#else
#define dumpFrame(dir, buf, len)
#endif

HuaweiESM::HuaweiESM(HardwareSerial &port, int8_t rxPin, int8_t txPin)
    : _port(port), _rxPin(rxPin), _txPin(txPin) {}

void HuaweiESM::setTarget(uint8_t addr, bool evenParity) {
    _addr = addr;
    uint32_t wanted = evenParity ? SERIAL_8E1 : SERIAL_8N1;
    if (!_started || _parity != wanted) {
        reconfigureSerial(wanted);
    }
}

void HuaweiESM::reconfigureSerial(uint32_t parityMode) {
    _parity = parityMode;
    if (_started) _port.end();
    _port.begin(ESM_BAUD, _parity, _rxPin, _txPin);
    _started = true;
}

uint16_t HuaweiESM::crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

// Send req. This function adds the CRC. Collect the reply until the
// line stays quiet for 25 ms (about 20 character times at 9600 baud,
// past a frame gap). Return the reply length after the CRC check.
// Return 0 for a timeout or a CRC error.
size_t HuaweiESM::transact(const uint8_t *req, size_t reqLen,
                           uint8_t *resp, size_t respMax, uint32_t timeoutMs) {
    uint8_t frame[64];
    if (reqLen + 2 > sizeof(frame)) return 0;
    memcpy(frame, req, reqLen);
    uint16_t crc = crc16(frame, reqLen);
    frame[reqLen]     = crc & 0xFF;
    frame[reqLen + 1] = crc >> 8;

    while (_port.available()) _port.read();  // Remove old bytes.
    dumpFrame("TX", frame, reqLen + 2);
    _port.write(frame, reqLen + 2);
    _port.flush();
    txBytes += reqLen + 2;

    size_t   len   = 0;
    uint32_t start = millis();
    uint32_t lastByte = 0;
    while (true) {
        if (_port.available()) {
            if (len < respMax) resp[len++] = _port.read();
            else _port.read();
            rxBytes++;
            lastByte = millis();
        } else if (len > 0 && millis() - lastByte > 25) {
            break;  // The frame is complete.
        } else if (millis() - start > timeoutMs) {
            if (len == 0) return 0;
            break;
        } else {
            // Yield while the function waits for bytes. A busy loop starves
            // the RTOS idle task when this code runs in a pinned FreeRTOS
            // task (the translator). This trips the ESP32 task watchdog.
            // The UART FIFO holds incoming bytes, so a 1 ms poll loses none.
            delay(1);
        }
    }
    dumpFrame("RX", resp, len);

    if (len < 4) return 0;
    uint16_t rxCrc = resp[len - 2] | (resp[len - 1] << 8);
    if (rxCrc != crc16(resp, len - 2)) return 0;
    if (resp[0] != _addr) return 0;
    return len;
}

bool HuaweiESM::fc03(uint16_t startReg, uint16_t count, uint16_t *out,
                     uint32_t timeoutMs) {
    uint8_t req[6] = { _addr, 0x03,
                       (uint8_t)(startReg >> 8), (uint8_t)(startReg & 0xFF),
                       (uint8_t)(count >> 8),    (uint8_t)(count & 0xFF) };
    uint8_t resp[8 + 2 * 32];
    size_t len = transact(req, sizeof(req), resp, sizeof(resp), timeoutMs);
    if (len < (size_t)(5 + 2 * count)) return false;
    if (resp[1] != 0x03 || resp[2] != 2 * count) return false;
    for (uint16_t i = 0; i < count; i++)
        out[i] = (resp[3 + 2 * i] << 8) | resp[4 + 2 * i];
    return true;
}

bool HuaweiESM::probe() {
    const uint32_t parities[] = { SERIAL_8N1, SERIAL_8E1 };
    for (uint32_t p : parities) {
        reconfigureSerial(p);
        for (uint8_t addr = 214; addr <= 218; addr++) {
            _addr = addr;
            uint16_t v;
            g_esmLog->printf("  probing addr %u, parity %s... ",
                          addr, p == SERIAL_8N1 ? "8N1" : "8E1");
            if (fc03(0x0000, 1, &v, 800)) {
                g_esmLog->printf("answered (reg0 = %u)\n", v);
                return true;
            }
            g_esmLog->println("no reply");
        }
    }
    return false;
}

bool HuaweiESM::wake() {
    // Use larger delays each time. The BMS communication can need time to start.
    const uint32_t delays[] = { 1000, 2000, 4000, 8000, 16000 };
    for (uint32_t d : delays) {
        uint16_t v;
        if (fc03(0x0000, 1, &v, 800) && v > 0) return true;
        delay(d);
    }
    uint16_t v;
    return fc03(0x0000, 1, &v, 800);
}

bool HuaweiESM::readClock(uint16_t out[6]) {
    return fc03(0x1000, 6, out);
}

bool HuaweiESM::fc10TimeSync() {
    // The BMS expects a wall-clock write during the handshake.
    // Write its own current time back first. This keeps the battery
    // clock. If the read fails, use the firmware build time.
    uint16_t vals[6];
    if (!readClock(vals)) {
        // __DATE__ = "Jul  4 2026", __TIME__ = "12:34:56"
        static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
        char monStr[4] = { __DATE__[0], __DATE__[1], __DATE__[2], 0 };
        vals[1] = (strstr(months, monStr) - months) / 3 + 1;
        vals[2] = atoi(__DATE__ + 4);
        vals[0] = atoi(__DATE__ + 7);
        vals[3] = atoi(__TIME__);
        vals[4] = atoi(__TIME__ + 3);
        vals[5] = atoi(__TIME__ + 6);
    }
    delay(100);

    uint8_t req[7 + 12] = { _addr, 0x10, 0x10, 0x00, 0x00, 0x06, 0x0C };
    for (int i = 0; i < 6; i++) {
        req[7 + 2 * i]     = vals[i] >> 8;
        req[7 + 2 * i + 1] = vals[i] & 0xFF;
    }
    uint8_t resp[16];
    size_t len = transact(req, sizeof(req), resp, sizeof(resp), 1000);
    return len >= 8 && resp[1] == 0x10;
}

bool HuaweiESM::fc41Access() {
    // Documented frame: [addr] 41 05 01 04
    uint8_t req[5] = { _addr, 0x41, 0x05, 0x01, 0x04 };
    uint8_t resp[32];
    size_t len = transact(req, sizeof(req), resp, sizeof(resp), 2000);
    return len >= 5 && resp[1] == 0x41;
}

bool HuaweiESM::authenticate() {
    // Phase 1: unlock read of register 0x0106.
    uint16_t v;
    if (!fc03(0x0106, 1, &v, 1000)) {
        g_esmLog->println("  auth phase 1 (FC03 0x0106): no/bad reply");
        return false;
    }
    delay(600);

    // Phase 2: date and time sync.
    if (!fc10TimeSync()) {
        g_esmLog->println("  auth phase 2 (FC10 time sync): no/bad reply");
        return false;
    }
    delay(600);

    // Phase 3: FC41 access grant.
    if (!fc41Access()) {
        g_esmLog->println("  auth phase 3 (FC41 access): no/bad reply");
        return false;
    }
    delay(300);
    return true;
}

bool HuaweiESM::readRegisters(uint16_t startReg, uint16_t count, uint16_t *out) {
    return fc03(startReg, count, out);
}

bool HuaweiESM::readMainData(EsmMainData &out) {
    uint16_t r[7];
    if (!fc03(0x0000, 7, r)) return false;
    out.busbarVoltage  = r[0];
    out.batteryVoltage = r[1];
    out.current        = (int16_t)r[2];
    out.soc            = r[3];
    out.soh            = r[4];
    out.cellTempMax    = (int16_t)r[5];
    out.cellTempMin    = (int16_t)r[6];
    return true;
}

bool HuaweiESM::readCells(uint16_t mv[15], int16_t temps[16]) {
    uint16_t t[16];
    if (!fc03(0x0012, 16, t)) return false;
    for (int i = 0; i < 16; i++) temps[i] = (int16_t)t[i];
    return fc03(0x0022, 15, mv);
}

bool HuaweiESM::readStats(uint32_t &cycles, uint32_t &ahTotal) {
    uint16_t r[4];
    if (!fc03(0x0042, 4, r)) return false;
    cycles  = ((uint32_t)r[0] << 16) | r[1];
    ahTotal = ((uint32_t)r[2] << 16) | r[3];
    return true;
}

bool HuaweiESM::readStatusBits(EsmStatusBits &out) {
    uint16_t r[5];
    if (!fc03(0x0046, 5, r)) return false;
    out.systemFaults      = r[0];
    out.cellFaults        = r[1];
    out.activeProtections = r[2];
    out.activeWarnings    = r[3];
    out.specialStates     = r[4];
    return true;
}

size_t HuaweiESM::readArchiveInfo(char *buf, size_t maxLen) {
    size_t n = 0;
    for (uint8_t idx = 0; idx <= 7 && n + 1 < maxLen; idx++) {
        uint8_t req[7] = { _addr, 0x41, 0x06, 0x03, 0x04, 0x00, idx };
        uint8_t resp[256];
        size_t len = transact(req, sizeof(req), resp, sizeof(resp), 2000);
        if (len < 9) break;
        // The payload starts at [6]. Bytes [3..5] are a chunk header
        // (length byte 0x43 = 'C' plus counters) that litters the text.
        for (size_t i = 6; i < len - 2 && n + 1 < maxLen; i++) {
            char c = (char)resp[i];
            if (c >= 32 && c < 127) {
                buf[n++] = c;
            } else if (n && buf[n - 1] != '\n') {
                buf[n++] = '\n';   // Collapse runs of control bytes.
            }
        }
        delay(150);
    }
    buf[n] = 0;
    return n;
}

void HuaweiESM::printDeviceInfo(Stream &console) {
    // FC41 sub-command 03 04: device info by index (ASCII key=value data).
    for (uint8_t idx = 0; idx <= 5; idx++) {
        // Request layout: [addr] 41 06 03 04 00 [index]
        uint8_t req[7] = { _addr, 0x41, 0x06, 0x03, 0x04, 0x00, idx };
        uint8_t resp[256];
        size_t len = transact(req, sizeof(req), resp, sizeof(resp), 2000);
        if (len < 9) continue;
        console.printf("  info[%u]: ", idx);
        for (size_t i = 6; i < len - 2; i++) {
            char c = (char)resp[i];
            console.print(isPrintable(c) ? c : '.');
        }
        console.println();
        delay(200);
    }
}
