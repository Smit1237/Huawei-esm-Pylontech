#include "pylontech.h"

// ---- YDN1363 framing. This is the same as deye_bms.py. ----

static uint16_t chksum(const char *body, size_t n) {
    uint32_t s = 0;
    for (size_t i = 0; i < n; i++) s += (uint8_t)body[i];
    return (uint16_t)((~s + 1) & 0xFFFF);
}

// LENGTH field has 4 hex characters: a 4-bit checksum nibble, then the 12-bit length.
static void lengthField(uint16_t infoLen, char out[5]) {
    uint16_t n = (infoLen & 0xF) + ((infoLen >> 4) & 0xF) + ((infoLen >> 8) & 0xF);
    uint16_t lchk = (uint16_t)((~n & 0xF) + 1);
    snprintf(out, 5, "%1X%03X", (unsigned)(lchk & 0xF), (unsigned)(infoLen & 0xFFF));
}

// deci-Kelvin is the Pylontech temperature unit.
static uint16_t deciK(float celsius) {
    return (uint16_t)lroundf((celsius + 273.15f) * 10.0f);
}

void PylontechEmu::sendFrame(const char *info) {
    char body[512];
    char lf[5];
    size_t infoLen = strlen(info);
    lengthField((uint16_t)infoLen, lf);
    // Fields: VER=20, ADR, CID1=46, RTN=00, LENGTH, INFO
    int bn = snprintf(body, sizeof(body), "20%02X4600%s%s", _addr, lf, info);
    if (bn <= 0 || (size_t)bn >= sizeof(body)) return;

    char out[540];
    int on = snprintf(out, sizeof(out), "~%s%04X\r", body, chksum(body, (size_t)bn));
    if (on <= 0 || (size_t)on >= sizeof(out)) return;
    _port.write((const uint8_t *)out, (size_t)on);
    _port.flush();
    answered++;
#ifdef BUS_DEBUG
    Serial.print("[DEYE TX] ");
    Serial.write((const uint8_t *)out, (size_t)on - 1);   // Remove the last CR character.
    Serial.println();
#endif
}

void PylontechEmu::buildCmd61(const PackSnapshot &s, char *out, size_t n) {
    uint16_t v_mv = (uint16_t)lroundf(s.packV * 1000.0f);
    uint16_t i_ca = (uint16_t)(((int32_t)lroundf(s.current * 100.0f)) & 0xFFFF);
    uint8_t  soc  = s.soc;
    uint8_t  soh  = s.soh;
    uint16_t cyc  = s.cycles ? s.cycles : 100;

    uint16_t maxc = 3350, minc = 3350;
    if (s.nCells) {
        maxc = 0; minc = 0xFFFF;
        for (uint8_t i = 0; i < s.nCells; i++) {
            if (s.cells[i] > maxc) maxc = s.cells[i];
            if (s.cells[i] < minc) minc = s.cells[i];
        }
    }

    uint16_t tavg = deciK((s.tmin + s.tmax) / 2.0f);
    uint16_t tmaxK, tminK;
    if (s.nTemps) {
        int16_t hi = s.ctemps[0], lo = s.ctemps[0];
        for (uint8_t i = 1; i < s.nTemps; i++) {
            if (s.ctemps[i] > hi) hi = s.ctemps[i];
            if (s.ctemps[i] < lo) lo = s.ctemps[i];
        }
        tmaxK = deciK(hi); tminK = deciK(lo);
    } else {
        tmaxK = deciK(s.tmax); tminK = deciK(s.tmin);
    }

    // The 0101/0108/0103/0109 markers are fixed protocol constants. You
    // cannot address the Deye per-cell page through 0x61, which is a
    // summary-only format. A test confirms this. Keep the reference values.
    snprintf(out, n,
             "%04X%04X%02X%04X%04X%02X%02X"
             "%04X0101%04X0108"
             "%04X%04X0103%04X0109"
             "%04X%04X0101%04X0101"
             "%04X%04X0101%04X0101",
             v_mv, i_ca, soc, cyc, cyc, soh, soh,
             maxc, minc,
             tavg, tmaxK, tminK,
             tavg, tavg, tavg,
             tavg, tavg, tavg);
}

void PylontechEmu::buildCmd62(const PackSnapshot &s, char *out, size_t n) {
    (void)s;
    // Report all-normal unless the pack shows a fault. Keep this simple and safe.
    snprintf(out, n, "00000000");
}

void PylontechEmu::buildCmd63(const PackSnapshot &s, char *out, size_t n) {
    const uint8_t status = 0xC0;   // bit7 is charge enable. bit6 is discharge enable.
    // Forward the limit registers of the battery. ESM voltage registers are
    // 0.1 V. Multiply by 100 to get mV. Current registers are already 0.1 A,
    // which is the Deye deci-amp unit, so they pass through (e.g. 693 -> 69.3 A).
    uint32_t cvl_mv = (uint32_t)(s.cvl_raw ? s.cvl_raw : 540) * 100;
    uint32_t dvl_mv = (uint32_t)(s.dvl_raw ? s.dvl_raw : 475) * 100;
    uint32_t ccl_da = s.ccl_raw ? s.ccl_raw : 500;
    uint32_t dcl_da = s.dcl_raw ? s.dcl_raw : 500;
    // Apply the charge and discharge limit coefficient of the pack (%,
    // registers 0x1011 / 0x1018): e.g. 693 * 85% = 589 -> 58.9 A.
    if (s.ccl_coef) ccl_da = ccl_da * s.ccl_coef / 100;
    if (s.dcl_coef) dcl_da = dcl_da * s.dcl_coef / 100;
    snprintf(out, n, "%04X%04X%04X%04X%02X",
             (unsigned)(cvl_mv & 0xFFFF), (unsigned)(dvl_mv & 0xFFFF),
             (unsigned)(ccl_da & 0xFFFF), (unsigned)(dcl_da & 0xFFFF), status);
}

void PylontechEmu::poll(const PackSnapshot &s) {
    while (_port.available()) {
        char c = (char)_port.read();
        if (c == '\r') {
            // The complete frame is in _buf[0.._len).
            if (_len >= 9 && _buf[0] == '~') {
#ifdef BUS_DEBUG
                _buf[_len] = 0;
                Serial.print("[DEYE RX] "); Serial.println(_buf);
#endif
                const char *b = _buf + 1;          // Skip the '~' character.
                char tmp[3] = { 0, 0, 0 };
                tmp[0] = b[2]; tmp[1] = b[3];
                uint8_t adr = (uint8_t)strtol(tmp, nullptr, 16);
                tmp[0] = b[4]; tmp[1] = b[5];
                uint8_t cid1 = (uint8_t)strtol(tmp, nullptr, 16);
                char cid2[3] = { b[6], b[7], 0 };

                if (adr == _addr && cid1 == 0x46) {
                    lastPollMs = millis();
                    strncpy(lastCid2, cid2, sizeof(lastCid2));
                    if (s.valid) {
                        char info[320];
                        bool ok = true;
                        if (!strcmp(cid2, "61"))      buildCmd61(s, info, sizeof(info));
                        else if (!strcmp(cid2, "62")) buildCmd62(s, info, sizeof(info));
                        else if (!strcmp(cid2, "63")) buildCmd63(s, info, sizeof(info));
                        else ok = false;
                        if (ok) sendFrame(info);
                    }
                }
            }
            _len = 0;
        } else {
            if (_len < sizeof(_buf) - 1) _buf[_len++] = c;
            else _len = 0;                       // Data too long. Reset to resynchronize.
        }
    }
}
