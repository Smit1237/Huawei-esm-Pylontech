#include "pylontech.h"

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

// Pylontech RS485 temperature unit: 0.1 K where 0.0 degC = 2731 (273.1 K).
static uint16_t deciK(float celsius) {
    int32_t dk = (int32_t)lroundf(celsius * 10.0f) + 2731;
    if (dk < 0)     dk = 0;
    if (dk > 65535) dk = 65535;
    return (uint16_t)dk;
}

static void cellExtremes(const PackSnapshot &s, uint16_t &maxc, uint16_t &minc,
                         uint8_t *maxIdxOut = nullptr, uint8_t *minIdxOut = nullptr) {
    maxc = 3350;
    minc = 3350;
    uint8_t hiIdx = 1, loIdx = 1;
    if (s.nCells) {
        maxc = 0;
        minc = 0xFFFF;
        for (uint8_t i = 0; i < s.nCells; i++) {
            if (s.cells[i] > maxc) { maxc = s.cells[i]; hiIdx = i + 1; }
            if (s.cells[i] < minc) { minc = s.cells[i]; loIdx = i + 1; }
        }
        if (minc == 0xFFFF) minc = 3350;
    }
    if (maxIdxOut) *maxIdxOut = s.maxCellIdx ? s.maxCellIdx : hiIdx;
    if (minIdxOut) *minIdxOut = s.minCellIdx ? s.minCellIdx : loIdx;
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

    // Wait 2 ms (~2 char times @ 9600 baud) so Deye's RS485 transceiver releases TX
    delay(2);
    _port.write((const uint8_t *)out, (size_t)on);
    _port.flush();
    answered++;
#ifdef BUS_DEBUG
    Serial.print("[DEYE TX] ");
    Serial.write((const uint8_t *)out, (size_t)on - 1);
    Serial.println();
#endif
}

void PylontechEmu::buildCmd61(const PackSnapshot &s, char *out, size_t n) {
    uint16_t v_mv = (uint16_t)lroundf(s.packV * 1000.0f);
    // Clamp centi-amps to int16_t range [-32768, +32767] so >327.67 A never wraps sign
    int32_t raw_ca = (int32_t)lroundf(s.current * 100.0f);
    if (raw_ca > 32767)  raw_ca = 32767;
    if (raw_ca < -32768) raw_ca = -32768;
    uint16_t i_ca = (uint16_t)(raw_ca & 0xFFFF);

    uint8_t  soc    = s.soc;
    uint8_t  avgSoh = s.soh;
    uint8_t  minSoh = s.sohMin ? s.sohMin : avgSoh;
    uint16_t maxCyc = s.cycles ? s.cycles : 100;
    uint16_t avgCyc = s.cyclesAvg ? s.cyclesAvg : maxCyc;

    uint16_t maxc, minc;
    uint8_t  maxCIdx = 1, minCIdx = 1;
    cellExtremes(s, maxc, minc, &maxCIdx, &minCIdx);
    uint8_t maxCPack = s.maxCellPack ? s.maxCellPack : 1;
    uint8_t minCPack = s.minCellPack ? s.minCellPack : 1;

    uint16_t tavgK, tmaxK, tminK;
    uint8_t  maxTPack = s.maxTempPack ? s.maxTempPack : 1;
    uint8_t  minTPack = s.minTempPack ? s.minTempPack : 1;
    uint8_t  maxTIdx  = s.maxTempIdx  ? s.maxTempIdx  : 1;
    uint8_t  minTIdx  = s.minTempIdx  ? s.minTempIdx  : 1;

    if (s.nTemps) {
        int16_t hi = s.ctemps[0], lo = s.ctemps[0];
        int32_t sumT = s.ctemps[0];
        uint8_t hiI = 1, loI = 1;
        for (uint8_t i = 1; i < s.nTemps; i++) {
            sumT += s.ctemps[i];
            if (s.ctemps[i] > hi) { hi = s.ctemps[i]; hiI = i + 1; }
            if (s.ctemps[i] < lo) { lo = s.ctemps[i]; loI = i + 1; }
        }
        float meanC = !isnan(s.tavg) ? s.tavg : ((float)sumT / (float)s.nTemps);
        tavgK = deciK(meanC);
        tmaxK = deciK(hi);
        tminK = deciK(lo);
        if (!s.maxTempIdx) maxTIdx = hiI;
        if (!s.minTempIdx) minTIdx = loI;
    } else {
        float meanC = !isnan(s.tavg) ? s.tavg : ((s.tmin + s.tmax) * 0.5f);
        tavgK = deciK(meanC);
        tmaxK = deciK(s.tmax);
        tminK = deciK(s.tmin);
    }

    snprintf(out, n,
             "%04X%04X%02X%04X%04X%02X%02X"
             "%04X%02X%02X%04X%02X%02X"
             "%04X%04X%02X%02X%04X%02X%02X"
             "%04X%04X0101%04X0101"
             "%04X%04X0101%04X0101",
             v_mv, i_ca, soc, avgCyc, maxCyc, avgSoh, minSoh,
             maxc, maxCPack, maxCIdx, minc, minCPack, minCIdx,
             tavgK, tmaxK, maxTPack, maxTIdx, tminK, minTPack, minTIdx,
             tavgK, tavgK, tavgK,
             tavgK, tavgK, tavgK);
}

void PylontechEmu::buildCmd62(const PackSnapshot &s, char *out, size_t n) {
    uint16_t maxc, minc;
    cellExtremes(s, maxc, minc);
    uint16_t spread = (s.maxSpread != 0xFFFF) ? s.maxSpread
                    : ((s.nCells >= 2 && maxc >= minc) ? (maxc - minc) : 0);

    uint8_t alarm1 = 0, alarm2 = 0, prot1 = 0, prot2 = 0;

    // Alarm Status 1 (matches Pylontech 0x62 layout)
    if (s.packV >= 55.5f)                               alarm1 |= (1 << 7); // Total V High
    if (s.packV > 10.0f && s.packV <= 47.5f)            alarm1 |= (1 << 6); // Total V Low
    if (s.nCells && maxc >= 3600)                       alarm1 |= (1 << 5); // Cell V High
    if (s.nCells && minc <= 2950)                       alarm1 |= (1 << 4); // Cell V Low
    if (s.tmax >= 50)                                   alarm1 |= (1 << 3); // Cell Temp High
    if (s.tmin <= 2)                                    alarm1 |= (1 << 2); // Cell Temp Low
    if (spread >= 300)                                  alarm1 |= (1 << 0); // Within-pack Cell Imbalance >= 300mV

    // Alarm Status 2
    if (s.warn != 0)                                    alarm2 |= (1 << 3); // Pack warning active

    // Protection Status 1
    if (s.packV >= 56.5f)                               prot1  |= (1 << 7); // Module OV Prot
    if (s.packV > 10.0f && s.packV <= 45.0f)            prot1  |= (1 << 6); // Module UV Prot
    if (s.nCells && maxc >= 3650)                       prot1  |= (1 << 5); // Cell OV Prot
    if (s.nCells && minc <= 2800)                       prot1  |= (1 << 4); // Cell UV Prot
    if (s.tmax >= 55)                                   prot1  |= (1 << 3); // Cell OT Prot
    if (s.tmin <= -5)                                   prot1  |= (1 << 2); // Cell UT Prot

    // Protection Status 2
    if ((s.sysFault | s.cellFault | s.prot) != 0)       prot2  |= (1 << 3); // Hardware / BMS Fault

    snprintf(out, n, "%02X%02X%02X%02X", alarm1, alarm2, prot1, prot2);
}

void PylontechEmu::buildCmd63(const PackSnapshot &s, char *out, size_t n) {
    uint16_t maxc, minc;
    cellExtremes(s, maxc, minc);

    uint32_t cvl_mv = (uint32_t)(s.cvl_raw ? s.cvl_raw : 540) * 100;
    uint32_t dvl_mv = (uint32_t)(s.dvl_raw ? s.dvl_raw : 475) * 100;
    uint32_t ccl_da = s.haveLimits ? s.ccl_raw : 0;
    uint32_t dcl_da = s.haveLimits ? s.dcl_raw : 0;

    // Apply Huawei NVRAM coef when configured (1..100%); 0 in Huawei 0x1011 means unconfigured
    uint16_t cclC = s.ccl_coef > 100 ? 100 : s.ccl_coef;
    uint16_t dclC = s.dcl_coef > 100 ? 100 : s.dcl_coef;
    if (cclC > 0) ccl_da = ccl_da * cclC / 100;
    if (dclC > 0) dcl_da = dcl_da * dclC / 100;

    // LiFePO4 cell-voltage & temperature protection envelopes
    bool hardFault = (s.sysFault | s.cellFault | s.prot) != 0;
    if (hardFault || s.tmax >= 55 || s.tmin <= 0 || (s.nCells && maxc >= 3620) || s.fullStop) {
        ccl_da = 0;
    } else {
        // Cold temperature charge derate
        if (s.tmin <= 5 && ccl_da > 100)       ccl_da = 100; // <=5C: max 10A
        else if (s.tmin <= 12 && ccl_da > 300) ccl_da = 300; // <=12C: max 30A
        // High cell voltage top-of-charge taper (15S LiFePO4)
        if (s.nCells && maxc >= 3550 && ccl_da > 50)       ccl_da = 50;  // >=3.55V: 5A balance taper
        else if (s.nCells && maxc >= 3500 && ccl_da > 150) ccl_da = 150; // >=3.50V: 15A taper
    }

    if ((s.sysFault | s.cellFault) != 0 || s.tmax >= 58 || (s.nCells && minc <= 2800)) {
        dcl_da = 0;
    }

    // Dynamic Pylontech 0x63 management flags:
    // bit7 = Charge Enable, bit6 = Discharge Enable, bit5 = Force Charge Request
    uint8_t status = 0;
    if (ccl_da > 0) status |= 0x80;
    if (dcl_da > 0) status |= 0x40;
    if (s.soc <= 5 || (s.nCells && minc <= 2950)) status |= 0x20;

    snprintf(out, n, "%04X%04X%04X%04X%02X",
             (unsigned)(cvl_mv & 0xFFFF), (unsigned)(dvl_mv & 0xFFFF),
             (unsigned)(ccl_da & 0xFFFF), (unsigned)(dcl_da & 0xFFFF), status);
}

static int hexByte(char hi, char lo) {
    auto nib = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    };
    int h = nib(hi), l = nib(lo);
    return (h < 0 || l < 0) ? -1 : ((h << 4) | l);
}

void PylontechEmu::poll(const PackSnapshot &s) {
    while (_port.available()) {
        char c = (char)_port.read();
        if (c == '~') {
            // Always resync frame start on '~' so any RS485 turnaround glitch byte before '~' is discarded!
            _len = 0;
            _buf[_len++] = '~';
        } else if (c == '\r') {
            // Minimum valid Pylontech request: '~' + 12 body chars (VER,ADR,CID1,CID2,LEN) + 4 CHKSUM chars = 17 chars
            if (_len >= 17 && _buf[0] == '~') {
#ifdef BUS_DEBUG
                _buf[_len] = 0;
                Serial.print("[DEYE RX] "); Serial.println(_buf);
#endif
                const char *b = _buf + 1;           // Skip '~'
                size_t bodyLen = _len - 5;          // Exclude '~' and 4-char trailing checksum
                int chkHi = hexByte(_buf[_len - 4], _buf[_len - 3]);
                int chkLo = hexByte(_buf[_len - 2], _buf[_len - 1]);
                uint16_t expectedChk = chksum(b, bodyLen);

                if (chkHi >= 0 && chkLo >= 0 && (((uint16_t)chkHi << 8) | (uint16_t)chkLo) == expectedChk) {
                    int adr  = hexByte(b[2], b[3]);
                    int cid1 = hexByte(b[4], b[5]);
                    char cid2[3] = { b[6], b[7], 0 };

                    if (adr == (int)_addr && cid1 == 0x46) {
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
                } else {
                    badFrames++;
                }
            }
            _len = 0;
        } else if (_len > 0) {
            // Only buffer payload characters after '~' has been seen
            if (_len < sizeof(_buf) - 1) _buf[_len++] = c;
            else _len = 0;
        }
    }
}
