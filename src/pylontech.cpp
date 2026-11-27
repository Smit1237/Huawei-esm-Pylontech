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

    // Alarm Status 1 — aligned with Huawei ESM-48100B1 BMS thresholds (0x1200..0x122F, 0x0049)
    if (s.packV >= 56.4f || (s.warn & (1 << 4)))                   alarm1 |= (1 << 7); // Pack V High (56.4V manual / 0x0049_4)
    if ((s.packV > 10.0f && s.packV <= 47.5f) || (s.warn & (1 << 6))) alarm1 |= (1 << 6); // Pack V Low (47.5V DVL / 0x0049_6)
    if ((s.nCells && maxc >= 3600) || (s.warn & (1 << 5)))         alarm1 |= (1 << 5); // Cell V High (0x1200 = 3600mV / 0x0049_5)
    if ((s.nCells && minc <= 3000) || (s.warn & (1 << 7)))         alarm1 |= (1 << 4); // Cell V Low  (0x1203 = 3000mV / 0x0049_7)
    if (s.tmax >= 50 || (s.warn & ((1 << 1) | (1 << 3))))          alarm1 |= (1 << 3); // Temp High   (0x1220/0x1228 = 50C)
    if (s.tmin <= 0  || (s.warn & ((1 << 0) | (1 << 2))))          alarm1 |= (1 << 2); // Temp Low    (0x122B = 0C / 0x0049_0,2)
    if (spread >= 300)                                             alarm1 |= (1 << 0); // Within-pack Cell Imbalance >= 300mV

    // Alarm Status 2 — Huawei OC warnings (0x004A) & general BMS warnings (0x0049)
    if (s.special & (1 << 10))                                     alarm2 |= (1 << 0); // Charge OC Alarm (0x004A_10)
    if (s.special & (1 << 4))                                      alarm2 |= (1 << 1); // Discharge OC Alarm (0x004A_4)
    if (s.warn != 0)                                               alarm2 |= (1 << 3); // Pack warning active

    // Protection Status 1 — aligned with Huawei ESM-48100B1 BMS protections (0x1204..0x122F, 0x0048)
    if (s.packV >= 57.6f || (s.prot & ((1 << 4) | (1 << 8))))      prot1  |= (1 << 7); // Pack OV Prot (0x1214 = 57.6V / 0x0048_4,8)
    if ((s.packV > 10.0f && s.packV <= 43.2f) || (s.prot & ((1 << 6) | (1 << 11)))) prot1 |= (1 << 6); // Pack UV Prot (0x1217 = 43.2V)
    if ((s.nCells && maxc >= 3800) || (s.prot & (1 << 5)))         prot1  |= (1 << 5); // Cell OV Prot (0x1204 = 3900mV / 0x0048_5)
    if ((s.nCells && minc <= 2500) || (s.prot & (1 << 7)))         prot1  |= (1 << 4); // Cell UV Prot (0x1207 = 2500mV / 0x0048_7)
    if (s.tmax >= 60 || (s.prot & ((1 << 1) | (1 << 3) | (1 << 9)))) prot1 |= (1 << 3); // Cell/MOS OT Prot (0x1224 = 60C)
    if (s.tmin <= -20 || (s.prot & ((1 << 0) | (1 << 2))))         prot1  |= (1 << 2); // Cell UT Prot (0x1227 = 0C chg / 0x122F = -20C dis)

    // Protection Status 2 — Huawei OC, SC & hardware faults (0x0046, 0x0047, 0x0048, 0x004A)
    if (s.special & (1 << 11))                                     prot2  |= (1 << 0); // Charge OC Prot (0x004A_11)
    if ((s.special & ((1 << 5) | (1 << 6))) || (s.prot & (1 << 14))) prot2 |= (1 << 1); // Discharge OC / Overload Prot
    if ((s.prot & (1 << 10)) || (s.sysFault & (1 << 3)) || (s.special & (1 << 13))) prot2 |= (1 << 2); // Short Circuit Prot
    if ((s.sysFault | s.cellFault) != 0 || (s.prot & 0x3000) != 0) prot2  |= (1 << 3); // Hardware / Cell / Lock Fault

    snprintf(out, n, "%02X%02X%02X%02X", alarm1, alarm2, prot1, prot2);
}

void PylontechEmu::effectiveLimits(const PackSnapshot &s, uint16_t &cclDaOut, uint16_t &dclDaOut) {
    uint16_t maxc, minc;
    cellExtremes(s, maxc, minc);
    uint32_t np = s.nPacks ? s.nPacks : 1;

    // Official Huawei ESM-48100B1 datasheet rating:
    //   Nominal capacity = 100 Ah (0x0107), Max charge/discharge current limited = 100.0 A (1.0C) @ 35 C.
    //   BMS 0x100D/0x100B read 1020 (102.0 A = 102% hardware OC margin at 0x1274); cap to 1.0C manual spec
    //   while still honoring any lower user-configured limit in 0x100D/0x100B.
    //   Note: 0x1018 (=85) is NOT a discharge coefficient in Huawei's register map (oficial1.xml).
    uint32_t capDa  = (uint32_t)(s.capAh ? s.capAh : 100) * 10; // 1000 (100.0 A) per 100 Ah pack
    uint32_t bankDa = capDa * np;                               // 1.0C ceiling across active packs

    uint32_t ccl_da = s.haveLimits ? min<uint32_t>(s.ccl_raw, bankDa) : 0;
    uint32_t dcl_da = s.haveLimits ? min<uint32_t>(s.dcl_raw, bankDa) : 0;

    // Direction-specific Huawei protection masks (oficial1.xml 0x0048 & 0x004A) so an undervoltage
    // trip never blocks charging, and an overvoltage trip never blocks discharging.
    static constexpr uint16_t PROT_CHG_MASK = 0x7733; // Chg UT/OT, Pack/Cell/Out OV, Mod OT, SC, Rev, Lock, Overload
    static constexpr uint16_t PROT_DIS_MASK = 0x7ECC; // Dis UT/OT, Pack/Cell/LVD UV, Mod OT, SC, Rev, Lock, Overload
    static constexpr uint16_t SPEC_CHG_MASK = 0x288B; // Cmd/Abn shutdown, VBUS OV/lock, Chg OC prot, FET short
    static constexpr uint16_t SPEC_DIS_MASK = 0x206B; // Cmd/Abn shutdown, VBUS lock, Dis OC prot/lock, FET short

    bool hwFault  = (s.sysFault | s.cellFault) != 0;
    bool chgBlock = hwFault || (s.prot & PROT_CHG_MASK) != 0 || (s.special & SPEC_CHG_MASK) != 0;
    bool disBlock = hwFault || (s.prot & PROT_DIS_MASK) != 0 || (s.special & SPEC_DIS_MASK) != 0;

    // 1. CHARGE ENVELOPE — Huawei Manual: Operating Temp 0 C..45 C (0x1227 = 0.0 C UT protect),
    //    rated 100 A @ 35 C, Cell High-Voltage Alarm 0x1200 = 3600 mV.
    if (chgBlock || s.tmin <= 0 || s.tmax > 45 || (s.nCells && maxc >= 3600) || s.fullStop) {
        ccl_da = 0;
    } else {
        auto capChgC = [&](uint32_t centiC) {
            uint32_t lim = bankDa * centiC / 100;
            if (ccl_da > lim) ccl_da = lim;
        };
        // Cold charge derating aligned with Huawei 0x1223 (5 C alarm) & 0x1222 (8 C restore)
        if (s.tmin <= 5)       capChgC(10);  // 1..5 C  -> 0.10C (10 A/pack)
        else if (s.tmin <= 8)  capChgC(20);  // 6..8 C  -> 0.20C (20 A/pack)
        else if (s.tmin <= 12) capChgC(50);  // 9..12 C -> 0.50C (50 A/pack)

        // Warm charge derating above 35 C rated point up to 45 C manual limit (Note 3)
        if (s.tmax >= 41)      capChgC(30);  // 41..45 C -> 0.30C (30 A/pack)
        else if (s.tmax >= 36) capChgC(50);  // 36..40 C -> 0.50C (50 A/pack)

        // Top-of-charge cell-voltage & SOC taper approaching 0x1200 = 3600 mV (CVL = 54.0 V)
        if (s.nCells) {
            if (maxc >= 3550)      capChgC(5);   // >= 3.55 V -> 0.05C (5 A/pack)
            else if (maxc >= 3500) capChgC(15);  // >= 3.50 V -> 0.15C (15 A/pack)
            else if (maxc >= 3450) capChgC(30);  // >= 3.45 V -> 0.30C (30 A/pack)
        }
        if (s.soc >= 100)      capChgC(5);       // 100% SOC  -> 0.05C (5 A/pack absorption finish)
        else if (s.soc >= 99)  capChgC(20);      // 99% SOC   -> 0.20C (20 A/pack)
        else if (s.soc >= 98)  capChgC(50);      // 98% SOC   -> 0.50C (50 A/pack)
    }

    // 2. DISCHARGE ENVELOPE — Huawei Manual: Operating Temp -20 C..45 C (0x122F = -20.0 C UT protect,
    //    0x122B = 0.0 C low-temp alarm, 0x1228 = 50.0 C high-temp alarm), Default LVD = 46.0 V,
    //    Cell LV ladder: 0x1202 = 3100 mV, 0x1203 = 3000 mV, 0x1206 = 2900 mV, 0x1217 = 43.2 V.
    if (disBlock || s.tmin <= -20 || s.tmax >= 55 ||
        (s.nCells && minc <= 2800) || (s.packV > 10.0f && s.packV <= 43.5f)) {
        dcl_da = 0;
    } else {
        auto capDisC = [&](uint32_t centiC) {
            uint32_t lim = bankDa * centiC / 100;
            if (dcl_da > lim) dcl_da = lim;
        };
        // Sub-zero / cold discharge derating (Note 3, 0x122B = 0 C alarm, 0x122F = -20 C protect)
        if (s.tmin <= -10)     capDisC(20);  // -19..-10 C -> 0.20C (20 A/pack)
        else if (s.tmin <= 0)  capDisC(50);  //  -9..0 C   -> 0.50C (50 A/pack)

        // High-temperature discharge derating (Note 3 & 4: rated 100 A @ 35 C, max op 45 C, 50 C alarm)
        if (s.tmax >= 50)      capDisC(20);  // 50..54 C -> 0.20C (20 A/pack)
        else if (s.tmax >= 46) capDisC(30);  // 46..49 C -> 0.30C (30 A/pack)
        else if (s.tmax >= 40) capDisC(70);  // 40..45 C -> 0.70C (70 A/pack)

        // Dynamic low-voltage & low-SOC discharge taper (Huawei 0x1202/0x1203/0x1206 & 46.0 V LVD)
        if ((s.nCells && minc <= 2900) || (s.packV > 10.0f && s.packV <= 46.0f) || s.soc <= 2) {
            capDisC(10);                     // <= 2.90 V / <= 46.0 V / <= 2%  -> 0.10C (10 A/pack)
        } else if ((s.nCells && minc <= 3000) || (s.packV > 10.0f && s.packV <= 47.0f) || s.soc <= 5) {
            capDisC(20);                     // <= 3.00 V / <= 47.0 V / <= 5%  -> 0.20C (20 A/pack)
        } else if ((s.nCells && minc <= 3100) || s.soc <= 10) {
            capDisC(50);                     // <= 3.10 V / <= 10% SOC         -> 0.50C (50 A/pack)
        }
    }

    cclDaOut = ccl_da > 65535 ? 65535 : (uint16_t)ccl_da;
    dclDaOut = dcl_da > 65535 ? 65535 : (uint16_t)dcl_da;
}

void PylontechEmu::buildCmd63(const PackSnapshot &s, char *out, size_t n) {
    uint16_t maxc, minc;
    cellExtremes(s, maxc, minc);

    // Honor pack's configured 0x100A (54.0 V) & 0x100E (47.5 V); fallback to 54.0 V / 46.0 V (Huawei manual LVD)
    uint32_t cvl_mv = (uint32_t)(s.cvl_raw ? s.cvl_raw : 540) * 100;
    uint32_t dvl_mv = (uint32_t)(s.dvl_raw ? s.dvl_raw : 460) * 100;
    uint16_t ccl_da = 0, dcl_da = 0;
    effectiveLimits(s, ccl_da, dcl_da);

    // Dynamic Pylontech 0x63 management flags:
    // bit7 = Charge Enable, bit6 = Discharge Enable, bit5 = Force Charge Request (at Huawei 0x1203 3000mV or <=5% SOC)
    uint8_t status = 0;
    if (ccl_da > 0) status |= 0x80;
    if (dcl_da > 0) status |= 0x40;
    if (s.soc <= 5 || (s.nCells && minc <= 3000) || (s.packV > 10.0f && s.packV <= 46.0f)) status |= 0x20;

    snprintf(out, n, "%04X%04X%04X%04X%02X",
             (unsigned)(cvl_mv & 0xFFFF), (unsigned)(dvl_mv & 0xFFFF),
             (unsigned)ccl_da, (unsigned)dcl_da, status);
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
