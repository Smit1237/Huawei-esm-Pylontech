#pragma once
#include <Arduino.h>
#include <math.h>

// Pylontech RS485 (YDN1363) battery emulator for the Deye BMS port,
// "Lithium mode 12". The emulator answers the inverter polls
// CID1=0x46 / CID2=0x61,0x62,0x63 with real Huawei ESM data.

struct PackSnapshot {
    bool     valid      = false;
    bool     haveLimits = false;
    bool     fullStop   = false; // hysteresis latch when 100% full & idle
    float    packV      = 0;     // V
    float    current    = 0;     // A, + = charge (ESM native sign)
    uint8_t  soc        = 0;     // % (SOH-weighted rounded average in bank mode)
    uint8_t  soh        = 0;     // % (average SOH in bank mode)
    uint8_t  sohMin     = 0;     // % (minimum pack SOH; 0 = same as soh)
    int16_t  tmin       = 0;     // degC
    int16_t  tmax       = 0;     // degC
    float    tavg       = NAN;   // degC true mean across all NTCs (NAN = compute from ctemps[])
    uint16_t cells[15];          // mV
    uint8_t  nCells     = 0;
    uint16_t maxSpread  = 0xFFFF;// max within-pack cell delta in mV (0xFFFF = compute from cells[])
    uint8_t  maxCellPack = 1, maxCellIdx = 0; // 1-based Pylontech coordinates
    uint8_t  minCellPack = 1, minCellIdx = 0;
    uint8_t  maxTempPack = 1, maxTempIdx = 0;
    uint8_t  minTempPack = 1, minTempIdx = 0;
    int16_t  ctemps[16];         // degC (Huawei NTC sensors)
    uint8_t  nTemps     = 0;
    uint16_t cycles     = 0;     // max cycles
    uint16_t cyclesAvg  = 0;     // average cycles (0 = same as cycles)
    // Raw limit registers of the pack (0x100A block)
    uint16_t cvl_raw    = 0;     // 0.1 V  charge voltage
    uint16_t dvl_raw    = 0;     // 0.1 V  discharge cutoff
    uint16_t ccl_raw    = 0;     // 0.1 A  charge current
    uint16_t dcl_raw    = 0;     // 0.1 A  discharge current
    uint16_t ccl_coef   = 0;     // %      0x1011 (0 = unconfigured in Huawei NVRAM)
    uint16_t dcl_coef   = 0;     // %      0x1018
    // Extended telemetry & Huawei fault/status words
    float    busV       = 0;     // busbar voltage, register 0x0000 (V)
    uint32_t ahTotal    = 0;     // lifetime Ah throughput (register 0x0044)
    uint16_t sysFault   = 0;     // 0x0046 system faults
    uint16_t cellFault  = 0;     // 0x0047 cell faults
    uint16_t prot       = 0;     // 0x0048 active protections
    uint16_t warn       = 0;     // 0x0049 active warnings
    uint16_t special    = 0;     // 0x004A special states
    uint32_t ts         = 0;     // millis() value of the last good read
};

class PylontechEmu {
public:
    PylontechEmu(HardwareSerial &port, uint8_t respAddr = 0x02)
        : _port(port), _addr(respAddr) {}

    // Read all data that the inverter sent. Answer each frame with our address.
    // This call is cheap and non-blocking. Call it as often as possible.
    void poll(const PackSnapshot &s);

    uint32_t answered   = 0;
    uint32_t badFrames  = 0;
    uint32_t lastPollMs = 0;   // millis() value of the last valid frame from the inverter
    char     lastCid2[3] = "";

private:
    void sendFrame(const char *info);
    void buildCmd61(const PackSnapshot &s, char *out, size_t n);
    void buildCmd62(const PackSnapshot &s, char *out, size_t n);
    void buildCmd63(const PackSnapshot &s, char *out, size_t n);

    HardwareSerial &_port;
    uint8_t _addr;
    char    _buf[256];
    size_t  _len = 0;
};
