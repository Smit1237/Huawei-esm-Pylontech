#pragma once
#include <Arduino.h>

// Pylontech RS485 (YDN1363) battery emulator for the Deye BMS port,
// "Lithium mode 12". The emulator answers the inverter polls
// CID1=0x46 / CID2=0x61,0x62,0x63 with real Huawei ESM data.
//
// This code is a direct port of the proven tools/deye/deye_bms.py.
// That script ran against the real Deye SUN-6K/8K-SG05LP1. Keep the
// frame layout, the units and the fixed protocol marker constants
// byte-identical. Do not change them.

// Most recent reading of the ESM pack. The poller and the responder
// share this data. The fields match the JSON snapshot that the Python
// emulator used.
struct PackSnapshot {
    bool     valid   = false;
    float    packV   = 0;     // V
    float    current = 0;     // A, + = charge (ESM native sign)
    uint8_t  soc     = 0;
    uint8_t  soh     = 0;
    int16_t  tmin    = 0;     // degC
    int16_t  tmax    = 0;     // degC
    uint16_t cells[15];       // mV
    uint8_t  nCells  = 0;
    int16_t  ctemps[15];      // degC
    uint8_t  nTemps  = 0;
    uint16_t cycles  = 0;
    // Raw limit registers of the pack (0x100A block)
    uint16_t cvl_raw = 0;     // 0.1 V  charge voltage
    uint16_t dvl_raw = 0;     // 0.1 V  discharge cutoff
    uint16_t ccl_raw = 0;     // 0.1 A  charge current
    uint16_t dcl_raw = 0;     // 0.1 A  discharge current
    uint16_t ccl_coef = 0;    // %      0x1011
    uint16_t dcl_coef = 0;    // %      0x1018
    // Extended telemetry. The bank firmware sets these values. The emulator ignores them.
    float    busV     = 0;    // busbar voltage, register 0x0000 (V)
    uint32_t ahTotal  = 0;    // lifetime Ah throughput (register 0x0044)
    uint16_t sysFault = 0;    // 0x0046 system faults
    uint16_t cellFault= 0;    // 0x0047 cell faults
    uint16_t prot     = 0;    // 0x0048 active protections
    uint16_t warn     = 0;    // 0x0049 active warnings
    uint16_t special  = 0;    // 0x004A special states
    uint32_t ts      = 0;     // millis() value of the last good read
};

class PylontechEmu {
public:
    PylontechEmu(HardwareSerial &port, uint8_t respAddr = 0x02)
        : _port(port), _addr(respAddr) {}

    // Read all data that the inverter sent. Answer each frame with our address.
    // This call is cheap and non-blocking. Call it as often as possible.
    void poll(const PackSnapshot &s);

    uint32_t answered = 0;
    uint32_t lastPollMs = 0;   // millis() value of the last frame from the inverter
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
