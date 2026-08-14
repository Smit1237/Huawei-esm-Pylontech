#pragma once
#include <Arduino.h>

// Pylontech RS485 (YDN1363) battery emulator for the Deye BMS port,
// "Lithium mode 12". Answers the inverter's CID1=0x46 / CID2=0x61,0x62,0x63
// polls with real Huawei ESM data.
//
// Direct port of the PROVEN tools/deye/deye_bms.py, which ran against the
// real Deye SUN-6K/8K-SG05LP1. Frame layout, units and the fixed protocol
// marker constants are kept byte-identical — do not "tidy" them.

// Latest reading of the ESM pack, shared between the poller and the
// responder. Fields mirror the JSON snapshot the Python emulator consumed.
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
    // battery's own limit registers, raw (0x100A block)
    uint16_t cvl_raw = 0;     // 0.1 V  charge voltage
    uint16_t dvl_raw = 0;     // 0.1 V  discharge cutoff
    uint16_t ccl_raw = 0;     // 0.1 A  charge current
    uint16_t dcl_raw = 0;     // 0.1 A  discharge current
    uint16_t ccl_coef = 0;    // %      0x1011
    uint16_t dcl_coef = 0;    // %      0x1018
    // extended telemetry — populated by the bank firmware; the emu ignores these
    float    busV     = 0;    // busbar voltage reg 0x0000 (V)
    uint32_t ahTotal  = 0;    // lifetime Ah throughput (reg 0x0044)
    uint16_t sysFault = 0;    // 0x0046 system faults
    uint16_t cellFault= 0;    // 0x0047 cell faults
    uint16_t prot     = 0;    // 0x0048 active protections
    uint16_t warn     = 0;    // 0x0049 active warnings
    uint16_t special  = 0;    // 0x004A special states
    uint32_t ts      = 0;     // millis() of last good read
};

class PylontechEmu {
public:
    PylontechEmu(HardwareSerial &port, uint8_t respAddr = 0x02)
        : _port(port), _addr(respAddr) {}

    // Drain whatever the inverter sent, answer any frame addressed to us.
    // Cheap and non-blocking — call it as often as possible.
    void poll(const PackSnapshot &s);

    uint32_t answered = 0;
    uint32_t lastPollMs = 0;   // millis() of the last frame the Deye sent us
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
