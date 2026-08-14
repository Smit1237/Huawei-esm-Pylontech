#pragma once
#include <Arduino.h>

// Where the driver's frame dumps and progress messages go. Defaults to the
// USB console; the remote-debug firmware points it at a tee (Serial + WiFi)
// so the RS485 traffic can be watched over the network.
extern Print *g_esmLog;

// Huawei ESM battery Modbus RTU driver.
// The BMS speaks Modbus RTU at 9600 baud but requires a proprietary
// handshake (FC03 unlock read + FC10 time sync + FC41 access grant)
// before it serves data. Slave address is typically 214..218.
// Protocol reverse-engineered by the community:
//   https://github.com/williamsioSapo/Huawei_Battery_monitor

// Registers 0x0000..0x0006 read as one block
struct EsmMainData {
    uint16_t busbarVoltage;   // 0x0000
    uint16_t batteryVoltage;  // 0x0001
    int16_t  current;         // 0x0002 (signed: - discharge / + charge)
    uint16_t soc;             // 0x0003 %
    uint16_t soh;             // 0x0004 %
    int16_t  cellTempMax;     // 0x0005
    int16_t  cellTempMin;     // 0x0006
};

// Registers 0x0046..0x004A read as one block
struct EsmStatusBits {
    uint16_t systemFaults;      // 0x0046
    uint16_t cellFaults;        // 0x0047
    uint16_t activeProtections; // 0x0048
    uint16_t activeWarnings;    // 0x0049
    uint16_t specialStates;     // 0x004A
};

class HuaweiESM {
public:
    HuaweiESM(HardwareSerial &port, int8_t rxPin, int8_t txPin);

    // Scans slave addresses 214..218 under 8N1 then 8E1 until the battery
    // answers an FC03 read. Returns true and locks in addr/parity on success.
    bool probe();

    // Repeated reads with escalating delays until the BMS starts answering
    // (it can take several seconds to wake its comms after silence).
    bool wake();

    // Three-phase unlock: FC03 0x0106 -> FC10 time sync -> FC41 access.
    // Must be re-run after any comms loss or battery reset.
    bool authenticate();

    bool readMainData(EsmMainData &out);
    bool readStatusBits(EsmStatusBits &out);
    bool readRegisters(uint16_t startReg, uint16_t count, uint16_t *out);

    // Confirmed on ESM-4875A1: 15 cell voltages (mV) at 0x0022,
    // 16 temp sensors at 0x0012, 32-bit cycles/Ah at 0x0042/0x0044.
    bool readCells(uint16_t mv[15], int16_t temps[16]);
    bool readStats(uint32_t &cycles, uint32_t &ahTotal);

    // FC41 device info dump (vendor, board type, barcode, manufacture date).
    // Only works after authenticate(). Prints raw ASCII to console.
    void printDeviceInfo(Stream &console);

    // Concatenated printable FC41 archive text (control bytes -> '\n').
    // Returns bytes written. Only works after authenticate().
    size_t readArchiveInfo(char *buf, size_t maxLen);

    // Battery's own RTC (registers 0x1000..0x1005: Y M D h m s)
    bool readClock(uint16_t out[6]);

    uint8_t  slaveAddr() const { return _addr; }
    bool     usesEvenParity() const { return _parity == SERIAL_8E1; }
    void     setTarget(uint8_t addr, bool evenParity);

    // lifetime byte counters (diagnostics): every byte written to and
    // received from the bus, including frames that later fail CRC
    uint32_t txBytes = 0;
    uint32_t rxBytes = 0;

private:
    void   reconfigureSerial(uint32_t parityMode);
    size_t transact(const uint8_t *req, size_t reqLen,
                    uint8_t *resp, size_t respMax, uint32_t timeoutMs);
    bool   fc03(uint16_t startReg, uint16_t count, uint16_t *out,
                uint32_t timeoutMs = 1000);
    bool   fc10TimeSync();
    bool   fc41Access();

    static uint16_t crc16(const uint8_t *data, size_t len);

    HardwareSerial &_port;
    int8_t   _rxPin, _txPin;
    uint8_t  _addr   = 217;
    uint32_t _parity = SERIAL_8N1;
};
