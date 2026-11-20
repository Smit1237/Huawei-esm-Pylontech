#pragma once
#include <Arduino.h>

// This pointer sets the destination for the frame dumps and progress
// messages of the driver. The default destination is the USB console. The
// remote-debug firmware sends the output to a tee (Serial and WiFi). This
// lets you monitor the RS485 traffic on the network.
extern Print *g_esmLog;

// Huawei ESM battery Modbus RTU driver.
// The BMS uses Modbus RTU at 9600 baud. The BMS needs a proprietary
// handshake before it sends data. The handshake has three steps: FC03
// unlock read, FC10 time sync, and FC41 access grant. The slave address is
// usually 214 to 218.
// The community found the protocol:
//   https://github.com/williamsioSapo/Huawei_Battery_monitor

// Read registers 0x0000 to 0x0006 as one block
struct EsmMainData {
    uint16_t busbarVoltage;   // 0x0000
    uint16_t batteryVoltage;  // 0x0001
    int16_t  current;         // 0x0002 (signed: minus is discharge, plus is charge)
    uint16_t soc;             // 0x0003 %
    uint16_t soh;             // 0x0004 %
    int16_t  cellTempMax;     // 0x0005
    int16_t  cellTempMin;     // 0x0006
};

// Read registers 0x0046 to 0x004A as one block
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

    // Scan the slave addresses 214 to 218 with 8N1, then with 8E1. Stop when
    // the battery answers an FC03 read. On success, return true and keep the
    // address and the parity.
    bool probe();

    // Do repeated reads with longer delays until the BMS answers. The BMS
    // can need several seconds to start its communication after a silent
    // period.
    bool wake();

    // Three-step unlock: FC03 0x0106, then FC10 time sync, then FC41 access.
    // Run this again after each communication loss or battery reset.
    bool authenticate();

    bool readMainData(EsmMainData &out);
    bool readStatusBits(EsmStatusBits &out);
    bool readRegisters(uint16_t startReg, uint16_t count, uint16_t *out);

    // Confirmed on the ESM-4875A1: 15 cell voltages (mV) at 0x0022,
    // 16 temperature sensors at 0x0012, and 32-bit cycles and Ah at
    // 0x0042 and 0x0044.
    bool readCells(uint16_t mv[15], int16_t temps[16]);
    bool readStats(uint32_t &cycles, uint32_t &ahTotal);

    // FC41 device information dump (vendor, board type, barcode, manufacture
    // date). This works only after authenticate(). It prints raw ASCII to the
    // console.
    void printDeviceInfo(Stream &console);

    // Joined printable FC41 archive text (control bytes become '\n').
    // Return the number of bytes written. This works only after authenticate().
    size_t readArchiveInfo(char *buf, size_t maxLen);

    // The RTC of the battery (registers 0x1000 to 0x1005: Y M D h m s)
    bool readClock(uint16_t out[6]);

    uint8_t  slaveAddr() const { return _addr; }
    bool     usesEvenParity() const { return _parity == SERIAL_8E1; }
    void     setTarget(uint8_t addr, bool evenParity);

    // Lifetime byte counters for diagnostics. These count each byte written
    // to the bus and received from the bus. These include frames that fail
    // CRC.
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
    uint8_t  _addr    = 217;
    uint32_t _parity  = SERIAL_8N1;
    bool     _started = false;
};
