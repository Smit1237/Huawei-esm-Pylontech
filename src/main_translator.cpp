// Standalone ESM <-> Pylontech translator for the Deye BMS port.
//
// Replaces the gateway PC entirely: the ESP32 reads the Huawei ESM pack on
// one RS485 bus and emulates a Pylontech battery to the Deye inverter on a
// second RS485 bus (Deye "Lithium mode 12"). Both sides are the proven
// implementations — huawei_esm.cpp (Modbus + FC41 handshake) and
// pylontech.cpp (ported from tools/deye/deye_bms.py).
//
// WHY DUAL-CORE: the Deye polls ~1.7x/s and expects a prompt answer, while an
// ESM Modbus transaction can block for up to a second. The two jobs run as
// separate FreeRTOS tasks pinned to different cores, sharing one mutexed
// snapshot — so a slow battery read can never delay an inverter reply. This
// is the on-chip equivalent of the two separate Python processes + HTTP cache.
//
// WIRING — two auto-direction RS485 modules (VCC/GND/TXD/RXD only).
// STRAIGHT THROUGH, not crossed: these modules name their pins by signal
// function, so TXD takes the MCU's TX and RXD feeds the MCU's RX. This is
// the wiring proven in README.md ("TXD | GPIO4 (TX)", "RXD | GPIO5 (RX)").
//   ESM  side: ESP32 GPIO33 (TX) -> module TXD,  GPIO32 (RX) <- module RXD
//   Deye side: ESP32 GPIO26 (TX) -> module TXD,  GPIO25 (RX) <- module RXD
//   Both modules: VCC -> 5V (as proven; see README), GND -> ESP32 GND
//   ESM module  A+/B- -> battery RJ45 pins 1+4 / 2+5
//   Deye module A+/B- -> inverter BMS RJ45 (see README for its pinout)

#include <Arduino.h>
#include "huawei_esm.h"
#include "pylontech.h"
#ifdef USE_DISPLAY
#include "display.h"
#endif

#ifndef ESM_TX_PIN
#define ESM_TX_PIN 33
#endif
#ifndef ESM_RX_PIN
#define ESM_RX_PIN 32
#endif
#ifndef DEYE_TX_PIN
#define DEYE_TX_PIN 26
#endif
#ifndef DEYE_RX_PIN
#define DEYE_RX_PIN 25
#endif
#ifndef DEYE_ADDR
#define DEYE_ADDR 0x02
#endif

static HuaweiESM   esm(Serial1, ESM_RX_PIN, ESM_TX_PIN);
static PylontechEmu deye(Serial2, DEYE_ADDR);

static PackSnapshot     shared;          // guarded by snapMux
static SemaphoreHandle_t snapMux;

// ESM link state for the display (pointer to a literal -> atomic to publish)
static volatile const char *g_esmState = "PROBE";
static volatile int         g_esmAddr  = 0;

static const uint32_t SLOW_MS = 10000;   // cells / cycles / limits cadence

// ---------------- ESM side (core 0) ----------------

// One-shot inventory printed when a pack first links: reads and dumps EVERY
// queryable field so we can confirm each Modbus query individually, not just
// that the link came up. Any "READ FAILED" line pinpoints exactly which query
// the battery won't serve.
static void dumpEverything() {
    Serial.println("\n===== ESM FULL QUERY DUMP =====");
    Serial.printf("addr %u  parity %s\n", esm.slaveAddr(),
                  esm.usesEvenParity() ? "8E1" : "8N1");

    EsmMainData d;
    if (esm.readMainData(d))
        Serial.printf("main : busbar=%.2fV pack=%.2fV I=%+.2fA SOC=%u%% SOH=%u%% "
                      "Tmax=%dC Tmin=%dC\n",
                      d.busbarVoltage * 0.01, d.batteryVoltage * 0.01,
                      d.current * 0.01, d.soc, d.soh, d.cellTempMax, d.cellTempMin);
    else Serial.println("main : READ FAILED");

    uint16_t mv[15]; int16_t t[16];
    if (esm.readCells(mv, t)) {
        uint16_t lo = 0xFFFF, hi = 0;
        Serial.print("cells: ");
        for (int i = 0; i < 15; i++) {
            Serial.printf("%u ", mv[i]);
            if (mv[i]) { if (mv[i] < lo) lo = mv[i]; if (mv[i] > hi) hi = mv[i]; }
        }
        Serial.printf(" mV  spread=%u mV\n", (lo != 0xFFFF) ? (hi - lo) : 0);
        Serial.print("temps: ");
        for (int i = 0; i < 16; i++) Serial.printf("%d ", t[i]);
        Serial.println("C");
    } else Serial.println("cells: READ FAILED");

    uint32_t cyc, ah;
    if (esm.readStats(cyc, ah))
        Serial.printf("stats: cycles=%lu  ahTotal=%lu\n",
                      (unsigned long)cyc, (unsigned long)ah);
    else Serial.println("stats: READ FAILED");

    uint16_t lim[15];
    if (esm.readRegisters(0x100A, 15, lim)) {
        Serial.printf("limit: CVL=%.1fV DCL=%.1fA CCL=%.1fA DVL=%.1fV "
                      "ccoef=%u%% dcoef=%u%%\n",
                      lim[0] * 0.1, lim[1] * 0.1, lim[3] * 0.1, lim[4] * 0.1,
                      lim[7], lim[14]);
        Serial.print("  0x100A raw:");
        for (int i = 0; i < 15; i++) Serial.printf(" [%d]=%u", i, lim[i]);
        Serial.println();
    } else Serial.println("limit: READ FAILED");

    EsmStatusBits sb;
    if (esm.readStatusBits(sb))
        Serial.printf("stat : sysFlt=%04X cellFlt=%04X prot=%04X warn=%04X spc=%04X\n",
                      sb.systemFaults, sb.cellFaults, sb.activeProtections,
                      sb.activeWarnings, sb.specialStates);
    else Serial.println("stat : READ FAILED");

    uint16_t clk[6];
    if (esm.readClock(clk))
        Serial.printf("clock: %04u-%02u-%02u %02u:%02u:%02u\n",
                      clk[0], clk[1], clk[2], clk[3], clk[4], clk[5]);
    else Serial.println("clock: READ FAILED");

    static char blob[1024];
    if (esm.readArchiveInfo(blob, sizeof(blob))) {
        Serial.println("nameplate (FC41 archive):");
        Serial.println(blob);
    } else Serial.println("nameplate: READ FAILED");
    Serial.println("===============================\n");
}

static void esmTask(void *) {
    // probe() opens the port and sweeps addresses 214..218 across 8N1/8E1
    for (;;) {
        g_esmState = "PROBE"; g_esmAddr = 0;
        Serial.println("[esm] probing bus ...");
        if (!esm.probe()) {
            Serial.println("[esm] no battery found, retrying in 5 s");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        g_esmAddr = esm.slaveAddr();
        g_esmState = "AUTH";
        Serial.printf("[esm] battery at addr %u\n", esm.slaveAddr());
        esm.wake();
        if (!esm.authenticate()) {
            Serial.println("[esm] auth failed, retrying");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        g_esmState = "LINK";
        Serial.println("[esm] authenticated");
        dumpEverything();      // one-shot full inventory for verification

        uint32_t lastSlow = 0;
        bool     firstSlow = true;   // force the slow reads on iteration 1 so
                                     // the first snapshot carries REAL limits/
                                     // cells/cycles, not the fallbacks — the
                                     // Python poller gets this for free by
                                     // gating on a unix timestamp
        uint8_t  fails = 0;
        PackSnapshot local;

        while (fails < 5) {
            EsmMainData d;
            if (!esm.readMainData(d)) { fails++; vTaskDelay(pdMS_TO_TICKS(500)); continue; }
            fails = 0;

            local.packV   = d.batteryVoltage * 0.01f;
            local.current = d.current * 0.01f;
            local.soc     = (uint8_t)d.soc;
            local.soh     = (uint8_t)d.soh;
            local.tmin    = d.cellTempMin;
            local.tmax    = d.cellTempMax;

            if (firstSlow || millis() - lastSlow > SLOW_MS) {
                firstSlow = false;
                lastSlow = millis();
                uint16_t mv[15]; int16_t t[16];
                if (esm.readCells(mv, t)) {
                    local.nCells = 0;
                    for (int i = 0; i < 15; i++)
                        if (mv[i]) local.cells[local.nCells++] = mv[i];
                    local.nTemps = 0;
                    for (int i = 0; i < 15 && local.nTemps < 15; i++)
                        if (t[i] > -100 && t[i] < 200) local.ctemps[local.nTemps++] = t[i];
                }
                uint32_t cyc, ah;
                if (esm.readStats(cyc, ah))
                    local.cycles = cyc > 65535 ? 65535 : (uint16_t)cyc;

                // limit registers 0x100A..0x1018 — indices per esm_web.py
                uint16_t lim[15];
                if (esm.readRegisters(0x100A, 15, lim)) {
                    local.cvl_raw  = lim[0];    // 0.1 V charge voltage
                    local.dcl_raw  = lim[1];    // 0.1 A discharge current
                    local.ccl_raw  = lim[3];    // 0.1 A charge current
                    local.dvl_raw  = lim[4];    // 0.1 V discharge cutoff
                    local.ccl_coef = lim[7];    // %  0x1011
                    local.dcl_coef = lim[14];   // %  0x1018
                }
            }

            local.valid = true;
            local.ts    = millis();
            if (xSemaphoreTake(snapMux, pdMS_TO_TICKS(50)) == pdTRUE) {
                shared = local;
                xSemaphoreGive(snapMux);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        g_esmState = "STALE";
        Serial.println("[esm] lost the battery, re-probing");
        if (xSemaphoreTake(snapMux, pdMS_TO_TICKS(50)) == pdTRUE) {
            shared.valid = false;           // stop feeding the Deye stale data
            xSemaphoreGive(snapMux);
        }
    }
}

// ---------------- Deye side (core 1) ----------------

static void deyeTask(void *) {
    PackSnapshot mine;
    for (;;) {
        if (xSemaphoreTake(snapMux, pdMS_TO_TICKS(5)) == pdTRUE) {
            mine = shared;
            xSemaphoreGive(snapMux);
        }
        // Refuse to answer with stale data: if the pack has been unreadable
        // for >15 s, go silent so the inverter raises a BMS-comms fault
        // rather than acting on numbers that no longer reflect the battery.
        if (mine.valid && millis() - mine.ts > 15000) mine.valid = false;
        deye.poll(mine);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== ESM <-> Pylontech translator ===");
    Serial.printf("ESM  UART1: TX=%d RX=%d\n", ESM_TX_PIN, ESM_RX_PIN);
    Serial.printf("Deye UART2: TX=%d RX=%d  (responding as addr 0x%02X)\n",
                  DEYE_TX_PIN, DEYE_RX_PIN, DEYE_ADDR);

    Serial2.begin(9600, SERIAL_8N1, DEYE_RX_PIN, DEYE_TX_PIN);

#ifdef USE_DISPLAY
    displayInit();
    displayStatus("ESM <-> Deye", "starting...");
#endif

    snapMux = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(esmTask,  "esm",  6144, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(deyeTask, "deye", 4096, nullptr, 3, nullptr, 1);
}

void loop() {
    static uint32_t lastStat = 0, lastDisp = 0;
    uint32_t now = millis();

    PackSnapshot s;
    if (xSemaphoreTake(snapMux, pdMS_TO_TICKS(20)) == pdTRUE) {
        s = shared;
        xSemaphoreGive(snapMux);
    }
    bool fresh = s.valid && (now - s.ts < 15000);

#ifdef USE_DISPLAY
    // Refresh at ~2.5 Hz from loopTask (core 1, prio 1). The Deye responder
    // runs at prio 3, so it always preempts a redraw — a slow SPI push can
    // never delay an inverter reply.
    if (now - lastDisp >= 400) {
        lastDisp = now;
        uint32_t cc = s.ccl_coef ? s.ccl_coef : 100;
        uint32_t dc = s.dcl_coef ? s.dcl_coef : 100;
        TranslatorView tv;
        tv.packValid = fresh;
        tv.esmState  = (const char *)g_esmState;
        tv.addr      = g_esmAddr;
        tv.soc = s.soc; tv.soh = s.soh;
        tv.packV = s.packV; tv.current = s.current;
        tv.tmin = s.tmin; tv.tmax = s.tmax; tv.cycles = s.cycles;
        tv.deyeAnswered = deye.answered;
        tv.deyeAgeMs = deye.lastPollMs ? (now - deye.lastPollMs) : UINT32_MAX;
        tv.deyeCid2  = deye.lastCid2[0] ? deye.lastCid2 : "-";
        tv.haveLimits = fresh && s.ccl_raw;
        tv.cclEff = (s.ccl_raw * cc / 100) / 10.0f;
        tv.dclEff = (s.dcl_raw * dc / 100) / 10.0f;
        tv.esmTx = esm.txBytes;
        tv.esmRx = esm.rxBytes;
        displayTranslator(tv);
    }
#endif

    if (now - lastStat >= 5000) {
        lastStat = now;
        uint32_t sincePoll = deye.lastPollMs ? (now - deye.lastPollMs) : 0;
        Serial.printf("[stat] pack %s %.2fV %+.2fA soc=%u%% soh=%u%% %d..%dC cyc=%u | "
                      "deye polls: %lu answered, last %lus ago (cid2=%s)\n",
                      fresh ? "OK" : "STALE", s.packV, s.current, s.soc, s.soh,
                      s.tmin, s.tmax, s.cycles,
                      (unsigned long)deye.answered,
                      (unsigned long)(sincePoll / 1000),
                      deye.lastCid2[0] ? deye.lastCid2 : "-");
        if (fresh && s.ccl_raw)
            Serial.printf("[stat] limits: CVL %.1fV DVL %.1fV CCL %u(0.1A)x%u%% DCL %u(0.1A)x%u%%\n",
                          s.cvl_raw * 0.1f, s.dvl_raw * 0.1f,
                          s.ccl_raw, s.ccl_coef, s.dcl_raw, s.dcl_coef);
    }
    delay(20);
}
