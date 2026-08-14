#include <TFT_eSPI.h>
#include "display.h"

// Full-screen sprite (240x135x16bit = ~65 KB) so redraws don't flicker
static TFT_eSPI tft;
static TFT_eSprite spr(&tft);

void displayInit() {
    tft.init();
    tft.setRotation(1);  // landscape, 240x135, USB to the right
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    spr.setColorDepth(16);
    spr.createSprite(240, 135);
    spr.fillSprite(TFT_BLACK);
    spr.pushSprite(0, 0);
}

void displayStatus(const char *line1, const char *line2) {
    spr.fillSprite(TFT_BLACK);
    spr.setTextDatum(TC_DATUM);
    spr.setTextColor(TFT_CYAN, TFT_BLACK);
    spr.drawString("Huawei ESM monitor", 120, 8, 2);
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.drawString(line1, 120, 55, 4);
    spr.setTextColor(TFT_SILVER, TFT_BLACK);
    spr.drawString(line2, 120, 95, 2);
    spr.pushSprite(0, 0);
}

void displayTranslator(const TranslatorView &v) {
    spr.fillSprite(TFT_BLACK);
    char buf[48];

    // --- header: title + ESM link state ---
    spr.setTextDatum(TL_DATUM);
    spr.setTextColor(TFT_CYAN, TFT_BLACK);
    spr.drawString("ESM", 6, 2, 2);
    spr.setTextColor(TFT_DARKGREY, TFT_BLACK);
    spr.drawString(">>", 40, 2, 2);
    spr.setTextColor(TFT_CYAN, TFT_BLACK);
    spr.drawString("Deye", 62, 2, 2);

    uint16_t stCol = v.packValid ? TFT_GREEN
                     : (!strcmp(v.esmState, "STALE") ? TFT_RED : TFT_YELLOW);
    spr.setTextColor(stCol, TFT_BLACK);
    spr.setTextDatum(TR_DATUM);
    if (v.addr > 0) snprintf(buf, sizeof(buf), "%s @%d", v.esmState, v.addr);
    else            snprintf(buf, sizeof(buf), "%s", v.esmState);
    spr.drawString(buf, 234, 2, 2);
    spr.setTextDatum(TL_DATUM);

    spr.drawFastHLine(0, 20, 240, TFT_DARKGREY);

    // --- ESM pack block ---
    if (v.packValid) {
        uint16_t socCol = v.soc > 40 ? TFT_GREEN : v.soc > 15 ? TFT_YELLOW : TFT_RED;
        spr.setTextColor(socCol, TFT_BLACK);
        snprintf(buf, sizeof(buf), "%d%%", v.soc);
        spr.drawString(buf, 6, 26, 6);            // big SOC digits

        spr.setTextColor(TFT_WHITE, TFT_BLACK);
        snprintf(buf, sizeof(buf), "%.2fV", v.packV);
        spr.setTextDatum(TR_DATUM);
        spr.drawString(buf, 234, 28, 4);
        uint16_t iCol = v.current > 0 ? TFT_GREEN : v.current < 0 ? TFT_ORANGE : TFT_SILVER;
        spr.setTextColor(iCol, TFT_BLACK);
        snprintf(buf, sizeof(buf), "%+.2fA", v.current);
        spr.drawString(buf, 234, 54, 4);
        spr.setTextDatum(TL_DATUM);

        spr.setTextColor(TFT_SILVER, TFT_BLACK);
        snprintf(buf, sizeof(buf), "SOH %d%%  %d..%dC  cyc %d",
                 v.soh, v.tmin, v.tmax, v.cycles);
        spr.drawString(buf, 6, 78, 2);
    } else {
        spr.setTextColor(TFT_YELLOW, TFT_BLACK);
        spr.drawString(v.esmState, 6, 34, 4);
        spr.setTextColor(TFT_DARKGREY, TFT_BLACK);
        spr.drawString("waiting for battery on RS485", 6, 66, 2);
    }

    spr.drawFastHLine(0, 96, 240, TFT_DARKGREY);

    // --- Deye side ---
    bool recent = v.deyeAgeMs != UINT32_MAX && v.deyeAgeMs < 3000;
    uint16_t dCol = recent ? TFT_GREEN
                    : (v.deyeAnswered ? TFT_YELLOW : TFT_DARKGREY);
    spr.setTextColor(dCol, TFT_BLACK);
    if (v.deyeAgeMs == UINT32_MAX)
        snprintf(buf, sizeof(buf), "Deye: no polls yet");
    else
        snprintf(buf, sizeof(buf), "Deye: %lu ans  cid%s  %lus",
                 (unsigned long)v.deyeAnswered, v.deyeCid2,
                 (unsigned long)(v.deyeAgeMs / 1000));
    spr.drawString(buf, 6, 100, 2);

    // bottom line: forwarded limits once linked, else raw bus byte counters
    // (the bring-up view — rx staying 0 means the battery isn't answering).
    if (v.haveLimits) {
        spr.setTextColor(TFT_SILVER, TFT_BLACK);
        snprintf(buf, sizeof(buf), "CCL %.1fA  DCL %.1fA", v.cclEff, v.dclEff);
        spr.drawString(buf, 6, 118, 2);
    } else {
        spr.setTextColor(TFT_SILVER, TFT_BLACK);
        snprintf(buf, sizeof(buf), "ESM tx%lu", (unsigned long)v.esmTx);
        spr.drawString(buf, 6, 118, 2);
        spr.setTextColor(v.esmRx ? TFT_GREEN : TFT_RED, TFT_BLACK);
        snprintf(buf, sizeof(buf), "rx%lu", (unsigned long)v.esmRx);
        spr.setTextDatum(TR_DATUM);
        spr.drawString(buf, 234, 118, 2);
        spr.setTextDatum(TL_DATUM);
    }

    spr.pushSprite(0, 0);
}

void displayBank(const BankView &v) {
    spr.fillSprite(TFT_BLACK);
    char buf[64];

    spr.setTextDatum(TL_DATUM);
    spr.setTextColor(TFT_CYAN, TFT_BLACK);
    spr.drawString("ESM bank", 6, 2, 2);
    // WiFi IP + MQTT dot, right side
    spr.setTextColor(v.mqttOn ? (v.mqttOk ? TFT_GREEN : TFT_RED) : TFT_DARKGREY, TFT_BLACK);
    spr.setTextDatum(TR_DATUM);
    spr.drawString(v.mqttOn ? (v.mqttOk ? "MQTT" : "mqtt?") : "", 234, 2, 2);
    spr.setTextColor(TFT_SILVER, TFT_BLACK);
    spr.drawString(v.ip, 234, 120, 1);
    spr.setTextDatum(TL_DATUM);
    spr.drawFastHLine(0, 20, 240, TFT_DARKGREY);

    if (!v.valid) {
        spr.setTextColor(TFT_YELLOW, TFT_BLACK);
        spr.drawString("reading packs...", 6, 40, 4);
        snprintf(buf, sizeof(buf), "%d found", v.nPacks);
        spr.setTextColor(TFT_SILVER, TFT_BLACK);
        spr.drawString(buf, 6, 78, 2);
        spr.pushSprite(0, 0);
        return;
    }

    // big aggregate SOC
    uint16_t socCol = v.soc > 40 ? TFT_GREEN : v.soc > 15 ? TFT_YELLOW : TFT_RED;
    spr.setTextColor(socCol, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%d%%", v.soc);
    spr.drawString(buf, 6, 24, 6);

    // V / A on the right
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%.1fV", v.packV);
    spr.setTextDatum(TR_DATUM);
    spr.drawString(buf, 234, 26, 4);
    uint16_t iCol = v.current > 0 ? TFT_GREEN : v.current < 0 ? TFT_ORANGE : TFT_SILVER;
    spr.setTextColor(iCol, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%+.1fA", v.current);
    spr.drawString(buf, 234, 52, 4);
    spr.setTextDatum(TL_DATUM);

    // per-pack SOC row
    spr.setTextColor(TFT_SILVER, TFT_BLACK);
    int x = 6;
    for (int i = 0; i < v.nPacks && i < 6; i++) {
        snprintf(buf, sizeof(buf), "%d:%d%%", v.packAddr[i], v.packSoc[i]);
        spr.drawString(buf, x, 80, 2);
        x += 58;
    }

    // limits + link line
    spr.setTextColor(TFT_DARKCYAN, TFT_BLACK);
    snprintf(buf, sizeof(buf), "SOH %d%%  %d..%dC  CCL %dA DCL %dA",
             v.soh, v.tmin, v.tmax, v.cclA, v.dclA);
    spr.drawString(buf, 6, 102, 1);
    spr.setTextColor(v.deyeAgeMs != UINT32_MAX && v.deyeAgeMs < 3000 ? TFT_GREEN : TFT_DARKGREY,
                     TFT_BLACK);
    if (v.deyeAgeMs == UINT32_MAX) snprintf(buf, sizeof(buf), "Deye: waiting");
    else snprintf(buf, sizeof(buf), "Deye: %lus", (unsigned long)(v.deyeAgeMs / 1000));
    spr.drawString(buf, 6, 114, 1);

    spr.pushSprite(0, 0);
}

// Register scaling for the ESM4875A1 is unconfirmed; pick a scale that
// lands a 48 V-class pack in a plausible range. Serial output keeps
// printing raw values so the guess can be verified and hardcoded later.
static float scaleVoltage(uint16_t raw) {
    if (raw > 1000) return raw * 0.01f;   // 5342 -> 53.42 V
    if (raw > 100)  return raw * 0.1f;    //  534 -> 53.4 V
    return (float)raw;                    //   53 -> 53 V
}

static float scaleCurrent(int16_t raw, uint8_t &decimals) {
    int16_t mag = abs(raw);
    if (mag > 1000) { decimals = 2; return raw * 0.01f; }
    if (mag > 150)  { decimals = 1; return raw * 0.1f; }
    decimals = 0;
    return (float)raw;
}

void displayData(const EsmMainData &d, const EsmStatusBits &s, bool statusValid) {
    spr.fillSprite(TFT_BLACK);

    // --- SOC row: big number + bar ---
    uint16_t socColor = d.soc > 40 ? TFT_GREEN : d.soc > 15 ? TFT_YELLOW : TFT_RED;
    spr.setTextDatum(TL_DATUM);
    spr.setTextColor(socColor, TFT_BLACK);
    char buf[32];
    snprintf(buf, sizeof(buf), "%u%%", d.soc);
    spr.drawString(buf, 6, 4, 6);

    int barX = 110, barY = 12, barW = 124, barH = 22;
    spr.drawRect(barX, barY, barW, barH, TFT_SILVER);
    int fill = (barW - 4) * min<uint16_t>(d.soc, 100) / 100;
    spr.fillRect(barX + 2, barY + 2, fill, barH - 4, socColor);

    // --- voltage / current row ---
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%.2fV", scaleVoltage(d.batteryVoltage));
    spr.drawString(buf, 6, 52, 4);

    uint8_t dec;
    float amps = scaleCurrent(d.current, dec);
    // charging = positive, discharging = negative
    spr.setTextColor(d.current > 0 ? TFT_GREEN : d.current < 0 ? TFT_ORANGE : TFT_SILVER,
                     TFT_BLACK);
    snprintf(buf, sizeof(buf), "%+.*fA", dec, amps);
    spr.setTextDatum(TR_DATUM);
    spr.drawString(buf, 234, 52, 4);
    spr.setTextDatum(TL_DATUM);

    // --- SOH / temperature row ---
    spr.setTextColor(TFT_SILVER, TFT_BLACK);
    snprintf(buf, sizeof(buf), "SOH %u%%", d.soh);
    spr.drawString(buf, 6, 86, 2);
    snprintf(buf, sizeof(buf), "cell %d..%dC", d.cellTempMin, d.cellTempMax);
    spr.setTextDatum(TR_DATUM);
    spr.drawString(buf, 234, 86, 2);
    spr.setTextDatum(TL_DATUM);

    // --- fault / warning row ---
    if (statusValid) {
        bool anyFault = s.systemFaults || s.cellFaults || s.activeProtections;
        if (anyFault || s.activeWarnings) {
            spr.setTextColor(anyFault ? TFT_RED : TFT_YELLOW, TFT_BLACK);
            snprintf(buf, sizeof(buf), "FLT %04X/%04X PROT %04X WRN %04X",
                     s.systemFaults, s.cellFaults, s.activeProtections,
                     s.activeWarnings);
        } else {
            spr.setTextColor(TFT_DARKGREEN, TFT_BLACK);
            snprintf(buf, sizeof(buf), "no faults / warnings");
        }
        spr.drawString(buf, 6, 115, 2);
    } else {
        spr.setTextColor(TFT_DARKGREY, TFT_BLACK);
        spr.drawString("status read failed", 6, 115, 2);
    }

    spr.pushSprite(0, 0);
}
