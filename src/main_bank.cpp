// ESM battery BANK -> Deye translator, with WiFi / WebUI / MQTT / OTA.
//
// The four Huawei ESM packs self-address to 214..217 on ONE shared RS485 bus
// (this works once the Huawei CAN matching-resistor is installed on the last
// pack's COM_OUT). This firmware:
//   * polls each pack (wake + FC41 auth per address, then FC03 reads),
//   * aggregates them into ONE virtual battery,
//   * emits Pylontech RS485 (Deye "Lithium mode 12") on a second bus,
//   * serves a web UI to configure optional MQTT (aggregate + per-pack),
//   * supports OTA (ArduinoOTA via espota, AND browser upload) so it never
//     needs a USB cable again.
//
// Hardware (unchanged from the translator):
//   ESM  bus: Serial1  GPIO33 TX -> module TXD, GPIO32 RX <- module RXD
//   Deye bus: Serial2  GPIO26 TX -> module TXD, GPIO25 RX <- module RXD
//   (straight-through auto-direction RS485 modules, VCC 5V, common GND)

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <time.h>
#include "huawei_esm.h"
#include "pylontech.h"
#ifdef USE_DISPLAY
#include "display.h"
#endif

// WiFi is OPTIONAL — the RS485 translation to the Deye works without it. WiFi
// only enables the WebUI, OTA and MQTT. Set your credentials by copying
// src/wifi_config.example.h to src/wifi_config.h (git-ignored) and editing it,
// or by passing -DWIFI_SSID=... -DWIFI_PASS=... build flags.
#if defined(__has_include)
#  if __has_include("wifi_config.h")
#    include "wifi_config.h"
#  endif
#endif
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#endif
#ifndef OTA_PASS
#define OTA_PASS "esmbank-ota"
#endif
// Fallback setup access point: if no working WiFi is configured, the device
// hosts this AP so you can join it and enter your network on the config page.
#ifndef AP_SSID
#define AP_SSID "ESM-Bank-Setup"
#endif
#ifndef AP_PASS
#define AP_PASS "esmbank-setup"   // >= 8 chars (WPA2); the setup-AP password
#endif
// Bump this on every build so you can confirm — in the WebUI header and the
// serial banner — exactly which image is live (proves an OTA push actually took).
#ifndef FW_VERSION
#define FW_VERSION "bank-1.7"
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

static const uint8_t PACK_ADDR[] = { 214, 215, 216, 217 };
#define NUM_PACKS (sizeof(PACK_ADDR) / sizeof(PACK_ADDR[0]))
#define STALE_MS  15000       // pack unreadable this long -> drop from bank

HuaweiESM  esm(Serial1, ESM_RX_PIN, ESM_TX_PIN);
PylontechEmu deye(Serial2, DEYE_ADDR);

struct Pack {
    uint8_t  addr;
    bool     authed = false;
    uint8_t  fails = 0;
    char     serial[24] = "";
    PackSnapshot s;           // s.valid = fresh data; s.ts = last read
};
static Pack pk[NUM_PACKS];
static PackSnapshot agg;                 // aggregate handed to the Deye emu
static SemaphoreHandle_t mux;            // guards pk[] + agg
static TaskHandle_t esmH = nullptr, deyeH = nullptr;  // suspended during web OTA

// ---------------- config (persisted) ----------------
struct Config {
    bool     mqttEnable = false;
    char     host[64] = "";
    uint16_t port = 1883;
    char     user[32] = "";
    char     pass[64] = "";
    char     topic[32] = "esmbank";
    bool     perPack = true;
    uint16_t interval = 5;               // publish period, seconds
    // time: up to 3 NTP servers (only ntp1 needed) + POSIX TZ string
    char     ntp1[48] = "pool.ntp.org";
    char     ntp2[48] = "";
    char     ntp3[48] = "";
    char     tz[40]   = "Europe/Moscow"; // IANA name or raw POSIX (offset sign is INVERTED)
    // WiFi credentials set at runtime (setup AP / WebUI). Empty -> fall back to
    // the compile-time WIFI_SSID/WIFI_PASS, and if those are unset, the setup AP.
    char     wifiSsid[33] = "";
    char     wifiPass[65] = "";
};
static Config cfg;
static Preferences prefs;

static void loadCfg() {
    prefs.begin("bank", true);
    cfg.mqttEnable = prefs.getBool("en", false);
    prefs.getString("host", cfg.host, sizeof(cfg.host));
    cfg.port = prefs.getUShort("port", 1883);
    prefs.getString("user", cfg.user, sizeof(cfg.user));
    prefs.getString("pass", cfg.pass, sizeof(cfg.pass));
    String t = prefs.getString("topic", "esmbank"); t.toCharArray(cfg.topic, sizeof(cfg.topic));
    cfg.perPack = prefs.getBool("pp", true);
    cfg.interval = prefs.getUShort("intv", 5);
    prefs.getString("ntp1", cfg.ntp1, sizeof(cfg.ntp1));
    prefs.getString("ntp2", cfg.ntp2, sizeof(cfg.ntp2));
    prefs.getString("ntp3", cfg.ntp3, sizeof(cfg.ntp3));
    { String z = prefs.getString("tz", "Europe/Moscow"); z.toCharArray(cfg.tz, sizeof(cfg.tz)); }
    prefs.getString("wssid", cfg.wifiSsid, sizeof(cfg.wifiSsid));
    prefs.getString("wpass", cfg.wifiPass, sizeof(cfg.wifiPass));
    prefs.end();
    if (cfg.interval < 1) cfg.interval = 1;
    if (!cfg.ntp1[0]) strcpy(cfg.ntp1, "pool.ntp.org");
    if (!cfg.tz[0])   strcpy(cfg.tz, "Europe/Moscow");
}
static void saveCfg() {
    prefs.begin("bank", false);
    prefs.putBool("en", cfg.mqttEnable);
    prefs.putString("host", cfg.host);
    prefs.putUShort("port", cfg.port);
    prefs.putString("user", cfg.user);
    prefs.putString("pass", cfg.pass);
    prefs.putString("topic", cfg.topic);
    prefs.putBool("pp", cfg.perPack);
    prefs.putUShort("intv", cfg.interval);
    prefs.putString("ntp1", cfg.ntp1);
    prefs.putString("ntp2", cfg.ntp2);
    prefs.putString("ntp3", cfg.ntp3);
    prefs.putString("tz", cfg.tz);
    prefs.putString("wssid", cfg.wifiSsid);
    prefs.putString("wpass", cfg.wifiPass);
    prefs.end();
}

// Readable IANA zone names -> POSIX TZ strings (the ESP has no tzdata db, so we
// carry a curated map). Anything not listed is passed through verbatim, so a raw
// POSIX string like "<+03>-3" still works. DST-observing entries use current
// rules; fixed-offset zones never drift. Heavy on Europe/Russia (Smit's region).
struct TzEntry { const char *name; const char *posix; };
static const TzEntry TZ_TABLE[] = {
    { "UTC",                 "UTC0" },
    // --- Europe / Atlantic (EU DST last-Sun-Mar..last-Sun-Oct unless noted) ---
    { "Europe/London",       "GMT0BST,M3.5.0/1,M10.5.0" },
    { "Europe/Dublin",       "GMT0BST,M3.5.0/1,M10.5.0" },
    { "Europe/Lisbon",       "WET0WEST,M3.5.0/1,M10.5.0" },
    { "Atlantic/Reykjavik",  "GMT0" },
    { "Atlantic/Azores",     "<-01>1<+00>,M3.5.0/0,M10.5.0/1" },
    { "Europe/Berlin",       "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Paris",        "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Madrid",       "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Rome",         "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Amsterdam",    "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Brussels",     "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Vienna",       "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Zurich",       "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Prague",       "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Warsaw",       "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Budapest",     "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Stockholm",    "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Oslo",         "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Copenhagen",   "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Belgrade",     "CET-1CEST,M3.5.0,M10.5.0/3" },
    { "Europe/Athens",       "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Helsinki",     "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Bucharest",    "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Sofia",        "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Riga",         "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Vilnius",      "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Tallinn",      "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Chisinau",     "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Kyiv",         "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Kiev",         "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Europe/Kaliningrad",  "<+02>-2" },
    { "Europe/Minsk",        "<+03>-3" },
    { "Europe/Istanbul",     "<+03>-3" },
    { "Europe/Moscow",       "<+03>-3" },
    { "Europe/Samara",       "<+04>-4" },
    // --- Russia (east) / Central Asia (fixed, no DST) ---
    { "Asia/Yekaterinburg",  "<+05>-5" },
    { "Asia/Omsk",           "<+06>-6" },
    { "Asia/Novosibirsk",    "<+07>-7" },
    { "Asia/Krasnoyarsk",    "<+07>-7" },
    { "Asia/Irkutsk",        "<+08>-8" },
    { "Asia/Yakutsk",        "<+09>-9" },
    { "Asia/Vladivostok",    "<+10>-10" },
    { "Asia/Magadan",        "<+11>-11" },
    { "Asia/Kamchatka",      "<+12>-12" },
    { "Asia/Almaty",         "<+05>-5" },
    { "Asia/Tashkent",       "<+05>-5" },
    { "Asia/Bishkek",        "<+06>-6" },
    // --- Middle East ---
    { "Asia/Tbilisi",        "<+04>-4" },
    { "Asia/Yerevan",        "<+04>-4" },
    { "Asia/Baku",           "<+04>-4" },
    { "Asia/Dubai",          "<+04>-4" },
    { "Asia/Tehran",         "<+0330>-3:30" },
    { "Asia/Baghdad",        "<+03>-3" },
    { "Asia/Riyadh",         "<+03>-3" },
    { "Asia/Jerusalem",      "IST-2IDT,M3.4.4/26,M10.5.0" },
    { "Asia/Beirut",         "EET-2EEST,M3.5.0/3,M10.5.0/4" },
    { "Asia/Kabul",          "<+0430>-4:30" },
    // --- South / SE / East Asia ---
    { "Asia/Karachi",        "PKT-5" },
    { "Asia/Kolkata",        "IST-5:30" },
    { "Asia/Kathmandu",      "<+0545>-5:45" },
    { "Asia/Dhaka",          "<+06>-6" },
    { "Asia/Bangkok",        "<+07>-7" },
    { "Asia/Jakarta",        "WIB-7" },
    { "Asia/Ho_Chi_Minh",    "<+07>-7" },
    { "Asia/Shanghai",       "CST-8" },
    { "Asia/Hong_Kong",      "HKT-8" },
    { "Asia/Taipei",         "CST-8" },
    { "Asia/Singapore",      "<+08>-8" },
    { "Asia/Kuala_Lumpur",   "<+08>-8" },
    { "Asia/Manila",         "PST-8" },
    { "Asia/Tokyo",          "JST-9" },
    { "Asia/Seoul",          "KST-9" },
    // --- Oceania (southern-hemisphere DST Oct..Apr) ---
    { "Australia/Perth",     "AWST-8" },
    { "Australia/Darwin",    "ACST-9:30" },
    { "Australia/Adelaide",  "ACST-9:30ACDT,M10.1.0,M4.1.0/3" },
    { "Australia/Brisbane",  "AEST-10" },
    { "Australia/Sydney",    "AEST-10AEDT,M10.1.0,M4.1.0/3" },
    { "Australia/Melbourne", "AEST-10AEDT,M10.1.0,M4.1.0/3" },
    { "Pacific/Auckland",    "NZST-12NZDT,M9.5.0,M4.1.0/3" },
    { "Pacific/Fiji",        "<+12>-12" },
    { "Pacific/Guam",        "ChST-10" },
    { "Pacific/Honolulu",    "HST10" },
    // --- North America (US/CA DST 2nd-Sun-Mar..1st-Sun-Nov) ---
    { "America/St_Johns",    "NST3:30NDT,M3.2.0,M11.1.0" },
    { "America/Halifax",     "AST4ADT,M3.2.0,M11.1.0" },
    { "America/New_York",    "EST5EDT,M3.2.0,M11.1.0" },
    { "America/Toronto",     "EST5EDT,M3.2.0,M11.1.0" },
    { "America/Chicago",     "CST6CDT,M3.2.0,M11.1.0" },
    { "America/Denver",      "MST7MDT,M3.2.0,M11.1.0" },
    { "America/Phoenix",     "MST7" },
    { "America/Los_Angeles", "PST8PDT,M3.2.0,M11.1.0" },
    { "America/Vancouver",   "PST8PDT,M3.2.0,M11.1.0" },
    { "America/Anchorage",   "AKST9AKDT,M3.2.0,M11.1.0" },
    { "America/Mexico_City", "CST6" },
    // --- Central / South America ---
    { "America/Bogota",      "<-05>5" },
    { "America/Lima",        "<-05>5" },
    { "America/Santiago",    "<-04>4<-03>,M9.1.6/24,M4.1.6/24" },
    { "America/Sao_Paulo",   "<-03>3" },
    { "America/Argentina/Buenos_Aires", "<-03>3" },
    // --- Africa ---
    { "Africa/Casablanca",   "<+01>-1" },
    { "Africa/Algiers",      "CET-1" },
    { "Africa/Lagos",        "WAT-1" },
    { "Africa/Cairo",        "EET-2EEST,M4.5.5/0,M10.5.4/24" },
    { "Africa/Johannesburg", "SAST-2" },
    { "Africa/Nairobi",      "EAT-3" },
};
#define TZ_TABLE_N (sizeof(TZ_TABLE) / sizeof(TZ_TABLE[0]))

// Resolve an IANA name to POSIX; pass through unknown values (already POSIX).
static const char *tzToPosix(const char *tz) {
    for (unsigned i = 0; i < TZ_TABLE_N; i++)
        if (!strcmp(TZ_TABLE[i].name, tz)) return TZ_TABLE[i].posix;
    return tz;
}

// (Re)apply the NTP + timezone config. Safe to call repeatedly (e.g. on save).
static void applyNtp() {
    const char *s1 = cfg.ntp1[0] ? cfg.ntp1 : "pool.ntp.org";
    const char *s2 = cfg.ntp2[0] ? cfg.ntp2 : nullptr;
    const char *s3 = cfg.ntp3[0] ? cfg.ntp3 : nullptr;
    configTzTime(tzToPosix(cfg.tz[0] ? cfg.tz : "Europe/Moscow"), s1, s2, s3);
}
// UTC epoch seconds, or 0 until NTP has actually synced (guards the 1970 window).
static uint32_t epochNow() {
    time_t t = time(nullptr);
    return t > 1600000000 ? (uint32_t)t : 0;   // 1600000000 = 2020-09
}

// ---------------- ESM polling (core 0) ----------------
static void extractSerial(const char *blob, char *out, size_t n) {
    out[0] = 0;
    const char *p = strstr(blob, "BarCode=");
    if (!p) return;
    p += 8;
    size_t i = 0;
    while (i < n - 1 && (isalnum((unsigned char)p[i]) || p[i] == '-')) { out[i] = p[i]; i++; }
    out[i] = 0;
}

static void pollPack(Pack &p, bool deep) {
    esm.setTarget(p.addr, false);        // 8N1 (also (re)starts the UART)
    if (!p.authed) {
        esm.wake();
        p.authed = esm.authenticate();
        if (!p.authed) { p.fails++; return; }
        static char blob[1024];
        char sn[24] = "";
        if (esm.readArchiveInfo(blob, sizeof(blob))) extractSerial(blob, sn, sizeof(sn));
        if (sn[0] && xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) {
            strncpy(p.serial, sn, sizeof(p.serial)); p.serial[sizeof(p.serial) - 1] = 0;
            xSemaphoreGive(mux);                 // guard the cross-task serial read
        }
    }
    EsmMainData d;
    if (!esm.readMainData(d)) {
        if (++p.fails >= 3) { p.authed = false; p.fails = 0; }  // force re-auth
        return;
    }
    p.fails = 0;

    PackSnapshot n = p.s;                 // build into a local, publish atomically
    n.packV = d.batteryVoltage * 0.01f;
    n.busV = d.busbarVoltage * 0.01f;
    n.current = d.current * 0.01f;
    n.soc = d.soc > 255 ? 255 : (uint8_t)d.soc;
    n.soh = d.soh > 255 ? 255 : (uint8_t)d.soh;
    n.tmin = (int8_t)d.cellTempMin;
    n.tmax = (int8_t)d.cellTempMax;
    n.valid = true;
    n.ts = millis();

    if (deep) {
        uint16_t mv[15]; int16_t t[16];
        if (esm.readCells(mv, t)) {
            n.nCells = 0;
            for (int i = 0; i < 15; i++) if (mv[i]) n.cells[n.nCells++] = mv[i];
            n.nTemps = 0;
            for (int i = 0; i < 15 && n.nTemps < 15; i++)
                if (t[i] > -100 && t[i] < 200) n.ctemps[n.nTemps++] = t[i];
        }
        uint32_t cyc, ah;
        if (esm.readStats(cyc, ah)) { n.cycles = cyc > 65535 ? 65535 : (uint16_t)cyc; n.ahTotal = ah; }
        uint16_t lim[15];
        if (esm.readRegisters(0x100A, 15, lim)) {
            n.cvl_raw = lim[0]; n.dcl_raw = lim[1]; n.ccl_raw = lim[3];
            n.dvl_raw = lim[4]; n.ccl_coef = lim[7]; n.dcl_coef = lim[14];
        }
        EsmStatusBits sb;
        if (esm.readStatusBits(sb)) {
            n.sysFault = sb.systemFaults; n.cellFault = sb.cellFaults;
            n.prot = sb.activeProtections; n.warn = sb.activeWarnings; n.special = sb.specialStates;
        }
    }

    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) { p.s = n; xSemaphoreGive(mux); }
}

// Combine the fresh packs into `agg`. Called on the ESM task, under mux.
// Rules: paralleled bus so V averages; current & capacity & limits sum; SOC
// averages (all 100Ah); charge-cutoff on the highest cell, discharge on the
// lowest; temps take the extremes; a fault on any pack raises the bank fault.
static void aggregate() {
    uint32_t now = millis();
    int n = 0;
    float sumV = 0, sumI = 0; uint32_t sumSoc = 0; int minSoh = 100;
    uint16_t bankMaxCell = 0, bankMinCell = 0xFFFF;
    int16_t  bankMaxT = -128, bankMinT = 127;
    uint32_t sumCcl = 0, sumDcl = 0; uint16_t maxCyc = 0;
    uint16_t cvl = 0, dvl = 0;

    for (unsigned i = 0; i < NUM_PACKS; i++) {
        Pack &p = pk[i];
        if (!p.s.valid || now - p.s.ts > STALE_MS) continue;
        n++;
        sumV += p.s.packV; sumI += p.s.current; sumSoc += p.s.soc;
        if (p.s.soh < minSoh) minSoh = p.s.soh;
        bankMaxT = max<int16_t>(bankMaxT, p.s.tmax);
        bankMinT = min<int16_t>(bankMinT, p.s.tmin);
        if (maxCyc < p.s.cycles) maxCyc = p.s.cycles;
        // per-pack cell extremes
        for (uint8_t c = 0; c < p.s.nCells; c++) {
            if (p.s.cells[c] > bankMaxCell) bankMaxCell = p.s.cells[c];
            if (p.s.cells[c] < bankMinCell) bankMinCell = p.s.cells[c];
        }
        // Limits: sum EFFECTIVE per-pack amps (deci-A); the bank coef is set to
        // 100 (no-op) below so buildCmd63 doesn't apply it a second time. A full
        // pack tapers its own CCL, so summing the effective values gives the true
        // bank limit. Fail-safe: a pack whose limit block never read (raw==0)
        // contributes ZERO headroom rather than an invented default — under
        // normal operation every reachable pack is deep-read on the first pass.
        uint32_t cclE = p.s.ccl_raw;
        uint32_t dclE = p.s.dcl_raw;
        // Coefficient (regs 0x1011/0x1018, %) only DERATES. Clamp to 100 so a
        // corrupt/oversized read (e.g. 0xFFFF) can never inflate the limit; a 0
        // coef leaves the raw value unscaled (matches the proven emitter).
        uint16_t cclC = p.s.ccl_coef > 100 ? 100 : p.s.ccl_coef;
        uint16_t dclC = p.s.dcl_coef > 100 ? 100 : p.s.dcl_coef;
        if (cclE && cclC) cclE = cclE * cclC / 100;
        if (dclE && dclC) dclE = dclE * dclC / 100;
        sumCcl += cclE;
        sumDcl += dclE;
        // Voltages are common to the parallel bus, not summed. Charge to the
        // lowest CVL and stop discharge at the highest DVL any pack reports —
        // the most conservative choice when packs disagree slightly.
        if (p.s.cvl_raw && (!cvl || p.s.cvl_raw < cvl)) cvl = p.s.cvl_raw;
        if (p.s.dvl_raw && (!dvl || p.s.dvl_raw > dvl)) dvl = p.s.dvl_raw;
    }
    if (!cvl) cvl = 540;      // 54.0 V fallback if no pack reported a limit
    if (!dvl) dvl = 475;      // 47.5 V

    PackSnapshot a;
    a.valid = (n > 0);
    if (n > 0) {
        a.packV = sumV / n;
        a.current = sumI;
        a.soc = sumSoc / n;
        a.soh = minSoh;
        a.tmin = bankMinT; a.tmax = bankMaxT;
        a.nTemps = 0;                                // emu falls back to tmin/tmax
        a.cycles = maxCyc;
        if (bankMaxCell) { a.cells[0] = bankMaxCell; a.cells[1] = bankMinCell; a.nCells = 2; }
        else a.nCells = 0;
        a.cvl_raw = cvl; a.dvl_raw = dvl;
        a.ccl_raw = sumCcl > 65535 ? 65535 : sumCcl;
        a.dcl_raw = sumDcl > 65535 ? 65535 : sumDcl;
        a.ccl_coef = 100; a.dcl_coef = 100;          // already effective
        a.ts = now;
    }
    agg = a;
}

static void esmTask(void *) {
    for (unsigned i = 0; i < NUM_PACKS; i++) pk[i].addr = PACK_ADDR[i];
    esm.setTarget(PACK_ADDR[0], false);  // start UART1 on the ESM bus
    delay(200);
    uint32_t deepCtr = 0;
    for (;;) {
        for (unsigned i = 0; i < NUM_PACKS; i++) {
            bool deep = (deepCtr++ % (NUM_PACKS * 3)) < NUM_PACKS;   // cells/limits ~1/3 of passes
            pollPack(pk[i], deep);
        }
        if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) { aggregate(); xSemaphoreGive(mux); }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// ---------------- Deye responder (core 1) ----------------
static void deyeTask(void *) {
    PackSnapshot mine;
    for (;;) {
        if (xSemaphoreTake(mux, pdMS_TO_TICKS(5)) == pdTRUE) { mine = agg; xSemaphoreGive(mux); }
        if (mine.valid && millis() - mine.ts > STALE_MS) mine.valid = false;
        deye.poll(mine);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ---------------- WiFi / MQTT ----------------
static WiFiClient   netClient;
static PubSubClient mqtt(netClient);
static bool         mqttWasOk = false;

static void mqttReconnect() {
    if (!cfg.mqttEnable || !cfg.host[0] || !WiFi.isConnected()) return;
    mqtt.setServer(cfg.host, cfg.port);
    mqtt.setBufferSize(1024);            // room for the full per-pack cell/temp arrays
    String id = "esmbank-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    bool ok = cfg.user[0] ? mqtt.connect(id.c_str(), cfg.user, cfg.pass)
                          : mqtt.connect(id.c_str());
    (void)ok;
}

static void mqttPublish() {
    if (!cfg.mqttEnable || !mqtt.connected()) return;
    char topic[80], buf[768];            // per-pack payload w/ full cell+temp arrays + faults

    PackSnapshot a; Pack local[NUM_PACKS];
    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        a = agg;
        for (unsigned i = 0; i < NUM_PACKS; i++) local[i] = pk[i];
        xSemaphoreGive(mux);
    } else return;

    uint32_t now = millis();
    uint32_t ts = epochNow();            // 0 until NTP has synced
    int nfresh = 0;
    for (unsigned i = 0; i < NUM_PACKS; i++)
        if (local[i].s.valid && now - local[i].s.ts < STALE_MS) nfresh++;

    // aggregate
    if (a.valid) {
        snprintf(buf, sizeof(buf),
                 "{\"soc\":%d,\"v\":%.2f,\"i\":%.2f,\"soh\":%d,\"tmin\":%d,\"tmax\":%d,"
                 "\"cycles\":%u,\"ccl\":%.1f,\"dcl\":%.1f,\"packs\":%d,\"ts\":%u}",
                 a.soc, a.packV, a.current, a.soh, a.tmin, a.tmax, a.cycles,
                 a.ccl_raw / 10.0, a.dcl_raw / 10.0, nfresh, ts);
        snprintf(topic, sizeof(topic), "%s/bank", cfg.topic);
        mqtt.publish(topic, buf);
    }
    // per pack
    if (cfg.perPack) {
        for (unsigned i = 0; i < NUM_PACKS; i++) {
            Pack &p = local[i];
            if (!p.s.valid || now - p.s.ts > STALE_MS) continue;
            uint16_t hi = 0, lo = 0xFFFF;
            for (uint8_t c = 0; c < p.s.nCells; c++) { if (p.s.cells[c] > hi) hi = p.s.cells[c]; if (p.s.cells[c] < lo) lo = p.s.cells[c]; }
            int spread = (lo != 0xFFFF) ? (hi - lo) : 0;
            char cellsJson[120]; size_t cn = 0; cellsJson[cn++] = '[';
            for (uint8_t c = 0; c < p.s.nCells && cn < sizeof(cellsJson) - 8; c++)
                cn += snprintf(cellsJson + cn, sizeof(cellsJson) - cn, "%s%u", c ? "," : "", p.s.cells[c]);
            cellsJson[cn++] = ']'; cellsJson[cn] = 0;
            char tempsJson[90]; size_t tn = 0; tempsJson[tn++] = '[';
            for (uint8_t c = 0; c < p.s.nTemps && tn < sizeof(tempsJson) - 8; c++)
                tn += snprintf(tempsJson + tn, sizeof(tempsJson) - tn, "%s%d", c ? "," : "", p.s.ctemps[c]);
            tempsJson[tn++] = ']'; tempsJson[tn] = 0;
            snprintf(buf, sizeof(buf),
                     "{\"addr\":%d,\"sn\":\"%s\",\"soc\":%d,\"v\":%.2f,\"busv\":%.2f,\"i\":%.2f,\"soh\":%d,"
                     "\"tmin\":%d,\"tmax\":%d,\"cellmax\":%d,\"cellmin\":%d,\"spread\":%d,\"cycles\":%u,\"ah\":%u,"
                     "\"ccl\":%.1f,\"dcl\":%.1f,\"cvl\":%.1f,\"dvl\":%.1f,\"cclcoef\":%d,\"dclcoef\":%d,"
                     "\"sysf\":%u,\"cellf\":%u,\"prot\":%u,\"warn\":%u,\"spec\":%u,"
                     "\"ncells\":%d,\"ntemps\":%d,\"cells\":%s,\"temps\":%s,\"ts\":%u}",
                     p.addr, p.serial, p.s.soc, p.s.packV, p.s.busV, p.s.current, p.s.soh,
                     p.s.tmin, p.s.tmax, hi, (lo == 0xFFFF ? 0 : lo), spread, p.s.cycles, p.s.ahTotal,
                     p.s.ccl_raw / 10.0, p.s.dcl_raw / 10.0, p.s.cvl_raw / 10.0, p.s.dvl_raw / 10.0,
                     p.s.ccl_coef, p.s.dcl_coef,
                     p.s.sysFault, p.s.cellFault, p.s.prot, p.s.warn, p.s.special,
                     p.s.nCells, p.s.nTemps, cellsJson, tempsJson, ts);
            snprintf(topic, sizeof(topic), "%s/pack/%d", cfg.topic, p.addr);
            mqtt.publish(topic, buf);
        }
    }
}

// ---------------- Web UI ----------------
static WebServer web(80);
static DNSServer dnsServer;              // captive portal in setup-AP mode
static bool      apMode = false;         // true = hosting the setup AP

// Bring up WiFi: try the configured station, and if that fails (or nothing is
// configured) fall back to hosting the setup AP with a captive portal so the
// user can enter their network from a phone. RS485 translation runs either way.
static void startWifi() {
    const char *ss = cfg.wifiSsid[0] ? cfg.wifiSsid : WIFI_SSID;
    const char *pw = cfg.wifiSsid[0] ? cfg.wifiPass : WIFI_PASS;
    bool haveCreds = ss[0] && strcmp(ss, "YOUR_WIFI_SSID") != 0;
    if (haveCreds) {
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        WiFi.begin(ss, pw);
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print('.'); }
        Serial.println();
        if (WiFi.isConnected()) {
            apMode = false;
            Serial.printf("WiFi '%s' ip=%s\n", ss, WiFi.localIP().toString().c_str());
            return;
        }
        Serial.printf("WiFi '%s' connect failed -> ", ss);
    }
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    dnsServer.start(53, "*", WiFi.softAPIP());
    apMode = true;
    Serial.printf("setup AP '%s' (pass '%s') at http://%s/\n",
                  AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

static String htmlPage() {
    PackSnapshot a; Pack local[NUM_PACKS];
    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        a = agg; for (unsigned i = 0; i < NUM_PACKS; i++) local[i] = pk[i]; xSemaphoreGive(mux);
    }
    String h = F("<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
                 "<style>body{font-family:system-ui;margin:16px;max-width:640px}"
                 "table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:4px 6px;font-size:14px;text-align:right}"
                 "th:first-child,td:first-child{text-align:left}input{padding:4px}fieldset{margin:12px 0}"
                 ".b{font-size:22px;font-weight:bold}</style>"
                 "<h2>ESM bank <span style='font-size:13px;color:#888;font-weight:normal'>fw " FW_VERSION "</span></h2>");
    if (apMode)
        h += F("<p style='background:#fee3e3;padding:8px;border-radius:4px'><b>Setup mode.</b> "
               "Enter your WiFi below and press Save &mdash; the device reboots and joins your network.</p>");
    else
        h += "<p style='color:#0a0;font-size:13px'>WiFi: " + WiFi.SSID() + " &nbsp; " + WiFi.localIP().toString() + "</p>";
    if (a.valid) {
        h += "<p class=b>" + String(a.soc) + "% &nbsp; " + String(a.packV, 1) + " V &nbsp; "
             + String(a.current, 1) + " A</p>";
        h += "<p>SOH " + String(a.soh) + "% &nbsp; " + String(a.tmin) + ".." + String(a.tmax)
             + " C &nbsp; CCL " + String(a.ccl_raw / 10.0, 1) + " A &nbsp; DCL "
             + String(a.dcl_raw / 10.0, 1) + " A</p>";
    } else h += "<p>reading packs...</p>";
    h += F("<table><tr><th>Pack</th><th>Serial</th><th>SOC</th><th>V</th><th>A</th><th>SOH</th><th>Temp</th></tr>");
    uint32_t now = millis();
    for (unsigned i = 0; i < NUM_PACKS; i++) {
        Pack &p = local[i];
        bool ok = p.s.valid && now - p.s.ts < STALE_MS;
        h += "<tr><td>" + String(p.addr) + "</td><td>" + String(p.serial) + "</td>";
        if (ok) h += "<td>" + String(p.s.soc) + "%</td><td>" + String(p.s.packV, 2) + "</td><td>"
                     + String(p.s.current, 2) + "</td><td>" + String(p.s.soh) + "%</td><td>"
                     + String(p.s.tmin) + ".." + String(p.s.tmax) + "</td>";
        else h += "<td colspan=5 style='color:#999'>--</td>";
        h += "</tr>";
    }
    h += F("</table>"
           "<form method=POST action=/save>"
           "<fieldset><legend>WiFi</legend>"
           "SSID <input name=wssid value='%WSSID%'><br>"
           "Password <input name=wpass type=password placeholder='(unchanged)'><br>"
           "<span style='color:#999;font-size:12px'>setting a new network reboots the device to connect</span>"
           "</fieldset>"
           "<fieldset><legend>MQTT</legend>"
           "<label><input type=checkbox name=en %EN%> enable</label><br>"
           "Broker <input name=host value='%HOST%'> : <input name=port size=5 value='%PORT%'><br>"
           "User <input name=user value='%USER%'> Pass <input name=pass type=password value='%PASS%'><br>"
           "Topic <input name=topic value='%TOPIC%'> Interval <input name=intv size=4 value='%INTV%'>s<br>"
           "<label><input type=checkbox name=pp %PP%> publish each pack separately</label>"
           "</fieldset>"
           "<fieldset><legend>Time (NTP)</legend>"
           "Server 1 <input name=ntp1 value='%NTP1%'><br>"
           "Server 2 <input name=ntp2 value='%NTP2%'> <span style='color:#999;font-size:12px'>optional</span><br>"
           "Server 3 <input name=ntp3 value='%NTP3%'> <span style='color:#999;font-size:12px'>optional</span><br>"
           "Timezone <input name=tz list=tzs value='%TZ%' size=28> "
           "<datalist id=tzs>%TZOPTS%</datalist><br>"
           "<span style='color:#999;font-size:12px'>pick a name e.g. Europe/Moscow (autocomplete), or type a raw POSIX string</span><br>"
           "<span style='font-size:12px'>device time now: <b>%NOW%</b></span>"
           "</fieldset><button>Save</button></form>"
           "<fieldset><legend>Firmware</legend>"
           "<form method=POST action=/update enctype='multipart/form-data'>"
           "<input type=file name=fw accept=.bin> <button>Upload &amp; reboot</button></form>"
           "<p style='color:#999;font-size:12px'>OTA also available via espota (port 3232).</p></fieldset>");
    char nowStr[26] = "not synced yet";
    uint32_t ep = epochNow();
    if (ep) { time_t tt = ep; struct tm lt; localtime_r(&tt, &lt); strftime(nowStr, sizeof(nowStr), "%Y-%m-%d %H:%M:%S", &lt); }
    h.replace("%EN%", cfg.mqttEnable ? "checked" : "");
    h.replace("%HOST%", cfg.host); h.replace("%PORT%", String(cfg.port));
    h.replace("%USER%", cfg.user); h.replace("%PASS%", cfg.pass);
    h.replace("%TOPIC%", cfg.topic); h.replace("%INTV%", String(cfg.interval));
    h.replace("%PP%", cfg.perPack ? "checked" : "");
    String tzOpts;
    for (unsigned i = 0; i < TZ_TABLE_N; i++) { tzOpts += "<option value='"; tzOpts += TZ_TABLE[i].name; tzOpts += "'>"; }
    h.replace("%NTP1%", cfg.ntp1); h.replace("%NTP2%", cfg.ntp2); h.replace("%NTP3%", cfg.ntp3);
    h.replace("%TZ%", cfg.tz); h.replace("%NOW%", nowStr); h.replace("%TZOPTS%", tzOpts);
    h.replace("%WSSID%", WiFi.isConnected() ? WiFi.SSID() : String(cfg.wifiSsid));
    return h;
}

static void handleSave() {
    cfg.mqttEnable = web.hasArg("en");
    web.arg("host").toCharArray(cfg.host, sizeof(cfg.host));
    cfg.port = web.arg("port").toInt() ?: 1883;
    web.arg("user").toCharArray(cfg.user, sizeof(cfg.user));
    web.arg("pass").toCharArray(cfg.pass, sizeof(cfg.pass));
    web.arg("topic").toCharArray(cfg.topic, sizeof(cfg.topic));
    cfg.interval = web.arg("intv").toInt() ?: 5;
    cfg.perPack = web.hasArg("pp");
    web.arg("ntp1").toCharArray(cfg.ntp1, sizeof(cfg.ntp1));
    web.arg("ntp2").toCharArray(cfg.ntp2, sizeof(cfg.ntp2));
    web.arg("ntp3").toCharArray(cfg.ntp3, sizeof(cfg.ntp3));
    if (web.arg("tz").length()) web.arg("tz").toCharArray(cfg.tz, sizeof(cfg.tz));
    // WiFi: update SSID if given; update password only if a new one was typed
    // (a blank password field means "keep current"). Reboot to apply a change.
    bool wifiChanged = false;
    if (web.hasArg("wssid") && web.arg("wssid").length()) {
        String ns = web.arg("wssid"), np = web.arg("wpass");
        if (ns != String(cfg.wifiSsid)) wifiChanged = true;
        ns.toCharArray(cfg.wifiSsid, sizeof(cfg.wifiSsid));
        if (np.length()) { np.toCharArray(cfg.wifiPass, sizeof(cfg.wifiPass)); wifiChanged = true; }
    }
    saveCfg();
    applyNtp();                          // re-sync with the new servers/TZ immediately
    if (mqtt.connected()) mqtt.disconnect();
    if (wifiChanged) {
        web.send(200, "text/html",
                 "<meta http-equiv=refresh content='8;url=/'>WiFi saved &mdash; rebooting to connect. "
                 "If it joins your network, reopen this page at its new address.");
        delay(600);
        ESP.restart();
    }
    web.sendHeader("Location", "/"); web.send(303);
}

// Machine-readable diagnostics: aggregate limits (incl. CVL/DVL not shown on
// the HTML page), the Deye poll counters, and each pack's raw limit registers
// so the 0x100A block can be sanity-checked remotely. curl http://<ip>/status
static void handleStatus() {
    PackSnapshot a; Pack local[NUM_PACKS];
    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        a = agg; for (unsigned i = 0; i < NUM_PACKS; i++) local[i] = pk[i]; xSemaphoreGive(mux);
    }
    uint32_t now = millis();
    uint32_t dPoll = deye.lastPollMs ? (now - deye.lastPollMs) : 0xFFFFFFFF;
    uint32_t ep = epochNow();
    char localStr[26] = "";
    if (ep) { time_t tt = ep; struct tm lt; localtime_r(&tt, &lt); strftime(localStr, sizeof(localStr), "%Y-%m-%d %H:%M:%S", &lt); }
    String j = "{\"fw\":\"" FW_VERSION "\",\"up\":" + String(now / 1000);
    j += ",\"net\":{\"ap\":" + String(apMode ? 1 : 0) +
         ",\"ssid\":\"" + (apMode ? String(AP_SSID) : WiFi.SSID()) +
         "\",\"ip\":\"" + (apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) +
         "\",\"rssi\":" + String(apMode ? 0 : WiFi.RSSI()) + "}";
    j += ",\"time\":{\"epoch\":" + String(ep) + ",\"synced\":" + String(ep ? 1 : 0) +
         ",\"local\":\"" + String(localStr) + "\",\"tz\":\"" + String(cfg.tz) +
         "\",\"posix\":\"" + String(tzToPosix(cfg.tz)) + "\",\"ntp\":\"" + String(cfg.ntp1) + "\"}";
    j += ",\"agg\":{\"valid\":" + String(a.valid ? 1 : 0) +
         ",\"soc\":" + String(a.soc) + ",\"v\":" + String(a.packV, 2) +
         ",\"i\":" + String(a.current, 2) + ",\"soh\":" + String(a.soh) +
         ",\"tmin\":" + String(a.tmin) + ",\"tmax\":" + String(a.tmax) +
         ",\"ccl\":" + String(a.ccl_raw / 10.0, 1) + ",\"dcl\":" + String(a.dcl_raw / 10.0, 1) +
         ",\"cvl\":" + String(a.cvl_raw / 10.0, 1) + ",\"dvl\":" + String(a.dvl_raw / 10.0, 1) +
         ",\"age\":" + String(a.valid ? now - a.ts : 0) + "}";
    j += ",\"deye\":{\"answered\":" + String(deye.answered) +
         ",\"pollAge\":" + String((int32_t)(dPoll == 0xFFFFFFFF ? -1 : (int32_t)dPoll)) +
         ",\"cid\":\"" + String(deye.lastCid2) + "\"}";
    j += ",\"packs\":[";
    for (unsigned i = 0; i < NUM_PACKS; i++) {
        Pack &p = local[i];
        bool ok = p.s.valid && now - p.s.ts < STALE_MS;
        uint16_t hi = 0, lo = 0xFFFF;
        for (uint8_t c = 0; c < p.s.nCells; c++) { if (p.s.cells[c] > hi) hi = p.s.cells[c]; if (p.s.cells[c] < lo) lo = p.s.cells[c]; }
        String cells = "["; for (uint8_t c = 0; c < p.s.nCells; c++) { if (c) cells += ","; cells += String(p.s.cells[c]); } cells += "]";
        String temps = "["; for (uint8_t c = 0; c < p.s.nTemps; c++) { if (c) temps += ","; temps += String(p.s.ctemps[c]); } temps += "]";
        if (i) j += ",";
        j += "{\"addr\":" + String(p.addr) + ",\"sn\":\"" + String(p.serial) +
             "\",\"ok\":" + String(ok ? 1 : 0) + ",\"age\":" + String(now - p.s.ts) +
             ",\"soc\":" + String(p.s.soc) + ",\"v\":" + String(p.s.packV, 2) +
             ",\"busv\":" + String(p.s.busV, 2) +
             ",\"i\":" + String(p.s.current, 2) + ",\"soh\":" + String(p.s.soh) +
             ",\"tmin\":" + String(p.s.tmin) + ",\"tmax\":" + String(p.s.tmax) +
             ",\"cellmax\":" + String(hi) + ",\"cellmin\":" + String(lo == 0xFFFF ? 0 : lo) +
             ",\"cyc\":" + String(p.s.cycles) + ",\"ah\":" + String(p.s.ahTotal) +
             ",\"ccl_raw\":" + String(p.s.ccl_raw) + ",\"ccl_coef\":" + String(p.s.ccl_coef) +
             ",\"dcl_raw\":" + String(p.s.dcl_raw) + ",\"dcl_coef\":" + String(p.s.dcl_coef) +
             ",\"cvl_raw\":" + String(p.s.cvl_raw) + ",\"dvl_raw\":" + String(p.s.dvl_raw) +
             ",\"sysf\":" + String(p.s.sysFault) + ",\"cellf\":" + String(p.s.cellFault) +
             ",\"prot\":" + String(p.s.prot) + ",\"warn\":" + String(p.s.warn) + ",\"spec\":" + String(p.s.special) +
             ",\"ncells\":" + String(p.s.nCells) + ",\"ntemps\":" + String(p.s.nTemps) +
             ",\"cells\":" + cells + ",\"temps\":" + temps + "}";
    }
    j += "]}";
    web.send(200, "application/json", j);
}

static void handleUpdateDone() {
    web.sendHeader("Connection", "close");
    web.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK, rebooting");
    delay(500);
    ESP.restart();
}
// The Arduino WebServer runs the WHOLE multipart upload inside one blocking
// handleClient() call. With the two RS485 tasks hammering CPU/heap (esmTask
// reconfigures UART1 every poll; deyeTask preempts at prio 3) a long upload
// starves the loop-task watchdog and resets mid-transfer. So suspend both
// tasks for the duration and feed the WDT on each chunk; resume on abort.
static void handleUpdateUpload() {
    HTTPUpload &up = web.upload();
    if (up.status == UPLOAD_FILE_START) {
        if (esmH)  vTaskSuspend(esmH);
        if (deyeH) vTaskSuspend(deyeH);
        Update.begin(UPDATE_SIZE_UNKNOWN);
    } else if (up.status == UPLOAD_FILE_WRITE) {
        Update.write(up.buf, up.currentSize);
        vTaskDelay(1);                      // yield so IDLE1 runs -> feeds its WDT
    } else if (up.status == UPLOAD_FILE_END) {
        Update.end(true);                   // success -> handleUpdateDone reboots
    } else if (up.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        if (esmH)  vTaskResume(esmH);        // upload failed: keep translating
        if (deyeH) vTaskResume(deyeH);
    }
}

// ---------------- setup / loop ----------------
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.printf("\n== ESM bank translator fw %s ==\nESM UART1 TX=%d RX=%d | Deye UART2 TX=%d RX=%d addr 0x%02X\n",
                  FW_VERSION,
                  ESM_TX_PIN, ESM_RX_PIN, DEYE_TX_PIN, DEYE_RX_PIN, DEYE_ADDR);

#ifdef USE_DISPLAY
    displayInit();
    displayStatus("ESM bank", "starting...");
#endif
    loadCfg();

    Serial2.begin(9600, SERIAL_8N1, DEYE_RX_PIN, DEYE_TX_PIN);

    startWifi();                         // station, or fall back to the setup AP
    if (!apMode) applyNtp();             // SNTP only makes sense once online

    ArduinoOTA.setHostname("esm-bank");
    ArduinoOTA.setPassword(OTA_PASS);
    ArduinoOTA.begin();

    web.on("/", []() { web.send(200, "text/html", htmlPage()); });
    web.on("/save", HTTP_POST, handleSave);
    web.on("/status", handleStatus);
    // captive portal: send any other URL to the config page (setup-AP UX)
    web.onNotFound([]() { web.sendHeader("Location", "/"); web.send(302, "text/plain", ""); });
    web.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
    web.begin();

    mux = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(esmTask,  "esm",  8192, nullptr, 2, &esmH,  0);
    xTaskCreatePinnedToCore(deyeTask, "deye", 4096, nullptr, 3, &deyeH, 1);
}

void loop() {
    static uint32_t lastWifi = 0, lastMqtt = 0, lastPub = 0, lastDisp = 0, lastStat = 0;
    uint32_t now = millis();

    ArduinoOTA.handle();
    web.handleClient();
    if (apMode) dnsServer.processNextRequest();
    if (mqtt.connected()) mqtt.loop();

    // Station keepalive only in station mode; AP-mode retries would drop clients.
    if (!apMode && now - lastWifi > 5000) {
        lastWifi = now;
        if (!WiFi.isConnected()) {
            const char *ss = cfg.wifiSsid[0] ? cfg.wifiSsid : WIFI_SSID;
            const char *pw = cfg.wifiSsid[0] ? cfg.wifiPass : WIFI_PASS;
            WiFi.begin(ss, pw);
        }
    }
    if (!apMode && cfg.mqttEnable && !mqtt.connected() && now - lastMqtt > 5000) { lastMqtt = now; mqttReconnect(); }
    if (!apMode && now - lastPub > cfg.interval * 1000UL) { lastPub = now; mqttPublish(); }

#ifdef USE_DISPLAY
    if (now - lastDisp > 1000) {
        lastDisp = now;
        BankView v{};
        PackSnapshot a; Pack local[NUM_PACKS];
        if (xSemaphoreTake(mux, pdMS_TO_TICKS(30)) == pdTRUE) {
            a = agg; for (unsigned i = 0; i < NUM_PACKS; i++) local[i] = pk[i]; xSemaphoreGive(mux);
        }
        int np = 0;
        for (unsigned i = 0; i < NUM_PACKS && i < 6; i++)
            if (local[i].s.valid && now - local[i].s.ts < STALE_MS) {
                v.packAddr[np] = local[i].addr; v.packSoc[np] = local[i].s.soc; np++;
            }
        v.valid = a.valid; v.nPacks = np;
        v.soc = a.soc; v.soh = a.soh; v.packV = a.packV; v.current = a.current;
        v.tmin = a.tmin; v.tmax = a.tmax;
        v.cclA = a.ccl_raw / 10; v.dclA = a.dcl_raw / 10;
        static String ip; ip = WiFi.isConnected() ? WiFi.localIP().toString() : String("WiFi...");
        v.ip = ip.c_str();
        v.mqttOn = cfg.mqttEnable; v.mqttOk = mqtt.connected();
        v.deyeAgeMs = deye.lastPollMs ? (now - deye.lastPollMs) : UINT32_MAX;
        displayBank(v);
    }
#endif

    if (now - lastStat > 10000) {
        lastStat = now;
        PackSnapshot a;
        if (xSemaphoreTake(mux, pdMS_TO_TICKS(30)) == pdTRUE) { a = agg; xSemaphoreGive(mux); }
        Serial.printf("[bank] %s soc=%d%% %.2fV %+.2fA soh=%d %d..%dC CCL=%.1f DCL=%.1f | deye=%lu ip=%s mqtt=%d\n",
                      a.valid ? "OK" : "--", a.soc, a.packV, a.current, a.soh, a.tmin, a.tmax,
                      a.ccl_raw / 10.0, a.dcl_raw / 10.0, (unsigned long)deye.answered,
                      WiFi.localIP().toString().c_str(), mqtt.connected());
    }
    delay(2);
}
