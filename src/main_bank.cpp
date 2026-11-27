// ESM battery bank to Deye translator. It adds WiFi, WebUI, MQTT, and OTA.
//
// The four Huawei ESM packs self-address to 214..217 on ONE shared RS485 bus.
// This works after you install the Huawei CAN matching resistor on the
// COM_OUT of the last pack. This firmware does these tasks:
//   * It polls each pack: wake and FC41 auth per address, then FC03 reads.
//   * It aggregates the packs into ONE virtual battery.
//   * It sends Pylontech RS485 (Deye "Lithium mode 12") on a second bus.
//   * It serves a web UI to set optional MQTT (aggregate and per-pack).
//   * It supports OTA (ArduinoOTA through espota, AND browser upload), so it
//     does not need a USB cable again.
//
// Hardware (same as the translator):
//   ESM  bus: Serial1  GPIO33 TX -> module TXD, GPIO32 RX <- module RXD
//   Deye bus: Serial2  GPIO26 TX -> module TXD, GPIO25 RX <- module RXD
//   (straight-through auto-direction RS485 modules, VCC 5V, common GND)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <esp_timer.h>
#include <time.h>
#include "huawei_esm.h"
#include "pylontech.h"
#ifdef USE_DISPLAY
#include "display.h"
#endif

// WiFi is OPTIONAL. The RS485 translation to the Deye works without WiFi. WiFi
// only enables the WebUI, OTA, and MQTT. To set your credentials, copy
// src/wifi_config.example.h to src/wifi_config.h (git-ignored) and edit it.
// As an alternative, pass -DWIFI_SSID=... -DWIFI_PASS=... build flags.
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
// Fallback setup access point: if no WiFi is configured or works, the device
// hosts this AP. Join the AP and enter your network on the config page.
#ifndef AP_SSID
#define AP_SSID "ESM-Bank-Setup"
#endif
#ifndef AP_PASS
#define AP_PASS "esmbank-setup"   // The setup AP password. WPA2 needs 8 characters minimum.
#endif
// Increase this on each build. Then you can confirm the live image in the
// WebUI header and the serial banner. This proves that an OTA push worked.
#ifndef FW_VERSION
#define FW_VERSION "bank-2.5"
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
#define STALE_MS  15000       // If a pack stays unreadable this long, drop it from the bank.

HuaweiESM  esm(Serial1, ESM_RX_PIN, ESM_TX_PIN);
PylontechEmu deye(Serial2, DEYE_ADDR);

struct Pack {
    uint8_t  addr;
    bool     authed = false;
    uint8_t  fails = 0;
    uint32_t nextProbeMs = 0; // cooldown before re-probing an offline pack
    char     serial[24] = "";
    PackSnapshot s;           // s.valid = fresh data. s.ts = time of last read.
};

static Pack pk[NUM_PACKS];
static PackSnapshot agg;                 // aggregate given to the Deye emulator
static SemaphoreHandle_t mux;            // guards pk[] and agg
static TaskHandle_t esmH = nullptr, deyeH = nullptr;  // suspended during a web OTA

// ---------------- config (saved) ----------------
enum ChargeMode : uint8_t {
    CHG_MODE_UPS_STOP    = 0,  // EOC Stop: CCL = 0 A when full until Rebulk SOC / timer
    CHG_MODE_SOLAR_FLOAT = 1,  // Follow-Float CVL: step CVL down to Float V, keep gentle CCL open
};

struct Config {
    bool     mqttEnable = false;
    char     host[64] = "";
    uint16_t port = 1883;
    char     user[32] = "";
    char     pass[64] = "";
    char     topic[32] = "esmbank";
    bool     perPack = true;
    uint16_t interval = 5;               // publish period in seconds
    // time: up to 3 NTP servers (ntp1 is enough) and a POSIX TZ string
    char     ntp1[48] = "pool.ntp.org";
    char     ntp2[48] = "";
    char     ntp3[48] = "";
    char     tz[40]   = "Europe/Moscow"; // IANA name or raw POSIX (the offset sign is INVERTED)
    // Charge & BMS reporting policy (persisted in NVS "bank")
    uint8_t  chgMode      = CHG_MODE_UPS_STOP;
    uint8_t  rebulkSoc    = 96;          // % bank SOC threshold to start a new Bulk cycle (70..99)
    uint8_t  rebulkDays   = 14;          // periodic top-balance interval in days (0 = disabled, 1..60)
    uint16_t floatVoltDv  = 501;         // target Float CVL in 0.1 V (501 = 50.1 V = 3.340 V/cell)
    bool     ahCycles     = true;        // true = report Ah-equivalent cycles (ah/cap) to Deye 0x61
    uint32_t lastEocEpoch = 0;           // UTC epoch of last completed full charge
    // WiFi credentials persisted in NVS ("wifi" + "bank" + esp_wifi net80211),
    // surviving OTA updates and normal USB reflashes.
    char     wifiSsid[33] = "";
    char     wifiPass[65] = "";
};
static Config cfg;
static Preferences prefs;
static bool wifiNvsExplicit = false;     // true once "wifi" NVS namespace is initialized
static volatile bool pendingSaveEocTs = false;

struct ChargeCtrlState {
    bool     eocLatched     = false;     // true = EOC reached, waiting for Rebulk (Stop or Float)
    bool     forceBulk      = false;     // manual Force Rebulk requested via WebUI
    uint16_t activeFloatDv  = 501;       // current stepped Follow-Float CVL in 0.1 V
    uint64_t lastEocMonoS   = 0;         // monotonic seconds at last EOC
    uint64_t bulkHoldUntilS = 0;         // min Bulk hold window when manual/timer rebulk fires at 100%
    uint64_t lastFloatStepS = 0;         // monotonic seconds of last Follow-Float 60s decrement
};
static ChargeCtrlState chgSt;

static void initNvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("[nvs] partition full or version mismatch -> reinitializing");
        nvs_flash_erase();
        nvs_flash_init();
    }
}

// Persist WiFi credentials to dedicated NVS namespace "wifi" AND mirror to "bank"
// so credentials stay isolated from routine MQTT/NTP saves and backward-compatible.
static void saveWifiNvs(const char *ssid, const char *pass) {
    strncpy(cfg.wifiSsid, ssid ? ssid : "", sizeof(cfg.wifiSsid) - 1);
    cfg.wifiSsid[sizeof(cfg.wifiSsid) - 1] = 0;
    strncpy(cfg.wifiPass, pass ? pass : "", sizeof(cfg.wifiPass) - 1);
    cfg.wifiPass[sizeof(cfg.wifiPass) - 1] = 0;

    if (prefs.begin("wifi", false)) {
        prefs.putBool("init", true);
        prefs.putString("ssid", cfg.wifiSsid);
        prefs.putString("pass", cfg.wifiPass);
        prefs.end();
    }
    if (prefs.begin("bank", false)) {
        prefs.putString("wssid", cfg.wifiSsid);
        prefs.putString("wpass", cfg.wifiPass);
        prefs.end();
    }
    wifiNvsExplicit = true;
}

static void loadCfg() {
    if (prefs.begin("bank", true)) {
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
        cfg.chgMode      = prefs.getUChar("cmode", CHG_MODE_UPS_STOP);
        cfg.rebulkSoc    = prefs.getUChar("rbsoc", 96);
        cfg.rebulkDays   = prefs.getUChar("rbdays", 14);
        cfg.floatVoltDv  = prefs.getUShort("fltdv", 501);
        cfg.ahCycles     = prefs.getBool("ahcyc", true);
        cfg.lastEocEpoch = prefs.getULong("eocts", 0);
        prefs.getString("wssid", cfg.wifiSsid, sizeof(cfg.wifiSsid));
        prefs.getString("wpass", cfg.wifiPass, sizeof(cfg.wifiPass));
        prefs.end();
    }
    if (cfg.chgMode > CHG_MODE_SOLAR_FLOAT) cfg.chgMode = CHG_MODE_UPS_STOP;
    if (cfg.rebulkSoc < 70)  cfg.rebulkSoc = 70;
    if (cfg.rebulkSoc > 99)  cfg.rebulkSoc = 99;
    if (cfg.rebulkDays > 60) cfg.rebulkDays = 60;
    if (cfg.floatVoltDv < 490) cfg.floatVoltDv = 490;
    if (cfg.floatVoltDv > 535) cfg.floatVoltDv = 535;
    chgSt.activeFloatDv = cfg.floatVoltDv;

    // Dedicated "wifi" NVS namespace takes precedence over legacy "bank" keys.
    char wSsid[33] = "", wPass[65] = "";
    if (prefs.begin("wifi", true)) {
        wifiNvsExplicit = prefs.getBool("init", false);
        prefs.getString("ssid", wSsid, sizeof(wSsid));
        prefs.getString("pass", wPass, sizeof(wPass));
        prefs.end();
    }
    if (wifiNvsExplicit) {
        strncpy(cfg.wifiSsid, wSsid, sizeof(cfg.wifiSsid) - 1);
        strncpy(cfg.wifiPass, wPass, sizeof(cfg.wifiPass) - 1);
    } else if (cfg.wifiSsid[0]) {
        // Migrate legacy "bank" WiFi keys into dedicated "wifi" NVS namespace
        saveWifiNvs(cfg.wifiSsid, cfg.wifiPass);
    }

    if (cfg.interval < 1) cfg.interval = 1;
    if (!cfg.ntp1[0]) strcpy(cfg.ntp1, "pool.ntp.org");
    if (!cfg.tz[0])   strcpy(cfg.tz, "Europe/Moscow");
}

static void saveCfg() {
    if (prefs.begin("bank", false)) {
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
        prefs.putUChar("cmode", cfg.chgMode);
        prefs.putUChar("rbsoc", cfg.rebulkSoc);
        prefs.putUChar("rbdays", cfg.rebulkDays);
        prefs.putUShort("fltdv", cfg.floatVoltDv);
        prefs.putBool("ahcyc", cfg.ahCycles);
        prefs.putULong("eocts", cfg.lastEocEpoch);
        prefs.putString("wssid", cfg.wifiSsid);
        prefs.putString("wpass", cfg.wifiPass);
        prefs.end();
    }
}

// Readable IANA zone names -> POSIX TZ strings. The ESP32 has no tzdata
// database, so the code carries this map. It passes any unlisted name through
// unchanged, so a raw POSIX string like "<+03>-3" still works. DST zones use
// current rules. Fixed-offset zones do not drift. The map favors Europe and
// Russia (Smit's region).
struct TzEntry { const char *name; const char *posix; };
static const TzEntry TZ_TABLE[] = {
    { "UTC",                 "UTC0" },
    // --- Europe / Atlantic (EU DST: last Sun Mar to last Sun Oct, unless noted) ---
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
    // --- Russia (east) / Central Asia (fixed offset, no DST) ---
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
    // --- Oceania (southern hemisphere DST: Oct to Apr) ---
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
    // --- North America (US/CA DST: 2nd Sun Mar to 1st Sun Nov) ---
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

// Convert an IANA name to POSIX. Pass unknown values through (already POSIX).
static const char *tzToPosix(const char *tz) {
    for (unsigned i = 0; i < TZ_TABLE_N; i++)
        if (!strcmp(TZ_TABLE[i].name, tz)) return TZ_TABLE[i].posix;
    return tz;
}

// Apply the NTP and timezone config. You can call this again safely, e.g. on save.
static void applyNtp() {
    const char *s1 = cfg.ntp1[0] ? cfg.ntp1 : "pool.ntp.org";
    const char *s2 = cfg.ntp2[0] ? cfg.ntp2 : nullptr;
    const char *s3 = cfg.ntp3[0] ? cfg.ntp3 : nullptr;
    configTzTime(tzToPosix(cfg.tz[0] ? cfg.tz : "Europe/Moscow"), s1, s2, s3);
}
// UTC epoch seconds. It returns 0 until NTP syncs. This guards the 1970 window.
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

static void pollPack(unsigned idx, bool deep) {
    Pack &p = pk[idx];
    uint32_t now = millis();
    if (!p.authed && p.nextProbeMs && (int32_t)(now - p.nextProbeMs) < 0) {
        return; // offline pack in cooldown: keep polling healthy packs without delay
    }
    esm.setTarget(p.addr, false);        // 8N1
    if (!p.authed) {
        // Fast 400 ms ping instead of 35 s wake() so 1 offline pack never stalls the bank!
        if (!esm.ping(400)) {
            p.fails++;
            p.nextProbeMs = millis() + (p.fails > 2 ? 10000 : 2000);
            return;
        }
        p.authed = esm.authenticate();
        if (!p.authed) {
            p.fails++;
            p.nextProbeMs = millis() + 5000;
            return;
        }
        p.fails = 0;
        p.nextProbeMs = 0;
        deep = true;                     // populate cells/limits/faults immediately on link-up
        static char blob[1024];
        char sn[24] = "";
        if (esm.readArchiveInfo(blob, sizeof(blob))) extractSerial(blob, sn, sizeof(sn));
        if (sn[0] && xSemaphoreTake(mux, pdMS_TO_TICKS(100)) == pdTRUE) {
            strncpy(p.serial, sn, sizeof(p.serial)); p.serial[sizeof(p.serial) - 1] = 0;
            xSemaphoreGive(mux);         // guards the cross-task serial read
        }
    }
    EsmMainData d;
    if (!esm.readMainData(d)) {
        if (++p.fails >= 3) { p.authed = false; p.fails = 0; }  // force a re-auth
        return;
    }
    p.fails = 0;

    PackSnapshot n = p.s;                 // build into a local copy, then publish atomically
    n.packV = d.batteryVoltage * 0.01f;
    n.busV = d.busbarVoltage * 0.01f;
    n.current = d.current * 0.01f;
    n.soc = d.soc > 255 ? 255 : (uint8_t)d.soc;
    n.soh = d.soh > 255 ? 255 : (uint8_t)d.soh;
    n.tmin = d.cellTempMin;
    n.tmax = d.cellTempMax;
    n.valid = true;
    n.ts = millis();

    if (deep) {
        if (!n.capAh) {
            uint16_t cap = 0;
            if (esm.readRegisters(0x0107, 1, &cap, 300) && cap >= 20 && cap <= 500) n.capAh = cap;
            else n.capAh = 100; // ESM-48100B1 nominal capacity (100 Ah)
        }
        uint16_t mv[15]; int16_t t[16];
        if (esm.readCells(mv, t)) {
            n.nCells = 0;
            for (int i = 0; i < 15; i++)
                if (mv[i] >= 500 && mv[i] <= 5000) n.cells[n.nCells++] = mv[i];
            n.nTemps = 0;
            for (int i = 0; i < 16 && n.nTemps < 16; i++)
                if (t[i] >= -50 && t[i] <= 125) n.ctemps[n.nTemps++] = t[i];
        }
        uint32_t cyc, ah;
        if (esm.readStats(cyc, ah)) {
            n.rawCycles = cyc > 65535 ? 65535 : (uint16_t)cyc;
            n.ahTotal   = ah;
            uint32_t cap = n.capAh ? n.capAh : 100;
            uint32_t efc = (ah + cap / 2) / cap;
            n.efcCycles = efc > 65535 ? 65535 : (uint16_t)efc;
            n.cycles    = n.rawCycles;
        }
        uint16_t lim[15];
        if (esm.readRegisters(0x100A, 15, lim)) {
            n.cvl_raw = lim[0]; n.dcl_raw = lim[1]; n.ccl_raw = lim[3];
            n.dvl_raw = lim[4]; n.ccl_coef = lim[7]; n.dcl_coef = lim[14];
            n.haveLimits = true;
        }
        EsmStatusBits sb;
        if (esm.readStatusBits(sb)) {
            n.sysFault = sb.systemFaults; n.cellFault = sb.cellFaults;
            n.prot = sb.activeProtections; n.warn = sb.activeWarnings; n.special = sb.specialStates;
        }
    }

    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) { p.s = n; xSemaphoreGive(mux); }
}

// Combine the fresh packs into `agg`. The ESM task calls this under mux.
// Rules for the parallel bus:
//   * Average terminal/pack voltage; sum parallel currents and charge/discharge limits.
//   * SOH-weighted, properly rounded SOC across active packs; report both avg & min SOH,
//     and both avg & max cycle counts.
//   * Report true global max/min cell voltage across all 60 cells (with exact pack+cell
//     position) for OV/UV protection, while tracking within-pack spread for cell imbalance.
//   * Compute true arithmetic mean across all valid NTC sensors (all 60 thermistors).
static void aggregate() {
    uint32_t now = millis();
    uint64_t monoS = (uint64_t)(esp_timer_get_time() / 1000000ULL);
    int n = 0, nWithLimits = 0;
    bool allFull = true;
    uint8_t minPackSoc = 100;
    float sumV = 0, sumBusV = 0, sumI = 0;
    uint32_t weightedSocNum = 0, weightedSocDen = 0;
    uint32_t sumSoh = 0, sumRawCyc = 0, sumEfcCyc = 0;
    int minSoh = 100;
    uint16_t maxRawCyc = 0, maxEfcCyc = 0;
    uint16_t bankMaxCell = 0, bankMinCell = 0xFFFF, bankMaxSpread = 0;
    uint8_t  maxCellPack = 1, maxCellIdx = 1, minCellPack = 1, minCellIdx = 1;
    int16_t  bankMaxT = -128, bankMinT = 127;
    int16_t  bankMaxSensorT = -128, bankMinSensorT = 127;
    uint8_t  maxTempPack = 1, maxTempIdx = 1, minTempPack = 1, minTempIdx = 1;
    int32_t  sumCellT = 0;
    uint32_t cntCellT = 0;
    uint32_t sumCcl = 0, sumDcl = 0, sumCapAh = 0;
    uint16_t cvl = 0, dvl = 0;
    uint16_t orSysF = 0, orCellF = 0, orProt = 0, orWarn = 0, orSpec = 0;

    for (unsigned i = 0; i < NUM_PACKS; i++) {
        Pack &p = pk[i];
        if (!p.s.valid || now - p.s.ts > STALE_MS) continue;
        n++;
        sumCapAh += (p.s.capAh ? p.s.capAh : 100);
        if (p.s.soc < 100) allFull = false;
        if (p.s.soc < minPackSoc) minPackSoc = p.s.soc;
        sumV += p.s.packV;
        sumBusV += p.s.busV;
        sumI += p.s.current;

        uint32_t w = p.s.soh > 0 ? p.s.soh : 100;
        weightedSocNum += (uint32_t)p.s.soc * w;
        weightedSocDen += w;

        sumSoh += p.s.soh;
        if (p.s.soh < minSoh) minSoh = p.s.soh;

        uint16_t pRaw = p.s.rawCycles ? p.s.rawCycles : p.s.cycles;
        uint32_t pCap = p.s.capAh ? p.s.capAh : 100;
        uint16_t pEfc = p.s.efcCycles ? p.s.efcCycles : (uint16_t)((p.s.ahTotal + pCap / 2) / pCap);
        sumRawCyc += pRaw;
        if (maxRawCyc < pRaw) maxRawCyc = pRaw;
        sumEfcCyc += pEfc;
        if (maxEfcCyc < pEfc) maxEfcCyc = pEfc;

        bankMaxT = max<int16_t>(bankMaxT, p.s.tmax);
        bankMinT = min<int16_t>(bankMinT, p.s.tmin);

        orSysF  |= p.s.sysFault;
        orCellF |= p.s.cellFault;
        orProt  |= p.s.prot;
        orWarn  |= p.s.warn;
        orSpec  |= p.s.special;

        uint16_t pMaxC = 0, pMinC = 0xFFFF;
        for (uint8_t c = 0; c < p.s.nCells; c++) {
            uint16_t mv = p.s.cells[c];
            if (mv > bankMaxCell) { bankMaxCell = mv; maxCellPack = i + 1; maxCellIdx = c + 1; }
            if (mv < bankMinCell) { bankMinCell = mv; minCellPack = i + 1; minCellIdx = c + 1; }
            if (mv > pMaxC) pMaxC = mv;
            if (mv < pMinC) pMinC = mv;
        }
        if (p.s.nCells >= 2 && pMaxC >= pMinC) {
            uint16_t sp = pMaxC - pMinC;
            if (sp > bankMaxSpread) bankMaxSpread = sp;
        }

        for (uint8_t t = 0; t < p.s.nTemps; t++) {
            int16_t tc = p.s.ctemps[t];
            sumCellT += tc;
            cntCellT++;
            if (tc > bankMaxSensorT) { bankMaxSensorT = tc; maxTempPack = i + 1; maxTempIdx = t + 1; }
            if (tc < bankMinSensorT) { bankMinSensorT = tc; minTempPack = i + 1; minTempIdx = t + 1; }
        }

        if (p.s.haveLimits) {
            nWithLimits++;
            uint16_t cclE = 0, dclE = 0;
            PylontechEmu::effectiveLimits(p.s, cclE, dclE);
            sumCcl += cclE;
            sumDcl += dclE;
        }
        if (p.s.cvl_raw && (!cvl || p.s.cvl_raw < cvl)) cvl = p.s.cvl_raw;
        if (p.s.dvl_raw && (!dvl || p.s.dvl_raw > dvl)) dvl = p.s.dvl_raw;
    }
    if (!cvl) cvl = 540;      // fallback 54.0 V (3.60 V/cell = 0x1200)
    if (!dvl) dvl = 460;      // fallback 46.0 V (Huawei manual default LVD)

    PackSnapshot a;
    a.valid = (n > 0);
    if (n > 0) {
        uint8_t  avgSoc    = weightedSocDen ? (uint8_t)((weightedSocNum + weightedSocDen / 2) / weightedSocDen) : 0;
        uint8_t  avgSoh    = (uint8_t)((sumSoh + (uint32_t)n / 2) / (uint32_t)n);
        uint16_t avgRawCyc = (uint16_t)((sumRawCyc + (uint32_t)n / 2) / (uint32_t)n);
        uint16_t avgEfcCyc = (uint16_t)((sumEfcCyc + (uint32_t)n / 2) / (uint32_t)n);
        float    avgPackV  = sumV / n;

        if (cntCellT > 0) {
            bankMaxT = max<int16_t>(bankMaxT, bankMaxSensorT);
            bankMinT = min<int16_t>(bankMinT, bankMinSensorT);
        }

        // Dual-mode post-EOC controller (selectable in WebUI, persisted in NVS):
        //   * Mode 0 (UPS Stop):   CCL = 0 A when full; stays parked across brief brownouts
        //                          until Rebulk SOC / periodic Rebulk Days / low-cell floor.
        //   * Mode 1 (Follow-Float): YamBMS-style Follow-Float CVL step-down (0.1 V / 60s
        //                          behind relaxing pack voltage so Deye never overvoltage-dumps)
        //                          while keeping gentle CCL open until Rebulk triggers.
        uint32_t ep = epochNow();
        if (ep > 0 && cfg.lastEocEpoch == 0 && chgSt.eocLatched) {
            cfg.lastEocEpoch = ep;
            pendingSaveEocTs = true;
        }

        uint8_t packRebulkSoc = (cfg.rebulkSoc > 2) ? (cfg.rebulkSoc - 2) : 0;
        bool cellFloorTrip = (bankMinCell != 0xFFFF) &&
                             ((fabsf(sumI) <= 1.0f && bankMinCell < 3260) || bankMinCell < 3150);
        bool daysDue = false;
        if (cfg.rebulkDays > 0) {
            if (ep > 0 && cfg.lastEocEpoch > 0 && ep > cfg.lastEocEpoch) {
                daysDue = (ep - cfg.lastEocEpoch) >= (uint32_t)cfg.rebulkDays * 86400UL;
            } else if (ep == 0 && chgSt.lastEocMonoS > 0) {
                daysDue = (monoS - chgSt.lastEocMonoS) >= (uint64_t)cfg.rebulkDays * 86400ULL;
            }
        }

        if (chgSt.eocLatched) {
            if (chgSt.forceBulk || avgSoc <= cfg.rebulkSoc || minPackSoc <= packRebulkSoc ||
                cellFloorTrip || daysDue) {
                bool holdAt100 = (chgSt.forceBulk || daysDue) && allFull;
                chgSt.eocLatched     = false;
                chgSt.forceBulk      = false;
                chgSt.bulkHoldUntilS = holdAt100 ? (monoS + 300) : 0; // 5 min Bulk window if already 100%
            }
        } else {
            if (chgSt.forceBulk) {
                chgSt.forceBulk = false;
                if (allFull && chgSt.bulkHoldUntilS < monoS + 300) chgSt.bulkHoldUntilS = monoS + 300;
            }
            if (monoS >= chgSt.bulkHoldUntilS && allFull && fabsf(sumI) <= 0.5f &&
                (bankMinCell == 0xFFFF || bankMinCell >= 3310)) {
                bool hadBulkSession  = (chgSt.bulkHoldUntilS > 0) || (chgSt.lastEocMonoS > 0);
                chgSt.eocLatched     = true;
                chgSt.bulkHoldUntilS = 0;
                chgSt.lastEocMonoS   = monoS;
                if (ep > 0 && (hadBulkSession || cfg.lastEocEpoch == 0)) {
                    cfg.lastEocEpoch = ep;
                    pendingSaveEocTs = true;
                }
                uint16_t liveDv = (uint16_t)ceilf(avgPackV * 10.0f);
                chgSt.activeFloatDv  = max<uint16_t>(cfg.floatVoltDv, min<uint16_t>(cvl, liveDv));
                chgSt.lastFloatStepS = monoS;
            }
        }

        bool useStop  = false;
        bool useFloat = false;
        if (chgSt.eocLatched) {
            uint16_t liveRoundDv = (uint16_t)lroundf(avgPackV * 10.0f);
            if (cfg.chgMode == CHG_MODE_SOLAR_FLOAT) {
                useFloat = true;
                if (chgSt.activeFloatDv < cfg.floatVoltDv) chgSt.activeFloatDv = cfg.floatVoltDv;
                if (chgSt.activeFloatDv > cvl)             chgSt.activeFloatDv = cvl;
                if (monoS - chgSt.lastFloatStepS >= 60) {
                    chgSt.lastFloatStepS = monoS;
                    if (chgSt.activeFloatDv > cfg.floatVoltDv && liveRoundDv <= chgSt.activeFloatDv) {
                        chgSt.activeFloatDv--;
                    }
                }
                cvl = min<uint16_t>(cvl, chgSt.activeFloatDv);
            } else {
                useStop = true;
                sumCcl  = 0;
                // Keep activeFloatDv tracking live voltage so switching to Float is seamless
                uint16_t ceilDv = (uint16_t)ceilf(avgPackV * 10.0f);
                chgSt.activeFloatDv  = max<uint16_t>(cfg.floatVoltDv, min<uint16_t>(cvl, ceilDv));
                chgSt.lastFloatStepS = monoS;
            }
        }

        a.haveLimits = (nWithLimits > 0);
        a.fullStop = useStop;
        a.floatMode = useFloat;
        a.nPacks = (uint8_t)n;
        a.capAh = (uint16_t)((sumCapAh + (uint32_t)n / 2) / (uint32_t)n);
        a.packV = avgPackV;
        a.busV = sumBusV / n;
        a.current = sumI;
        a.soc = avgSoc;
        a.soh = avgSoh;
        a.sohMin = (uint8_t)minSoh;
        a.tmin = bankMinT;
        a.tmax = bankMaxT;
        a.tavg = cntCellT > 0 ? ((float)sumCellT / (float)cntCellT) : ((bankMinT + bankMaxT) * 0.5f);
        a.nTemps = 0;
        a.maxTempPack = maxTempPack; a.maxTempIdx = maxTempIdx;
        a.minTempPack = minTempPack; a.minTempIdx = minTempIdx;
        a.rawCycles = maxRawCyc;
        a.rawCyclesAvg = avgRawCyc;
        a.efcCycles = maxEfcCyc;
        a.efcCyclesAvg = avgEfcCyc;
        a.cycles = cfg.ahCycles ? maxEfcCyc : maxRawCyc;
        a.cyclesAvg = cfg.ahCycles ? avgEfcCyc : avgRawCyc;
        if (bankMaxCell && bankMinCell != 0xFFFF) {
            a.cells[0] = bankMaxCell;
            a.cells[1] = bankMinCell;
            a.nCells = 2;
            a.maxSpread = bankMaxSpread;
            a.maxCellPack = maxCellPack; a.maxCellIdx = maxCellIdx;
            a.minCellPack = minCellPack; a.minCellIdx = minCellIdx;
        } else {
            a.nCells = 0;
            a.maxSpread = 0;
        }
        a.cvl_raw = cvl; a.dvl_raw = dvl;
        a.ccl_raw = sumCcl > 65535 ? 65535 : sumCcl;
        a.dcl_raw = sumDcl > 65535 ? 65535 : sumDcl;
        a.ccl_coef = 0; a.dcl_coef = 0;
        a.sysFault = orSysF; a.cellFault = orCellF;
        a.prot = orProt; a.warn = orWarn; a.special = orSpec;
        // Synchronize aggregate CCL/DCL with the exact Pylontech 0x63 effective limits
        // (note: populate fault/prot/special BEFORE effectiveLimits so bank-level protections apply!)
        PylontechEmu::effectiveLimits(a, a.ccl_raw, a.dcl_raw);
        a.ts = now;
    }
    agg = a;
}

static void esmTask(void *) {
    for (unsigned i = 0; i < NUM_PACKS; i++) pk[i].addr = PACK_ADDR[i];
    esm.setTarget(PACK_ADDR[0], false);  // starts UART1 on the ESM bus
    delay(200);
    uint32_t deepCtr = 0;
    for (;;) {
        for (unsigned i = 0; i < NUM_PACKS; i++) {
            bool deep = !pk[i].s.haveLimits || !pk[i].s.valid || ((deepCtr++ % (NUM_PACKS * 3)) < NUM_PACKS);
            pollPack(i, deep);
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
    mqtt.setBufferSize(1024);            // room for the full per-pack cell and temp arrays
    String id = "esmbank-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    bool ok = cfg.user[0] ? mqtt.connect(id.c_str(), cfg.user, cfg.pass)
                          : mqtt.connect(id.c_str());
    (void)ok;
}

static void packEffectiveDa(const PackSnapshot &ps, const PackSnapshot &bankAgg,
                            uint16_t &cclDa, uint16_t &dclDa,
                            uint16_t &nomCclDa, uint16_t &nomDclDa) {
    uint16_t capDa = (uint16_t)((ps.capAh ? ps.capAh : 100) * 10); // 1.0C in 0.1 A (1000 = 100.0 A)
    nomCclDa = ps.haveLimits ? min<uint16_t>(ps.ccl_raw, capDa) : 0;
    nomDclDa = ps.haveLimits ? min<uint16_t>(ps.dcl_raw, capDa) : 0;
    PackSnapshot tmp = ps;
    tmp.fullStop  = bankAgg.fullStop;
    tmp.floatMode = bankAgg.floatMode;
    PylontechEmu::effectiveLimits(tmp, cclDa, dclDa);
}

static const char *chargeStateName(const PackSnapshot &a) {
    if (a.fullStop)  return "eoc_stop";
    if (a.floatMode) return "float";
    return "bulk";
}

static void mqttPublish() {
    if (!cfg.mqttEnable || !mqtt.connected()) return;
    char topic[80], buf[896];            // per-pack payload with full cell and temp arrays and faults

    PackSnapshot a; Pack local[NUM_PACKS];
    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        a = agg;
        for (unsigned i = 0; i < NUM_PACKS; i++) local[i] = pk[i];
        xSemaphoreGive(mux);
    } else return;

    uint32_t now = millis();
    uint32_t ts = epochNow();            // 0 until NTP syncs
    int nfresh = 0;
    for (unsigned i = 0; i < NUM_PACKS; i++)
        if (local[i].s.valid && now - local[i].s.ts < STALE_MS) nfresh++;

    // aggregate
    if (a.valid) {
        float tavgVal = !isnan(a.tavg) ? a.tavg : ((a.tmin + a.tmax) * 0.5f);
        snprintf(buf, sizeof(buf),
                 "{\"soc\":%d,\"v\":%.2f,\"busv\":%.2f,\"i\":%.2f,\"soh\":%d,\"soh_min\":%d,"
                 "\"tavg\":%.1f,\"tmin\":%d,\"tmax\":%d,\"cycles_avg\":%u,\"cycles\":%u,"
                 "\"efc_avg\":%u,\"efc_max\":%u,\"raw_cycles_avg\":%u,\"raw_cycles\":%u,"
                 "\"spread_max\":%u,\"ccl\":%.1f,\"dcl\":%.1f,\"cvl\":%.1f,\"chg_state\":\"%s\","
                 "\"packs\":%d,\"ts\":%u}",
                 a.soc, a.packV, a.busV, a.current, a.soh, a.sohMin ? a.sohMin : a.soh,
                 tavgVal, a.tmin, a.tmax, a.cyclesAvg, a.cycles,
                 a.efcCyclesAvg, a.efcCycles, a.rawCyclesAvg, a.rawCycles,
                 a.maxSpread == 0xFFFF ? 0 : a.maxSpread,
                 a.ccl_raw / 10.0, a.dcl_raw / 10.0, a.cvl_raw / 10.0, chargeStateName(a),
                 nfresh, ts);
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
            uint16_t pCclDa = 0, pDclDa = 0, pNomC = 0, pNomD = 0;
            packEffectiveDa(p.s, a, pCclDa, pDclDa, pNomC, pNomD);
            uint32_t cap = p.s.capAh ? p.s.capAh : 100;
            uint16_t efc = p.s.efcCycles ? p.s.efcCycles : (uint16_t)((p.s.ahTotal + cap / 2) / cap);
            uint16_t raw = p.s.rawCycles ? p.s.rawCycles : p.s.cycles;
            snprintf(buf, sizeof(buf),
                     "{\"addr\":%d,\"sn\":\"%s\",\"soc\":%d,\"v\":%.2f,\"busv\":%.2f,\"i\":%.2f,\"soh\":%d,"
                     "\"tmin\":%d,\"tmax\":%d,\"cellmax\":%d,\"cellmin\":%d,\"spread\":%d,"
                     "\"cycles\":%u,\"efc\":%u,\"ah\":%u,"
                     "\"ccl\":%.1f,\"dcl\":%.1f,\"cvl\":%.1f,\"dvl\":%.1f,\"cap_ah\":%u,"
                     "\"sysf\":%u,\"cellf\":%u,\"prot\":%u,\"warn\":%u,\"spec\":%u,"
                     "\"ncells\":%d,\"ntemps\":%d,\"cells\":%s,\"temps\":%s,\"ts\":%u}",
                     p.addr, p.serial, p.s.soc, p.s.packV, p.s.busV, p.s.current, p.s.soh,
                     p.s.tmin, p.s.tmax, hi, (lo == 0xFFFF ? 0 : lo), spread,
                     raw, efc, p.s.ahTotal,
                     pCclDa / 10.0, pDclDa / 10.0, a.cvl_raw / 10.0, p.s.dvl_raw / 10.0,
                     (unsigned)cap,
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
static bool      apMode = false;         // true = the device hosts the setup AP

static String htmlEsc(const char *s) {
    String out;
    if (!s) return out;
    for (; *s; s++) {
        switch (*s) {
            case '&':  out += F("&amp;");  break;
            case '<':  out += F("&lt;");   break;
            case '>':  out += F("&gt;");   break;
            case '"':  out += F("&quot;"); break;
            case '\'': out += F("&#39;");  break;
            default:   out += *s;          break;
        }
    }
    return out;
}

static String jsonEsc(const char *s) {
    String out;
    if (!s) return out;
    for (; *s; s++) {
        if (*s == '"' || *s == '\\') { out += '\\'; out += *s; }
        else if (*s == '\n')         { out += "\\n"; }
        else if (*s == '\r')         { /* skip CR */ }
        else if ((uint8_t)*s >= 0x20) out += *s;
    }
    return out;
}

// Start WiFi with multi-layer NVS persistence:
//   1. Dedicated NVS namespace "wifi" (+ mirrored "bank" keys).
//   2. Auto-recovery from ESP-IDF nvs.net80211 on upgrade if "wifi" NVS is new.
//   3. One-time seed from compile-time WIFI_SSID/WIFI_PASS into NVS if set.
//   4. Runtime WiFi storage locked to RAM so fallback AP never overwrites NVS.
//   5. If saved WiFi is temporarily unreachable (e.g. router booting after a
//      power outage), runs AP+STA fallback and auto-rejoins when router comes up.
static void startWifi() {
    WiFi.setHostname("esm-bank");
    WiFi.mode(WIFI_STA);   // initializes esp_wifi with flash NVS open so we can inspect/sync net80211

    wifi_config_t idfCfg{};
    char idfSsid[33] = "", idfPass[65] = "";
    if (esp_wifi_get_config(WIFI_IF_STA, &idfCfg) == ESP_OK) {
        memcpy(idfSsid, idfCfg.sta.ssid, 32);     idfSsid[32] = 0;
        memcpy(idfPass, idfCfg.sta.password, 64); idfPass[64] = 0;
    }

    // First boot on NVS-managed firmware: recover from esp_wifi NVS or compile-time macros
    if (!wifiNvsExplicit && !cfg.wifiSsid[0]) {
        if (idfSsid[0] && strcmp(idfSsid, AP_SSID) != 0 && strcmp(idfSsid, "YOUR_WIFI_SSID") != 0) {
            Serial.printf("[nvs] recovered WiFi '%s' from esp_wifi NVS -> saving to NVS\n", idfSsid);
            saveWifiNvs(idfSsid, idfPass);
        } else if (WIFI_SSID[0] && strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0) {
            Serial.printf("[nvs] seeding compile-time WiFi '%s' into NVS\n", WIFI_SSID);
            saveWifiNvs(WIFI_SSID, WIFI_PASS);
        }
    }

    // Mirror authoritative NVS credentials into esp_wifi flash config once, then lock runtime to RAM
    if (cfg.wifiSsid[0]) {
        if (strcmp(idfSsid, cfg.wifiSsid) != 0 || strcmp(idfPass, cfg.wifiPass) != 0) {
            wifi_config_t newCfg{};
            strncpy((char *)newCfg.sta.ssid, cfg.wifiSsid, sizeof(newCfg.sta.ssid));
            strncpy((char *)newCfg.sta.password, cfg.wifiPass, sizeof(newCfg.sta.password));
            esp_wifi_set_storage(WIFI_STORAGE_FLASH);
            esp_wifi_set_config(WIFI_IF_STA, &newCfg);
        }
    } else if (wifiNvsExplicit && idfSsid[0]) {
        wifi_config_t emptyCfg{};
        esp_wifi_set_storage(WIFI_STORAGE_FLASH);
        esp_wifi_set_config(WIFI_IF_STA, &emptyCfg);
    }
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    WiFi.persistent(false);

    if (cfg.wifiSsid[0]) {
        WiFi.setSleep(false);
        WiFi.setAutoReconnect(true);
        WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
        Serial.printf("[wifi] connecting to NVS SSID '%s'", cfg.wifiSsid);
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
            delay(250);
            Serial.print('.');
        }
        Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
            apMode = false;
            Serial.printf("[wifi] connected '%s' ip=%s (NVS persisted)\n",
                          cfg.wifiSsid, WiFi.localIP().toString().c_str());
            return;
        }
        Serial.printf("[wifi] '%s' unreachable at boot -> AP+STA fallback (auto-retrying every 30s)\n",
                      cfg.wifiSsid);
        WiFi.mode(WIFI_AP_STA);
        WiFi.setSleep(false);
        WiFi.softAP(AP_SSID, AP_PASS);
        dnsServer.start(53, "*", WiFi.softAPIP());
        apMode = true;
        return;
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    dnsServer.start(53, "*", WiFi.softAPIP());
    apMode = true;
    Serial.printf("[wifi] setup AP '%s' (pass '%s') at http://%s/\n",
                  AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

static String htmlPage() {
    PackSnapshot a; Pack local[NUM_PACKS]; ChargeCtrlState cs;
    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        a = agg; cs = chgSt;
        for (unsigned i = 0; i < NUM_PACKS; i++) local[i] = pk[i];
        xSemaphoreGive(mux);
    }
    String h = F("<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
                 "<style>body{font-family:system-ui;margin:16px;max-width:700px}"
                 "table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:4px 6px;font-size:13px;text-align:right}"
                 "th:first-child,td:first-child{text-align:left}input,select{padding:4px 6px;margin:3px 0}fieldset{margin:12px 0}"
                 ".lbl{display:inline-block;min-width:110px}.b{font-size:22px;font-weight:bold;margin:8px 0 4px}"
                 ".pill{display:inline-block;padding:2px 8px;border-radius:10px;font-size:12px;font-weight:600}</style>"
                 "<h2>ESM bank <span style='font-size:13px;color:#888;font-weight:normal'>fw " FW_VERSION "</span></h2>");
    if (apMode) {
        if (cfg.wifiSsid[0])
            h += "<p style='background:#fff4d6;padding:8px;border-radius:4px'><b>Fallback AP mode.</b> "
                 "Saved WiFi <b>" + htmlEsc(cfg.wifiSsid) + "</b> was unreachable at boot (retrying automatically every 30s), "
                 "or enter new WiFi credentials below.</p>";
        else
            h += F("<p style='background:#fee3e3;padding:8px;border-radius:4px'><b>Setup mode.</b> "
                   "Enter your WiFi below and press Save &mdash; credentials are stored in NVS flash and survive reflashing.</p>");
    } else {
        h += "<p style='color:#0a0;font-size:13px'>WiFi: " + htmlEsc(WiFi.SSID().c_str()) + " &nbsp; "
             + WiFi.localIP().toString() + " &nbsp; <span style='color:#666'>(saved in NVS)</span></p>";
    }
    uint32_t now = millis();
    uint32_t nomCcl = 0, nomDcl = 0;
    for (unsigned i = 0; i < NUM_PACKS; i++) {
        Pack &p = local[i];
        if (p.s.valid && now - p.s.ts < STALE_MS && p.s.haveLimits) {
            uint16_t pc, pd, nc, nd;
            packEffectiveDa(p.s, a, pc, pd, nc, nd);
            nomCcl += nc;
            nomDcl += nd;
        }
    }
    h += F("<div id=live>");
    if (a.valid) {
        float tavgVal = !isnan(a.tavg) ? a.tavg : ((a.tmin + a.tmax) * 0.5f);
        String badge;
        if (a.fullStop) {
            badge = "<span class=pill style='background:#dff6e4;color:#0f6a28'>UPS EOC Stop &bull; Rebulk &le; "
                    + String(cfg.rebulkSoc) + "%</span>";
        } else if (a.floatMode) {
            badge = "<span class=pill style='background:#e0f0ff;color:#0b5394'>Follow-Float &bull; CVL "
                    + String(a.cvl_raw / 10.0, 1) + " V (target " + String(cfg.floatVoltDv / 10.0, 1) + " V)</span>";
        } else {
            badge = "<span class=pill style='background:#fff0d6;color:#9c5700'>Bulk Charge &bull; CVL "
                    + String(a.cvl_raw / 10.0, 1) + " V</span>";
        }
        h += "<p class=b id=lv_main>" + String(a.soc) + "% &nbsp; " + String(a.packV, 2) + " V &nbsp; "
             + String(a.current, 1) + " A &nbsp; " + badge + "</p>";
        String cclTxt = String(a.ccl_raw / 10.0, 1) + " A";
        if (a.fullStop && nomCcl > 0) {
            cclTxt += " <span style='color:#666;font-size:12px'>(parked &bull; max " + String(nomCcl / 10.0, 1) + " A)</span>";
        } else if (a.floatMode && nomCcl > a.ccl_raw) {
            cclTxt += " <span style='color:#666;font-size:12px'>(float limit &bull; max " + String(nomCcl / 10.0, 1) + " A)</span>";
        } else if (nomCcl > a.ccl_raw) {
            cclTxt += " <span style='color:#666;font-size:12px'>(tapered &bull; max " + String(nomCcl / 10.0, 1) + " A)</span>";
        }
        String dclTxt = String(a.dcl_raw / 10.0, 1) + " A";
        if (nomDcl > a.dcl_raw) {
            dclTxt += " <span style='color:#666;font-size:12px'>(tapered &bull; max " + String(nomDcl / 10.0, 1) + " A)</span>";
        }
        h += "<p id=lv_sub style='margin:4px 0 10px;line-height:1.5'>SOH " + String(a.soh) + "% (min " + String(a.sohMin ? a.sohMin : a.soh) + "%) &nbsp; "
             + String(tavgVal, 1) + " &deg;C (" + String(a.tmin) + ".." + String(a.tmax) + " &deg;C) &nbsp; "
             + "Cells " + String(a.nCells >= 2 ? a.cells[1] : 0) + ".." + String(a.nCells ? a.cells[0] : 0) + " mV (&Delta;"
             + String(a.maxSpread == 0xFFFF ? 0 : a.maxSpread) + " mV)<br>"
             + "Cycles: <b>" + String(a.efcCyclesAvg) + " avg / " + String(a.efcCycles) + " max</b> (Ah eq.) &bull; "
             + "<span style='color:#666'>" + String(a.rawCyclesAvg) + " avg / " + String(a.rawCycles) + " max (BMS events)</span><br>"
             + "CVL " + String(a.cvl_raw / 10.0, 1) + " V &nbsp; CCL " + cclTxt + " &nbsp; DCL " + dclTxt + "</p>";
    } else h += "<p class=b id=lv_main>--</p><p id=lv_sub>reading packs...</p>";
    h += F("<table><thead><tr><th>Pack</th><th>Serial</th><th>SOC</th><th>V</th><th>A</th><th>SOH</th><th>Temp</th><th>Cycles (Ah / Evt)</th><th>CCL / DCL</th></tr></thead><tbody id=lv_tb>");
    for (unsigned i = 0; i < NUM_PACKS; i++) {
        Pack &p = local[i];
        bool ok = p.s.valid && now - p.s.ts < STALE_MS;
        h += "<tr><td>" + String(p.addr) + "</td><td>" + String(p.serial) + "</td>";
        if (ok) {
            uint16_t pc = 0, pd = 0, nc = 0, nd = 0;
            packEffectiveDa(p.s, a, pc, pd, nc, nd);
            uint32_t cap = p.s.capAh ? p.s.capAh : 100;
            uint16_t efc = p.s.efcCycles ? p.s.efcCycles : (uint16_t)((p.s.ahTotal + cap / 2) / cap);
            uint16_t raw = p.s.rawCycles ? p.s.rawCycles : p.s.cycles;
            h += "<td>" + String(p.s.soc) + "%</td><td>" + String(p.s.packV, 2) + "</td><td>"
                 + String(p.s.current, 2) + "</td><td>" + String(p.s.soh) + "%</td><td>"
                 + String(p.s.tmin) + ".." + String(p.s.tmax) + "</td><td>"
                 + String(efc) + " <span style='color:#888'>/ " + String(raw) + "</span></td><td>"
                 + String(pc / 10.0f, 0) + " / " + String(pd / 10.0f, 0) + " A</td>";
        } else h += "<td colspan=7 style='color:#999'>--</td>";
        h += "</tr>";
    }
    h += F("</tbody></table></div>"
           "<div style='margin:8px 0;display:flex;gap:8px;align-items:center;flex-wrap:wrap'>"
           "<form method=POST action=/rebulk style='margin:0'><button name=act value=bulk>Force Rebulk (Top-Balance Now)</button></form>"
           "<form method=POST action=/rebulk style='margin:0'><button name=act value=park>Park / EOC Now</button></form>"
           "<span style='color:#777;font-size:12px' id=lv_eoc>%EOCINFO%</span>"
           "</div>"
           "<script>"
           "setInterval(async()=>{"
           "try{const r=await fetch('/status',{cache:'no-store'});const j=await r.json();const a=j.agg;const c=j.chg;"
           "if(!a||!a.valid)return;"
           "let nomC=0,nomD=0,tb='';"
           "for(const p of j.packs){"
           "tb+='<tr><td>'+p.addr+'</td><td>'+p.sn+'</td>';"
           "if(p.ok){nomC+=p.nom_ccl;nomD+=p.nom_dcl;"
           "tb+='<td>'+p.soc+'%</td><td>'+p.v.toFixed(2)+'</td><td>'+p.i.toFixed(2)+'</td><td>'+p.soh+'%</td><td>'+p.tmin+'..'+p.tmax+'</td><td>'+p.efc+' <span style=\"color:#888\">/ '+p.cyc+'</span></td><td>'+p.ccl.toFixed(0)+' / '+p.dcl.toFixed(0)+' A</td>';}"
           "else tb+=\"<td colspan=7 style='color:#999'>--</td>\";"
           "tb+='</tr>';}"
           "document.getElementById('lv_tb').innerHTML=tb;"
           "let bg='';"
           "if(j.deye&&j.deye.fullStop)bg=\"<span class=pill style='background:#dff6e4;color:#0f6a28'>UPS EOC Stop &bull; Rebulk &le; \"+(c?c.rebulk_soc:96)+\"%</span>\";"
           "else if(a.float_mode)bg=\"<span class=pill style='background:#e0f0ff;color:#0b5394'>Follow-Float &bull; CVL \"+a.cvl.toFixed(1)+\" V (target \"+(c?c.float_target_v.toFixed(1):'50.1')+\" V)</span>\";"
           "else bg=\"<span class=pill style='background:#fff0d6;color:#9c5700'>Bulk Charge &bull; CVL \"+a.cvl.toFixed(1)+\" V</span>\";"
           "const m=document.getElementById('lv_main');if(m)m.innerHTML=a.soc+'% &nbsp; '+a.v.toFixed(2)+' V &nbsp; '+a.i.toFixed(1)+' A &nbsp; '+bg;"
           "let ct=a.ccl.toFixed(1)+' A';"
           "if(j.deye&&j.deye.fullStop&&nomC>0)ct+=\" <span style='color:#666;font-size:12px'>(parked &bull; max \"+nomC.toFixed(1)+\" A)</span>\";"
           "else if(a.float_mode&&nomC>a.ccl+0.2)ct+=\" <span style='color:#666;font-size:12px'>(float limit &bull; max \"+nomC.toFixed(1)+\" A)</span>\";"
           "else if(nomC>a.ccl+0.2)ct+=\" <span style='color:#666;font-size:12px'>(tapered &bull; max \"+nomC.toFixed(1)+\" A)</span>\";"
           "let dt=a.dcl.toFixed(1)+' A';"
           "if(nomD>a.dcl+0.2)dt+=\" <span style='color:#666;font-size:12px'>(tapered &bull; max \"+nomD.toFixed(1)+\" A)</span>\";"
           "const s=document.getElementById('lv_sub');if(s)s.innerHTML='SOH '+a.soh+'% (min '+a.soh_min+'%) &nbsp; '+a.tavg.toFixed(1)+' &deg;C ('+a.tmin+'..'+a.tmax+' &deg;C) &nbsp; Cells '+a.cellmin+'..'+a.cellmax+' mV (&Delta;'+a.spread_max+' mV)<br>'"
           "+'Cycles: <b>'+a.efc_avg+' avg / '+a.efc_max+' max</b> (Ah eq.) &bull; <span style=\"color:#666\">'+a.raw_cyc_avg+' avg / '+a.raw_cyc_max+' max (BMS events)</span><br>'"
           "+'CVL '+a.cvl.toFixed(1)+' V &nbsp; CCL '+ct+' &nbsp; DCL '+dt;"
           "if(c&&c.last_eoc_str){const el=document.getElementById('lv_eoc');if(el)el.textContent='Last full top-balance: '+c.last_eoc_str;}"
           "}catch(e){}},3000);"
           "</script>"
           "<form method=POST action=/save>"
           "<fieldset><legend>Charge &amp; BMS Control (NVS Persistent)</legend>"
           "<span class=lbl>Charge Mode</span><select name=cmode>"
           "<option value=0 %CM0%>UPS Stop Mode (CCL = 0 A when full until Rebulk SOC &mdash; Grid/UPS)</option>"
           "<option value=1 %CM1%>Solar Follow-Float CVL (Step CVL to Float V, keep CCL open &mdash; Solar PV)</option>"
           "</select><br>"
           "<span class=lbl>Rebulk SOC</span><input name=rbsoc type=number min=70 max=99 value='%RBSOC%' style='width:58px'> % "
           "<span style='color:#777;font-size:12px'>(start new Bulk cycle at &le; this SOC, or single pack &le; SOC&minus;2%)</span><br>"
           "<span class=lbl>Float CVL</span><input name=fltv type=number step=0.1 min=49.0 max=53.5 value='%FLTV%' style='width:68px'> V "
           "<span style='color:#777;font-size:12px'>(Solar Follow-Float target, e.g. 50.1 V = 3.34 V/cell for 15S)</span><br>"
           "<span class=lbl>Periodic Rebulk</span><input name=rbdays type=number min=0 max=60 value='%RBDAYS%' style='width:58px'> days "
           "<span style='color:#777;font-size:12px'>(top-balance interval if parked continuously; 0 = disabled)</span><br>"
           "<span class=lbl>Deye Cycles</span><select name=ahcyc>"
           "<option value=1 %AH1%>True Ah-Equivalent Cycles (ahTotal / 100 Ah &mdash; Recommended)</option>"
           "<option value=0 %AH0%>Huawei Raw Event Counter (0x0042)</option>"
           "</select>"
           "</fieldset>"
           "<fieldset><legend>WiFi (NVS Persistent)</legend>"
           "<span class=lbl>SSID</span><input name=wssid value='%WSSID%' size=28 maxlength=32><br>"
           "<span class=lbl>Password</span><input name=wpass type=password placeholder='%WPPH%' size=28 maxlength=64 autocomplete=new-password><br>"
           "<span style='color:#666;font-size:12px'>%WNVS%</span><br>"
           "<label style='color:#999;font-size:12px'><input type=checkbox name=wclr> forget saved WiFi (boot to setup AP)</label>"
           "</fieldset>"
           "<fieldset><legend>MQTT</legend>"
           "<label><input type=checkbox name=en %EN%> enable</label><br>"
           "Broker <input name=host value='%HOST%'> : <input name=port size=5 value='%PORT%'><br>"
           "User <input name=user value='%USER%'> Pass <input name=pass type=password placeholder='%MPPH%' autocomplete=new-password><br>"
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
           "<p style='color:#999;font-size:12px'>OTA &amp; USB firmware uploads keep NVS WiFi/MQTT/Charge settings intact. Also available via espota (port 3232).</p></fieldset>");
    char nowStr[26] = "not synced yet";
    uint32_t ep = epochNow();
    if (ep) { time_t tt = ep; struct tm lt; localtime_r(&tt, &lt); strftime(nowStr, sizeof(nowStr), "%Y-%m-%d %H:%M:%S", &lt); }
    String eocInfo;
    if (cfg.lastEocEpoch > 0) {
        char eb[24]; time_t et = cfg.lastEocEpoch; struct tm elt; localtime_r(&et, &elt);
        strftime(eb, sizeof(eb), "%Y-%m-%d %H:%M", &elt);
        eocInfo = "Last full top-balance: " + String(eb);
        if (ep > cfg.lastEocEpoch) eocInfo += " (" + String((ep - cfg.lastEocEpoch) / 86400.0f, 1) + " d ago)";
    } else {
        eocInfo = "Last full top-balance: waiting for NTP / EOC";
    }
    h.replace("%EOCINFO%", eocInfo);
    h.replace("%CM0%", cfg.chgMode == CHG_MODE_UPS_STOP ? "selected" : "");
    h.replace("%CM1%", cfg.chgMode == CHG_MODE_SOLAR_FLOAT ? "selected" : "");
    h.replace("%RBSOC%", String(cfg.rebulkSoc));
    h.replace("%FLTV%", String(cfg.floatVoltDv / 10.0f, 1));
    h.replace("%RBDAYS%", String(cfg.rebulkDays));
    h.replace("%AH1%", cfg.ahCycles ? "selected" : "");
    h.replace("%AH0%", !cfg.ahCycles ? "selected" : "");
    h.replace("%EN%", cfg.mqttEnable ? "checked" : "");
    h.replace("%HOST%", htmlEsc(cfg.host)); h.replace("%PORT%", String(cfg.port));
    h.replace("%USER%", htmlEsc(cfg.user));
    h.replace("%MPPH%", cfg.pass[0] ? "(saved, blank = keep)" : "(none)");
    h.replace("%TOPIC%", htmlEsc(cfg.topic)); h.replace("%INTV%", String(cfg.interval));
    h.replace("%PP%", cfg.perPack ? "checked" : "");
    String tzOpts;
    for (unsigned i = 0; i < TZ_TABLE_N; i++) { tzOpts += "<option value='"; tzOpts += TZ_TABLE[i].name; tzOpts += "'>"; }
    h.replace("%NTP1%", htmlEsc(cfg.ntp1)); h.replace("%NTP2%", htmlEsc(cfg.ntp2)); h.replace("%NTP3%", htmlEsc(cfg.ntp3));
    h.replace("%TZ%", htmlEsc(cfg.tz)); h.replace("%NOW%", nowStr); h.replace("%TZOPTS%", tzOpts);
    h.replace("%WSSID%", htmlEsc(cfg.wifiSsid));
    h.replace("%WPPH%", cfg.wifiPass[0] ? "(saved in NVS, blank = keep)" : "(password)");
    String wnvs = cfg.wifiSsid[0]
        ? ("Saved in NVS: <b>" + htmlEsc(cfg.wifiSsid) + "</b> (" + (cfg.wifiPass[0] ? "password set" : "open network") + " &bull; survives OTA &amp; USB reflash)")
        : String("No WiFi saved in NVS yet");
    h.replace("%WNVS%", wnvs);
    return h;
}

static void handleRebulk() {
    String act = web.arg("act");
    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (act == "park") {
            if (agg.valid && agg.soc >= 90) {
                chgSt.eocLatched     = true;
                chgSt.forceBulk      = false;
                chgSt.bulkHoldUntilS = 0;
                uint16_t ceilDv      = (uint16_t)ceilf(agg.packV * 10.0f);
                chgSt.activeFloatDv  = max<uint16_t>(cfg.floatVoltDv, ceilDv);
            }
        } else {
            chgSt.forceBulk = true;
        }
        aggregate();
        xSemaphoreGive(mux);
    }
    web.sendHeader("Location", "/");
    web.send(303);
}

static void handleSave() {
    if (web.hasArg("cmode")) {
        int cm = web.arg("cmode").toInt();
        cfg.chgMode = (cm == 1) ? CHG_MODE_SOLAR_FLOAT : CHG_MODE_UPS_STOP;
    }
    if (web.hasArg("rbsoc")) {
        int rb = web.arg("rbsoc").toInt();
        if (rb < 70) rb = 70;
        if (rb > 99) rb = 99;
        cfg.rebulkSoc = (uint8_t)rb;
    }
    if (web.hasArg("fltv")) {
        float fv = web.arg("fltv").toFloat();
        int fdv = (int)lroundf(fv * 10.0f);
        if (fdv < 490) fdv = 490;
        if (fdv > 535) fdv = 535;
        cfg.floatVoltDv = (uint16_t)fdv;
    }
    if (web.hasArg("rbdays")) {
        int rd = web.arg("rbdays").toInt();
        if (rd < 0)  rd = 0;
        if (rd > 60) rd = 60;
        cfg.rebulkDays = (uint8_t)rd;
    }
    if (web.hasArg("ahcyc")) {
        cfg.ahCycles = (web.arg("ahcyc").toInt() != 0);
    }

    cfg.mqttEnable = web.hasArg("en");
    web.arg("host").toCharArray(cfg.host, sizeof(cfg.host));
    cfg.port = web.arg("port").toInt() ?: 1883;
    web.arg("user").toCharArray(cfg.user, sizeof(cfg.user));
    if (web.arg("pass").length()) {
        web.arg("pass").toCharArray(cfg.pass, sizeof(cfg.pass));
    } else if (!cfg.user[0]) {
        cfg.pass[0] = 0;
    }
    if (web.arg("topic").length()) web.arg("topic").toCharArray(cfg.topic, sizeof(cfg.topic));
    cfg.interval = web.arg("intv").toInt() ?: 5;
    cfg.perPack = web.hasArg("pp");
    web.arg("ntp1").toCharArray(cfg.ntp1, sizeof(cfg.ntp1));
    web.arg("ntp2").toCharArray(cfg.ntp2, sizeof(cfg.ntp2));
    web.arg("ntp3").toCharArray(cfg.ntp3, sizeof(cfg.ntp3));
    if (web.arg("tz").length()) web.arg("tz").toCharArray(cfg.tz, sizeof(cfg.tz));

    // WiFi NVS update: never wipe the saved password when the password field is left blank.
    bool wifiChanged = false;
    if (web.hasArg("wclr")) {
        saveWifiNvs("", "");
        wifiChanged = true;
    } else if (web.hasArg("wssid")) {
        String ns = web.arg("wssid"), np = web.arg("wpass");
        if (ns.length() > 0) {
            bool ssidDiff = (ns != String(cfg.wifiSsid));
            if (ssidDiff || np.length() > 0) {
                char newSsid[33] = "", newPass[65] = "";
                ns.toCharArray(newSsid, sizeof(newSsid));
                if (np.length() > 0) {
                    np.toCharArray(newPass, sizeof(newPass));
                } else if (!ssidDiff) {
                    strncpy(newPass, cfg.wifiPass, sizeof(newPass) - 1);
                }
                // Note: if ssidDiff && np is empty, newPass stays "" (open WiFi network).
                saveWifiNvs(newSsid, newPass);
                wifiChanged = true;
            } else if (apMode && cfg.wifiSsid[0]) {
                // Pressing Save in fallback AP mode with existing NVS creds triggers a fresh connect attempt
                wifiChanged = true;
            }
        }
    }

    saveCfg();
    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        aggregate();
        xSemaphoreGive(mux);
    }
    if (!apMode) applyNtp();             // re-sync with the new servers and TZ immediately
    if (mqtt.connected()) mqtt.disconnect();
    if (wifiChanged) {
        web.send(200, "text/html; charset=utf-8",
                 "<meta charset=utf-8><meta http-equiv=refresh content='8;url=/'>WiFi saved to NVS &mdash; rebooting to connect. "
                 "If it joins your network, reopen this page at its LAN address.");
        delay(600);
        ESP.restart();
    }
    web.sendHeader("Location", "/"); web.send(303);
}

// Machine-readable diagnostics: the aggregate limits (with CVL/DVL, which the
// HTML page does not show), the Deye poll counters, and the raw limit registers
// of each pack. This lets you check the 0x100A block remotely. curl http://<ip>/status
static void handleStatus() {
    PackSnapshot a; Pack local[NUM_PACKS]; ChargeCtrlState cs;
    if (xSemaphoreTake(mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        a = agg; cs = chgSt;
        for (unsigned i = 0; i < NUM_PACKS; i++) local[i] = pk[i];
        xSemaphoreGive(mux);
    }
    uint32_t now = millis();
    uint32_t dPoll = deye.lastPollMs ? (now - deye.lastPollMs) : 0xFFFFFFFF;
    uint32_t ep = epochNow();
    char localStr[26] = "";
    if (ep) { time_t tt = ep; struct tm lt; localtime_r(&tt, &lt); strftime(localStr, sizeof(localStr), "%Y-%m-%d %H:%M:%S", &lt); }
    String eocStr;
    float daysSinceEoc = -1.0f;
    if (cfg.lastEocEpoch > 0) {
        char eb[24]; time_t et = cfg.lastEocEpoch; struct tm elt; localtime_r(&et, &elt);
        strftime(eb, sizeof(eb), "%Y-%m-%d %H:%M", &elt);
        eocStr = String(eb);
        if (ep > cfg.lastEocEpoch) {
            daysSinceEoc = (ep - cfg.lastEocEpoch) / 86400.0f;
            eocStr += " (" + String(daysSinceEoc, 1) + " d ago)";
        }
    }
    String j = "{\"fw\":\"" FW_VERSION "\",\"up\":" + String(now / 1000);
    j += ",\"net\":{\"ap\":" + String(apMode ? 1 : 0) +
         ",\"ssid\":\"" + jsonEsc(apMode ? AP_SSID : WiFi.SSID().c_str()) +
         "\",\"ip\":\"" + (apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) +
         "\",\"rssi\":" + String(apMode ? 0 : WiFi.RSSI()) +
         ",\"nvs_saved\":" + String(cfg.wifiSsid[0] ? 1 : 0) +
         ",\"nvs_ssid\":\"" + jsonEsc(cfg.wifiSsid) + "\"}";
    j += ",\"time\":{\"epoch\":" + String(ep) + ",\"synced\":" + String(ep ? 1 : 0) +
         ",\"local\":\"" + String(localStr) + "\",\"tz\":\"" + String(cfg.tz) +
         "\",\"posix\":\"" + String(tzToPosix(cfg.tz)) + "\",\"ntp\":\"" + String(cfg.ntp1) + "\"}";
    j += ",\"chg\":{\"mode\":" + String(cfg.chgMode) +
         ",\"mode_name\":\"" + String(cfg.chgMode == CHG_MODE_SOLAR_FLOAT ? "solar_float" : "ups_stop") +
         "\",\"state\":\"" + String(chargeStateName(a)) +
         "\",\"eoc_latched\":" + String(cs.eocLatched ? 1 : 0) +
         ",\"rebulk_soc\":" + String(cfg.rebulkSoc) +
         ",\"rebulk_days\":" + String(cfg.rebulkDays) +
         ",\"float_target_v\":" + String(cfg.floatVoltDv / 10.0f, 1) +
         ",\"float_active_v\":" + String(cs.activeFloatDv / 10.0f, 1) +
         ",\"ah_cycles\":" + String(cfg.ahCycles ? 1 : 0) +
         ",\"last_eoc_epoch\":" + String(cfg.lastEocEpoch) +
         ",\"days_since_eoc\":" + String(daysSinceEoc, 2) +
         ",\"last_eoc_str\":\"" + jsonEsc(eocStr.c_str()) + "\"}";
    float tavgVal = !isnan(a.tavg) ? a.tavg : ((a.tmin + a.tmax) * 0.5f);
    j += ",\"agg\":{\"valid\":" + String(a.valid ? 1 : 0) +
         ",\"soc\":" + String(a.soc) + ",\"v\":" + String(a.packV, 2) +
         ",\"busv\":" + String(a.busV, 2) +
         ",\"i\":" + String(a.current, 2) +
         ",\"soh\":" + String(a.soh) + ",\"soh_min\":" + String(a.sohMin ? a.sohMin : a.soh) +
         ",\"tavg\":" + String(tavgVal, 2) +
         ",\"tmin\":" + String(a.tmin) + ",\"tmax\":" + String(a.tmax) +
         ",\"cyc_avg\":" + String(a.cyclesAvg) + ",\"cyc_max\":" + String(a.cycles) +
         ",\"efc_avg\":" + String(a.efcCyclesAvg) + ",\"efc_max\":" + String(a.efcCycles) +
         ",\"raw_cyc_avg\":" + String(a.rawCyclesAvg) + ",\"raw_cyc_max\":" + String(a.rawCycles) +
         ",\"float_mode\":" + String(a.floatMode ? 1 : 0) +
         ",\"cellmax\":" + String(a.nCells ? a.cells[0] : 0) +
         ",\"cellmin\":" + String(a.nCells >= 2 ? a.cells[1] : 0) +
         ",\"spread_max\":" + String(a.maxSpread == 0xFFFF ? 0 : a.maxSpread) +
         ",\"ccl\":" + String(a.ccl_raw / 10.0, 1) + ",\"dcl\":" + String(a.dcl_raw / 10.0, 1) +
         ",\"cvl\":" + String(a.cvl_raw / 10.0, 1) + ",\"dvl\":" + String(a.dvl_raw / 10.0, 1) +
         ",\"age\":" + String(a.valid ? now - a.ts : 0) + "}";
    j += ",\"deye\":{\"answered\":" + String(deye.answered) +
         ",\"badFrames\":" + String(deye.badFrames) +
         ",\"fullStop\":" + String(a.fullStop ? 1 : 0) +
         ",\"floatMode\":" + String(a.floatMode ? 1 : 0) +
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
        uint16_t pCclDa = 0, pDclDa = 0, pNomC = 0, pNomD = 0;
        packEffectiveDa(p.s, a, pCclDa, pDclDa, pNomC, pNomD);
        uint32_t cap = p.s.capAh ? p.s.capAh : 100;
        uint16_t efc = p.s.efcCycles ? p.s.efcCycles : (uint16_t)((p.s.ahTotal + cap / 2) / cap);
        uint16_t raw = p.s.rawCycles ? p.s.rawCycles : p.s.cycles;
        if (i) j += ",";
        j += "{\"addr\":" + String(p.addr) + ",\"sn\":\"" + String(p.serial) +
             "\",\"ok\":" + String(ok ? 1 : 0) + ",\"age\":" + String(p.s.valid ? (int32_t)(now - p.s.ts) : -1) +
             ",\"cap_ah\":" + String(cap) +
             ",\"soc\":" + String(p.s.soc) + ",\"v\":" + String(p.s.packV, 2) +
             ",\"busv\":" + String(p.s.busV, 2) +
             ",\"i\":" + String(p.s.current, 2) + ",\"soh\":" + String(p.s.soh) +
             ",\"tmin\":" + String(p.s.tmin) + ",\"tmax\":" + String(p.s.tmax) +
             ",\"cellmax\":" + String(hi) + ",\"cellmin\":" + String(lo == 0xFFFF ? 0 : lo) +
             ",\"cyc\":" + String(raw) + ",\"efc\":" + String(efc) + ",\"ah\":" + String(p.s.ahTotal) +
             ",\"ccl\":" + String(pCclDa / 10.0, 1) + ",\"dcl\":" + String(pDclDa / 10.0, 1) +
             ",\"nom_ccl\":" + String(pNomC / 10.0, 1) + ",\"nom_dcl\":" + String(pNomD / 10.0, 1) +
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
// handleClient() call. The two RS485 tasks load the CPU and heap heavily
// (esmTask reconfigures UART1 each poll; deyeTask preempts at priority 3). Thus
// a long upload starves the loop-task watchdog and resets mid-transfer. So
// suspend both tasks for the upload and feed the WDT on each chunk. Resume on abort.
static void handleUpdateUpload() {
    HTTPUpload &up = web.upload();
    if (up.status == UPLOAD_FILE_START) {
        if (esmH)  vTaskSuspend(esmH);
        if (deyeH) vTaskSuspend(deyeH);
        Update.begin(UPDATE_SIZE_UNKNOWN);
    } else if (up.status == UPLOAD_FILE_WRITE) {
        Update.write(up.buf, up.currentSize);
        vTaskDelay(1);                      // yield so IDLE1 runs and feeds its WDT
    } else if (up.status == UPLOAD_FILE_END) {
        Update.end(true);                   // on success, handleUpdateDone reboots
    } else if (up.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        if (esmH)  vTaskResume(esmH);        // upload failed: continue translation
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
    initNvs();
    loadCfg();

    // Start RS485 translation tasks BEFORE waiting on WiFi so the Deye inverter
    // receives BMS frames immediately on boot even if the router is offline.
    Serial2.begin(9600, SERIAL_8N1, DEYE_RX_PIN, DEYE_TX_PIN);
    mux = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(esmTask,  "esm",  8192, nullptr, 2, &esmH,  0);
    xTaskCreatePinnedToCore(deyeTask, "deye", 4096, nullptr, 3, &deyeH, 1);

    startWifi();                         // station from NVS, or fallback AP
    if (!apMode) applyNtp();             // SNTP only works once online

    ArduinoOTA.setHostname("esm-bank");
    ArduinoOTA.setPassword(OTA_PASS);
    ArduinoOTA.onStart([]() {
        if (esmH)  vTaskSuspend(esmH);
        if (deyeH) vTaskSuspend(deyeH);
    });
    ArduinoOTA.onError([](ota_error_t) {
        if (esmH)  vTaskResume(esmH);
        if (deyeH) vTaskResume(deyeH);
    });
    ArduinoOTA.begin();

    web.on("/", []() {
        web.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
        web.send(200, "text/html; charset=utf-8", htmlPage());
    });
    web.on("/save", HTTP_POST, handleSave);
    web.on("/rebulk", HTTP_POST, handleRebulk);
    web.on("/status", handleStatus);
    // captive portal: send any other URL to the config page (setup-AP use)
    web.onNotFound([]() { web.sendHeader("Location", "/"); web.send(302, "text/plain", ""); });
    web.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
    web.begin();
}

void loop() {
    static uint32_t lastWifi = 0, lastMqtt = 0, lastPub = 0, lastDisp = 0, lastStat = 0;
    static uint32_t lastSavedEocEpoch = 0;
    static bool wasConnected = false;
    uint32_t now = millis();

    ArduinoOTA.handle();
    web.handleClient();
    if (apMode) dnsServer.processNextRequest();
    if (mqtt.connected()) mqtt.loop();

    if (pendingSaveEocTs) {
        pendingSaveEocTs = false;
        uint32_t eocEp = 0;
        if (xSemaphoreTake(mux, pdMS_TO_TICKS(20)) == pdTRUE) { eocEp = cfg.lastEocEpoch; xSemaphoreGive(mux); }
        if (eocEp > 0 && (lastSavedEocEpoch == 0 || eocEp - lastSavedEocEpoch >= 3600)) {
            if (prefs.begin("bank", false)) {
                prefs.putULong("eocts", eocEp);
                prefs.end();
                lastSavedEocEpoch = eocEp;
            }
        }
    }

    if (apMode) {
        // If we fell back to AP+STA because the home router was still booting
        // (e.g. after a grid outage), close the setup AP as soon as STA connects,
        // or retry saved NVS WiFi every 30s whenever no client is on the AP.
        if (cfg.wifiSsid[0] && WiFi.status() == WL_CONNECTED) {
            Serial.printf("[wifi] '%s' connected (ip=%s) -> closing fallback AP\n",
                          cfg.wifiSsid, WiFi.localIP().toString().c_str());
            dnsServer.stop();
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_STA);
            WiFi.setSleep(false);
            apMode = false;
            wasConnected = true;
            applyNtp();
        } else if (cfg.wifiSsid[0] && now - lastWifi > 30000) {
            lastWifi = now;
            if (WiFi.softAPgetStationNum() == 0) {
                WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
            }
        }
    } else {
        bool up = (WiFi.status() == WL_CONNECTED);
        if (up && !wasConnected) {
            wasConnected = true;
            applyNtp();
        } else if (!up) {
            wasConnected = false;
            // Space reconnect calls 25s apart so WPA2 handshakes are never aborted mid-flight
            if (cfg.wifiSsid[0] && now - lastWifi > 25000) {
                lastWifi = now;
                WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
            }
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
        static String ip;
        ip = WiFi.isConnected() ? WiFi.localIP().toString()
           : (apMode ? ("AP " + WiFi.softAPIP().toString()) : String("WiFi..."));
        v.ip = ip.c_str();
        v.chgTag = a.valid ? (a.fullStop ? "STOP" : (a.floatMode ? "FLT" : "BULK")) : "";
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
                      apMode ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str(),
                      mqtt.connected());
    }
    delay(2);
}
