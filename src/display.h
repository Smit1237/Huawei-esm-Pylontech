#pragma once
#include "huawei_esm.h"

void displayInit();

// Full-screen status message (probe/wake/auth/error states)
void displayStatus(const char *line1, const char *line2 = "");

// Main data screen. statusValid gates the fault/warning row.
void displayData(const EsmMainData &d, const EsmStatusBits &s, bool statusValid);

// ---- ESM <-> Deye translator debug screen ----
// Plain values only, so this header stays independent of pylontech.h.
struct TranslatorView {
    // ESM side
    bool        packValid;      // have we got fresh pack data?
    const char *esmState;       // "PROBE" / "AUTH" / "LINK" / "STALE"
    int         soc, soh;
    float       packV, current; // V, A (+ = charge)
    int         tmin, tmax, cycles;
    int         addr;           // 0 if not yet found
    // Deye side
    uint32_t    deyeAnswered;   // total frames answered
    uint32_t    deyeAgeMs;      // ms since last poll; UINT32_MAX = never
    const char *deyeCid2;       // last command ("61"/"62"/"63" or "-")
    // forwarded limits (effective, A) — shown only if haveLimits
    bool        haveLimits;
    float       cclEff, dclEff;
    // raw ESM bus byte counters (rx==0 while tx climbs => no reply)
    uint32_t    esmTx, esmRx;
};
void displayTranslator(const TranslatorView &v);

// ---- multi-pack bank screen ----
struct BankView {
    bool     valid;
    int      nPacks;          // packs currently reporting
    int      soc, soh;        // aggregate
    float    packV, current;  // V, A (+ = charge)
    int      tmin, tmax;
    int      cclA, dclA;      // aggregate charge/discharge limits (A)
    // per-pack SOC for the little row (index 0..nPacks-1), -1 = absent
    int      packSoc[6];
    int      packAddr[6];
    // link status
    const char *ip;           // WiFi IP or "WiFi..."
    bool     mqttOn, mqttOk;
    uint32_t deyeAgeMs;       // ms since last Deye poll; UINT32_MAX = never
};
void displayBank(const BankView &v);
