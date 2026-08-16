#pragma once
#include "huawei_esm.h"

void displayInit();

// Full-screen status message. It shows the probe, wake, auth, and error states.
void displayStatus(const char *line1, const char *line2 = "");

// Main data screen. The statusValid flag controls the fault and warning row.
void displayData(const EsmMainData &d, const EsmStatusBits &s, bool statusValid);

// ---- ESM to and from Deye translator debug screen ----
// This screen uses plain values only. Thus the header stays independent of pylontech.h.
struct TranslatorView {
    // ESM side
    bool        packValid;      // Set to true when fresh pack data is available.
    const char *esmState;       // State text: "PROBE", "AUTH", "LINK", or "STALE".
    int         soc, soh;
    float       packV, current; // Pack voltage in V and current in A. A positive current shows charge.
    int         tmin, tmax, cycles;
    int         addr;           // Address. A value of 0 shows the address is not found yet.
    // Deye side
    uint32_t    deyeAnswered;   // Total number of frames answered.
    uint32_t    deyeAgeMs;      // Time in ms since the last poll. UINT32_MAX shows no poll yet.
    const char *deyeCid2;       // Last command: "61", "62", "63", or "-".
    // Forwarded limits in A (effective). Shown only when haveLimits is true.
    bool        haveLimits;
    float       cclEff, dclEff;
    // Raw ESM bus byte counters. If rx stays 0 while tx climbs, there is no reply.
    uint32_t    esmTx, esmRx;
};
void displayTranslator(const TranslatorView &v);

// ---- multi-pack bank screen ----
struct BankView {
    bool     valid;
    int      nPacks;          // Number of packs that report now.
    int      soc, soh;        // Aggregate SOC and SOH.
    float    packV, current;  // Pack voltage in V and current in A. A positive current shows charge.
    int      tmin, tmax;
    int      cclA, dclA;      // Aggregate charge and discharge limits in A.
    // Per-pack SOC for the row. Index 0 to nPacks-1. A value of -1 shows absent.
    int      packSoc[6];
    int      packAddr[6];
    // Link status.
    const char *ip;           // WiFi IP address, or the text "WiFi...".
    bool     mqttOn, mqttOk;
    uint32_t deyeAgeMs;       // Time in ms since the last Deye poll. UINT32_MAX shows no poll yet.
};
void displayBank(const BankView &v);
