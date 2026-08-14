# Huawei ESM → Pylontech (Deye) battery translator

Turn one or more **Huawei ESM** telecom lithium packs (ESM-48100B1, ESM-4875A1
and relatives) into a battery your **Deye / SunSynk hybrid inverter** understands
— no genuine Huawei controller (SMU/ECC) required.

An ESP32 reads each pack over RS485/Modbus (including Huawei's proprietary FC41
unlock), **aggregates several packs into ONE virtual battery**, and presents it to
the inverter as a **Pylontech RS485 BMS** ("Lithium mode 12"). Optional WiFi adds
a web dashboard, MQTT (down to per-cell voltages), NTP, and over-the-air updates.

```
  ┌────────┐   ┌────────┐   ┌────────┐   ┌────────┐
  │ ESM 214│   │ ESM 215│   │ ESM 216│   │ ESM 217│   4× Huawei ESM packs
  └───┬────┘   └───┬────┘   └───┬────┘   └───┬────┘   self-address over CAN
      │  RS485 (Modbus, 9600 8N1) — one shared bus  │  (needs CAN terminator!)
      └────────────┴──────┬──────┴────────────┘
                          │
                   ┌──────┴───────┐
                   │    ESP32     │  reads all packs, aggregates,
                   │  (this repo) │  emulates one Pylontech battery
                   └──────┬───────┘
                          │  RS485 (Pylontech YDN1363, "Lithium mode 12")
                   ┌──────┴───────┐
                   │  Deye / Sun- │  sees ONE battery: SOC, V, A, temps,
                   │ Synk inverter│  cell min/max, charge/discharge limits
                   └──────────────┘
```

---

## Features

- **Multi-pack aggregation** — presents N packs as a single Pylontech battery
  (voltage averaged, current/capacity/limits summed, SOC averaged, SOH the
  worst pack, charge/discharge cut-offs from the highest/lowest cell across the
  whole bank). Also runs great with a **single** pack.
- **Real Huawei telemetry** — Modbus RTU + the FC41 unlock handshake
  (FC03 `0x0106` → FC10 time-sync → FC41 access), so you get true SOC, per-cell
  voltages, per-sensor temperatures, cycles, Ah and the packs' own limit
  registers — not a guessed voltage curve.
- **Fail-safe** — if every pack goes unreadable for 15 s the emulator goes
  **silent**, so the inverter faults to a safe state instead of trusting stale
  data. Charge-current limits are clamped so a bad read can never *inflate* them.
- **Web dashboard** — live per-pack table, aggregate summary, and a JSON
  `/status` endpoint for Home Assistant / scripting.
- **MQTT** (optional) — one aggregate topic plus a fully-detailed topic per
  pack, including every cell and every temperature sensor.
- **NTP + timezone** — configurable servers and IANA zone names (`Europe/Moscow`
  …), so timestamps are correct.
- **OTA everywhere** — flash once over USB, then update wirelessly via
  `espota`, a browser upload page, **or** the built-in Wi-Fi **setup AP** — no
  more trips to the basement.

---

## ⚠️ The one thing everybody gets wrong: CAN-bus termination

Multiple ESM packs **self-address** to `214, 215, 216, 217…` **only when the CAN
bus between them is terminated.** The packs negotiate their addresses over the
CAN pair; without the terminating resistor they all stay stuck at the default
address **214** and collide on the shared RS485 bus (you'll see garbled,
ever-changing frames).

- Fit a **120 Ω resistor across CAN-H and CAN-L** at the **last pack's
  `COM_OUT`** port. On the ESM RJ45 that is **pin 7 (CAN-H) ↔ pin 8 (CAN-L)**.
- This terminator ships in the accessory bag with the batteries — it is part of
  Huawei's designed system. **Keep it installed.**
- Daisy-chain the packs' COM ports normally (`COM_OUT` → `COM_IN`), terminator
  on the very last `COM_OUT`. Power-cycle the whole bank once with it in place
  and let the addresses settle.

With the terminator in, a single ESP32 on one shared RS485 bus reads
`214…217` cleanly. **No SMU, no ECC500, no maintenance tool.** (This overturns
the common advice that a Huawei controller is mandatory — it is not; correct CAN
termination alone is sufficient.)

> RS485 termination (120 Ω across A/B) is a *different* resistor and is normally
> unnecessary at 9600 baud over short indoor runs. Add it only if you have long
> or electrically noisy cabling and see CRC errors.

---

## Hardware

### Bill of materials

| Qty | Item |
|----:|------|
| 1 | **ESP32** dev board — classic ESP32 (WROOM-32 / WROVER / DevKitC / NodeMCU-32S), or a **LilyGO T-Display** for an on-board screen |
| **2** | **RS485 ↔ UART** modules, **auto-direction** type (no DE/RE pin) — e.g. HW-519, or a MAX485 board with automatic flow control. **You need two: one for the battery bus, one for the inverter bus.** |
| 1 | 120 Ω CAN terminator (ships with the batteries — see above) |
| — | Dupont wires; a common ground between the ESP32 and both RS485 modules |

A classic ESP32 is recommended because it has **three hardware UARTs** (console +
two spare). The design needs two spare UARTs — one per RS485 bus. ESP32-S3 works
too; ESP32-C3/C2/H2 do **not** (only ~2 usable UARTs).

### RS485 modules & wiring

Use **auto-direction** modules — they switch TX/RX automatically, so you only
wire power and the two data lines. Wiring is **straight-through** (the module's
`TXD` pad goes to the ESP32's **TX** pin, `RXD` to the ESP32's **RX** pin — *not*
crossed):

| RS485 module pad | Connect to |
|------------------|------------|
| `VCC`            | ESP32 **5V** (these modules want 5 V; logic is 3.3 V-safe) |
| `GND`            | ESP32 **GND** (common ground is essential) |
| `TXD`            | ESP32 **TX** GPIO |
| `RXD`            | ESP32 **RX** GPIO |
| `A` / `B`        | RS485 `A`/`B` on the pack (or inverter) — A→A, B→B |

### You need TWO RS485 converters — one per bus

The ESP32 sits between two separate RS485 buses, so it needs **two** converters,
each on its own UART:

- **Converter A → the batteries** (ESM COM port) — UART1
- **Converter B → the inverter** (Deye BMS port) — UART2

They never share pins. Each converter needs 4 wires: `VCC`(5V), `GND`, `TXD`,
`RXD`. Wiring is **straight-through**: module `TXD` → ESP32 **TX** GPIO, module
`RXD` → ESP32 **RX** GPIO (not crossed). All grounds must be common.

### Per-board pin map

The firmware default pins are **GPIO33/32** (ESM) and **GPIO26/25** (Deye).
These land in convenient places on the common boards:

**Generic ESP32 DevKit (WROOM-32 / WROVER / NodeMCU-32S / DevKitC)**

| Converter | module pad | ESP32 pin |
|-----------|-----------|-----------|
| A — ESM (battery) | `TXD` | **GPIO33** |
| A — ESM (battery) | `RXD` | **GPIO32** |
| A — ESM (battery) | `VCC` / `GND` | 5V / GND |
| B — Deye (inverter) | `TXD` | **GPIO26** |
| B — Deye (inverter) | `RXD` | **GPIO25** |
| B — Deye (inverter) | `VCC` / `GND` | 5V / GND |

GPIO 32/33/25/26 are free, non-strapping and output-capable on WROOM-32 and
WROVER (the WROVER's PSRAM uses 16/17, which are deliberately avoided).

**LilyGO T-Display v1.1**

Same GPIOs — and conveniently, **GPIO 32, 33, 25, 26 plus 5V and GND are all on
the right-hand header**, so both converters land on one contiguous run. The TFT
uses 4/5/16/18/19/23 internally (not broken out), so there is no clash.

| Converter | module pad | header pin |
|-----------|-----------|-----------|
| A — ESM | `TXD` / `RXD` | **GPIO33** / **GPIO32** |
| B — Deye | `TXD` / `RXD` | **GPIO26** / **GPIO25** |
| both | `VCC` / `GND` | 5V / GND (right header) |

**ESP32-S3 (e.g. DevKitC-1)** — has enough UARTs, but different GPIOs. Pick four
free pins (avoid the USB, strapping, PSRAM and flash pins; GPIO 4/5/6/7 are a
safe set on most S3 boards) and set them at build time:

```ini
build_flags = -DESM_TX_PIN=4 -DESM_RX_PIN=5 -DDEYE_TX_PIN=6 -DDEYE_RX_PIN=7
```

Any pin can be remapped on any board with the
`-DESM_TX_PIN=` / `-DESM_RX_PIN=` / `-DDEYE_TX_PIN=` / `-DDEYE_RX_PIN=` flags.

On the **inverter** side, connect converter B's `A`/`B` to the Deye's **BMS/CAN
RS485 port** (the port used for lithium comms) and set the inverter to **Lithium
mode 12** (see below).

### ESM COM-port (RJ45) reference

| Pin | Signal |
|----:|--------|
| 7   | CAN-H  ← 120 Ω terminator here … |
| 8   | CAN-L  ← … to here, on the last pack |
| —   | RS485 A / B for Modbus telemetry (this is what the ESP32 reads) |

The exact RS485 A/B pins vary slightly by model — **check your pack's manual or
silkscreen** and connect A→A, B→B. The CAN terminator pins (7/8) are consistent
across the ESM family.

---

## Build & flash

### 1. Install PlatformIO

Use the [PlatformIO IDE](https://platformio.org/) (VS Code extension) or the CLI:

```bash
pip install platformio
```

### 2. Set your WiFi (optional)

WiFi is **optional** — the RS485 translation runs without it. WiFi adds the
WebUI, MQTT and OTA. Three ways to configure it, in order of precedence:

1. **Setup AP (easiest, no rebuild).** Flash the firmware as-is; if it can't
   join a network it hosts an access point **`ESM-Bank-Setup`** (password
   `esmbank-setup`). Join it from a phone, the config page pops up (captive
   portal), enter your WiFi, Save — the device reboots and joins your network.
2. **Compile it in.** Copy `src/wifi_config.example.h` → `src/wifi_config.h`
   (git-ignored) and fill in `WIFI_SSID` / `WIFI_PASS` / `OTA_PASS`.
3. **Build flags.** Pass `-DWIFI_SSID='"…"' -DWIFI_PASS='"…"'`.

### 3. Pick your board & build

| PlatformIO env | Board | Screen |
|----------------|-------|--------|
| `bank_esp32` | generic ESP32 (WROOM/WROVER/…) | no |
| `bank_tdisplay` | LilyGO T-Display v1.1 | yes |
| `translator_esp32` | generic ESP32 — single-pack, no WiFi | no |
| `translator_tdisplay` | LilyGO T-Display — single-pack | yes |

```bash
pio run -e bank_esp32                 # build
pio run -e bank_esp32 -t upload       # flash over USB
pio device monitor                    # watch the serial log
```

### 4. Update over the air (after the first USB flash)

Three options, all wireless:

- **espota** (CLI): `pio run -e bank_esp32_ota -t upload`
  (uses mDNS `esm-bank.local`; set `upload_port` to the device IP if mDNS is
  flaky; `--auth` must match `OTA_PASS`).
- **Browser**: open `http://<device-ip>/`, *Firmware → Choose file → Upload*.
- **Setup AP**: same browser upload while joined to `ESM-Bank-Setup`.

---

## Configuration & the web dashboard

Browse to `http://<device-ip>/` (or `http://esm-bank.local/`):

- **Live view** — aggregate SOC / V / A and a per-pack table with serials.
- **WiFi** — change network (reboots to reconnect).
- **MQTT** — enable, broker host/port, user/pass, topic prefix, publish
  interval, and a "publish each pack separately" toggle. Off by default.
- **Time (NTP)** — up to 3 servers + a timezone (IANA name like
  `Europe/Berlin`, autocomplete, or a raw POSIX string). Applied live.
- **Firmware** — browser OTA upload.

`GET /status` returns everything as JSON (aggregate, Deye poll counters, network
state, and every pack's registers incl. all cells/temps and fault words) — handy
for Home Assistant or `curl`.

---

## MQTT

When enabled, every `interval` seconds the device publishes (default prefix
`esmbank`):

**`esmbank/bank`** — the virtual battery the inverter sees:

```json
{"soc":100,"v":51.64,"i":0.00,"soh":95,"tmin":18,"tmax":22,
 "cycles":92,"ccl":325.9,"dcl":346.8,"packs":4,"ts":1786669362}
```

**`esmbank/pack/<addr>`** — one message per pack, everything it reports:

```json
{"addr":214,"sn":"EX19B0004574","soc":100,"v":51.11,"busv":53.69,"i":0.00,
 "soh":95,"tmin":20,"tmax":22,"cellmax":3413,"cellmin":3344,"spread":69,
 "cycles":92,"ah":2316,"ccl":102.0,"dcl":102.0,"cvl":54.0,"dvl":47.5,
 "cclcoef":0,"dclcoef":85,"sysf":0,"cellf":0,"prot":0,"warn":0,"spec":0,
 "ncells":15,"ntemps":15,
 "cells":[3412,3412,3411,3412,3412,3411,3411,3412,3344,3412,3412,3413,3412,3412,3412],
 "temps":[21,21,21,20,20,20,20,21,21,21,21,22,22,22,22],"ts":1786669362}
```

Fields: `v` = pack terminal volts, `busv` = shared DC-bus volts, cell values in
**mV**, temps in **°C**, `ah` = lifetime throughput, `ccl/dcl/cvl/dvl` =
charge/discharge current & voltage limits, `*coef` = the pack's own derate %,
and `sysf/cellf/prot/warn/spec` = the fault/warning/protection bit-words
(0 = healthy). `ts` is UTC epoch seconds (0 until NTP has synced).

---

## Deye / SunSynk inverter setup

1. Battery type: **Lithium**.
2. BMS protocol: **Mode 12** (= Pylontech RS485). (Mode 00 is Pylontech CAN —
   not what this firmware speaks.)
3. Wire the Deye-side RS485 module to the inverter's BMS RS485 port.

The emulator answers as Pylontech address `0x02` (`-DDEYE_ADDR=` to change) to
the inverter's `CID1=0x46`, `CID2=0x61/62/63` polls.

---

## How aggregation works

| Quantity | Rule |
|----------|------|
| Voltage | average (packs are paralleled) |
| Current | sum |
| SOC | average (equal-capacity packs) |
| SOH | minimum (worst pack) |
| Charge cut-off cell | highest cell across the bank |
| Discharge cut-off cell | lowest cell across the bank |
| Temps | min / max across all sensors |
| CCL / DCL | sum of each pack's **effective** limit (its own coefficient folded in, clamped ≤ 100 %) |
| CVL / DVL | most conservative (lowest CVL, highest DVL) |
| Faults | logical OR |

A pack that hasn't been read for 15 s drops out of the aggregate; if none remain,
the emulator stops answering the inverter.

---

## Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| Only address 214 answers, frames are garbled and keep changing | Missing/!bad **CAN terminator** — the packs are colliding at 214. Fit the 120 Ω across pins 7/8 on the last pack, power-cycle the bank. |
| Nothing at all on the ESM bus (`tx` climbs, `rx` stays 0) | RS485 A/B swapped, no common ground, or wrong pins. Try swapping A/B. |
| A pack reads but SOC/limits are 0 | It answered telemetry but not its limit block yet — it fills in within a few poll cycles. |
| Inverter shows "battery not connected" | Deye not in Lithium **Mode 12**, RS485 A/B swapped on the inverter side, or the emulator is silent because no pack is readable. |
| espota "No response from device" | The device is busy or mid-reboot; retry. If it just crashed, USB-flash a good build once, then OTA works again. |

---

## Protocol references & credits

- Huawei ESM Modbus + FC41 handshake reverse-engineering:
  [williamsioSapo/Huawei_Battery_monitor](https://github.com/williamsioSapo/Huawei_Battery_monitor)
- Pylontech YDN1363 / Deye RS485 emulation modelled on
  [Fahmula/esphome-pylontech-rs485](https://github.com/Fahmula/esphome-pylontech-rs485).

Built and tested against a bank of 4× ESM-48100B1 (100 Ah) and a Deye
SUN-*K-SG05LP1.

## License

MIT — see [LICENSE](LICENSE). Provided as-is; you are working with live mains
inverters and lithium batteries at your own risk.
