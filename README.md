# Huawei ESM to Pylontech (Deye) battery translator

This firmware makes one or more **Huawei ESM** lithium packs work with a **Deye
or SunSynk hybrid inverter**. You do not need a Huawei controller (SMU or ECC).

An ESP32 reads each pack over RS485/Modbus. It uses the Huawei FC41 unlock
sequence. It joins the packs into **one virtual battery**. It sends this battery
to the inverter as a **Pylontech RS485 BMS** ("Lithium mode 12"). WiFi is
optional. WiFi adds a web dashboard, MQTT (down to each cell voltage), NTP, and
updates over the air.

```
  ┌────────┐   ┌────────┐   ┌────────┐   ┌────────┐
  │ ESM 214│   │ ESM 215│   │ ESM 216│   │ ESM 217│   4x Huawei ESM packs
  └───┬────┘   └───┬────┘   └───┬────┘   └───┬────┘   self-address over CAN
      │  RS485 (Modbus, 9600 8N1) — one shared bus  │  (CAN terminator required)
      └────────────┴──────┬──────┴────────────┘
                          │
                   ┌──────┴───────┐
                   │    ESP32     │  reads all packs, aggregates,
                   │  (this repo) │  emulates one Pylontech battery
                   └──────┬───────┘
                          │  RS485 (Pylontech YDN1363, "Lithium mode 12")
                   ┌──────┴───────┐
                   │  Deye / Sun- │  reads ONE battery: SOC, V, A, temps,
                   │ Synk inverter│  cell min/max, charge/discharge limits
                   └──────────────┘
```

---

## Features

- **Multi-pack aggregation.** The firmware shows N packs as one Pylontech
  battery. It averages the voltage. It sums the current, the capacity, and the
  limits. It averages the SOC. It uses the SOH of the worst pack. It sets the
  charge and discharge cut-offs from the highest and lowest cell in the bank.
  The firmware also works with one pack.
- **Real Huawei telemetry.** The firmware uses Modbus RTU and the FC41 unlock
  sequence (FC03 `0x0106`, then FC10 time sync, then FC41 access). You get the
  true SOC, the cell voltages, the sensor temperatures, the cycles, the Ah, and
  the pack limit registers.
- **Fail-safe.** If all packs stay unreadable for 15 seconds, the emulator
  becomes silent. Then the inverter goes to a safe state. It does not use old
  data. The firmware clamps the charge-current limit. A bad read cannot increase
  the limit.
- **Web dashboard.** The dashboard shows a live per-pack table and an aggregate
  summary. A JSON endpoint (`/status`) gives the data to Home Assistant or a
  script.
- **MQTT (optional).** The firmware sends one aggregate topic. It also sends one
  topic for each pack. Each pack topic includes every cell and every temperature
  sensor.
- **NTP and timezone.** Set the servers and an IANA zone name (for example
  `Europe/Moscow`). The timestamps are then correct.
- **OTA (updates over the air).** Flash the firmware one time over USB. After
  that, update it over WiFi. Use `espota`, the browser upload page, or the WiFi
  setup access point.

---

## Important: CAN-bus termination

The packs self-address to `214, 215, 216, 217`, and higher. This works only when
the CAN bus between the packs has termination. The packs set their addresses over
the CAN pair. Without the terminating resistor, all packs stay at the default
address **214**. Then they collide on the shared RS485 bus. You see garbled
frames that change continuously.

- Install a **120 Ω resistor across CAN-H and CAN-L**. Install it at the
  `COM_OUT` port of the **last pack**. On the ESM RJ45 connector this is
  **pin 7 (CAN-H) and pin 8 (CAN-L)**.
- The terminator comes in the accessory bag with the batteries. It is part of the
  Huawei system. **Keep it installed.**
- Connect the pack COM ports in a chain (`COM_OUT` to `COM_IN`). Install the
  terminator on the last `COM_OUT`. Do a full power cycle of the bank. Let the
  addresses settle.

Result: one ESP32 on one shared RS485 bus reads `214` to `217` correctly. You do
not need an SMU, an ECC500, or a maintenance tool. Correct CAN termination is
sufficient.

> RS485 termination (120 Ω across A and B) is a different resistor. You usually
> do not need it at 9600 baud on short indoor cables. Add it only for long or
> noisy cables that give CRC errors.

---

## Hardware

### Bill of materials

| Qty | Item |
|----:|------|
| 1 | **ESP32** board. Use a classic ESP32 (WROOM-32, WROVER, DevKitC, or NodeMCU-32S). Or use a **LilyGO T-Display** for a screen. |
| **2** | **RS485-to-UART converters**, auto-direction type (no DE/RE pin). For example the HW-519, or a MAX485 board with automatic flow control. You need two converters. One is for the battery bus. One is for the inverter bus. |
| 1 | 120 Ω CAN terminator (comes with the batteries — see above). |
| — | Dupont wires. Use a common ground between the ESP32 and both converters. |

Use a classic ESP32. It has **three hardware UARTs** (one console and two spare).
The design needs two spare UARTs, one for each RS485 bus. The ESP32-S3 also
works. The ESP32-C3, C2, and H2 do not work. They have only about two usable
UARTs.

### RS485 converters and wiring

Use **auto-direction** converters. They switch between transmit and receive
automatically. You wire only the power and the two data lines. The wiring is
**straight-through**. Connect the converter `TXD` pad to the ESP32 **TX** pin.
Connect the converter `RXD` pad to the ESP32 **RX** pin. Do not cross them.

| Converter pad | Connect to |
|---------------|------------|
| `VCC`         | ESP32 **5V**. The converters need 5 V. The logic is 3.3 V safe. |
| `GND`         | ESP32 **GND**. A common ground is necessary. |
| `TXD`         | ESP32 **TX** GPIO |
| `RXD`         | ESP32 **RX** GPIO |
| `A` / `B`     | RS485 `A`/`B` on the pack or the inverter. Connect A to A and B to B. |

### You need two RS485 converters — one for each bus

The ESP32 is between two separate RS485 buses. It needs **two** converters. Each
converter uses its own UART.

- **Converter A goes to the batteries** (ESM COM port). It uses UART1.
- **Converter B goes to the inverter** (Deye BMS port). It uses UART2.

The converters do not share pins. Each converter needs four wires: `VCC` (5V),
`GND`, `TXD`, and `RXD`. Connect `TXD` to the ESP32 **TX** GPIO. Connect `RXD` to
the ESP32 **RX** GPIO. Do not cross them. All grounds must be common.

### Pin map for each board

The default firmware pins are **GPIO33 and GPIO32** (ESM), and **GPIO26 and
GPIO25** (Deye). These pins are in good positions on the common boards.

**Generic ESP32 DevKit (WROOM-32, WROVER, NodeMCU-32S, DevKitC)**

| Converter | Converter pad | ESP32 pin |
|-----------|---------------|-----------|
| A — ESM (battery) | `TXD` | **GPIO33** |
| A — ESM (battery) | `RXD` | **GPIO32** |
| A — ESM (battery) | `VCC` / `GND` | 5V / GND |
| B — Deye (inverter) | `TXD` | **GPIO26** |
| B — Deye (inverter) | `RXD` | **GPIO25** |
| B — Deye (inverter) | `VCC` / `GND` | 5V / GND |

GPIO 32, 33, 25, and 26 are free, non-strapping, and output-capable on the
WROOM-32 and the WROVER. The WROVER PSRAM uses GPIO 16 and 17, so the design
avoids them.

**LilyGO T-Display v1.1**

The GPIOs are the same. GPIO 32, 33, 25, 26, plus 5V and GND are all on the
right header. Both converters connect to one continuous row. The TFT uses GPIO 4,
5, 16, 18, 19, and 23 internally. These pins are not on the header, so there is
no conflict.

| Converter | Converter pad | Header pin |
|-----------|---------------|-----------|
| A — ESM | `TXD` / `RXD` | **GPIO33** / **GPIO32** |
| B — Deye | `TXD` / `RXD` | **GPIO26** / **GPIO25** |
| both | `VCC` / `GND` | 5V / GND (right header) |

**ESP32-S3 (for example the DevKitC-1)** has enough UARTs, but the GPIOs are
different. Choose four free pins. Avoid the USB, strapping, PSRAM, and flash
pins. GPIO 4, 5, 6, and 7 are a safe set on most S3 boards. Set them at build
time:

```ini
build_flags = -DESM_TX_PIN=4 -DESM_RX_PIN=5 -DDEYE_TX_PIN=6 -DDEYE_RX_PIN=7
```

You can remap any pin on any board. Use the `-DESM_TX_PIN=`, `-DESM_RX_PIN=`,
`-DDEYE_TX_PIN=`, and `-DDEYE_RX_PIN=` flags.

On the inverter side, connect the `A` and `B` wires of converter B to the Deye
**BMS/CAN RS485 port**. This is the port for lithium communication. Set the
inverter to **Lithium mode 12** (see below).

### ESM COM port (RJ45) reference

| Pin | Signal |
|----:|--------|
| 7   | CAN-H  — install the 120 Ω terminator here … |
| 8   | CAN-L  — … to here, on the last pack |
| —   | RS485 A / B for Modbus telemetry. The ESP32 reads this pair. |

The exact RS485 A and B pins change with the model. Check the pack manual or the
silkscreen. Connect A to A and B to B. The CAN terminator pins (7 and 8) are the
same for all ESM packs.

---

## Build and flash

### 1. Install PlatformIO

Use the [PlatformIO IDE](https://platformio.org/) (VS Code extension) or the
command line:

```bash
pip install platformio
```

### 2. Set your WiFi (optional)

WiFi is optional. The RS485 translation works without WiFi. WiFi adds the web UI,
MQTT, and OTA. There are three ways to set the WiFi. This is the order of
precedence:

1. **Setup access point (easiest, no rebuild).** Flash the firmware. If it cannot
   join a network, it makes an access point (**`ESM-Bank-Setup`**, password
   `esmbank-setup`). Join it with a phone. The configuration page opens
   automatically (captive portal). Enter your WiFi and press Save. The device
   restarts and joins your network.
2. **Compile the credentials.** Copy `src/wifi_config.example.h` to
   `src/wifi_config.h` (git-ignored). Set `WIFI_SSID`, `WIFI_PASS`, and
   `OTA_PASS`.
3. **Build flags.** Pass `-DWIFI_SSID='"…"'` and `-DWIFI_PASS='"…"'`.

### 3. Select the board and build

| PlatformIO env | Board | Screen |
|----------------|-------|--------|
| `bank_esp32` | generic ESP32 (WROOM/WROVER/…) | no |
| `bank_tdisplay` | LilyGO T-Display v1.1 | yes |
| `translator_esp32` | generic ESP32 — single pack, no WiFi | no |
| `translator_tdisplay` | LilyGO T-Display — single pack | yes |

```bash
pio run -e bank_esp32                 # build the firmware
pio run -e bank_esp32 -t upload       # flash over USB
pio device monitor                    # read the serial log
```

### 4. Update over the air (after the first USB flash)

Use one of three wireless methods:

- **espota** (command line): `pio run -e bank_esp32_ota -t upload`. It uses the
  mDNS name `esm-bank.local`. Set `upload_port` to the device IP if mDNS fails.
  The `--auth` value must match `OTA_PASS`.
- **Browser:** open `http://<device-ip>/`. Select Firmware, then Choose file,
  then Upload.
- **Setup access point:** use the same browser upload after you join
  `ESM-Bank-Setup`.

---

## Configuration and the web dashboard

Open `http://<device-ip>/` (or `http://esm-bank.local/`):

- **Live view.** It shows the aggregate SOC, V, and A, and a per-pack table with
  the serial numbers.
- **WiFi.** Change the network. The device restarts to reconnect.
- **MQTT.** Enable it. Set the broker host and port, the user and password, the
  topic prefix, and the publish interval. There is a "publish each pack
  separately" option. MQTT is off by default.
- **Time (NTP).** Set up to three servers and a timezone. Use an IANA name (for
  example `Europe/Berlin`) or a raw POSIX string. The change is applied live.
- **Firmware.** Upload new firmware from the browser.

`GET /status` returns all data as JSON. It includes the aggregate, the Deye poll
counters, the network state, and every pack register (all cells, all
temperatures, and the fault words). Use it with Home Assistant or `curl`.

---

## MQTT

When MQTT is on, the firmware publishes every `interval` seconds. The default
topic prefix is `esmbank`.

**`esmbank/bank`** — the virtual battery that the inverter reads:

```json
{"soc":100,"v":51.64,"i":0.00,"soh":95,"tmin":18,"tmax":22,
 "cycles":92,"ccl":325.9,"dcl":346.8,"packs":4,"ts":1786669362}
```

**`esmbank/pack/<addr>`** — one message for each pack, with all its data:

```json
{"addr":214,"sn":"EX19B0004574","soc":100,"v":51.11,"busv":53.69,"i":0.00,
 "soh":95,"tmin":20,"tmax":22,"cellmax":3413,"cellmin":3344,"spread":69,
 "cycles":92,"ah":2316,"ccl":102.0,"dcl":102.0,"cvl":54.0,"dvl":47.5,
 "cclcoef":0,"dclcoef":85,"sysf":0,"cellf":0,"prot":0,"warn":0,"spec":0,
 "ncells":15,"ntemps":15,
 "cells":[3412,3412,3411,3412,3412,3411,3411,3412,3344,3412,3412,3413,3412,3412,3412],
 "temps":[21,21,21,20,20,20,20,21,21,21,21,22,22,22,22],"ts":1786669362}
```

Fields: `v` is the pack terminal voltage. `busv` is the shared DC-bus voltage.
The cell values are in **mV**. The temperatures are in **°C**. `ah` is the
lifetime throughput. `ccl`, `dcl`, `cvl`, and `dvl` are the charge and discharge
current and voltage limits. The `*coef` values are the pack derate percentage.
`sysf`, `cellf`, `prot`, `warn`, and `spec` are the fault, warning, and
protection words (`0` means healthy). `ts` is the UTC time in epoch seconds
(`0` until NTP syncs).

---

## Deye / SunSynk inverter setup

1. Set the battery type to **Lithium**.
2. Set the BMS protocol to **Mode 12** (Pylontech RS485). Mode 00 is Pylontech
   CAN. This firmware does not use Mode 00.
3. Connect the Deye-side RS485 converter to the inverter BMS RS485 port.

The emulator answers as Pylontech address `0x02`. Change it with `-DDEYE_ADDR=`.
The inverter sends `CID1=0x46` and `CID2=0x61/62/63` polls.

---

## How aggregation works

| Quantity | Rule |
|----------|------|
| Voltage | average (the packs are in parallel) |
| Current | sum |
| SOC | average (the packs have equal capacity) |
| SOH | minimum (the worst pack) |
| Charge cut-off cell | the highest cell in the bank |
| Discharge cut-off cell | the lowest cell in the bank |
| Temperatures | minimum and maximum of all sensors |
| CCL / DCL | sum of each pack effective limit (the firmware applies the coefficient and clamps it to 100 % maximum) |
| CVL / DVL | most conservative (lowest CVL, highest DVL) |
| Faults | logical OR |

A pack that stays unread for 15 seconds leaves the aggregate. If no pack remains,
the emulator stops the answers to the inverter.

---

## Troubleshooting

| Symptom | Cause and fix |
|---------|---------------|
| Only address 214 answers. The frames are garbled and change. | The CAN terminator is missing or bad. The packs collide at 214. Install the 120 Ω resistor across pins 7 and 8 on the last pack. Do a power cycle. |
| No data on the ESM bus (`tx` increases, `rx` stays 0). | The RS485 A and B are swapped, the ground is not common, or the pins are wrong. Swap A and B. |
| A pack reads, but the SOC and limits are 0. | The pack answered the telemetry but not the limit block. The values fill in after a few poll cycles. |
| The inverter shows "battery not connected". | The Deye is not in Lithium Mode 12, the RS485 A and B are swapped on the inverter side, or the emulator is silent because no pack is readable. |
| espota shows "No response from device". | The device is busy or it restarts. Try again. If it crashed, flash a good build over USB one time. Then OTA works again. |

---

## Protocol references and credits

- Huawei ESM Modbus and FC41 handshake research:
  [williamsioSapo/Huawei_Battery_monitor](https://github.com/williamsioSapo/Huawei_Battery_monitor).
- Pylontech YDN1363 and Deye RS485 emulation, based on
  [Fahmula/esphome-pylontech-rs485](https://github.com/Fahmula/esphome-pylontech-rs485).

The firmware was built and tested with a bank of 4 ESM-48100B1 packs (100 Ah)
and a Deye SUN-*K-SG05LP1 inverter.

## License

MIT. See [LICENSE](LICENSE). The firmware is provided as-is. You work with live
mains inverters and lithium batteries at your own risk.
