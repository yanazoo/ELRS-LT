# ELRS Lap Timer

🌐 [日本語](README.md) | **English**

A lap timer that detects a drone's gate crossing from the RSSI of the drone's
**ELRS telemetry (TRSS) uplink**, captured by custom sniffer firmware flashed onto
a **HappyModel EP1/EP2 TCXO**.

> An evolution of ESP-NOW-Lap-Timer (the dedicated-beacon design).
> The drone keeps its stock ELRS firmware; a gate-side EP1/EP2 TCXO acts as the sniffer.
> Both the calibration graph and lap detection follow **the drone's telemetry signal only** —
> they do not react to the stationary transmitter's (TX) RSSI.

## Highlights

- **No drone modification** — keep the stock ELRS firmware. No XIAO beacon required
- **EP1/EP2 TCXO repurposed as the gate receiver** — ESP8285 + SX1280 + TCXO; custom firmware does FHSS following + RSSI measurement
- **Isolates the drone's telemetry (TRSS) only** — measures OTA packet type `0b11` (PACKET_TYPE_TLM) exclusively, so it never reacts to the stationary TX's signal level or movement. The calibration graph and lap detection both track the drone alone
- **Run the TX telemetry ratio at 1:2** — the gate captures the drone's telemetry *densely* only at **1:2** (a dwell then contains ~2 telemetry slots, so it survives the channel-retune that drops single-slot ratios), giving a smooth graph and a steady value. Higher ratios (1:4 / 1:8+) are captured sparsely and look choppy, so they are **not recommended**. An envelope filter further smooths the trace. Verified on ELRS 3.6.3
- Up to 4 pilots timed simultaneously, roster of up to 20
- Gate crossings detected via RSSI peak detection + a RotorHazard-style state machine
- Smooth RSSI processing with an EMA filter (α = 0.25)
- GitHub Dark themed Web UI (Japanese TTS, Canvas waveform graph, SD file browser)
- Automatic race CSV logging to SD card, plus roster backup/restore
- Switchable **HS mode** / **Immediate (timing) mode**

---

## Versions

| Branch | Stable telemetry-ratio range | Key differences |
|---|---|---|
| `1.0.0` | Solid up to ~1:4 | Required 2 samples per report window; misses increase above 1:8 |
| `1.0.1` | **Solid at 1:64 / functional at 1:128** | HOLD + ratio-adaptive silence (min=1), EMA α = 0.25 |
| `1.1.0` (= current `main`, multi-sniffer) | **Run the TX at 1:2** (dense capture = smooth) | telemetry-capture fix + envelope filter, SX1280 hang auto-recovery, UID gate, 4 sniffers at once, EMA α = 0.25 |

> `main` always tracks the latest (currently 1.1.0). Each release is preserved as a
> tag-like branch (`1.0.0` / `1.0.1` / `1.1.0`).
>
> Note: `1.1.0` improved telemetry **capture** after `1.0.1`. In practice, set the
> **TX telemetry ratio to 1:2** (dense capture, smooth trace — see below).

---

## Base Project

An improved version of [yanazoo/ESP-NOW-Lap-Timer](https://github.com/yanazoo/ESP-NOW-Lap-Timer).

| | ESP-NOW-Lap-Timer | ELRS Lap Timer (this repo) |
|---|---|---|
| Drone side | XIAO ESP32-C3 beacon mounted | Stock ELRS firmware (no modification) |
| Gate receive | ESP32 promiscuous mode | EP1/EP2 TCXO custom sniffer firmware |
| Identity key | ESP-NOW source MAC | 6-byte ELRS bind UID |
| Gate link | ESP-NOW | ESP-NOW (same) |

---

## Hardware Architecture

```
  Pilot①        Pilot②        Pilot③        Pilot④
  [TX radio]    [TX radio]    [TX radio]    [TX radio]
       ↕ ELRS RC link (downlink + uplink, FHSS 2.4 GHz LoRa)
  [EP1/EP2 TCXO, stock ELRS firmware, on the drone]

  ── the sniffer detects the telemetry uplink RSSI at gate crossing ──

  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐
  │EP1 sniffer │ │EP1 sniffer │ │EP1 sniffer │ │EP1 sniffer │
  │ #1 (Pilot1)│ │ #2 (Pilot2)│ │ #3 (Pilot3)│ │ #4 (Pilot4)│
  │ ESP8285    │ │ ESP8285    │ │ ESP8285    │ │ ESP8285    │
  │ +SX1280    │ │ +SX1280    │ │ +SX1280    │ │ +SX1280    │
  └─────┬──────┘ └─────┬──────┘ └─────┬──────┘ └─────┬──────┘
        └───────────────┴───────────────┴───────────────┘
                          ESP-NOW (2.4 GHz)
                                ↓
                  ┌─────────────────────────┐
                  │  Gate Node (TTGO T8)    │
                  │  ESP32-WROVER-E         │
                  │  EMA + lap detection    │
                  └────────────┬────────────┘
                               │ UART 115200 bps
                  ┌────────────▼────────────┐
                  │  Web Node (XIAO S3)     │
                  │  WiFi AP: ESP-NOW-LT    │
                  │  IP: 20.0.0.1           │
                  └────────────┬────────────┘
                               │ WiFi
                  ┌────────────▼────────────┐
                  │  Phone / PC browser     │
                  │  http://20.0.0.1        │
                  └─────────────────────────┘
```

### Wiring — Gate Node ↔ Web Node (UART)

| ESP32-WROVER-E (Gate) | Direction | XIAO ESP32-S3 (Web) |
|-----------------------|-----------|---------------------|
| GPIO26 (TX1)          | →        | GPIO3 / D2 (RX1)    |
| GPIO25 (RX1)          | ←        | GPIO2 / D1 (TX1)    |
| GND                   | —        | GND                 |

### Wiring — Gate Node ↔ SD card (SPI, LilyGo TTGO T8 V1.8)

| Pin  | GPIO |
|------|------|
| CS   | 13   |
| MOSI | 15   |
| MISO | 2    |
| SCK  | 14   |

Use a FAT32-formatted microSD card. Race CSVs are auto-saved as
`/race_001.csv`, `/race_002.csv`, … and the pilot backup is written to `/pilots.csv`.

---

## Build & Flash

Requires PlatformIO Core or the PlatformIO IDE (VS Code extension).

```bash
# Gate EP1 sniffer (EP1/EP2 TCXO = ESP8285) — flash one per pilot slot (4 total)
pio run -e gate_ep1 -t upload

# Gate Node (ESP32-WROVER-E / LilyGo TTGO T8 V1.8)
pio run -e gate_node -t upload

# Web Node (XIAO ESP32-S3)
pio run -e web_node -t upload
pio run -e web_node -t uploadfs   # required after changing JS/HTML
```

### Flashing an EP1/EP2 (USB-serial adapter, **3.3 V only**)

```
 USB-serial adapter            EP1/EP2 TCXO
 (CP2102 / CH340, etc.)        external pads
 ┌─────────────┐               ┌───────────┐
 │  3V3 ────────────────────→  VCC        │
 │  GND ────────────────────── GND        │
 │  TX  ────────────────────→  RX (GPIO3) │ ← doubles as bootloader entry
 │  RX  ←──────────────────── TX (GPIO1) │
 └─────────────┘               └───────────┘
```

> **Entering flash mode**
> 1. Tie the RX pad (GPIO3) to GND, then apply VCC
> 2. If the LED stays lit, it is in bootloader mode (waiting for UART)
> 3. Run `pio run -e gate_ep1 -t upload`
> 4. Remove the GND tie and reset when done

---

## TRSS (Drone Telemetry) Handling

The calibration graph and lap detection look at **only the telemetry uplink (TRSS)**
that the drone sends back to its transmitter.

- In ELRS 3.6.3 the low 2 bits of the OTA packet decide its type, and **the drone's
  telemetry is `0b11` (PACKET_TYPE_TLM)**. The TX sends only RC=`0b00` / MSP=`0b01` /
  SYNC=`0b10`. Measuring type `0b11` alone tracks the drone independent of the
  stationary TX's level or movement.
- **Capture density depends on the ratio.** The gate dwells 4 slots per channel and
  retunes (briefly stops receiving) at each hop. At **1:4 / 1:8+** a dwell holds one
  telemetry slot; when it collides with the retune it is missed, so capture is sparse
  (a few per second) and the graph is choppy. At **1:2** a dwell holds ~2 telemetry
  slots, so even if one is missed the other is caught — capture is dense (100+/s) and
  the trace is smooth.
- The report uses an **envelope follower**: it rises instantly to each telemetry sample
  and decays slowly between (`ENV_DECAY_DB` per 50 ms), so the trace is a curve rather
  than spiking to the floor — but real smoothness comes from the dense capture at 1:2.
- The SYNC packet is read for the ratio + UID validation, and its **fhssIndex+nonce
  re-anchor the hop phase to the TX** (`SYNC_PHASE_ALIGN`, fixes index drift; set to `0`
  to disable).
- **Recommended ratio: 1:2.** It captures densely, holds a steady value at a fixed
  distance, and forms a smooth peak on a pass. High ratios (1:8…1:128) capture sparsely
  and look choppy, so they are not recommended.

---

## Lap Modes

Switchable from the Global Settings tab.

### HS Mode (default)

```
Race start
  └─ 1st gate crossing → recorded as "HS (Hole Shot)"
       └─ 2nd onward    → recorded as "Lap 1", "Lap 2", ...
```

- Cumulative time accumulates **from the HS crossing** (travel time from start to HS is excluded)
- HS itself is excluded from best-lap evaluation

### Immediate (Timing) Mode

```
Race start
  └─ 1st gate crossing → recorded as "Lap 1" (time from start)
       └─ 2nd onward    → recorded as "Lap 2", "Lap 3", ...
```

- Cumulative time accumulates **from race start**

---

## Pilot Model

- **Roster**: stores up to 20 pilots in NVS (name, reading, ELRS bind UID, RSSI thresholds)
- **Active slots**: pick up to 4 from the roster and assign them to gate channels
- **Identity**: each pilot is uniquely identified by their **6-byte ELRS bind UID**
  (shown in `AA:BB:CC:DD:EE:FF` form). The gate sniffer needs the UID to compute the
  FHSS hop sequence and to validate SYNC packets
- **Scan**: an unregistered drone appears in the scan list automatically; already-registered
  entries are hidden, "online" is shown while RSSI is flowing, and power-on order is used
  for automatic channel assignment

---

## Web UI

**Connect:** WiFi SSID `ESP-NOW-LT` (PASS: `esp-now-lt`) → open `http://20.0.0.1` in a browser.

Notifications appear in the **header status bar**, not as bottom-of-screen popups.

### Race Tab

- 3-second countdown + race timer (start / stop / clear), double-start prevention
- **Pause/Resume**: Stop = pause; pressing Start again resumes with no countdown (laps retained); paused time is excluded from the timer and lap times
- **State-aware Clear**: disabled during a race, enabled after a stop; Clear saves results and resets for the next race
- 4-column pilot grid (CROSSING badge, RSSI bar, best-lap + delta) and a per-pilot lap table

### Config Tab

- **Drone scan**: only unregistered drones are listed; "Online" badge while RSSI flows
  - 🤖 Auto channel assignment (Ch1–4 by power-on order), ✖ clear all channels
  - Manual scan refresh; auto-refresh runs silently every 5 s
- **Pilots**: up to 20 (name, reading, ELRS bind UID, channel assignment)
- **Global Settings**: announce mode, speech rate, lap mode (HS / Immediate), cooldown (seconds), saved to `localStorage`
- **SD card**: shown only when a card is detected

### Calib Tab

- Per-pilot Canvas RSSI waveform graph (rAF loop, dynamic Y scale) — **the trace follows the drone's TRSS**
- Enter/Exit threshold sliders (auto-saved after an 800 ms debounce, pushed to the Gate Node immediately)

### SD Tab

- Lists, downloads (streamed over WebSocket, UTF-8 BOM) and deletes race CSV files

---

## RSSI Peak Detection

```
telemetry RSSI (type 0b11) → EMA filter (α = 0.25, every loop) → Enter/Exit state machine → gate crossing
```

### State Machine (RotorHazard-style)

```
CLEAR (idle)
 └─(ema > EnterAt)→ CROSSING
      ├─(ema > peak) → update peak, record peak time
      └─(ema < ExitAt and cooldown elapsed)→ sendLap(peakTime) → CLEAR
```

### Tunable Parameters

| Parameter        | Default  | Description                                          |
|------------------|----------|------------------------------------------------------|
| EnterAt          | -55 dBm  | RSSI threshold to start a crossing                   |
| ExitAt           | -62 dBm  | RSSI threshold to end a crossing                     |
| EMA_ALPHA        | 0.25     | Smoothing coefficient (lower = smoother / slower)    |
| COOLDOWN_MS      | 3000 ms  | Minimum lap interval (behavior varies by lap mode)   |
| RSSI_INTERVAL_MS | 50 ms    | Sniffer → gate report interval (20 Hz)               |

Defaults are set well above the TX background (typically -65 to -75 dBm) so a freshly
registered pilot does not immediately enter a permanent crossing state. Enter/Exit are
**per-pilot and adjustable at runtime** via the Calib tab sliders (applied to the Gate
Node immediately).

---

## UART Protocol

### Gate → Web

```json
{"type":"lap",            "pilot":0,"uid":"AA:BB:CC:DD:EE:FF","rssi":-72,"ts":123456,"lapMs":42100}
{"type":"rssi",           "pilot":0,"rssi":-85,"raw":-87,"crossing":false,"signal":true,"ts":123460}
{"type":"ready",          "pilots":4}
{"type":"race_start_ack", "ts":123000}
{"type":"sd_status",      "present":true}
{"type":"scan",           "uid":"AA:BB:CC:DD:EE:FF","rssi":-75,"ts":123470}
{"type":"sd_file_list",   "files":[{"name":"race_001.csv","size":1024}]}
{"type":"sd_file_line",   "path":"/race_001.csv","line":"0,Hayate,..."}
{"type":"sd_file_done",   "path":"/race_001.csv"}
{"type":"sd_delete_result","path":"/race_001.csv","ok":true}
```

### Web → Gate

```json
{"type":"cmd","action":"race_start"}
{"type":"cmd","action":"set_pilot",    "pilot":0,"uid":"AA:BB:CC:DD:EE:FF","name":"Hayate"}
{"type":"cmd","action":"set_threshold","pilot":0,"enter":-55,"exit":-62}
{"type":"cmd","action":"set_cooldown", "ms":3000}
{"type":"cmd","action":"scan_refresh"}
{"type":"cmd","action":"sd_poll",      "enable":true}
{"type":"cmd","action":"sd_list_files"}
{"type":"cmd","action":"sd_read_file", "path":"/race_001.csv"}
{"type":"cmd","action":"sd_delete_file","path":"/race_001.csv"}
```

---

## CSV File Format

### Race CSV (`/race_NNN.csv`)

```
Slot,Name,UID,Lap,LapTime_ms,RSSI_dBm,Timestamp_ms
0,Hayate,AA:BB:CC:DD:EE:FF,1,42135,-75,123456
```

- Starts with a UTF-8 BOM → opens in Excel without garbled text
- `LapTime_ms` is an integer in milliseconds (e.g. `12078` = 12.078 s). Keep the column
  format as "General"; to display `m:ss.000`, use a helper column with
  `=IF(E2=0,"HS",TEXT(E2/86400000,"m:ss.000"))`

### Pilot Backup (`/pilots.csv`)

```
name,yomi,uid,enter,exit,slot
Hayate,hayate,AA:BB:CC:DD:EE:FF,-55,-62,0
```

- Starts with a UTF-8 BOM; the `slot` column is the channel assignment (0–3, `-1` if unassigned)

---

## REST API (Web Node)

| Endpoint                    | Method                     | Description                                       |
|-----------------------------|----------------------------|---------------------------------------------------|
| `/api/pilots`               | GET/POST                   | Get roster / add or update (saved to NVS)         |
| `/api/pilots/delete`        | POST `{id}`                | Delete a pilot from the roster                    |
| `/api/active`               | GET/POST                   | Get / set active slots                            |
| `/api/calib`                | POST `{id,enter,exit}`     | Update RSSI thresholds (NVS save + push to Gate)  |
| `/api/race/start`           | POST                       | Start race (send race_start to Gate; reset laps)  |
| `/api/race/stop`            | POST                       | Pause race (record stop time)                     |
| `/api/race/resume`          | POST                       | Resume from pause (keep laps)                     |
| `/api/race/save`            | POST                       | Save results to SD and clear                       |
| `/api/laps`                 | GET                        | Get lap history                                   |
| `/api/scan`                 | GET                        | List scanned unregistered drones                  |
| `/api/scan/refresh`         | POST                       | Reset the scan timer (forwarded to Gate)          |
| `/api/scan/clear`           | POST                       | Clear the scan list                               |
| `/api/settings`             | POST `{lapMode,cooldownMs}`| Lap mode / cooldown settings                      |
| `/api/status`               | GET                        | System status (raceRunning, lapCount, etc.)       |
| `/api/sd/status`            | GET                        | SD card presence                                  |
| `/api/sd/poll`              | POST `{enable}`            | SD hot-plug polling on/off                        |
| `/api/sd/files/list`        | POST                       | Get SD file list (over WS)                        |
| `/api/sd/files/download`    | POST `{path}`              | Download a file (over WS)                          |
| `/api/sd/files/delete`      | POST `{path}`              | Delete a file (over WS)                            |

---

## Announcements (TTS)

Configurable in the Global Settings tab. Default is "name + lap + lap time".

| Mode                            | Example readout                                   |
|---------------------------------|---------------------------------------------------|
| Name + lap + lap time (default) | "Hayate, hole shot, 42.1" / "Hayate, lap 1, 40.5" |
| Name + lap time                 | "Hayate, 42.1"                                     |
| Beep only                       | Sound effect only                                 |
| Off                             | None                                              |

> The built-in TTS phrases are Japanese; the readouts above are translated for reference.

---

## Source Layout

```
src/
├─ gate_ep1/    EP1/EP2 TCXO sniffer firmware (ESP8285) — FHSS following + TRSS isolation
├─ gate_node/   Gate Node (TTGO T8) — ESP-NOW receive + EMA lap detection
└─ web_node/    Web Node (XIAO S3) — WiFi AP + race-management UI
data/           Web UI (LittleFS)
boards/         custom board definitions
```

See [HANDOFF.md](HANDOFF.md) and [ARCHITECTURE.md](ARCHITECTURE.md) for details.

---

## Related Repositories

- Base: [yanazoo/ESP-NOW-Lap-Timer](https://github.com/yanazoo/ESP-NOW-Lap-Timer)
- ExpressLRS (FHSS / SX1280 driver reference): [ExpressLRS/ExpressLRS](https://github.com/ExpressLRS/ExpressLRS)
- PhobosLT (4ch SPI reference): [yanazoo/PhobosLT_4ch](https://github.com/yanazoo/PhobosLT_4ch)
- RotorHazard: [RotorHazard/RotorHazard](https://github.com/RotorHazard/RotorHazard)
```

