// config.h - Gate EP1 sniffer (ESP8285 + SX1280)
// Compile-time constants and shared packet definitions.
#pragma once
#include <stdint.h>

// ---- SX1280 SPI pins (ESP8285) ----
// Confirmed from ExpressLRS generic 2.4GHz ESP8285 RX layout.
// HappyModel EP1/EP2 TCXO use this same reference pinout (identical PCB,
// antenna differs only). Verify against your unit before relying on it.
#define SX_PIN_NSS    15   // chip select
#define SX_PIN_SCK    14   // HSPI clock
#define SX_PIN_MOSI   13   // HSPI MOSI
#define SX_PIN_MISO   12   // HSPI MISO
#define SX_PIN_BUSY    5   // SX1280 BUSY
#define SX_PIN_DIO1    4   // SX1280 DIO1 (IRQ)
#define SX_PIN_RST     2   // SX1280 reset
#define PIN_SERIAL_RX  3   // also the bootloader-entry pad (hold LOW at reset)
#define PIN_SERIAL_TX  1
#define PIN_LED       16

// ---- Flashing note ----
// No separate GPIO0 test point is needed. Holding the RX pad (GPIO3) LOW at
// power-on drops the ESP8285 into the UART bootloader. Flash custom firmware
// over the exposed 5V / GND / RX / TX pads with esptool (UART method).
// A solid LED with the TX off = the unit is sitting in bootloader mode.

// ---- ELRS / FHSS ----
// Packet-rate dependent timing. Must match the TX (see sx1280_sniffer.cpp for
// the matching SF/CR/preamble). Currently configured for 500Hz.
//   500Hz: ELRS_SLOT_US 2000   250Hz: 4000   150Hz: 6666
#define FHSS_CHANNEL_COUNT   80                          // 2.4GHz ISM unique channels
#define FHSS_SEQUENCE_LEN    (FHSS_CHANNEL_COUNT * 3)   // 240: 3 complete blocks per ELRS
#define ELRS_SLOT_US         2000   // 500Hz: 2 ms per packet
#define SX_SWITCH_US          500   // approx SX1280 standby+freq+rx round trip
// ELRS TX stays on each FHSS channel for this many consecutive packets before hopping.
// Must match FHSShopInterval from ELRS expresslrs_mod_settings_s (4 for 500Hz).
#define FHSS_HOP_INTERVAL    4
// Time the TX spends on one channel = hopInterval slots. FOLLOW dwells exactly
// this long (slot-phase locked to the TX) so it catches all interleaved slots.
#define FHSS_HOP_PERIOD_US   (ELRS_SLOT_US * FHSS_HOP_INTERVAL)   // 8000us @ 500Hz
// Approx LoRa airtime of one 8-byte SF5/BW800 packet. Used to back-date a SYNC
// packet's reception to its slot boundary when re-anchoring the hop grid phase.
#define PKT_AIRTIME_US       1100

// ---- Lock-on tuning ----
// SCAN dwell must exceed one packet interval (500Hz = 2000us) so a parked
// channel reliably catches a packet while the TX is transmitting on it.
#define SCAN_DWELL_US        2600   // RX dwell per channel during SCAN phase
#define MISS_STREAK_RESYNC    8     // consecutive empty channel dwells -> back to SCAN

// ---- RSSI reporting ----
#define RSSI_REPORT_MS       50     // 20 Hz, matches existing RSSI_INTERVAL_MS

// ---- Hybrid TLM isolation ----
// A connected ELRS TX sends SYNC packets only intermittently (every few seconds
// when linked, more often while connecting/idle — which is exactly the state a
// pilot calibrates in).  Each SYNC anchors the nonce phase; we then keep
// classifying TLM slots by free-running the nonce grid for NONCE_FRESH_MS.
// The EP1/EP2 TCXO drift is far below one 2 ms slot over this window, so the
// slot phase stays valid between SYNCs.  While the nonce is "fresh" we report
// ONLY the drone's telemetry-slot RSSI (the stationary TX downlink is excluded);
// once it goes stale (sustained armed flight with no SYNC) we fall back to the
// proven max-RSSI-of-all-packets behaviour so lap detection never regresses.
#define NONCE_FRESH_MS    10000     // free-run the nonce phase this long after a SYNC

// ---- TX-background notch (SYNC-free RX isolation) ----
// On a linked TX that emits no catchable SYNC (nonce unavailable — the case seen
// on real hardware), the drone (RX telemetry) is still separable from the
// stationary handset (TX downlink) by LEVEL: the TX is a constant, dominant RSSI
// cluster, so its level is the statistical mode of all packets over a short
// window.  We suppress that cluster and report only the packets that deviate from
// it (the drone).  This removes the ~-67 dBm TX baseline from the signal so the
// drone-pass spike stands alone with maximum contrast, and lets a drone weaker
// than the TX (far / fast pass) still be read as the "below-TX" cluster instead
// of being masked by the constant TX floor.
#define TXBG_WINDOW_MS     1000     // recompute the TX-background (mode) this often
#define TXBG_GUARD_DB         5     // a packet must differ from txBg by > this to be RX
#define TXBG_MIN_PKTS        40     // min packets in a window to trust the txBg estimate
#define TXBG_RX_MIN_PKTS      2     // min RX packets per report interval (rejects 1-pkt noise)

// ---- Telemetry (drone) isolation by OTA type (ELRS 3.6.3) ----
// In ELRS 3.6.3 the drone's telemetry uplink is OTA type 0b11 (PACKET_TYPE_TLM);
// the TX only sends RC=0b00 / MSP=0b01 / SYNC=0b10.  So type 0b11 identifies the
// drone regardless of TX level or movement.  We hold the latest telemetry RSSI
// and only fall to the floor after no telemetry for an adaptive "silence" window.
//
// The window scales with the telemetry ratio (denom) read from the SYNC packet:
//   silence = (denom * ELRS_SLOT_US / 1000) * TLM_SILENCE_MARGIN, clamped to
//             [TLM_SILENCE_MIN_MS, TLM_SILENCE_MAX_MS]
// telemetry interval per ratio @500Hz: 1:8=16ms 1:16=32 1:32=64 1:64=128 1:128=256.
// So low ratios -> short, responsive timeout; high/sparse ratios -> long enough
// that the trace does not flicker to the floor between samples.  This is what
// lets ratios up to 1:128 (incl. the 500 Hz "Std" default) be measured at all.
// NOTE: a single telemetry packet refreshes the hold (no per-interval minimum);
// with LoRa CRC off a bit-flipped RC type byte can therefore inject a brief false
// sample — acceptable here, removable later by adding OTA CRC validation.
#define TLM_SILENCE_MARGIN      3   // hold timeout = telemetry interval x this
#define TLM_SILENCE_MIN_MS    120   // floor for low ratios (responsive drop-out)
#define TLM_SILENCE_MAX_MS    600   // cap for 1:128 (~256 ms interval)
#define TLM_SILENCE_DEFAULT_MS 300  // used until the first SYNC reveals the ratio

// ---- Telemetry-only RSSI (lap timing) ----
// The lap-timing signal must come from the DRONE, not the handset.  In an ELRS
// link the drone (RX) only transmits during telemetry slots (OTA type 0b11);
// everything else (RC_DATA/MSP/SYNC) is downlink from the stationary TX and is
// useless for position.  We therefore report RSSI only from TLM packets.
//   - The TX downlink is still used to keep FHSS lock (it is always present).
//   - When no telemetry has been heard for TLM_SILENCE_MS, we emit RSSI_FLOOR_DBM
//     so the trace falls back to baseline instead of holding the last peak
//     (required for the exit threshold to fire after a gate pass).
// NOTE: telemetry cadence = tlmRatio / packetRate.  For crisp lap timing set a
// high telemetry ratio on the TX (1:2…1:8 @ 500Hz = 4…16 ms between samples).
// TLM_SILENCE_MS (300) is > the 1:128 interval (256 ms) so it never false-floors.
#define RSSI_FLOOR_DBM      (-120)  // reported when the drone's telemetry is absent
#define TLM_SILENCE_MS       300    // no TLM for this long -> report floor

// ---- UID gate (multi-node: ignore other pilots' drones) ----
// Each sniffer follows ONE pilot's UID. With several sniffers but only some
// drones powered on, a sniffer whose drone is OFF cannot lock its own link and
// instead briefly catches OTHER drones' packets, polluting its slot with noise.
// The drone's TX sends SYNC packets carrying the bound UID; we report RSSI only
// while a SYNC matching OUR UID has been seen within this window. A sniffer whose
// TX is off never matches -> stays at the floor. The window is long (the TX emits
// SYNC continuously regardless of drone position) so the correct sniffer is never
// suppressed; telemetry silence still floors the trace promptly when the drone
// leaves, so a long gate window adds no lag to lap detection.
#define UID_GATE_MS        10000    // report only if own-UID SYNC seen this recently

// ---- Near-cluster bridge (steady reading) ----
// The lap signal is the per-interval peak of the drone's near cluster (packets
// above the TX background). That cluster is dense every interval, so at a fixed
// distance it reads as a near-constant level. This short hold bridges the
// occasional empty interval so the trace stays steady instead of dropping to the
// floor for a frame; it still floors quickly once the drone leaves.
#define NEAR_HOLD_MS         300    // bridge near-cluster gaps this long before flooring

// ---- ELRS OTA sync-channel auto-discovery ----
// Channel 41 is always position-0 of every FHSS block in ELRS 3.x.
// Frequency = 2400.4 MHz + 41 × 1 MHz = 2441.4 MHz.
#define SYNC_CHANNEL_IDX        41
#define SYNC_FREQ_HZ            2441400000UL

// ELRS 3.x 8-byte OTA packet layout (verified from OTA.h + rx_main.cpp):
//   byte[0]: packetType[1:0] | crcHigh[7:2]
//            0b00=RC_DATA  0b01=MSP  0b10=SYNC  0b11=TLM
// SYNC packet (byte[0] & 0x03 == 0x02):
//   byte[1]  fhssIndex    – TX's current hop counter in FHSS sequence
//   byte[2]  nonce        – packet timing counter
//   byte[3]  switchEncMode[0] | tlmRatio[3:1] | rateIndex[7:4]
//   byte[4]  UID[3]       – full byte (known after capture)
//   byte[5]  UID[4]       – full byte (known after capture)
//   byte[6]  UID5_field   – (UID[5] & 0xC0) | (modelId & 0x3F)
//            Only bits[7:6] carry UID[5]; bits[5:0] = model match ID
//   byte[7]  crcLow
#define OTA_TYPE_MASK           0x03
#define OTA_TYPE_SYNC           0x02
#define OTA_TYPE_TLM            0x03   // telemetry uplink (RX->TX) = drone's signal
#define OTA_SYNC_FHSS_BYTE      1
#define OTA_SYNC_NONCE_BYTE     2      // packet counter; (nonce % hopInterval) = slot in dwell
#define OTA_SYNC_UID3_BYTE      4
#define OTA_SYNC_UID4_BYTE      5
#define OTA_SYNC_UID5_BYTE      6
#define OTA_SYNC_UID5_HIBITS    0xC0   // only bits[7:6] of byte[6] are UID[5]
// byte[3]: rateIndex[7:4] | tlmRatioIdx[3:1] | switchEncMode[0]
// tlmRatioIdx: 0=off 1=1:128 2=1:64 3=1:32 4=1:16 5=1:8 6=1:4 7=1:2
#define OTA_SYNC_TLMRATIO_BYTE  3
#define OTA_SYNC_TLMRATIO_SHIFT 1
#define OTA_SYNC_TLMRATIO_MASK  0x07

// Auto-discovery candidate space:
//   UID[0:1] = 0x00 always (ELRS convention)
//   UID[3:4] = known from SYNC packet
//   UID[5] bits[7:6] = known; bits[5:0] = unknown (64 values)
//   UID[2]  = fully unknown (256 values)
//   Total: 256 × 64 = 16384 candidates
// Candidate index: uid[2]*64 + (uid[5]&0x3F)
#define AUTO_CANDIDATE_COUNT    16384
// Scan length: 320 hops × 8 ms = 2.56 s → expected ~4 lucky hits.
// Each "got" hit narrows: 16384→205→3→1.
#define AUTO_SCAN_HOPS          320
#define AUTO_MAX_GOT_OBS        8

// ---- ESP-NOW channel (must match Gate Node ESPNOW_CHANNEL) ----
#define ESPNOW_CHANNEL       1

// ---- Identity ----
typedef struct { uint8_t uid[6]; bool valid; } SnifferIdentity_t;

// ---- ESP-NOW packet: EP1 -> Gate ESP32 (RSSI report, 12 bytes) ----
// Keep in sync with the matching struct in src/gate_node/promiscuous.*
typedef struct __attribute__((packed)) {
    uint8_t  pilot_uid[6];   // which pilot's EP1 this RSSI belongs to
    int8_t   rssi;           // measured RSSI (dBm)
    uint8_t  lq;             // link quality 0-100
    uint32_t ts;             // sniffer millis() timestamp
} GateEP1Packet_t;           // 12 bytes

// ---- ESP-NOW packet: EP1 -> Gate ESP32 (presence beacon, 8 bytes) ----
#define EP1_BEACON_MAGIC  0xA5
typedef struct __attribute__((packed)) {
    uint8_t magic;   // EP1_BEACON_MAGIC = 0xA5
    uint8_t state;   // 0=PROVISION 1=SCAN 2=FOLLOW
    uint8_t uid[6];  // current UID (all-zero if not provisioned)
} GateEP1BeaconPacket_t;     // 8 bytes

// ---- ESP-NOW packet: Gate ESP32 -> EP1 (provisioning, 7 bytes) ----
#define GATE_PROV_MAGIC   0xB1
typedef struct __attribute__((packed)) {
    uint8_t magic;   // GATE_PROV_MAGIC = 0xB1
    uint8_t uid[6];  // ELRS bind UID to follow (all-zero = clear/stop)
} GateProvisionPacket_t;     // 7 bytes

// NOTE: EP1 no longer needs the Gate ESP32 MAC address. Beacons and RSSI
// reports are sent to broadcast (FF:FF:FF:FF:FF:FF); the Gate Node learns
// each EP1's MAC from the receive callback's src_addr.
