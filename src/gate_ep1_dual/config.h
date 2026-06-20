// config.h - Gate EP1 DUAL sniffer (ESP32-PICO-D4 + 2x SX1280)
//
// Target: HappyModel EP1 Dual TCXO 2.4GHz (true-diversity RX).
//   MCU   : ESP32-PICO-D4  (dual-core, native ESP-NOW)
//   Radios: 2x SX1280 sharing one SPI bus, addressed by separate NSS lines.
//
// Pins below are the ExpressLRS "Generic 2400 True Diversity PA" layout that the
// stock "HappyModel EP Dual 2.4GHz RX" target uses.  VERIFY against your unit
// before relying on them — RF-switch / PA-enable / LED pins are board specific.
//
// ── Why two radios ──────────────────────────────────────────────────────────
// In ELRS the TX downlink (RC/MSP/SYNC) and the drone's telemetry uplink (TLM)
// both happen on the SAME FHSS channel during one dwell.  A single radio that
// parks on that channel for the whole dwell catches both — but it goes deaf for
// ~0.5-1 ms at every hop boundary (SetStandby+SetFrequency+SetRx).  At a sparse
// telemetry ratio (1:128 = one TLM packet every 256 ms @500Hz) that retune blind
// spot, plus any inter-SYNC grid drift, is enough to drop the rare TLM sample and
// make lap detection flaky.
//
// The dual radio removes the blind spot:
//   Radio A (SYNC anchor)  : holds the current channel for the first part of the
//                            dwell to capture SYNC (phase/UID/ratio), then moves
//                            ahead to the NEXT channel for the last slot so it is
//                            already listening when the boundary is crossed.
//                            Radio A also owns SCAN / re-sync.
//   Radio B (TLM capture)  : stays on the current channel for the WHOLE dwell and
//                            lingers a little past the hop boundary (TAIL_LINGER)
//                            so it catches a trailing telemetry slot that Radio A
//                            would miss while retuning.  RSSI is read here.
// Together they cover every slot of every dwell with no global deaf window, so the
// 1-per-256 ms telemetry packet at 1:128 is captured reliably.
#pragma once
#include <stdint.h>

// ---- Shared SPI bus (both SX1280 radios) ----
#define SX_PIN_SCK    25   // shared SPI clock
#define SX_PIN_MISO   33   // shared SPI MISO
#define SX_PIN_MOSI   32   // shared SPI MOSI

// ---- Radio A (sync/phase anchor) ----
#define SXA_PIN_NSS   27
#define SXA_PIN_BUSY  36   // input-only GPIO (ok: BUSY is an input)
#define SXA_PIN_DIO1  37   // input-only GPIO
#define SXA_PIN_RST   26

// ---- Radio B (telemetry capture) ----
#define SXB_PIN_NSS   13
#define SXB_PIN_BUSY  39   // input-only GPIO
#define SXB_PIN_DIO1  34   // input-only GPIO
#define SXB_PIN_RST   21

// ---- Status LED ----
// EP1 Dual uses a WS2812 addressable RGB LED on GPIO22 (GRB order), per the
// ELRS "Generic 2400 True Diversity PA" target — NOT a plain GPIO LED. It is
// driven with neopixelWrite() (ESP32 RMT), so digitalWrite() does nothing.
#define PIN_LED       22

// ---- Optional TCXO control via SX1280 DIO3 ----
// EP1 *TCXO* boards usually power the TCXO directly (the proven single-radio
// firmware never touches DIO3 and works), so the default is OFF.  If a unit's
// radios stay dead (FW ver reads 0x0000) set this to the TCXO voltage code from
// the SX1280 datasheet (e.g. 0x02 = 1.8 V) to drive DIO3 as the TCXO supply.
//   0x00 = disabled (default)  0x02 = 1.8V  0x07 = 3.3V
#define SX_DIO3_TCXO_VOLTAGE   0x00

// ---- ELRS / FHSS (500Hz ELRS 2.4GHz) ----
// Packet-rate dependent timing. Must match the TX (see sx1280_radio.cpp for the
// matching SF/CR/preamble).
//   500Hz: ELRS_SLOT_US 2000   250Hz: 4000   150Hz: 6666
#define FHSS_CHANNEL_COUNT   80                          // 2.4GHz ISM unique channels
#define FHSS_SEQUENCE_LEN    (FHSS_CHANNEL_COUNT * 3)    // 240: 3 complete blocks per ELRS
#define ELRS_SLOT_US         2000   // 500Hz: 2 ms per packet
// ELRS TX stays on each FHSS channel for this many consecutive packets before hopping.
// Must match FHSShopInterval from ELRS expresslrs_mod_settings_s (4 for 500Hz).
#define FHSS_HOP_INTERVAL    4
// Time the TX spends on one channel = hopInterval slots.
#define FHSS_HOP_PERIOD_US   (ELRS_SLOT_US * FHSS_HOP_INTERVAL)   // 8000us @ 500Hz
// Approx LoRa airtime of one 8-byte SF5/BW800 packet. Used to back-date a SYNC
// packet's reception to its slot boundary when re-anchoring the hop grid phase.
#define PKT_AIRTIME_US       1100

// ---- Dual-radio coverage timing ----
// When this much dwell time remains, move the SYNC radio (A) ahead to the next
// channel so it is already listening when the boundary is crossed (one slot).
#define LEAD_AHEAD_US        ELRS_SLOT_US
// After the boundary, the TLM radio (B) keeps listening on the OLD channel this
// long to catch a telemetry slot that straddles the hop, then it rejoins A.
#define TAIL_LINGER_US       1500

// ---- SYNC-nonce slot-phase alignment ----
// Each own-UID SYNC carries fhssIndex (sequence position) + nonce (slot counter).
// We re-anchor the hop grid to the TX on every own SYNC: fix sequence-index drift
// and re-phase the dwell boundary.  Set to 0 to disable instantly.
#define SYNC_PHASE_ALIGN     1

// ---- Lock-on tuning ----
// SCAN dwell must exceed one packet interval (500Hz = 2000us) so a parked channel
// reliably catches a packet while the TX is transmitting on it.
#define SCAN_DWELL_US        2600   // RX dwell per channel during SCAN phase
#define MISS_STREAK_RESYNC    8     // consecutive empty dwells -> back to SCAN

// ---- RSSI reporting ----
#define RSSI_REPORT_MS       50     // 20 Hz, matches gate_node RSSI_INTERVAL_MS

// ---- Telemetry (drone) isolation by OTA type (ELRS 3.6.3) ----
// The drone's telemetry uplink is OTA type 0b11 (PACKET_TYPE_TLM); the TX only
// sends RC=0b00 / MSP=0b01 / SYNC=0b10.  type 0b11 alone identifies the drone,
// so RSSI for lap timing is read only from those packets.  The trace is governed
// by the envelope follower (below) + the UID gate; the telemetry ratio read from
// SYNC is logged for diagnostics (telemetry interval @500Hz: 1:8=16ms ... 1:128=256ms).

// ---- Telemetry RSSI squelch (noise gate) ----
// LoRa CRC is OFF (ELRS validates its own OTA CRC in the payload, which a passive
// sniffer can't fully check because the TLM CRC is XORed with the per-slot nonce).
// So an occasional TX (handset) packet whose type byte bit-flips to 0b11, or a
// noise-triggered RX, is mis-read as telemetry.  With the drone powered OFF these
// false samples are weak (near the noise floor) but make the idle RSSI trace
// wiggle.  We ignore any "telemetry" sample weaker than this gate: real gate-pass
// telemetry is far stronger (-40..-75 dBm), so lap detection is unaffected, while
// the idle trace cleanly rests at the floor.  Lower the magnitude (e.g. -85) if a
// little wiggle remains; raise it (e.g. -100) to also show very distant drones.
#define TLM_RSSI_GATE_DBM   (-90)

// ---- OTA CRC validation (reject CRC-invalid false telemetry) ----
// Validate each TLM packet against ELRS's own 14-bit OTA CRC so bit-flipped TX
// packets / noise mis-read as telemetry are dropped (the squelch alone can't
// catch false samples stronger than TLM_RSSI_GATE_DBM). Self-calibrating and
// self-testing from SYNC packets — if it can't lock, it safely does nothing.
// Set to 0 to disable entirely.
#define VALIDATE_OTA_CRC     1

// ---- Reported RSSI floor ----
#define RSSI_FLOOR_DBM      (-120)  // reported when the drone's telemetry is absent

// ---- Envelope follower (smooth trace from sparse telemetry) ----
// Rise instantly to each telemetry sample, decay ENV_DECAY_DB per 50 ms report
// interval between samples, so the trace is a smooth curve instead of a sawtooth.
// ~2 dB/interval reaches the floor in ~1.9 s.
#define ENV_DECAY_DB          2

// ---- UID gate (multi-node: ignore other pilots' drones) ----
// Report RSSI only while a SYNC matching OUR UID has been seen this recently, so
// a sniffer whose drone is off stays at the floor instead of catching another
// pilot's packets.
#define UID_GATE_MS        10000

// ---- ELRS OTA packet layout ----
// ELRS 3.x 8-byte OTA packet layout (verified from OTA.h + rx_main.cpp):
//   byte[0]: packetType[1:0] | crcHigh[7:2]
//            0b00=RC_DATA  0b01=MSP  0b10=SYNC  0b11=TLM
// SYNC packet (byte[0] & 0x03 == 0x02):
//   byte[1]  fhssIndex    – TX's current hop counter in FHSS sequence
//   byte[2]  nonce        – packet timing counter
//   byte[3]  switchEncMode[0] | tlmRatio[3:1] | rateIndex[7:4]
//   byte[4]  UID[3]       – full byte
//   byte[5]  UID[4]       – full byte
//   byte[6]  UID5_field   – (UID[5] & 0xC0) | (modelId & 0x3F)
//   byte[7]  crcLow
#define OTA_TYPE_MASK           0x03
#define OTA_TYPE_SYNC           0x02
#define OTA_TYPE_TLM            0x03   // telemetry uplink (RX->TX) = drone's signal
#define OTA_SYNC_FHSS_BYTE      1
#define OTA_SYNC_NONCE_BYTE     2      // (nonce % hopInterval) = slot in dwell
#define OTA_SYNC_UID3_BYTE      4
#define OTA_SYNC_UID4_BYTE      5
#define OTA_SYNC_UID5_BYTE      6
#define OTA_SYNC_UID5_HIBITS    0xC0   // only bits[7:6] of byte[6] are UID[5]
// byte[3]: rateIndex[7:4] | tlmRatioIdx[3:1] | switchEncMode[0]
// tlmRatioIdx: 0=off 1=1:128 2=1:64 3=1:32 4=1:16 5=1:8 6=1:4 7=1:2
#define OTA_SYNC_TLMRATIO_BYTE  3
#define OTA_SYNC_TLMRATIO_SHIFT 1
#define OTA_SYNC_TLMRATIO_MASK  0x07

// ---- ESP-NOW channel (must match Gate Node ESPNOW_CHANNEL) ----
#define ESPNOW_CHANNEL       1

// ---- Identity ----
typedef struct { uint8_t uid[6]; bool valid; } SnifferIdentity_t;

// ---- ESP-NOW packet: EP1 -> Gate ESP32 (RSSI report, 12 bytes) ----
// MUST stay byte-identical to GateEP1Packet_t in src/gate_node/promiscuous.*
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
