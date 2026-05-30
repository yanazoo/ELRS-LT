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

// ---- Lock-on tuning ----
// SCAN dwell must exceed one packet interval (500Hz = 2000us) so a parked
// channel reliably catches a packet while the TX is transmitting on it.
#define SCAN_DWELL_US        2600   // RX dwell per channel during SCAN phase
#define MISS_STREAK_RESYNC    8     // consecutive empty channel dwells -> back to SCAN

// ---- RSSI reporting ----
#define RSSI_REPORT_MS       50     // 20 Hz, matches existing RSSI_INTERVAL_MS

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
#define OTA_SYNC_FHSS_BYTE      1
#define OTA_SYNC_UID3_BYTE      4
#define OTA_SYNC_UID4_BYTE      5
#define OTA_SYNC_UID5_BYTE      6
#define OTA_SYNC_UID5_HIBITS    0xC0   // only bits[7:6] of byte[6] are UID[5]

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
