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
#define FHSS_CHANNEL_COUNT   80                          // 2.4GHz ISM unique channels
#define FHSS_SEQUENCE_LEN    (FHSS_CHANNEL_COUNT * 3)   // 240: 3 complete blocks per ELRS
#define ELRS_SLOT_US         4000   // 250Hz default; adjust per packet rate
#define SX_SWITCH_US         1000   // approx SX1280 frequency switch time

// ---- Lock-on tuning ----
#define SCAN_DWELL_US        1500   // RX dwell per channel during SCAN phase
#define MISS_STREAK_RESYNC   30     // consecutive misses -> drop back to SCAN

// ---- RSSI reporting ----
#define RSSI_REPORT_MS       50     // 20 Hz, matches existing RSSI_INTERVAL_MS

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
