#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "pilots.h"

// ---- Packet: EP1 -> Gate Node (RSSI report, 12 bytes) ----
// Must match GateEP1Packet_t in src/gate_ep1/config.h exactly.
typedef struct __attribute__((packed)) {
    uint8_t  pilot_uid[6];
    int8_t   rssi;
    uint8_t  lq;
    uint32_t ts;
} GateEP1Packet_t;

// ---- Packet: EP1 -> Gate Node (presence beacon, 9 bytes) ----
#define EP1_BEACON_MAGIC  0xA5
typedef struct __attribute__((packed)) {
    uint8_t magic;   // EP1_BEACON_MAGIC
    uint8_t state;   // 0=PROVISION 1=SCAN 2=FOLLOW
    uint8_t uid[6];  // current UID (all-zero if not provisioned)
    int8_t  noise;   // SCAN-state noise floor dBm; -127 if unavailable
} GateEP1BeaconPacket_t;

// ---- Packet: Gate Node -> EP1 (unicast provisioning, 7 bytes) ----
#define GATE_PROV_MAGIC   0xB1
typedef struct __attribute__((packed)) {
    uint8_t magic;   // GATE_PROV_MAGIC
    uint8_t uid[6];  // ELRS bind UID (all-zero = clear)
} GateProvisionPacket_t;

extern QueueHandle_t packetQueue;

void setupEspNowGate();

// Unicast a UID assignment to a specific EP1 by its MAC address.
// MAC is learned from GateEP1BeaconPacket_t source address.
// uid all-zero = clear / stop tracking.
void espnowProvisionMac(const uint8_t ep1Mac[6], const uint8_t uid[6]);
