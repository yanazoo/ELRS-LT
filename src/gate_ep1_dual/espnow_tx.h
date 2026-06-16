// espnow_tx.h - ESP-NOW send/receive for the EP1 Dual sniffer (ESP32 core API).
// Wire protocol is identical to the single-radio firmware, so gate_node and
// web_node need no changes.
#pragma once
#include "config.h"

// Callback invoked when a GateProvisionPacket_t arrives from Gate Node.
// uid: 6-byte ELRS UID to follow (all-zero = clear).
typedef void (*ProvisionCallback_t)(const uint8_t uid[6]);

// Init ESP-NOW (send RSSI/beacons + receive provision).
bool espnowBegin();

// Register callback for incoming provision packets from Gate Node.
void espnowSetProvisionCallback(ProvisionCallback_t cb);

// Send RSSI report to Gate Node. Non-blocking; drops silently on failure.
void espnowSendRssi(const uint8_t uid[6], int8_t rssi, uint8_t lq, uint32_t ts);

// Send presence beacon so Gate Node can discover this EP1's MAC.
// state: 0=PROVISION 1=SCAN 2=FOLLOW
void espnowSendBeacon(const uint8_t uid[6], bool uidValid, uint8_t state);
