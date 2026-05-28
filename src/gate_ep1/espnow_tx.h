// espnow_tx.h - ESP-NOW send/receive for EP1 sniffer (ESP8285)
#pragma once
#include "config.h"

// Callback invoked when a GateProvisionPacket_t arrives from Gate Node.
// uid: 6-byte ELRS UID to follow (all-zero = clear).
typedef void (*ProvisionCallback_t)(const uint8_t uid[6]);

// Init ESP-NOW (COMBO role: send RSSI/beacons + receive provision).
bool espnowBegin();

// Register callback for incoming provision packets from Gate Node.
void espnowSetProvisionCallback(ProvisionCallback_t cb);

// Send RSSI report to Gate Node. Non-blocking; drops silently on failure.
void espnowSendRssi(const uint8_t uid[6], int8_t rssi, uint8_t lq, uint32_t ts);

// Send presence beacon to Gate Node so Gate Node can discover this EP1's
// MAC and relay it to the Web UI. noise = noise floor dBm (-127 if unknown).
// state: 0=PROVISION 1=SCAN 2=FOLLOW
void espnowSendBeacon(const uint8_t uid[6], bool uidValid, uint8_t state, int8_t noise = -127);
