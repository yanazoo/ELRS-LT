// secrets.example.h - COMMIT THIS TEMPLATE.
// Copy to secrets.h (which is gitignored) and fill in real values locally.
// Never commit real UIDs or the Gate ESP32 MAC if you treat it as private.
#pragma once
#include <stdint.h>

// STA MAC of the Gate ESP32 (TTGO T8). Find via WiFi.macAddress() on that board.
const uint8_t GATE_ESP32_MAC[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Optional: compile-time UID for solo bring-up before runtime UART provisioning.
// Uncomment and fill in your pilot's ELRS bind UID (6 bytes shown in TX config).
// Leave commented in the template — NEVER commit a real UID here.
// #define BRINGUP_UID { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }
