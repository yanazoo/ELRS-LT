// secrets.example.h - COMMIT THIS TEMPLATE.
// Copy to secrets.h (which is gitignored) and fill in real values locally.
#pragma once
#include <stdint.h>

// Optional: compile-time UID for solo bring-up before runtime provisioning.
// Uncomment and fill in your pilot's ELRS bind UID (6 bytes shown in TX config).
// Leave commented in the template — NEVER commit a real UID here.
// #define BRINGUP_UID { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }

// NOTE: The Gate ESP32 MAC is no longer needed here. EP1 broadcasts its
// beacons and RSSI reports; the Gate Node learns each EP1's MAC from the
// receive callback automatically.
