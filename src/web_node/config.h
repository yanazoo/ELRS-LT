#pragma once
#include <IPAddress.h>

// UART to Gate Node
#define GATE_RX_PIN   3
#define GATE_TX_PIN   2
// MUST match UART_BAUD in src/gate_node/config.h — flash both nodes together.
#define GATE_BAUD     230400

// Pilot limits
#define MAX_REGISTERED  20
#define MAX_ACTIVE       4
#define MAX_LAPS       200
// Per-pilot threshold defaults applied at registration and sent to the Gate
// Node (they override the Gate's own DEFAULT_ENTRY_THR/DEFAULT_EXIT_THR).
// MUST stay above the TX downlink background (-65 to -75 dBm) or a freshly
// registered pilot enters a permanent crossing state and never records a lap.
// Keep in sync with DEFAULT_ENTRY_THR / DEFAULT_EXIT_THR in src/gate_node/config.h.
#define DEFAULT_ENTER  (-55)
#define DEFAULT_EXIT   (-62)

// Scan
#define MAX_SCAN_MACS 8

// WiFi Access Point
static const char*     AP_SSID    = "ESP-NOW-LT";
static const char*     AP_PASS    = "esp-now-lt";
static const IPAddress AP_IP      (20, 0, 0, 1);
static const IPAddress AP_GATEWAY (20, 0, 0, 1);
static const IPAddress AP_SUBNET  (255, 255, 255, 0);
