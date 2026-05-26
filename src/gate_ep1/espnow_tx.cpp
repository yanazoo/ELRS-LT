// espnow_tx.cpp - ESP-NOW sender for ESP8285 (ESP8266 core API)
#include "espnow_tx.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <string.h>

// GATE_ESP32_MAC is declared extern in config.h.
// Define it from the gitignored secrets.h when present, otherwise fall back
// to all-zeros so the firmware still links for the Step 1 bring-up build.
// Copy secrets.example.h to secrets.h and fill in the real STA MAC of the
// Gate ESP32 (TTGO T8) before ESP-NOW can actually reach the gate node.
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  const uint8_t GATE_ESP32_MAC[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#endif

bool espnowBegin() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != 0) return false;
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    // GATE_ESP32_MAC defined in secrets.h (gitignored).
    esp_now_add_peer((u8*)GATE_ESP32_MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    return true;
}

void espnowSendRssi(const uint8_t uid[6], int8_t rssi, uint8_t lq, uint32_t ts) {
    GateEP1Packet_t pkt;
    memcpy(pkt.pilot_uid, uid, 6);
    pkt.rssi = rssi;
    pkt.lq   = lq;
    pkt.ts   = ts;
    esp_now_send((u8*)GATE_ESP32_MAC, (u8*)&pkt, sizeof(pkt));
}
