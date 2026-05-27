// espnow_tx.cpp - ESP-NOW sender/receiver for EP1 sniffer (ESP8266 core API)
#include "espnow_tx.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <string.h>

// GATE_ESP32_MAC defined in secrets.h (gitignored).
// Falls back to all-zeros so the firmware links without secrets.h.
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  const uint8_t GATE_ESP32_MAC[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#endif

static ProvisionCallback_t s_provisionCb = nullptr;

void espnowSetProvisionCallback(ProvisionCallback_t cb) {
    s_provisionCb = cb;
}

// Fires for any received ESP-NOW packet (from any sender).
// Gate Node sends GateProvisionPacket_t (7 bytes, magic=GATE_PROV_MAGIC).
static void onRecv(u8 * /*mac*/, u8 *data, u8 len) {
    if (len != (u8)sizeof(GateProvisionPacket_t)) return;
    GateProvisionPacket_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic != GATE_PROV_MAGIC) return;
    if (s_provisionCb) s_provisionCb(pkt.uid);
}

bool espnowBegin() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != 0) return false;
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);  // send RSSI/beacons + receive provision
    esp_now_add_peer((u8*)GATE_ESP32_MAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    esp_now_register_recv_cb(onRecv);
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

// Sends a presence beacon so Gate Node can discover this EP1's MAC and
// relay it to the Web UI for dynamic node assignment.
void espnowSendBeacon(const uint8_t uid[6], bool uidValid, uint8_t state) {
    GateEP1BeaconPacket_t pkt;
    pkt.magic = EP1_BEACON_MAGIC;
    pkt.state = state;
    if (uidValid) memcpy(pkt.uid, uid, 6);
    else          memset(pkt.uid, 0, 6);
    esp_now_send((u8*)GATE_ESP32_MAC, (u8*)&pkt, sizeof(pkt));
}
