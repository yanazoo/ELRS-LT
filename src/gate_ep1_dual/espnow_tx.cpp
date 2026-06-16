// espnow_tx.cpp - ESP-NOW sender/receiver for the EP1 Dual sniffer (ESP32).
//
// Same wire protocol as src/gate_ep1/espnow_tx.cpp, ported from the ESP8266
// core API to the ESP32 esp_now/esp_wifi API:
//   EP1 -> Gate (broadcast FF:FF:FF:FF:FF:FF):
//     - GateEP1BeaconPacket_t (8B, magic=0xA5) periodically
//     - GateEP1Packet_t       (12B)            every RSSI_REPORT_MS in FOLLOW
//   Gate -> EP1 (broadcast/unicast):
//     - GateProvisionPacket_t (7B, magic=0xB1) when the user assigns a UID

#include "espnow_tx.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

static const uint8_t BCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static ProvisionCallback_t s_provisionCb = nullptr;

void espnowSetProvisionCallback(ProvisionCallback_t cb) { s_provisionCb = cb; }

// Decode an incoming provision packet (called from the ESP-NOW recv callback).
static void handleRecv(const uint8_t *srcMac, const uint8_t *data, int len) {
    if (len != (int)sizeof(GateProvisionPacket_t)) return;
    GateProvisionPacket_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic != GATE_PROV_MAGIC) return;
    Serial.printf("[espnow] provision from %02X:%02X:%02X:%02X:%02X:%02X\n",
                  srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5]);
    if (s_provisionCb) s_provisionCb(pkt.uid);
}

// Arduino-ESP32 changed the recv-callback signature in core 3.x.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    handleRecv(info->src_addr, data, len);
}
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
    handleRecv(mac, data, len);
}
#endif

// Keep the radio pinned to the Gate Node's channel and the broadcast peer alive.
static void ensureChannelAndPeer() {
    uint8_t ch; wifi_second_chan_t sc;
    if (esp_wifi_get_channel(&ch, &sc) == ESP_OK && ch != ESPNOW_CHANNEL) {
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    }
    if (!esp_now_is_peer_exist(BCAST_MAC)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, BCAST_MAC, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }
}

bool espnowBegin() {
    // STA mode, no AP association (association would pull us off ESPNOW_CHANNEL).
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[espnow] init failed");
        return false;
    }
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BCAST_MAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[espnow] add_peer(broadcast) failed");
    }

    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("[espnow] up ch=%d, my MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  ESPNOW_CHANNEL, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return true;
}

void espnowSendRssi(const uint8_t uid[6], int8_t rssi, uint8_t lq, uint32_t ts) {
    ensureChannelAndPeer();
    GateEP1Packet_t pkt;
    memcpy(pkt.pilot_uid, uid, 6);
    pkt.rssi = rssi;
    pkt.lq   = lq;
    pkt.ts   = ts;
    esp_now_send(BCAST_MAC, (const uint8_t *)&pkt, sizeof(pkt));
}

void espnowSendBeacon(const uint8_t uid[6], bool uidValid, uint8_t state) {
    ensureChannelAndPeer();
    GateEP1BeaconPacket_t pkt;
    pkt.magic = EP1_BEACON_MAGIC;
    pkt.state = state;
    if (uidValid) memcpy(pkt.uid, uid, 6);
    else          memset(pkt.uid, 0, 6);
    esp_err_t rc = esp_now_send(BCAST_MAC, (const uint8_t *)&pkt, sizeof(pkt));
    Serial.printf("[espnow] beacon tx state=%u rc=%d\n", (unsigned)state, (int)rc);
}
