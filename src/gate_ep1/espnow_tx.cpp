// espnow_tx.cpp - ESP-NOW sender/receiver for EP1 sniffer (ESP8266 core API).
//
// Wire protocol:
//   EP1 -> Gate (broadcast FF:FF:FF:FF:FF:FF):
//     - GateEP1BeaconPacket_t (8B, magic=0xA5) every BEACON_INTERVAL_MS
//     - GateEP1Packet_t       (12B)             every RSSI_REPORT_MS while in FOLLOW
//   Gate -> EP1 (unicast, EP1 MAC learned by Gate from beacon src_addr):
//     - GateProvisionPacket_t (7B, magic=0xB1) when user assigns UID
//
// Broadcasting outgoing traffic means no Gate MAC needs to be hard-coded
// or kept in secrets.h. The Gate's recv callback is invoked for any packet
// on the configured channel.

#include "espnow_tx.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <string.h>

extern "C" {
#include <user_interface.h>   // wifi_set_channel()
}

static const uint8_t BCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static ProvisionCallback_t s_provisionCb = nullptr;

void espnowSetProvisionCallback(ProvisionCallback_t cb) {
    s_provisionCb = cb;
}

// Fires for any received ESP-NOW packet (from any sender).
// Gate Node sends GateProvisionPacket_t (7 bytes, magic=GATE_PROV_MAGIC) unicast.
static void onRecv(u8 *srcMac, u8 *data, u8 len) {
    if (len != (u8)sizeof(GateProvisionPacket_t)) return;
    GateProvisionPacket_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic != GATE_PROV_MAGIC) return;

    Serial.printf("[espnow] provision from %02X:%02X:%02X:%02X:%02X:%02X\n",
                  srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5]);

    if (s_provisionCb) s_provisionCb(pkt.uid);
}

bool espnowBegin() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Force the radio onto the Gate Node's channel. Without this, ESP8266
    // STA mode may sit on a different channel from a stale stored AP and
    // ESP-NOW will silently fail to deliver.
    wifi_set_channel(ESPNOW_CHANNEL);

    if (esp_now_init() != 0) {
        Serial.println("[espnow] init failed");
        return false;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);  // both send + receive

    // Add broadcast as a peer so esp_now_send(BCAST_MAC, ...) succeeds.
    if (esp_now_add_peer((u8*)BCAST_MAC, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, NULL, 0) != 0) {
        Serial.println("[espnow] add_peer(broadcast) failed");
    }

    esp_now_register_recv_cb(onRecv);

    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("[espnow] up ch=%d, my MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  ESPNOW_CHANNEL, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return true;
}

void espnowSendRssi(const uint8_t uid[6], int8_t rssi, uint8_t lq, uint32_t ts) {
    GateEP1Packet_t pkt;
    memcpy(pkt.pilot_uid, uid, 6);
    pkt.rssi = rssi;
    pkt.lq   = lq;
    pkt.ts   = ts;
    esp_now_send((u8*)BCAST_MAC, (u8*)&pkt, sizeof(pkt));
}

// Sends a presence beacon so Gate Node can discover this EP1's MAC and
// relay it to the Web UI for dynamic node assignment.
void espnowSendBeacon(const uint8_t uid[6], bool uidValid, uint8_t state) {
    GateEP1BeaconPacket_t pkt;
    pkt.magic = EP1_BEACON_MAGIC;
    pkt.state = state;
    if (uidValid) memcpy(pkt.uid, uid, 6);
    else          memset(pkt.uid, 0, 6);
    int rc = esp_now_send((u8*)BCAST_MAC, (u8*)&pkt, sizeof(pkt));
    Serial.printf("[espnow] beacon tx state=%u rc=%d\n", (unsigned)state, rc);
}
