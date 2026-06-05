// espnow_tx.cpp - ESP-NOW sender/receiver for EP1 sniffer (ESP8266 core API).
//
// Wire protocol:
//   EP1 -> Gate:
//     - GateEP1BeaconPacket_t (8B, magic=0xA5) broadcast every BEACON_INTERVAL_MS
//       (broadcast so the Gate can discover this EP1's MAC; low rate).
//     - GateEP1Packet_t       (12B) every RSSI_REPORT_MS while in FOLLOW —
//       UNICAST to the Gate once its MAC is learned (broadcast fallback before
//       the first provision).  This is the 20 Hz hot path; keeping it unicast
//       stops other EP1s on the same channel from being interrupted by it.
//   Gate -> EP1 (unicast, EP1 MAC learned by Gate from beacon src_addr):
//     - GateProvisionPacket_t (7B, magic=0xB1) when user assigns UID.
//       EP1 learns the Gate MAC from this packet's src_addr.
//
// The Gate's recv callback is invoked for broadcast frames and for unicast
// frames addressed to it, so both paths reach the Gate on the configured
// channel; no Gate MAC needs to be hard-coded or kept in secrets.h.

#include "espnow_tx.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <string.h>

extern "C" {
#include <user_interface.h>   // wifi_set_channel(), wifi_get_channel()
}

static const uint8_t BCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Gate Node MAC, learned from the src_addr of the first provision packet.
// Once known, the high-rate (20 Hz) RSSI reports are sent UNICAST to the Gate
// instead of broadcast.  This is critical with >1 EP1 on the same channel:
// broadcast RSSI frames are accepted by every EP1's hardware MAC filter and
// fire a WiFi RX interrupt (~1-2 ms WiFi-stack stall each) that disrupts the
// SX1280 capture dwell and breaks FHSS lock.  A unicast frame addressed to the
// Gate is dropped in hardware by the other EP1s, so they no longer interfere.
static uint8_t s_gateMac[6]    = {};
static bool    s_gateMacKnown  = false;

// Beacons are normally unicast to the Gate once its MAC is known (so other EP1s
// drop them in hardware), but every BEACON_BCAST_EVERY-th beacon is broadcast
// so the Gate — or a replacement Gate — can always (re)discover this EP1.
#define BEACON_BCAST_EVERY 6

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

    // Learn the Gate's MAC so RSSI reports can be unicast (see s_gateMac note).
    // Peer registration is deferred to the send path (main-loop context) — the
    // ESP-NOW recv callback runs in WiFi-task context where esp_now_add_peer()
    // is best avoided.
    memcpy(s_gateMac, srcMac, 6);
    s_gateMacKnown = true;

    if (s_provisionCb) s_provisionCb(pkt.uid);
}

// Re-pin radio channel and ensure broadcast peer exists. ESP8266 STA stack can
// silently change channels (background scan, AP-reconnect attempts) and some
// ESP-NOW stack states drop peers. Calling this before every send is cheap and
// makes broadcast delivery reliable.
static void ensureChannelAndPeer() {
    if (wifi_get_channel() != ESPNOW_CHANNEL) {
        wifi_set_channel(ESPNOW_CHANNEL);
    }
    if (!esp_now_is_peer_exist((u8*)BCAST_MAC)) {
        esp_now_add_peer((u8*)BCAST_MAC, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, NULL, 0);
    }
}

// Ensure the Gate Node is registered as a unicast peer. Called from the send
// path (main-loop context) once the Gate MAC has been learned from a provision.
static void ensureGatePeer() {
    if (!s_gateMacKnown) return;
    if (!esp_now_is_peer_exist(s_gateMac)) {
        esp_now_add_peer(s_gateMac, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, NULL, 0);
    }
}

bool espnowBegin() {
    // Prevent the STA stack from auto-connecting / auto-reconnecting to any
    // saved AP — that would trigger background scanning and pull the radio
    // off ESPNOW_CHANNEL.
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setAutoConnect(false);
    WiFi.setAutoReconnect(false);

    // Force the radio onto the Gate Node's channel.
    wifi_set_channel(ESPNOW_CHANNEL);

    if (esp_now_init() != 0) {
        Serial.println("[espnow] init failed");
        return false;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);  // both send + receive

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
    ensureChannelAndPeer();
    GateEP1Packet_t pkt;
    memcpy(pkt.pilot_uid, uid, 6);
    pkt.rssi = rssi;
    pkt.lq   = lq;
    pkt.ts   = ts;
    // Unicast to the Gate once its MAC is known: this is the 20 Hz hot path, and
    // keeping it off the broadcast address is what prevents other EP1s on the
    // same channel from being interrupted by it (their MAC filter drops it).
    // Fall back to broadcast before the first provision (e.g. auto-discovery /
    // UART bring-up), where there is no Gate MAC to target yet.
    if (s_gateMacKnown) {
        ensureGatePeer();
        esp_now_send(s_gateMac, (u8*)&pkt, sizeof(pkt));
    } else {
        esp_now_send((u8*)BCAST_MAC, (u8*)&pkt, sizeof(pkt));
    }
}

// Sends a presence beacon so Gate Node can discover this EP1's MAC and
// relay it to the Web UI for dynamic node assignment.
void espnowSendBeacon(const uint8_t uid[6], bool uidValid, uint8_t state) {
    ensureChannelAndPeer();
    GateEP1BeaconPacket_t pkt;
    pkt.magic = EP1_BEACON_MAGIC;
    pkt.state = state;
    if (uidValid) memcpy(pkt.uid, uid, 6);
    else          memset(pkt.uid, 0, 6);

    // Unicast to the Gate when known (keeps beacons off the other EP1s' MAC
    // filters); periodically broadcast for (re)discovery; always broadcast
    // before the first provision, when the Gate MAC is not yet learned.
    static uint8_t beaconSeq = 0;
    bool bcast = !s_gateMacKnown || (beaconSeq++ % BEACON_BCAST_EVERY) == 0;
    const u8 *dst;
    if (bcast) {
        dst = (const u8*)BCAST_MAC;
    } else {
        ensureGatePeer();
        dst = (const u8*)s_gateMac;
    }
    int rc = esp_now_send((u8*)dst, (u8*)&pkt, sizeof(pkt));
    Serial.printf("[espnow] beacon tx state=%u ch=%u %s rc=%d\n",
                  (unsigned)state, (unsigned)wifi_get_channel(),
                  bcast ? "bcast" : "uni", rc);
}
