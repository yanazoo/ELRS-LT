#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include "promiscuous.h"
#include "config.h"

QueueHandle_t packetQueue;

static void fmtMac(const uint8_t m[6], char out[18]) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             m[0], m[1], m[2], m[3], m[4], m[5]);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *srcMac = info->src_addr;
#else
static void onEspNowRecv(const uint8_t *srcMac, const uint8_t *data, int len) {
#endif
    if (len == (int)sizeof(GateEP1Packet_t)) {
        GateEP1Packet_t pkt;
        memcpy(&pkt, data, sizeof(pkt));
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(packetQueue, &pkt, &woken);
        if (woken) portYIELD_FROM_ISR();

    } else if (len == (int)sizeof(GateEP1BeaconPacket_t)) {
        const GateEP1BeaconPacket_t *b = (const GateEP1BeaconPacket_t *)data;
        if (b->magic != EP1_BEACON_MAGIC) return;

        char macStr[18], uidStr[18] = "";
        fmtMac(srcMac, macStr);

        bool hasUid = false;
        for (int i = 0; i < 6; i++) if (b->uid[i]) { hasUid = true; break; }
        if (hasUid) fmtMac(b->uid, uidStr);

        Serial.printf("[Gate] beacon from %s state=%u uid=%s noise=%d\n",
                      macStr, (unsigned)b->state, uidStr[0] ? uidStr : "(none)", (int)b->noise);

        char json[120];
        snprintf(json, sizeof(json),
                 R"({"type":"ep1_hello","mac":"%s","state":%u,"uid":"%s","noise":%d})",
                 macStr, (unsigned)b->state, uidStr, (int)b->noise);
        Serial1.println(json);

    } else {
        // Unknown packet size — log so we can see if EP1 is reaching us at all
        Serial.printf("[Gate] unknown ESP-NOW pkt from %02X:%02X:%02X:%02X:%02X:%02X len=%d\n",
                      srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5], len);
    }
}

void setupEspNowGate() {
    packetQueue = xQueueCreate(64, sizeof(GateEP1Packet_t));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    esp_now_init();
    esp_now_register_recv_cb(onEspNowRecv);

    Serial.printf("[Gate] ESP-NOW gate receiver on channel %d\n", ESPNOW_CHANNEL);
}

void espnowProvisionMac(const uint8_t ep1Mac[6], const uint8_t uid[6]) {
    if (!esp_now_is_peer_exist(ep1Mac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, ep1Mac, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    GateProvisionPacket_t pkt;
    pkt.magic = GATE_PROV_MAGIC;
    memcpy(pkt.uid, uid, 6);
    esp_err_t err = esp_now_send(ep1Mac, (const uint8_t *)&pkt, sizeof(pkt));

    char macStr[18], uidStr[18];
    fmtMac(ep1Mac, macStr);
    fmtMac(uid, uidStr);
    Serial.printf("[Gate] provision EP1 mac=%s uid=%s err=%d\n", macStr, uidStr, (int)err);
}
