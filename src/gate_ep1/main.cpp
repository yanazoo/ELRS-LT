// main.cpp - Gate EP1 sniffer entry point.
// State machine: PROVISION -> SCAN -> FOLLOW, reporting RSSI over ESP-NOW.
//
// Provisioning (pick one):
//   A. Compile-time: define BRINGUP_UID in secrets.h (for solo bring-up).
//   B. Runtime UART: send "UID AABBCCDDEEFF\n" over the serial monitor.
//
// SCAN: steps through the FHSS sequence, dwelling SCAN_DWELL_US per channel,
//       until a packet arrives.  The hop position at lock-on anchors timing.
//
// FOLLOW: tunes SX_SWITCH_US before each expected slot boundary, polls for the
//         packet, and re-anchors s_nextSlot_us on every successful reception.
//         MISS_STREAK_RESYNC consecutive misses -> back to SCAN.

#include <Arduino.h>
#include "config.h"
#include "fhss.h"
#include "sx1280_sniffer.h"
#include "espnow_tx.h"

#if __has_include("secrets.h")
  #include "secrets.h"
#endif

// ---- State machine ----
enum State { ST_PROVISION, ST_SCAN, ST_FOLLOW };
static State    state      = ST_PROVISION;
static uint16_t hopIndex   = 0;
static uint16_t missStreak = 0;
static uint32_t lastReport = 0;
static SnifferIdentity_t ident = { {0}, false };

// s_nextSlot_us: micros() timestamp when the NEXT packet is expected.
// Set at lock-on, re-anchored on every received packet, advanced by
// ELRS_SLOT_US on each miss so timing stays roughly correct.
static uint32_t s_nextSlot_us = 0;

// ---- Link-quality: rolling window of LQ_WINDOW slots ----
#define LQ_WINDOW 50
static uint8_t s_lqBuf[LQ_WINDOW] = {};
static uint8_t s_lqHead = 0;
static uint8_t s_lqSum  = 0;

static void lqPush(bool got) {
    s_lqSum -= s_lqBuf[s_lqHead];
    uint8_t v = got ? 1 : 0;
    s_lqBuf[s_lqHead] = v;
    s_lqSum += v;
    s_lqHead = (s_lqHead + 1) % LQ_WINDOW;
}

static uint8_t lqPct() { return (uint8_t)((s_lqSum * 100U) / LQ_WINDOW); }

// ---- UART UID provisioning ----
// Accepts:  "UID AABBCCDDEEFF\n"  or  "UID AA:BB:CC:DD:EE:FF\n"
static char    s_rxBuf[24];
static uint8_t s_rxLen = 0;

static bool parseHexUid(const char *s, uint8_t out[6]) {
    uint8_t nibbles = 0;
    for (; *s && nibbles < 12; ++s) {
        uint8_t n;
        char c = *s;
        if      (c >= '0' && c <= '9') n = (uint8_t)(c - '0');
        else if (c >= 'a' && c <= 'f') n = (uint8_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') n = (uint8_t)(c - 'A' + 10);
        else continue;  // skip colons, spaces, etc.
        if (nibbles % 2 == 0) out[nibbles / 2]  = (uint8_t)(n << 4);
        else                  out[nibbles / 2] |= n;
        ++nibbles;
    }
    return nibbles == 12;
}

static bool tryProvisionUart() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            s_rxBuf[s_rxLen] = '\0';
            s_rxLen = 0;
            if (strncmp(s_rxBuf, "UID ", 4) == 0) {
                uint8_t tmp[6];
                if (parseHexUid(s_rxBuf + 4, tmp)) {
                    memcpy(ident.uid, tmp, 6);
                    ident.valid = true;
                    Serial.print(F("[gate_ep1] uid: "));
                    for (int i = 0; i < 6; i++) {
                        if (i) Serial.print(':');
                        if (ident.uid[i] < 0x10) Serial.print('0');
                        Serial.print(ident.uid[i], HEX);
                    }
                    Serial.println();
                    return true;
                }
                Serial.println(F("[gate_ep1] bad UID format"));
            }
        } else if (s_rxLen < (uint8_t)(sizeof(s_rxBuf) - 1)) {
            s_rxBuf[s_rxLen++] = c;
        }
    }
    return false;
}

static bool tryProvision() {
#ifdef BRINGUP_UID
    if (!ident.valid) {
        static const uint8_t kBup[6] = BRINGUP_UID;
        memcpy(ident.uid, kBup, 6);
        ident.valid = true;
        Serial.println(F("[gate_ep1] uid from BRINGUP_UID"));
    }
    return true;
#else
    return tryProvisionUart();
#endif
}

// ---- Arduino entry points ----

void setup() {
    Serial.begin(115200);
    delay(50);
    Serial.println(F("[gate_ep1] boot"));
#ifndef BRINGUP_UID
    Serial.println(F("[gate_ep1] awaiting: UID AABBCCDDEEFF"));
#endif
    if (!sxBegin())     Serial.println(F("[gate_ep1] SX1280 FAILED"));
    else                Serial.println(F("[gate_ep1] SX1280 OK"));
    if (!espnowBegin()) Serial.println(F("[gate_ep1] ESP-NOW FAILED"));
    else                Serial.println(F("[gate_ep1] ESP-NOW OK"));
}

void loop() {
    switch (state) {

    case ST_PROVISION:
        if (tryProvision()) {
            fhssGenerate(ident.uid);
            hopIndex   = 0;
            missStreak = 0;
            s_lqHead   = 0;
            s_lqSum    = 0;
            memset(s_lqBuf, 0, sizeof(s_lqBuf));
            state = ST_SCAN;
            Serial.println(F("[gate_ep1] -> SCAN"));
        }
        break;

    case ST_SCAN: {
        // Tune to this position in the FHSS sequence and listen briefly.
        // Because the sequence was generated from the known UID, hopIndex
        // directly maps to the TX's current hop position when we get a hit.
        sxSetFrequencyHz(fhssFreqHz(fhssChannelAt(hopIndex)));
        delayMicroseconds(SCAN_DWELL_US);

        if (sxPacketReceived()) {
            uint16_t lockHop = hopIndex;
            // Anchor: next packet expected one full slot from now.
            s_nextSlot_us = micros() + ELRS_SLOT_US;
            hopIndex = (hopIndex + 1) % FHSS_SEQUENCE_LEN;
            missStreak = 0;
            state = ST_FOLLOW;
            Serial.print(F("[gate_ep1] locked hop="));
            Serial.print(lockHop);
            Serial.println(F(" -> FOLLOW"));
        } else {
            hopIndex = (hopIndex + 1) % FHSS_SEQUENCE_LEN;
        }
        break;
    }

    case ST_FOLLOW: {
        // Tune SX_SWITCH_US before the expected slot boundary.
        int32_t waitUs = (int32_t)(s_nextSlot_us - SX_SWITCH_US - micros());
        if (waitUs > 0 && waitUs < (int32_t)ELRS_SLOT_US)
            delayMicroseconds((uint32_t)waitUs);

        sxSetFrequencyHz(fhssFreqHz(fhssChannelAt(hopIndex)));

        // Poll until the packet arrives or we reach the tune-out point for
        // the next slot (leaving SX_SWITCH_US + 500 µs margin).
        uint32_t deadline = s_nextSlot_us + ELRS_SLOT_US - SX_SWITCH_US - 500U;
        bool got = false;
        while ((int32_t)(deadline - micros()) > 0) {
            if (sxPacketReceived()) { got = true; break; }
            yield();
        }

        if (got) {
            s_nextSlot_us = micros() + ELRS_SLOT_US;   // re-anchor from real packet
            missStreak = 0;
            lqPush(true);

            int8_t   rssi = sxReadRssi();
            uint32_t now  = millis();
            if (now - lastReport >= RSSI_REPORT_MS) {
                espnowSendRssi(ident.uid, rssi, lqPct(), now);
                lastReport = now;
            }
        } else {
            lqPush(false);
            s_nextSlot_us += ELRS_SLOT_US;  // keep advancing even on miss
            if (++missStreak >= MISS_STREAK_RESYNC) {
                state = ST_SCAN;
                Serial.print(F("[gate_ep1] resync lq="));
                Serial.println(lqPct());
            }
        }
        hopIndex = (hopIndex + 1) % FHSS_SEQUENCE_LEN;
        break;
    }
    }
}
