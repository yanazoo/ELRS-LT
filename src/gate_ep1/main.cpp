// main.cpp - Gate EP1 sniffer entry point.
// State machine: PROVISION -> SCAN -> FOLLOW, reporting RSSI over ESP-NOW.
//
// Provisioning (pick one, in priority order):
//   A. Compile-time: define BRINGUP_UID in secrets.h (for solo bring-up).
//   B. Runtime ESP-NOW: Gate Node unicasts GateProvisionPacket_t on pilot assign.
//   C. Runtime UART: send "UID AABBCCDDEEFF\n" over the serial monitor.
//
// This EP1 sends a GateEP1BeaconPacket_t every BEACON_INTERVAL_MS so the
// Gate Node can discover its MAC and relay it to the Web UI for assignment.

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

static uint32_t s_nextSlot_us = 0;
// Slot counter within current hop (0..FHSS_HOP_INTERVAL-1).
// The ELRS TX stays on each channel for FHSS_HOP_INTERVAL consecutive packets
// before hopping; the sniffer must do the same to stay in sync.
static uint8_t s_slotInHop = 0;

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

// ---- ESP-NOW provision from Gate Node ----
// Written from ESP-NOW recv callback (network task context).
static volatile bool s_newProvision = false;
static uint8_t s_pendingUid[6] = {};

static void onProvision(const uint8_t uid[6]) {
    memcpy(s_pendingUid, uid, 6);
    s_newProvision = true;
}

// Apply a pending provision packet (called from main loop, not from ISR context).
static void applyProvision() {
    if (!s_newProvision) return;
    s_newProvision = false;

    bool isZero = true;
    for (uint8_t i = 0; i < 6; i++) if (s_pendingUid[i]) { isZero = false; break; }

    if (!isZero) {
        memcpy(ident.uid, s_pendingUid, 6);
        ident.valid = true;
        Serial.print(F("[gate_ep1] provisioned "));
        for (int i = 0; i < 6; i++) {
            if (i) Serial.print(':');
            if (ident.uid[i] < 0x10) Serial.print('0');
            Serial.print(ident.uid[i], HEX);
        }
        Serial.println();
        // Restart FHSS tracking with new UID from any state.
        fhssGenerate(ident.uid);
        hopIndex = 0; missStreak = 0; s_slotInHop = 0;
        s_lqHead = 0; s_lqSum = 0;
        memset(s_lqBuf, 0, sizeof(s_lqBuf));
        state = ST_SCAN;
        Serial.println(F("[gate_ep1] -> SCAN"));
    } else {
        ident.valid = false;
        state = ST_PROVISION;
        Serial.println(F("[gate_ep1] UID cleared -> PROVISION"));
    }
}

// ---- UART UID provisioning (fallback / bring-up) ----
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
        else continue;
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

// ---- Beacon timer ----
// PROVISION: 1 s (Gate discovers EP1 quickly on boot)
// SCAN:      2 s
// FOLLOW:    5 s (keep-alive only)
#define BEACON_INTERVAL_PROVISION_MS 1000U
#define BEACON_INTERVAL_SCAN_MS      2000U
#define BEACON_INTERVAL_FOLLOW_MS    5000U
static uint32_t s_lastBeaconMs = 0;

static void maybeSendBeacon() {
    uint32_t now = millis();
    uint32_t interval = (state == ST_PROVISION) ? BEACON_INTERVAL_PROVISION_MS
                      : (state == ST_SCAN)       ? BEACON_INTERVAL_SCAN_MS
                                                 : BEACON_INTERVAL_FOLLOW_MS;
    if (now - s_lastBeaconMs >= interval) {
        s_lastBeaconMs = now;
        espnowSendBeacon(ident.uid, ident.valid, (uint8_t)state);
    }
}

// ---- LED heartbeat (non-blocking) ----
// PROVISION: slow single pulse every 2 s
// SCAN:      rapid 200 ms blink (searching)
// FOLLOW:    double-pulse every 2 s (tracking)
static void updateLedHeartbeat() {
    uint32_t t = millis();
    bool on;
    switch (state) {
    case ST_PROVISION: { uint32_t p = t % 2000; on = (p < 50);  break; }
    case ST_SCAN:      { uint32_t p = t % 200;  on = (p < 100); break; }
    case ST_FOLLOW:    { uint32_t p = t % 2000; on = (p < 80) || (p >= 160 && p < 240); break; }
    default: on = false;
    }
    digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// ---- Arduino entry points ----

void setup() {
    Serial.begin(115200);
    delay(50);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    Serial.println(F("[gate_ep1] boot"));
#ifndef BRINGUP_UID
    Serial.println(F("[gate_ep1] awaiting UID (UART or ESP-NOW provision)"));
#endif
    if (!sxBegin())     Serial.println(F("[gate_ep1] SX1280 FAILED"));
    else                Serial.println(F("[gate_ep1] SX1280 OK"));
    if (!espnowBegin()) Serial.println(F("[gate_ep1] ESP-NOW FAILED"));
    else                Serial.println(F("[gate_ep1] ESP-NOW OK"));
    espnowSetProvisionCallback(onProvision);
    espnowSendBeacon(ident.uid, ident.valid, (uint8_t)state);
    s_lastBeaconMs = millis();
}

void loop() {
    applyProvision();       // handle ESP-NOW provision before state machine
    maybeSendBeacon();      // 5 s presence beacon
    updateLedHeartbeat();   // non-blocking state indicator

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
        sxSetFrequencyHz(fhssFreqHz(fhssChannelAt(hopIndex)));
        // Poll during the dwell window so micros() is captured at the moment
        // of actual receipt, not up to SCAN_DWELL_US later.  The offset matters:
        // s_nextSlot_us feeds the FOLLOW timing and a 1500µs error here causes
        // EP1 to consistently miss the first packets after lock and fall back to
        // SCAN before ever recalibrating.
        uint32_t dwellEnd = micros() + SCAN_DWELL_US;
        bool scanGot = false;
        while ((int32_t)(dwellEnd - micros()) > 0) {
            if (sxPacketReceived()) { scanGot = true; break; }
            yield();
        }

        if (scanGot) {
            uint16_t lockHop = hopIndex;
            s_nextSlot_us = micros() + ELRS_SLOT_US;
            // Don't advance hopIndex yet — stay on the locked channel for the
            // remainder of the current hop.  s_slotInHop counts received packets
            // and advances hopIndex every FHSS_HOP_INTERVAL hits.
            missStreak = 0;
            s_slotInHop = 0;
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
        int32_t waitUs = (int32_t)(s_nextSlot_us - SX_SWITCH_US - micros());
        if (waitUs > 0 && waitUs < (int32_t)ELRS_SLOT_US)
            delayMicroseconds((uint32_t)waitUs);

        sxSetFrequencyHz(fhssFreqHz(fhssChannelAt(hopIndex)));

        uint32_t deadline = s_nextSlot_us + ELRS_SLOT_US - SX_SWITCH_US - 500U;
        bool got = false;
        while ((int32_t)(deadline - micros()) > 0) {
            if (sxPacketReceived()) { got = true; break; }
            yield();
        }

        if (got) {
            s_nextSlot_us = micros() + ELRS_SLOT_US;
            missStreak = 0;
            lqPush(true);

            int8_t   rssi = sxReadRssi();
            uint32_t now  = millis();
            if (now - lastReport >= RSSI_REPORT_MS) {
                espnowSendRssi(ident.uid, rssi, lqPct(), now);
                lastReport = now;
            }

            // Advance FHSS channel every FHSS_HOP_INTERVAL received packets.
            // The ELRS TX stays on each channel for hopInterval=4 packets (16ms at
            // 250Hz) before hopping; the sniffer must mirror this exactly.
            if (++s_slotInHop >= FHSS_HOP_INTERVAL) {
                s_slotInHop = 0;
                hopIndex = (hopIndex + 1) % FHSS_SEQUENCE_LEN;
            }
        } else {
            lqPush(false);
            s_nextSlot_us += ELRS_SLOT_US;
            if (++missStreak >= MISS_STREAK_RESYNC) {
                state = ST_SCAN;
                s_slotInHop = 0;
                Serial.print(F("[gate_ep1] resync lq="));
                Serial.println(lqPct());
            }
        }
        break;
    }
    }
}
