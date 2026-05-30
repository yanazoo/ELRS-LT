// main.cpp - Gate EP1 sniffer entry point.
// State machine: PROVISION -> SCAN -> FOLLOW, reporting RSSI over ESP-NOW.
//
// Provisioning (pick one, in priority order):
//   A. Compile-time: define BRINGUP_UID in secrets.h (for solo bring-up).
//   B. Runtime ESP-NOW: Gate Node unicasts GateProvisionPacket_t on pilot assign.
//   C. Runtime UART: send "UID AABBCCDDEEFF\n" over the serial monitor.
//   D. Auto-discovery: EP1 parks on ELRS sync channel, reads OTA SYNC packets
//      to extract UID[4:5]+fhssIndex, then brute-forces UID[2:3] by scanning
//      all 80 channels and matching observed channel activity to candidates.
//
// This EP1 sends a GateEP1BeaconPacket_t every BEACON_INTERVAL_MS so the
// Gate Node can discover its MAC and relay it to the Web UI for assignment.

#include <Arduino.h>
#include "config.h"
#include "fhss.h"
#include "sx1280_sniffer.h"
#include "espnow_tx.h"
#include <string.h>

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

// ---- ESP-NOW provision flag (set from network-task ISR context) ----
// Declared here so auto-discovery can abort early when a provision arrives.
static volatile bool s_newProvision = false;
static uint8_t       s_pendingUid[6] = {};

// ---- Auto-discovery state ----
// Active only in ST_PROVISION when no UID is available from standard sources.
// Not compiled in BRINGUP_UID builds (single-pilot bring-up).
//
// Algorithm:
//   SYNC_WAIT  — park on ch41 (ELRS sync channel), wait for any packet, read
//                8-byte OTA payload to extract fhssIndex + UID[4:5].
//   HOP_SCAN   — rotate through ch0..79, 8 ms dwell per hop for AUTO_SCAN_HOPS
//                hops; record every hop where a packet is actually received.
//   BRUTE_FORCE— try all 65536 (UID[2],UID[3]) combinations; keep only the
//                candidate(s) whose FHSS sequence predicts the observed channels
//                at the observed hop offsets.  One match → done.
//
// UID[0:1] are not part of the FHSS seed, so they are set to 0x00.
// gate_node pilots.cpp matches on uid[2:5] when uid[0:1] are both zero.
#ifndef BRINGUP_UID

enum AutoPhase { AUTO_SYNC_WAIT, AUTO_HOP_SCAN, AUTO_BRUTE_FORCE, AUTO_DONE };

struct AutoGotObs { uint16_t hopOffset; uint8_t channel; };

static struct {
    AutoPhase   phase;
    bool        syncParked;     // radio already tuned to sync ch — don't retune
    uint8_t     uid4, uid5;     // from OTA SYNC packet
    uint8_t     syncFhssIdx;    // fhssIndex from OTA SYNC packet
    AutoGotObs  obs[AUTO_MAX_GOT_OBS];
    uint8_t     obsCount;
    uint16_t    hopOffset;      // hops elapsed since SYNC capture
    uint8_t     scanChan;       // next channel to dwell on [0..79]
    uint32_t    dwellEnd;       // micros() deadline for current dwell (0 = new)
} s_auto;

static void autoReset() {
    memset(&s_auto, 0, sizeof(s_auto));
    // phase = AUTO_SYNC_WAIT (== 0), syncParked = false (== 0)
}

// Brute-force UID[2:3]: test all 65536 seeds against stored observations.
// Returns true and fills ident if exactly one candidate matches.
// Aborts early (returns false) if s_newProvision is set by ESP-NOW ISR.
static bool autoBruteForce() {
    if (s_auto.obsCount == 0) return false;

    uint8_t foundU2 = 0, foundU3 = 0;
    uint8_t matchCount = 0;

    for (uint32_t idx = 0; idx < 65536UL; idx++) {
        // Check for incoming ESP-NOW provision — abort if one arrived.
        if (s_newProvision) return false;

        uint8_t  u2   = (uint8_t)(idx >> 8);
        uint8_t  u3   = (uint8_t)(idx & 0xFF);
        uint32_t seed = ((uint32_t)u2   << 24)
                      | ((uint32_t)u3   << 16)
                      | ((uint32_t)s_auto.uid4  <<  8)
                      | ((uint32_t)(s_auto.uid5 ^ 3));

        bool ok = true;
        for (uint8_t o = 0; o < s_auto.obsCount; o++) {
            uint16_t hopIdx = (uint16_t)(
                ((uint32_t)s_auto.syncFhssIdx + 1u + s_auto.obs[o].hopOffset)
                % FHSS_SEQUENCE_LEN);
            if (fhssChannelFromSeed(seed, hopIdx) != s_auto.obs[o].channel) {
                ok = false; break;
            }
        }
        if (ok) { foundU2 = u2; foundU3 = u3; matchCount++; }

        if ((idx & 0xFFu) == 0xFFu) yield();   // feed WDT every 256 candidates
    }

    if (matchCount != 1) {
        Serial.printf("[gate_ep1] auto: %u candidates (need 1)\n",
                      (unsigned)matchCount);
        return false;
    }

    // Unique candidate found: UID[0:1] are unused by FHSS seed → set to 0x00.
    ident.uid[0] = 0x00; ident.uid[1] = 0x00;
    ident.uid[2] = foundU2; ident.uid[3] = foundU3;
    ident.uid[4] = s_auto.uid4;  ident.uid[5] = s_auto.uid5;
    ident.valid  = true;
    Serial.printf("[gate_ep1] auto-discovered uid=[00:00:%02X:%02X:%02X:%02X]\n",
                  foundU2, foundU3, s_auto.uid4, s_auto.uid5);
    return true;
}

// Advance the auto-discovery state machine by one step.
// Called from loop() inside the ST_PROVISION case when tryProvision() fails.
static void autoStep() {
    switch (s_auto.phase) {

    // ---- Phase 1: park on sync channel, wait for packet ----
    case AUTO_SYNC_WAIT:
        if (!s_auto.syncParked) {
            sxSetFrequencyHz(SYNC_FREQ_HZ);
            s_auto.syncParked = true;
            Serial.println(F("[gate_ep1] auto: parking sync ch41"));
        }
        if (sxPacketReceived()) {
            uint8_t buf[8] = {};
            sxReadPayload(buf, 8);
            s_auto.syncFhssIdx = buf[OTA_SYNC_FHSS_IDX_BYTE];
            s_auto.uid4        = buf[OTA_SYNC_UID4_BYTE];
            s_auto.uid5        = buf[OTA_SYNC_UID5_BYTE];
            s_auto.obsCount    = 0;
            s_auto.hopOffset   = 0;
            s_auto.scanChan    = 0;
            s_auto.dwellEnd    = 0;
            s_auto.phase       = AUTO_HOP_SCAN;
            Serial.printf("[gate_ep1] auto: sync fhssIdx=%u uid4=%02X uid5=%02X\n",
                          (unsigned)s_auto.syncFhssIdx, s_auto.uid4, s_auto.uid5);
        }
        break;

    // ---- Phase 2: scan all 80 channels in rotation, 8 ms per hop ----
    case AUTO_HOP_SCAN:
        // Start a new dwell when dwellEnd==0 (fresh channel).
        if (s_auto.dwellEnd == 0) {
            sxSetFrequencyHz(fhssFreqHz(s_auto.scanChan));
            s_auto.dwellEnd = micros() + (uint32_t)ELRS_SLOT_US * FHSS_HOP_INTERVAL;
        }

        if ((int32_t)(s_auto.dwellEnd - micros()) > 0) {
            if (sxPacketReceived() && s_auto.obsCount < AUTO_MAX_GOT_OBS) {
                s_auto.obs[s_auto.obsCount].hopOffset = s_auto.hopOffset;
                s_auto.obs[s_auto.obsCount].channel   = s_auto.scanChan;
                s_auto.obsCount++;
                Serial.printf("[gate_ep1] auto: hit %u  hop=%u ch=%u\n",
                              (unsigned)s_auto.obsCount,
                              (unsigned)s_auto.hopOffset,
                              (unsigned)s_auto.scanChan);
            }
            yield();
            return;   // still inside dwell window — come back next loop()
        }

        // Dwell complete: advance channel and hop counter.
        s_auto.dwellEnd = 0;
        s_auto.scanChan = (uint8_t)((s_auto.scanChan + 1u) % FHSS_CHANNEL_COUNT);
        s_auto.hopOffset++;

        if (s_auto.hopOffset >= AUTO_SCAN_HOPS) {
            s_auto.phase = AUTO_BRUTE_FORCE;
            Serial.printf("[gate_ep1] auto: scan done hits=%u -> brute-force\n",
                          (unsigned)s_auto.obsCount);
        }
        break;

    // ---- Phase 3: brute-force UID[2:3] ----
    case AUTO_BRUTE_FORCE:
        Serial.println(F("[gate_ep1] auto: brute-force..."));
        if (autoBruteForce()) {
            s_auto.phase = AUTO_DONE;
        } else {
            autoReset();   // restart SYNC hunt
            Serial.println(F("[gate_ep1] auto: retry"));
        }
        break;

    case AUTO_DONE:
        break;  // caller transitions to ST_SCAN
    }
}
#endif // !BRINGUP_UID

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
        hopIndex = 0; missStreak = 0;
        s_lqHead = 0; s_lqSum = 0;
        memset(s_lqBuf, 0, sizeof(s_lqBuf));
        state = ST_SCAN;
        Serial.println(F("[gate_ep1] -> SCAN"));
    } else {
        ident.valid = false;
        state = ST_PROVISION;
        Serial.println(F("[gate_ep1] UID cleared -> PROVISION"));
#ifndef BRINGUP_UID
        autoReset();
#endif
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
    Serial.println(F("[gate_ep1] awaiting UID (auto-discover / UART / ESP-NOW)"));
#endif
    if (!sxBegin())     Serial.println(F("[gate_ep1] SX1280 FAILED"));
    else                Serial.println(F("[gate_ep1] SX1280 OK"));
    if (!espnowBegin()) Serial.println(F("[gate_ep1] ESP-NOW FAILED"));
    else                Serial.println(F("[gate_ep1] ESP-NOW OK"));
    espnowSetProvisionCallback(onProvision);
    espnowSendBeacon(ident.uid, ident.valid, (uint8_t)state);
    s_lastBeaconMs = millis();
#ifndef BRINGUP_UID
    autoReset();
#endif
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
#ifndef BRINGUP_UID
            autoReset();
#endif
        }
#ifndef BRINGUP_UID
        else {
            // Auto-discovery: reads SYNC OTA packet, scans 240 hops,
            // brute-forces UID[2:3].  Aborted immediately if ESP-NOW
            // provision arrives (applyProvision() runs at top of loop()).
            autoStep();
            if (s_auto.phase == AUTO_DONE) {
                fhssGenerate(ident.uid);
                hopIndex   = 0;
                missStreak = 0;
                s_lqHead   = 0;
                s_lqSum    = 0;
                memset(s_lqBuf, 0, sizeof(s_lqBuf));
                state = ST_SCAN;
                Serial.println(F("[gate_ep1] auto-discover -> SCAN"));
                autoReset();
            }
        }
#endif
        break;

    case ST_SCAN: {
        sxSetFrequencyHz(fhssFreqHz(fhssChannelAt(hopIndex)));
        // Park on this channel for one dwell window and poll for any packet.
        // SCAN_DWELL_US is set longer than one packet interval so that, if the TX
        // is currently transmitting on this channel, we are guaranteed to see it.
        uint32_t dwellEnd = micros() + SCAN_DWELL_US;
        bool scanGot = false;
        while ((int32_t)(dwellEnd - micros()) > 0) {
            if (sxPacketReceived()) { scanGot = true; break; }
            yield();
        }

        if (scanGot) {
            // Stay on the channel we just caught — FOLLOW resumes tracking here.
            missStreak = 0;
            state = ST_FOLLOW;
            Serial.print(F("[gate_ep1] locked hop="));
            Serial.print(hopIndex);
            Serial.println(F(" -> FOLLOW"));
        } else {
            hopIndex = (hopIndex + 1) % FHSS_SEQUENCE_LEN;
        }
        break;
    }

    case ST_FOLLOW: {
        // Per-channel dwell model (robust at 500Hz where a slot is only 2ms and
        // precise per-packet slot prediction is too brittle): tune once to the
        // predicted channel, then listen in continuous RX. Count packets until we
        // have a full hop's worth, or a gap shows the TX has hopped on, then step
        // hopIndex to the next channel in the FHSS sequence.
        sxSetFrequencyHz(fhssFreqHz(fhssChannelAt(hopIndex)));

        uint32_t dwellStart = micros();
        uint32_t lastRxUs   = dwellStart;
        uint8_t  rxCount    = 0;
        const uint32_t hardCap = (uint32_t)ELRS_SLOT_US * (FHSS_HOP_INTERVAL + 2);
        const uint32_t gapUs   = (uint32_t)ELRS_SLOT_US * 2;   // 2 missed slots = hopped

        while (rxCount < FHSS_HOP_INTERVAL &&
               (uint32_t)(micros() - dwellStart) < hardCap) {
            if (sxPacketReceived()) {
                rxCount++;
                lastRxUs = micros();
                lqPush(true);
                int8_t   rssi = sxReadRssi();
                uint32_t now  = millis();
                if (now - lastReport >= RSSI_REPORT_MS) {
                    espnowSendRssi(ident.uid, rssi, lqPct(), now);
                    lastReport = now;
                }
            } else if (rxCount > 0 && (uint32_t)(micros() - lastRxUs) > gapUs) {
                break;   // had packets then a gap → TX moved to the next channel
            }
            yield();
        }

        if (rxCount > 0) {
            missStreak = 0;
        } else {
            lqPush(false);
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
