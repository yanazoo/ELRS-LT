// main.cpp - Gate EP1 sniffer entry point.
// State machine: PROVISION -> SCAN -> FOLLOW, reporting RSSI over ESP-NOW.
//
// Lap-timing RSSI is isolated to the drone's telemetry uplink: while a SYNC has
// recently anchored the nonce phase we report only TLM-slot RSSI (TX downlink
// excluded); otherwise we fall back to max-RSSI-of-all-packets.  The TX downlink
// (RC/MSP/SYNC) is from the stationary handset and is used only to keep FHSS lock.
//
// Provisioning (pick one, in priority order):
//   A. Compile-time: define BRINGUP_UID in secrets.h (for solo bring-up).
//   B. Runtime ESP-NOW: Gate Node unicasts GateProvisionPacket_t on pilot assign.
//   C. Runtime UART: send "UID AABBCCDDEEFF\n" over the serial monitor.
//   D. Auto-discovery: EP1 parks on ELRS sync channel, reads OTA SYNC packets
//      to extract fhssIndex+UID[3:4]+UID[5]hi, then brute-forces the remaining
//      UID[2]+UID[5]lo by matching observed channel activity to candidates.
//
// This EP1 sends a GateEP1BeaconPacket_t every BEACON_INTERVAL_MS so the
// Gate Node can discover its MAC and relay it to the Web UI for assignment.
//
// Auto-discovery reads an ELRS OTA SYNC packet (type 0x02 in byte[0] bits[1:0])
// to learn fhssIndex (byte[1]), UID[3] (byte[4]), UID[4] (byte[5]), and the top
// 2 bits of UID[5] (byte[6] bits[7:6]).  Unknown fields: UID[2] (256 values) ×
// UID[5] lower 6 bits (64 values) = 16,384 brute-force candidates.
// FHSS seed = (UID[2]<<24)|(UID[3]<<16)|(UID[4]<<8)|(UID[5]^3).
// UID[0:1] are always 0x00 in ELRS and are not part of the seed.

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

// ---- RSSI tracking for ESP-NOW reporting (telemetry isolation) ----
// In ELRS 3.6.3 the drone's telemetry uplink is OTA type 0b11 (PACKET_TYPE_TLM);
// the TX only sends RC=0b00 / MSP=0b01 / SYNC=0b10.  So type 0b11 alone is the
// drone, independent of TX level/movement.  We report the latest telemetry RSSI
// and HOLD it until no telemetry has been heard for an adaptive silence timeout,
// then drop to the floor.  The timeout scales with the telemetry ratio (read from
// the SYNC packet): low ratios get a short, responsive timeout; high/sparse ratios
// (down to 1:128 = a sample every 256 ms) get a longer one so the trace does not
// flicker to the floor between samples.  A single telemetry packet is enough to
// refresh the hold (no per-interval minimum).
static int8_t   s_tlmRssiMax  = RSSI_FLOOR_DBM;  // max telemetry RSSI this report interval
static uint8_t  s_tlmIntervalCnt = 0;  // telemetry packets this report interval
static uint32_t s_silenceMs   = TLM_SILENCE_DEFAULT_MS;  // adaptive hold (diag/log only)
static int8_t   s_nearHoldRssi = RSSI_FLOOR_DBM; // held near-cluster peak (lap signal)
static uint32_t s_lastNearMs   = 0;    // last report interval with a near-cluster peak
static uint32_t s_tlmLogMs    = 0;     // 1 Hz serial debug timer
static uint16_t s_pktCount    = 0;     // all packets since last debug log
static uint16_t s_tlmPktCount = 0;     // TLM packets since last debug log (1 Hz)
static uint16_t s_syncCount   = 0;     // SYNC packets since last log
static uint8_t  s_tlmDenom    = 0;     // telemetry ratio denominator from SYNC (0=unknown)
// SYNC byte[3] bits[3:1] index -> denominator (1:N).  0=off 1=1:128 .. 7=1:2
static const uint8_t TLM_DENOM_LUT[8] = { 0, 128, 64, 32, 16, 8, 4, 2 };

// ---- Raw diagnostics (independent of classification) ----
static uint16_t s_rawType[4]   = {0, 0, 0, 0};
static uint8_t  s_maxPerDwell  = 0;
static int8_t   s_rawRssiMax   = RSSI_FLOOR_DBM;

// ---- TX-background notch (SYNC-free RX isolation) ----
// The stationary TX shows up as a constant, dominant RSSI cluster; its level is
// the mode of the per-window RSSI histogram (s_txBg).  Packets that deviate from
// that level by more than TXBG_GUARD_DB are the drone (RX): stronger = near pass,
// weaker = far/fast pass below the TX floor.  We report the drone level with the
// TX suppressed, so the trace baseline is the noise floor instead of the TX.
static uint16_t s_rssiHist[128];                 // RSSI histogram (index = -dBm)
static int8_t   s_txBg        = RSSI_FLOOR_DBM;  // estimated constant TX level
static bool     s_txBgValid   = false;           // true once enough traffic seen
static uint32_t s_txBgMs      = 0;               // last txBg recompute (millis)
static int8_t   s_rxNearMax   = RSSI_FLOOR_DBM;  // max above-TX RSSI this report interval
static uint8_t  s_rxNearCnt   = 0;               // above-TX packets this report interval
static int8_t   s_rxFarMax    = RSSI_FLOOR_DBM;  // max below-TX RSSI this report interval
static uint8_t  s_rxFarCnt    = 0;               // below-TX packets this report interval
// 1 Hz diagnostics for the notch
static int8_t   s_dbgNearMax  = RSSI_FLOOR_DBM;
static int8_t   s_dbgFarMax   = RSSI_FLOOR_DBM;
static uint16_t s_dbgNearCnt  = 0;
static uint16_t s_dbgFarCnt   = 0;

// ---- FOLLOW hop-timing grid (slot-phase locked to the TX) ----
static uint32_t s_nextHopUs    = 0;    // absolute deadline to hop to next channel
static uint32_t s_dwellStartUs = 0;    // micros() recorded just before sxSetFrequencyHz

// ---- ESP-NOW provision flag (set from network-task ISR context) ----
// Declared here so auto-discovery can abort early when a provision arrives.
static volatile bool s_newProvision = false;
static uint8_t       s_pendingUid[6] = {};

// ---- Auto-discovery state ----
// Active only in ST_PROVISION when no UID is available from standard sources.
// Not compiled in BRINGUP_UID builds (single-pilot bring-up).
//
// Algorithm:
//   SYNC_WAIT  — park on ch41 (ELRS sync channel), read OTA SYNC packets
//                (byte[0]&0x03==0x02) to extract fhssIndex + UID[3:4] + UID[5]hi.
//   HOP_SCAN   — rotate through ch0..79, 8 ms dwell per hop for AUTO_SCAN_HOPS
//                hops; record every hop where a packet is actually received.
//   BRUTE_FORCE— try all 16384 (UID[2] × UID[5]lo) combinations; keep only the
//                candidate(s) whose FHSS sequence predicts the observed channels
//                at the observed hop offsets.  One match → done.
//
// UID[0:1] are always 0x00 in ELRS and are not part of the FHSS seed.
// gate_node pilots.cpp matches on uid[2:5] when uid[0:1] are both zero.
#ifndef BRINGUP_UID

enum AutoPhase { AUTO_SYNC_WAIT, AUTO_HOP_SCAN, AUTO_BRUTE_FORCE, AUTO_DONE };

struct AutoGotObs { uint16_t hopOffset; uint8_t channel; };

static struct {
    AutoPhase   phase;
    bool        syncParked;     // radio already tuned to sync ch — don't retune
    uint8_t     uid3;           // UID[3] from OTA SYNC byte[4]
    uint8_t     uid4;           // UID[4] from OTA SYNC byte[5]
    uint8_t     uid5hi;         // UID[5] bits[7:6] from OTA SYNC byte[6]&0xC0
    uint8_t     syncFhssIdx;    // fhssIndex from OTA SYNC byte[1]
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

// Brute-force UID[2] and UID[5] lower 6 bits: test all 16384 seeds against
// stored observations.  Returns true and fills ident if exactly one candidate
// matches.  Aborts early (returns false) if s_newProvision is set by ESP-NOW ISR.
static bool autoBruteForce() {
    if (s_auto.obsCount == 0) return false;

    uint8_t foundU2 = 0, foundU5lo = 0;
    uint8_t matchCount = 0;

    for (uint32_t idx = 0; idx < AUTO_CANDIDATE_COUNT; idx++) {
        // Check for incoming ESP-NOW provision — abort if one arrived.
        if (s_newProvision) return false;

        uint8_t  uid2   = (uint8_t)(idx >> 6);
        uint8_t  uid5lo = (uint8_t)(idx & 0x3F);
        uint8_t  uid5   = s_auto.uid5hi | uid5lo;
        uint32_t seed   = ((uint32_t)uid2          << 24)
                        | ((uint32_t)s_auto.uid3   << 16)
                        | ((uint32_t)s_auto.uid4   <<  8)
                        | ((uint32_t)(uid5 ^ 3));

        bool ok = true;
        for (uint8_t o = 0; o < s_auto.obsCount; o++) {
            uint16_t hopIdx = (uint16_t)(
                ((uint32_t)s_auto.syncFhssIdx + 1u + s_auto.obs[o].hopOffset)
                % FHSS_SEQUENCE_LEN);
            if (fhssChannelFromSeed(seed, hopIdx) != s_auto.obs[o].channel) {
                ok = false; break;
            }
        }
        if (ok) { foundU2 = uid2; foundU5lo = uid5lo; matchCount++; }

        if ((idx & 0x7Fu) == 0x7Fu) yield();   // feed WDT every 128 candidates
    }

    if (matchCount != 1) {
        Serial.printf("[gate_ep1] auto: %u candidates (need 1)\n",
                      (unsigned)matchCount);
        return false;
    }

    // Unique candidate found: UID[0:1] are unused by FHSS seed → set to 0x00.
    uint8_t uid5 = s_auto.uid5hi | foundU5lo;
    ident.uid[0] = 0x00; ident.uid[1] = 0x00;
    ident.uid[2] = foundU2;
    ident.uid[3] = s_auto.uid3;
    ident.uid[4] = s_auto.uid4;
    ident.uid[5] = uid5;
    ident.valid  = true;
    Serial.printf("[gate_ep1] auto-discovered uid=[00:00:%02X:%02X:%02X:%02X]\n",
                  foundU2, s_auto.uid3, s_auto.uid4, uid5);
    return true;
}

// Advance the auto-discovery state machine by one step.
// Called from loop() inside the ST_PROVISION case when tryProvision() fails.
static void autoStep() {
    switch (s_auto.phase) {

    // ---- Phase 1: park on sync channel, wait for ELRS SYNC packet ----
    case AUTO_SYNC_WAIT:
        if (!s_auto.syncParked) {
            sxSetFrequencyHz(SYNC_FREQ_HZ);
            s_auto.syncParked = true;
            Serial.println(F("[gate_ep1] auto: parking sync ch41"));
        }
        if (sxPacketReceived()) {
            uint8_t buf[8] = {};
            sxReadPayload(buf, 8);
            // Only accept ELRS SYNC packets: byte[0] bits[1:0] == 0x02.
            if ((buf[0] & OTA_TYPE_MASK) != OTA_TYPE_SYNC) break;
            s_auto.syncFhssIdx = buf[OTA_SYNC_FHSS_BYTE];                    // byte[1]
            s_auto.uid3        = buf[OTA_SYNC_UID3_BYTE];                    // byte[4]
            s_auto.uid4        = buf[OTA_SYNC_UID4_BYTE];                    // byte[5]
            s_auto.uid5hi      = buf[OTA_SYNC_UID5_BYTE] & OTA_SYNC_UID5_HIBITS; // byte[6]&0xC0
            s_auto.obsCount    = 0;
            s_auto.hopOffset   = 0;
            s_auto.scanChan    = 0;
            s_auto.dwellEnd    = 0;
            s_auto.phase       = AUTO_HOP_SCAN;
            Serial.printf("[gate_ep1] auto: sync fhssIdx=%u uid3=%02X uid4=%02X uid5hi=%02X\n",
                          (unsigned)s_auto.syncFhssIdx,
                          s_auto.uid3, s_auto.uid4, s_auto.uid5hi);
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
            // Auto-discovery: reads OTA SYNC packet for UID[3:4]+UID[5]hi,
            // scans AUTO_SCAN_HOPS hops, brute-forces UID[2]+UID[5]lo.
            // Aborted immediately if ESP-NOW provision arrives.
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
        // Slot-phase-locked dwell.  We hop on an ABSOLUTE micros() grid
        // (s_nextHopUs) instead of by counting packets, so the EP1 stays on each
        // channel for the TX's full dwell and catches every interleaved slot —
        // including the telemetry (uplink) slots, not just the downlink slots.
        //
        // A SYNC packet carries fhssIndex + nonce.  nonce % FHSShopInterval is
        // the slot index within the current dwell, so it tells us exactly how
        // much time is left on this channel: we re-anchor s_nextHopUs (phase)
        // and hopIndex (sequence position) from every SYNC.  Between SYNCs the
        // grid free-runs; crystal drift over a sync interval is negligible.
        //
        // RSSI for lap timing is reported ONLY for TLM (the drone's uplink).
        uint32_t nowU = micros();
        // First entry, or grid fell badly behind (e.g. after a stall): re-seat it.
        if (s_nextHopUs == 0 ||
            (int32_t)(nowU - s_nextHopUs) > (int32_t)FHSS_HOP_PERIOD_US) {
            s_nextHopUs = nowU + FHSS_HOP_PERIOD_US;
        }

        s_dwellStartUs = micros();
        sxSetFrequencyHz(fhssFreqHz(fhssChannelAt(hopIndex)));

        bool    gotAny    = false;
        uint8_t dwellPkts = 0;     // packets caught this dwell (continuous-Rx check)
        while ((int32_t)(s_nextHopUs - micros()) > 0) {
            if (sxPacketReceived()) {
                gotAny = true;
                dwellPkts++;
                lqPush(true);
                uint8_t buf[8];
                // Read ONLY the type byte in the hot path.  Reading the full 8
                // bytes every packet (plus the per-second raw dump) overran the
                // dwell and cost FHSS lock; type (low 2 bits of byte 0) is all we
                // need to isolate the drone's telemetry.
                sxReadPayload(buf, 1);
                uint8_t pktType = buf[0] & OTA_TYPE_MASK;

                // Read RSSI once per packet.  s_tlmRssiMax accumulates only the
                // TLM (type 0b11 = drone) packets, which is what we report.
                int8_t rssiNow = sxReadRssiNow();
                s_rawType[pktType]++;
                if (rssiNow > s_rawRssiMax)  s_rawRssiMax  = rssiNow;
                s_pktCount++;

                // ELRS 3.6.3: the drone's telemetry uplink is OTA type 0b11
                // (PACKET_TYPE_TLM).  The TX only ever sends RC=0b00 / MSP=0b01 /
                // SYNC=0b10, so type 0b11 alone identifies the drone — independent
                // of TX level or movement.  This is the primary separation.
                if (pktType == OTA_TYPE_TLM) {
                    if (rssiNow > s_tlmRssiMax) s_tlmRssiMax = rssiNow;
                    s_tlmPktCount++;
                    if (s_tlmIntervalCnt < 255) s_tlmIntervalCnt++;
                }

                // TX-background notch (Tier-2 fallback): histogram every packet
                // (mode = constant TX level) and split packets that deviate from
                // the current TX level into the drone's near (above) / far (below)
                // clusters.
                int idx = -(int)rssiNow;
                if (idx < 0)   idx = 0;
                if (idx > 127) idx = 127;
                s_rssiHist[idx]++;
                if (s_txBgValid) {
                    if (rssiNow > (int)s_txBg + TXBG_GUARD_DB) {
                        if (rssiNow > s_rxNearMax) s_rxNearMax = rssiNow;
                        if (s_rxNearCnt < 255) s_rxNearCnt++;
                    } else if (rssiNow < (int)s_txBg - TXBG_GUARD_DB) {
                        if (rssiNow > s_rxFarMax) s_rxFarMax = rssiNow;
                        if (s_rxFarCnt < 255) s_rxFarCnt++;
                    }
                }

                // SYNC: read the telemetry ratio to size the silence timeout.  We
                // deliberately do NOT re-anchor the hop grid (hopIndex/s_nextHopUs)
                // — hardware showed that breaks FHSS lock; the free-running grid +
                // SCAN re-lock is solid.  SYNC is rare (1-5/s) so the extra full-
                // payload read here does not load the capture loop.
                if (pktType == OTA_TYPE_SYNC) {
                    s_syncCount++;
                    sxReadPayload(buf, 8);   // need bytes 3-6 (ratio + UID)
                    bool uidOk = buf[OTA_SYNC_UID3_BYTE] == ident.uid[3] &&
                                 buf[OTA_SYNC_UID4_BYTE] == ident.uid[4] &&
                                 ((buf[OTA_SYNC_UID5_BYTE] ^ ident.uid[5]) &
                                  OTA_SYNC_UID5_HIBITS) == 0;
                    if (uidOk) {
                        uint8_t tlmIdx = (buf[OTA_SYNC_TLMRATIO_BYTE] >> OTA_SYNC_TLMRATIO_SHIFT)
                                         & OTA_SYNC_TLMRATIO_MASK;
                        uint8_t denom = TLM_DENOM_LUT[tlmIdx];
                        if (denom >= 2) {
                            s_tlmDenom = denom;
                            uint32_t to = ((uint32_t)denom * ELRS_SLOT_US / 1000)
                                          * TLM_SILENCE_MARGIN;
                            if (to < TLM_SILENCE_MIN_MS) to = TLM_SILENCE_MIN_MS;
                            if (to > TLM_SILENCE_MAX_MS) to = TLM_SILENCE_MAX_MS;
                            s_silenceMs = to;
                        }
                    }
                }
            }
            // NOTE: deliberately no yield() inside the dwell.  On the ESP8285
            // yield() services the WiFi/ESP-NOW stack and can stall for ~1-2 ms
            // — about one 500Hz slot — which made this loop skip every other
            // slot and lock onto a single direction (downlink OR telemetry).
            // The dwell is ≤8 ms and we yield between dwells (loop() returns),
            // which feeds the soft WDT and lets ESP-NOW drain with ~8 ms latency.
        }

        if (dwellPkts > s_maxPerDwell) s_maxPerDwell = dwellPkts;

        if (gotAny) {
            missStreak = 0;
        } else {
            lqPush(false);
            if (++missStreak >= MISS_STREAK_RESYNC) {
                state = ST_SCAN;
                s_nextHopUs  = 0;    // force grid re-seat on next lock
                Serial.print(F("[gate_ep1] resync lq="));
                Serial.println(lqPct());
            }
        }

        // Advance to the next channel and the next grid slot.
        s_nextHopUs += FHSS_HOP_PERIOD_US;
        hopIndex = (hopIndex + 1) % FHSS_SEQUENCE_LEN;

        // Recompute the TX-background level (mode of the RSSI histogram) on its
        // own cadence.  The constant TX is the dominant cluster even while the
        // drone is present, so the mode tracks it robustly.  Requires a minimum
        // packet count so a sparse/lock-loss window does not produce a bogus txBg.
        uint32_t now = millis();
        if (now - s_txBgMs >= TXBG_WINDOW_MS) {
            uint16_t total = 0, best = 0; int bestIdx = -1;
            for (int k = 0; k < 128; k++) {
                total += s_rssiHist[k];
                if (s_rssiHist[k] > best) { best = s_rssiHist[k]; bestIdx = k; }
            }
            s_txBgValid = (total >= TXBG_MIN_PKTS && bestIdx >= 0);
            if (s_txBgValid) s_txBg = (int8_t)(-bestIdx);
            memset(s_rssiHist, 0, sizeof(s_rssiHist));
            s_txBgMs = now;
        }

        // Report RSSI over ESP-NOW once per interval, OUTSIDE the capture loop
        // so no WiFi TX competes with slot catching.
        if ((now - lastReport) >= RSSI_REPORT_MS) {
            // Lap-timing RSSI = the per-interval PEAK of the txBg-notch NEAR
            // cluster: packets on this drone's FHSS sequence whose RSSI rose
            // above the constant TX downlink background as the drone approached
            // the gate.  This is dense (tens of packets per interval at the
            // gate) so the trace is smooth, and it is 0/floor when the drone is
            // away (rxNear=0), giving a clean enter/exit for lap detection.
            int8_t reportRssi;
            if (s_txBgValid && s_rxNearCnt >= TXBG_RX_MIN_PKTS) {
                s_nearHoldRssi = s_rxNearMax;   // newest drone-pass peak
                s_lastNearMs   = now;
                reportRssi     = s_rxNearMax;
            } else if (s_lastNearMs != 0 && (now - s_lastNearMs) < NEAR_HOLD_MS) {
                reportRssi = s_nearHoldRssi;    // bridge brief sampling gaps
            } else {
                // Drone away (no packets above background): clean floor so the
                // exit threshold fires.  No telemetry fallback — a stray far
                // telemetry sample could sit above a tight enter threshold and
                // false-trigger a lap.
                reportRssi     = (int8_t)RSSI_FLOOR_DBM;
                s_nearHoldRssi = (int8_t)RSSI_FLOOR_DBM;
            }
            espnowSendRssi(ident.uid, reportRssi, lqPct(), now);
            // Roll up 1 Hz diagnostics before clearing the interval accumulators.
            if (s_rxNearMax > s_dbgNearMax) s_dbgNearMax = s_rxNearMax;
            if (s_rxFarMax  > s_dbgFarMax)  s_dbgFarMax  = s_rxFarMax;
            s_dbgNearCnt += s_rxNearCnt;
            s_dbgFarCnt  += s_rxFarCnt;
            s_tlmRssiMax = (int8_t)RSSI_FLOOR_DBM; s_tlmIntervalCnt = 0;
            s_rxNearMax  = (int8_t)RSSI_FLOOR_DBM; s_rxNearCnt = 0;
            s_rxFarMax   = (int8_t)RSSI_FLOOR_DBM; s_rxFarCnt  = 0;
            lastReport = now;
        }

        // 1 Hz serial debug.
        //   pkts   = total packets/s (TX downlink + drone uplink)
        //   sync   = TX SYNC packets/s (intermittent on a linked TX; 0 = no nonce)
        //   rmax   = strongest packet this second
        //   mode   = TLM (type-0b11 drone) / RXBG (TX-notch) / ALL (max-of-all)
        //   txBg   = estimated constant TX level (the cluster being suppressed)
        //   rxNear = count@maxRSSI of above-TX (near drone) packets this second
        //   rxFar  = count@maxRSSI of below-TX (far drone) packets this second
        if (now - s_tlmLogMs >= 1000) {
            const char *mode = (s_tlmPktCount > 0) ? "TLM" : "idle";
            Serial.printf("[gate_ep1] pkts=%u sync=%u/s rmax=%d lq=%u mode=%s txBg=%d%s\n",
                          (unsigned)s_pktCount, (unsigned)s_syncCount,
                          (int)s_rawRssiMax, (unsigned)lqPct(), mode,
                          (int)s_txBg, s_txBgValid ? "" : "(?)");
            Serial.printf("[gate_ep1]   rxNear=%u@%d rxFar=%u@%d tlm/s=%u t0=%u t2=%u t3=%u ratio=1:%u hold=%ums\n",
                          (unsigned)s_dbgNearCnt, (int)s_dbgNearMax,
                          (unsigned)s_dbgFarCnt,  (int)s_dbgFarMax,
                          (unsigned)s_tlmPktCount,
                          (unsigned)s_rawType[0], (unsigned)s_rawType[2],
                          (unsigned)s_rawType[3],
                          (unsigned)s_tlmDenom, (unsigned)s_silenceMs);
            s_pktCount   = 0;
            s_tlmPktCount = 0;
            s_syncCount  = 0;
            s_rawType[0] = s_rawType[1] = s_rawType[2] = s_rawType[3] = 0;
            s_maxPerDwell = 0;
            s_rawRssiMax  = RSSI_FLOOR_DBM;
            s_dbgNearMax  = RSSI_FLOOR_DBM; s_dbgNearCnt = 0;
            s_dbgFarMax   = RSSI_FLOOR_DBM; s_dbgFarCnt  = 0;
            s_tlmLogMs   = now;
        }

        break;
    }
    }
}
