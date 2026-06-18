// main.cpp - Gate EP1 DUAL sniffer entry point (ESP32-PICO-D4 + 2x SX1280).
//
// State machine: PROVISION -> SCAN -> FOLLOW, reporting telemetry RSSI over
// ESP-NOW (same wire protocol as the single-radio firmware).
//
// Two radios share one SPI bus and split the job that one radio had to
// time-share (see config.h for the full rationale):
//   Radio A (g_a) — SYNC/phase anchor.  Holds the current channel early in each
//                   dwell to capture SYNC (UID/phase/ratio), then moves to the
//                   NEXT channel for the final slot so it is already listening at
//                   the hop boundary.  Owns SCAN, auto-discovery and re-sync.
//   Radio B (g_b) — telemetry capture.  Stays on the current channel for the
//                   whole dwell and lingers past the hop boundary, so the rare
//                   telemetry slot (1 per 256 ms at 1:128) is never lost to a
//                   retune blind spot.  RSSI for lap timing is read here.
//
// Provisioning (priority order): BRINGUP_UID compile define / ESP-NOW from Gate
// Node / UART "UID AABBCCDDEEFF" / auto-discovery from OTA SYNC packets.

#include <Arduino.h>
#include "config.h"
#include "fhss.h"
#include "sx1280_radio.h"
#include "espnow_tx.h"
#include <string.h>

#if __has_include("secrets.h")
  #include "secrets.h"
#endif

// ---- Radios (shared SPI bus) ----
static SPIClass  g_spi(HSPI);
static SxRadio   g_a;   // sync / phase anchor
static SxRadio   g_b;   // telemetry capture

// ---- State machine ----
enum State { ST_PROVISION, ST_SCAN, ST_FOLLOW };
static State    state      = ST_PROVISION;
static uint16_t hopIndex   = 0;
static uint16_t missStreak = 0;
static uint32_t lastReport = 0;
static SnifferIdentity_t ident = { {0}, false };

// ---- FOLLOW hop-timing grid ----
static uint32_t s_nextHopUs = 0;   // absolute deadline to hop to next channel

// ---- Per-radio current channel (avoid redundant retunes / dwell-start gaps) ----
// 0xFF = unknown (force a retune).  Retuning a radio that is already on the
// target channel would re-enter standby and reopen a deaf window, so we only
// call sxSetFrequencyHz() when the channel actually changes.
static uint8_t s_chA = 0xFF;
static uint8_t s_chB = 0xFF;

static inline void tune(SxRadio &r, uint8_t &cur, uint8_t ch) {
    if (cur != ch) { sxSetFrequencyHz(r, fhssFreqHz(ch)); cur = ch; }
}

// ---- RSSI tracking (telemetry isolation) ----
static int8_t   s_tlmRssiMax     = RSSI_FLOOR_DBM;  // max telemetry RSSI this report interval
static uint8_t  s_tlmIntervalCnt = 0;               // telemetry packets this report interval
static int8_t   s_envRssi        = RSSI_FLOOR_DBM;  // envelope-follower output (reported)
static uint32_t s_lastUidSyncMs  = 0;               // last own-UID SYNC (UID gate)
static uint8_t  s_tlmDenom       = 0;               // telemetry ratio denominator from SYNC (diag, 0=unknown)

// SYNC byte[3] bits[3:1] index -> denominator (1:N).  0=off 1=1:128 .. 7=1:2
static const uint8_t TLM_DENOM_LUT[8] = { 0, 128, 64, 32, 16, 8, 4, 2 };

// ---- Diagnostics ----
static uint32_t s_tlmLogMs    = 0;
static uint16_t s_pktCount    = 0;
static uint16_t s_tlmPktCount = 0;
static uint16_t s_syncCount   = 0;
static uint16_t s_rawType[4]  = {0, 0, 0, 0};

// ---- ESP-NOW provision flag (set from network-task context) ----
static volatile bool s_newProvision = false;
static uint8_t       s_pendingUid[6] = {};

// ---- Link-quality: rolling window of LQ_WINDOW dwells ----
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

static void lqReset() { s_lqHead = 0; s_lqSum = 0; memset(s_lqBuf, 0, sizeof(s_lqBuf)); }

// ---- Auto-discovery (radio A only) ----
#ifndef BRINGUP_UID
enum AutoPhase { AUTO_SYNC_WAIT, AUTO_HOP_SCAN, AUTO_BRUTE_FORCE, AUTO_DONE };
struct AutoGotObs { uint16_t hopOffset; uint8_t channel; };
static struct {
    AutoPhase  phase;
    bool       syncParked;
    uint8_t    uid3, uid4, uid5hi, syncFhssIdx;
    AutoGotObs obs[AUTO_MAX_GOT_OBS];
    uint8_t    obsCount;
    uint16_t   hopOffset;
    uint8_t    scanChan;
    uint32_t   dwellEnd;
} s_auto;

static void autoReset() { memset(&s_auto, 0, sizeof(s_auto)); }

static bool autoBruteForce() {
    if (s_auto.obsCount == 0) return false;
    uint8_t foundU2 = 0, foundU5lo = 0, matchCount = 0;
    for (uint32_t idx = 0; idx < AUTO_CANDIDATE_COUNT; idx++) {
        if (s_newProvision) return false;
        uint8_t  uid2   = (uint8_t)(idx >> 6);
        uint8_t  uid5lo = (uint8_t)(idx & 0x3F);
        uint8_t  uid5   = s_auto.uid5hi | uid5lo;
        uint32_t seed   = ((uint32_t)uid2        << 24)
                        | ((uint32_t)s_auto.uid3 << 16)
                        | ((uint32_t)s_auto.uid4 <<  8)
                        | ((uint32_t)(uid5 ^ 3));
        bool ok = true;
        for (uint8_t o = 0; o < s_auto.obsCount; o++) {
            uint16_t hopIdx = (uint16_t)(
                ((uint32_t)s_auto.syncFhssIdx + 1u + s_auto.obs[o].hopOffset)
                % FHSS_SEQUENCE_LEN);
            if (fhssChannelFromSeed(seed, hopIdx) != s_auto.obs[o].channel) { ok = false; break; }
        }
        if (ok) { foundU2 = uid2; foundU5lo = uid5lo; matchCount++; }
        if ((idx & 0x7Fu) == 0x7Fu) yield();
    }
    if (matchCount != 1) {
        Serial.printf("[gate_ep1d] auto: %u candidates (need 1)\n", (unsigned)matchCount);
        return false;
    }
    uint8_t uid5 = s_auto.uid5hi | foundU5lo;
    ident.uid[0] = 0x00; ident.uid[1] = 0x00;
    ident.uid[2] = foundU2; ident.uid[3] = s_auto.uid3;
    ident.uid[4] = s_auto.uid4; ident.uid[5] = uid5;
    ident.valid = true;
    Serial.printf("[gate_ep1d] auto-discovered uid=[00:00:%02X:%02X:%02X:%02X]\n",
                  foundU2, s_auto.uid3, s_auto.uid4, uid5);
    return true;
}

static void autoStep() {
    switch (s_auto.phase) {
    case AUTO_SYNC_WAIT:
        if (!s_auto.syncParked) {
            sxSetFrequencyHz(g_a, SYNC_FREQ_HZ);
            s_auto.syncParked = true;
            Serial.println(F("[gate_ep1d] auto: parking sync ch41"));
        }
        if (sxPacketReceived(g_a)) {
            uint8_t buf[8] = {};
            sxReadPayload(g_a, buf, 8);
            if ((buf[0] & OTA_TYPE_MASK) != OTA_TYPE_SYNC) break;
            s_auto.syncFhssIdx = buf[OTA_SYNC_FHSS_BYTE];
            s_auto.uid3   = buf[OTA_SYNC_UID3_BYTE];
            s_auto.uid4   = buf[OTA_SYNC_UID4_BYTE];
            s_auto.uid5hi = buf[OTA_SYNC_UID5_BYTE] & OTA_SYNC_UID5_HIBITS;
            s_auto.obsCount = 0; s_auto.hopOffset = 0; s_auto.scanChan = 0; s_auto.dwellEnd = 0;
            s_auto.phase = AUTO_HOP_SCAN;
            Serial.printf("[gate_ep1d] auto: sync fhssIdx=%u uid3=%02X uid4=%02X uid5hi=%02X\n",
                          (unsigned)s_auto.syncFhssIdx, s_auto.uid3, s_auto.uid4, s_auto.uid5hi);
        }
        break;
    case AUTO_HOP_SCAN:
        if (s_auto.dwellEnd == 0) {
            sxSetFrequencyHz(g_a, fhssFreqHz(s_auto.scanChan));
            s_auto.dwellEnd = micros() + (uint32_t)ELRS_SLOT_US * FHSS_HOP_INTERVAL;
        }
        if ((int32_t)(s_auto.dwellEnd - micros()) > 0) {
            if (sxPacketReceived(g_a) && s_auto.obsCount < AUTO_MAX_GOT_OBS) {
                s_auto.obs[s_auto.obsCount].hopOffset = s_auto.hopOffset;
                s_auto.obs[s_auto.obsCount].channel   = s_auto.scanChan;
                s_auto.obsCount++;
                Serial.printf("[gate_ep1d] auto: hit %u hop=%u ch=%u\n",
                              (unsigned)s_auto.obsCount, (unsigned)s_auto.hopOffset,
                              (unsigned)s_auto.scanChan);
            }
            yield();
            return;
        }
        s_auto.dwellEnd = 0;
        s_auto.scanChan = (uint8_t)((s_auto.scanChan + 1u) % FHSS_CHANNEL_COUNT);
        s_auto.hopOffset++;
        if (s_auto.hopOffset >= AUTO_SCAN_HOPS) {
            s_auto.phase = AUTO_BRUTE_FORCE;
            Serial.printf("[gate_ep1d] auto: scan done hits=%u -> brute-force\n",
                          (unsigned)s_auto.obsCount);
        }
        break;
    case AUTO_BRUTE_FORCE:
        Serial.println(F("[gate_ep1d] auto: brute-force..."));
        if (autoBruteForce()) s_auto.phase = AUTO_DONE;
        else { autoReset(); Serial.println(F("[gate_ep1d] auto: retry")); }
        break;
    case AUTO_DONE: break;
    }
}
#endif // !BRINGUP_UID

// ---- Provisioning ----
static void onProvision(const uint8_t uid[6]) {
    memcpy(s_pendingUid, uid, 6);
    s_newProvision = true;
}

static void startTracking() {
    fhssGenerate(ident.uid);
    hopIndex = 0; missStreak = 0; s_nextHopUs = 0;
    s_chA = 0xFF; s_chB = 0xFF;
    lqReset();
    s_lastUidSyncMs = 0;   // require a fresh own-drone SYNC before reporting
    state = ST_SCAN;
    Serial.println(F("[gate_ep1d] -> SCAN"));
}

static void applyProvision() {
    if (!s_newProvision) return;
    s_newProvision = false;
    bool isZero = true;
    for (uint8_t i = 0; i < 6; i++) if (s_pendingUid[i]) { isZero = false; break; }
    if (!isZero) {
        memcpy(ident.uid, s_pendingUid, 6);
        ident.valid = true;
        Serial.print(F("[gate_ep1d] provisioned "));
        for (int i = 0; i < 6; i++) { if (i) Serial.print(':');
            if (ident.uid[i] < 0x10) Serial.print('0'); Serial.print(ident.uid[i], HEX); }
        Serial.println();
        startTracking();
    } else {
        ident.valid = false;
        state = ST_PROVISION;
        Serial.println(F("[gate_ep1d] UID cleared -> PROVISION"));
#ifndef BRINGUP_UID
        autoReset();
#endif
    }
}

static char    s_rxBuf[24];
static uint8_t s_rxLen = 0;

static bool parseHexUid(const char *s, uint8_t out[6]) {
    uint8_t nibbles = 0;
    for (; *s && nibbles < 12; ++s) {
        uint8_t n; char c = *s;
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
            s_rxBuf[s_rxLen] = '\0'; s_rxLen = 0;
            if (strncmp(s_rxBuf, "UID ", 4) == 0) {
                uint8_t tmp[6];
                if (parseHexUid(s_rxBuf + 4, tmp)) {
                    memcpy(ident.uid, tmp, 6); ident.valid = true;
                    Serial.print(F("[gate_ep1d] uid: "));
                    for (int i = 0; i < 6; i++) { if (i) Serial.print(':');
                        if (ident.uid[i] < 0x10) Serial.print('0'); Serial.print(ident.uid[i], HEX); }
                    Serial.println();
                    return true;
                }
                Serial.println(F("[gate_ep1d] bad UID format"));
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
        memcpy(ident.uid, kBup, 6); ident.valid = true;
        Serial.println(F("[gate_ep1d] uid from BRINGUP_UID"));
    }
    return true;
#else
    return tryProvisionUart();
#endif
}

// ---- Beacon + LED ----
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

// WS2812 RGB heartbeat (GPIO22). Same blink cadence as the single-radio EP1 but
// colour-coded by state: amber=PROVISION, blue=SCAN, green=FOLLOW. neopixelWrite()
// is a ~30 us RMT transfer, so we only push on a change, not every loop.
static void updateLedHeartbeat() {
    uint32_t t = millis();
    bool on;
    uint8_t r = 0, g = 0, b = 0;
    switch (state) {
    case ST_PROVISION: { uint32_t p = t % 2000; on = (p < 50);  r = 60; g = 30; b = 0; break; } // amber
    case ST_SCAN:      { uint32_t p = t % 200;  on = (p < 100); r = 0;  g = 0;  b = 80; break; } // blue
    case ST_FOLLOW:    { uint32_t p = t % 2000; on = (p < 80) || (p >= 160 && p < 240);
                         r = 0;  g = 70; b = 0; break; }                                          // green
    default: on = false;
    }
    uint8_t cr = on ? r : 0, cg = on ? g : 0, cb = on ? b : 0;
    static uint8_t lr = 1, lg = 1, lb = 1;   // force first write
    if (cr != lr || cg != lg || cb != lb) {
        rgbLedWrite(PIN_LED, cr, cg, cb);
        lr = cr; lg = cg; lb = cb;
    }
}

// ---- Packet handling (shared by both radios) ----
// phaseAnchor: true only when the radio is parked on the CURRENT dwell channel
// (so a SYNC's fhssIndex/nonce describes THIS dwell and may re-anchor the grid).
// During radio A's lead-ahead and radio B's tail-linger it is false: we still
// stamp the UID gate and read the ratio, but never re-anchor the grid from a
// SYNC that belongs to a different dwell.
//
// While both radios sit on chCur they hear the SAME packets, so per-second
// counters (pkts/tlm) roughly double — cosmetic only.  The RSSI side benefits:
// EP1 Dual is true diversity (two antennas), so taking the MAX of both radios'
// telemetry RSSI gives a real diversity gain on the reported lap-timing signal.
static void handlePacket(SxRadio &r, bool phaseAnchor) {
    uint8_t buf[8];
    sxReadPayload(r, buf, 1);            // type byte only in the hot path
    uint8_t t = buf[0] & OTA_TYPE_MASK;
    s_rawType[t]++; s_pktCount++;

    if (t == OTA_TYPE_TLM) {
        int8_t rssi = sxReadRssiNow(r);
        if (rssi > s_tlmRssiMax) s_tlmRssiMax = rssi;
        s_tlmPktCount++;
        if (s_tlmIntervalCnt < 255) s_tlmIntervalCnt++;
        return;
    }
    if (t == OTA_TYPE_SYNC) {
        s_syncCount++;
        sxReadPayload(r, buf, 8);        // need bytes 3-6 (ratio + UID)
        bool uidOk = buf[OTA_SYNC_UID3_BYTE] == ident.uid[3] &&
                     buf[OTA_SYNC_UID4_BYTE] == ident.uid[4] &&
                     ((buf[OTA_SYNC_UID5_BYTE] ^ ident.uid[5]) & OTA_SYNC_UID5_HIBITS) == 0;
        if (!uidOk) return;
        s_lastUidSyncMs = millis();      // UID gate: confirmed our drone
#if SYNC_PHASE_ALIGN
        if (phaseAnchor) {
            uint8_t syncFhss = buf[OTA_SYNC_FHSS_BYTE];
            uint8_t slot     = buf[OTA_SYNC_NONCE_BYTE] % FHSS_HOP_INTERVAL;
            hopIndex = (uint16_t)(syncFhss % FHSS_SEQUENCE_LEN);
            int32_t toNext = (int32_t)(FHSS_HOP_INTERVAL - slot) * ELRS_SLOT_US - PKT_AIRTIME_US;
            if (toNext < 0) toNext = 0;
            s_nextHopUs = micros() + (uint32_t)toNext;
        }
#else
        (void)phaseAnchor;
#endif
        uint8_t tlmIdx = (buf[OTA_SYNC_TLMRATIO_BYTE] >> OTA_SYNC_TLMRATIO_SHIFT) & OTA_SYNC_TLMRATIO_MASK;
        uint8_t denom  = TLM_DENOM_LUT[tlmIdx];
        if (denom >= 2) s_tlmDenom = denom;   // diagnostic: report the 1:N ratio
    }
}

// Report the envelope-followed telemetry RSSI over ESP-NOW, once per interval.
static void maybeReport(uint32_t now) {
    if ((now - lastReport) < RSSI_REPORT_MS) return;
    bool uidConfirmed = (s_lastUidSyncMs != 0) && (now - s_lastUidSyncMs) < UID_GATE_MS;
    int8_t tlmVal = (uidConfirmed && s_tlmIntervalCnt >= 1) ? s_tlmRssiMax : (int8_t)RSSI_FLOOR_DBM;
    if (tlmVal > s_envRssi) {
        s_envRssi = tlmVal;                          // rise instantly to telemetry
    } else {
        int16_t d = (int16_t)s_envRssi - ENV_DECAY_DB;
        if (d < tlmVal)         d = tlmVal;
        if (d < RSSI_FLOOR_DBM) d = RSSI_FLOOR_DBM;
        s_envRssi = (int8_t)d;
    }
    espnowSendRssi(ident.uid, s_envRssi, lqPct(), now);
    s_tlmRssiMax = (int8_t)RSSI_FLOOR_DBM;
    s_tlmIntervalCnt = 0;
    lastReport = now;
}

static void maybeDebugLog(uint32_t now) {
    if (now - s_tlmLogMs < 1000) return;
    const char *mode = (s_tlmPktCount > 0) ? "TLM" : "idle";
    Serial.printf("[gate_ep1d] pkts=%u sync=%u/s lq=%u mode=%s tlm/s=%u t0=%u t2=%u t3=%u ratio=1:%u\n",
                  (unsigned)s_pktCount, (unsigned)s_syncCount, (unsigned)lqPct(), mode,
                  (unsigned)s_tlmPktCount, (unsigned)s_rawType[0], (unsigned)s_rawType[2],
                  (unsigned)s_rawType[3], (unsigned)s_tlmDenom);
    s_pktCount = 0; s_tlmPktCount = 0; s_syncCount = 0;
    s_rawType[0] = s_rawType[1] = s_rawType[2] = s_rawType[3] = 0;
    s_tlmLogMs = now;
}

// ---- Arduino entry points ----
void setup() {
    Serial.begin(115200);
    delay(50);
    // LED self-test FIRST — before radios / ESP-NOW — so the WS2812 on GPIO22 is
    // proven even if a later init step misbehaves: a bright red→green→blue flash
    // on every boot. If you see NO flash at all, the unit is not running this code
    // (still boot-looping) or the LED is wired to a different pin.
    rgbLedWrite(PIN_LED, 120, 0, 0); delay(250);
    rgbLedWrite(PIN_LED, 0, 120, 0); delay(250);
    rgbLedWrite(PIN_LED, 0, 0, 120); delay(250);
    rgbLedWrite(PIN_LED, 0, 0, 0);
    Serial.println(F("[gate_ep1d] boot (ESP32 dual SX1280)"));
#ifndef BRINGUP_UID
    Serial.println(F("[gate_ep1d] awaiting UID (auto-discover / UART / ESP-NOW)"));
#endif

    sxBusBegin(&g_spi);
    bool okA = sxBegin(g_a, &g_spi, SXA_PIN_NSS, SXA_PIN_BUSY, SXA_PIN_DIO1, SXA_PIN_RST, "A");
    bool okB = sxBegin(g_b, &g_spi, SXB_PIN_NSS, SXB_PIN_BUSY, SXB_PIN_DIO1, SXB_PIN_RST, "B");
    Serial.printf("[gate_ep1d] radios: A=%s B=%s\n", okA ? "OK" : "FAIL", okB ? "OK" : "FAIL");

    if (!espnowBegin()) Serial.println(F("[gate_ep1d] ESP-NOW FAILED"));
    else                Serial.println(F("[gate_ep1d] ESP-NOW OK"));
    espnowSetProvisionCallback(onProvision);
    espnowSendBeacon(ident.uid, ident.valid, (uint8_t)state);
    s_lastBeaconMs = millis();
#ifndef BRINGUP_UID
    autoReset();
#endif
}

// ---- Per-radio hang recovery ----
#define SX_MAX_RECOVER 30
static uint16_t s_recoverCountA = 0;

static bool handleRecovery() {
    // Radio B: telemetry only — recover it in place, keep sync/state.
    if (sxNeedsRecovery(g_b)) {
        sxRecover(g_b);
        s_chB = 0xFF;   // unknown channel after reset -> force retune
        Serial.println(F("[gate_ep1d] radio B recovered"));
    }
    // Radio A: holds sync — its loss means we must re-acquire.
    if (sxNeedsRecovery(g_a)) {
        sxRecover(g_a);
        if (++s_recoverCountA >= SX_MAX_RECOVER) {
            Serial.println(F("[gate_ep1d] radio A unrecoverable -> reboot"));
            ESP.restart();
        }
        missStreak = 0; s_nextHopUs = 0; s_chA = 0xFF; s_chB = 0xFF; lqReset();
        if (ident.valid) { state = ST_SCAN; Serial.println(F("[gate_ep1d] A recovered -> SCAN")); }
        else {
            state = ST_PROVISION;
#ifndef BRINGUP_UID
            autoReset();
#endif
            Serial.println(F("[gate_ep1d] A recovered -> PROVISION"));
        }
        return true;   // skip this loop iteration's state work
    }
    return false;
}

void loop() {
    applyProvision();
    maybeSendBeacon();
    updateLedHeartbeat();
    if (handleRecovery()) return;

    switch (state) {

    case ST_PROVISION:
        if (tryProvision()) {
            startTracking();
#ifndef BRINGUP_UID
            autoReset();
#endif
        }
#ifndef BRINGUP_UID
        else {
            autoStep();
            if (s_auto.phase == AUTO_DONE) { startTracking(); autoReset(); }
        }
#endif
        break;

    case ST_SCAN: {
        uint8_t ch = fhssChannelAt(hopIndex);
        tune(g_a, s_chA, ch);
        tune(g_b, s_chB, ch);                    // both cover the candidate channel
        uint32_t dwellEnd = micros() + SCAN_DWELL_US;
        bool got = false;
        while ((int32_t)(dwellEnd - micros()) > 0) {
            if (sxPacketReceived(g_a) || sxPacketReceived(g_b)) { got = true; break; }
            yield();
        }
        if (got) {
            missStreak = 0; s_recoverCountA = 0;
            s_nextHopUs = 0;
            state = ST_FOLLOW;
            Serial.printf("[gate_ep1d] locked hop=%u -> FOLLOW\n", (unsigned)hopIndex);
        } else {
            hopIndex = (hopIndex + 1) % FHSS_SEQUENCE_LEN;
        }
        break;
    }

    case ST_FOLLOW: {
        uint32_t nowU = micros();
        if (s_nextHopUs == 0 ||
            (int32_t)(nowU - s_nextHopUs) > (int32_t)FHSS_HOP_PERIOD_US) {
            s_nextHopUs = nowU + FHSS_HOP_PERIOD_US;
        }

        uint8_t chCur  = fhssChannelAt(hopIndex);
        uint8_t chNext = fhssChannelAt(hopIndex + 1);

        // Both radios cover the current channel for the main dwell.  At steady
        // state both are already on chCur (handed over from the previous dwell),
        // so tune() is a no-op here and the dwell starts with no deaf window.
        tune(g_a, s_chA, chCur);
        tune(g_b, s_chB, chCur);

        bool gotAny      = false;
        bool aMovedAhead = false;
        while ((int32_t)(s_nextHopUs - micros()) > 0) {
            // Last slot: move the SYNC radio ahead to pre-listen the next channel.
            if (!aMovedAhead &&
                (int32_t)(s_nextHopUs - micros()) <= (int32_t)LEAD_AHEAD_US) {
                tune(g_a, s_chA, chNext);
                aMovedAhead = true;
            }
            if (sxPacketReceived(g_a)) { gotAny = true; handlePacket(g_a, !aMovedAhead); }
            if (sxPacketReceived(g_b)) { gotAny = true; handlePacket(g_b, true); }
            // No yield() inside the dwell: ESP-NOW runs on the other core, and
            // the dwell is <=8 ms; we yield by returning from loop() each dwell.
        }

        // Boundary crossed. Ensure A is on chNext (it is, unless the dwell was so
        // short it never reached the lead window). B lingers on the OLD chCur to
        // catch a telemetry slot straddling the boundary, then rejoins on chNext.
        tune(g_a, s_chA, chNext);
        uint32_t tailEnd = micros() + TAIL_LINGER_US;
        while ((int32_t)(tailEnd - micros()) > 0) {
            if (sxPacketReceived(g_b)) { gotAny = true; handlePacket(g_b, false); }
            if (sxPacketReceived(g_a)) { gotAny = true; handlePacket(g_a, false); }
        }
        tune(g_b, s_chB, chNext);

        lqPush(gotAny);
        if (gotAny) {
            missStreak = 0;
        } else if (++missStreak >= MISS_STREAK_RESYNC) {
            state = ST_SCAN; s_nextHopUs = 0;
            Serial.printf("[gate_ep1d] resync lq=%u\n", (unsigned)lqPct());
        }

        // Advance grid to the next channel/slot.
        s_nextHopUs += FHSS_HOP_PERIOD_US;
        hopIndex = (hopIndex + 1) % FHSS_SEQUENCE_LEN;

        uint32_t now = millis();
        maybeReport(now);
        maybeDebugLog(now);
        break;
    }
    }

    // Feed core-1's IDLE task once per iteration so the Task Watchdog does not
    // fire ("IDLE1 did not reset").  ESP32 yield() only switches to equal/higher
    // priority tasks, never the lower-priority idle task, so the continuous
    // busy-poll in SCAN/FOLLOW needs a real (1-tick) block here.  Between dwells
    // both radios sit in continuous RX, so this 1 ms costs no telemetry.
    delay(1);
}
