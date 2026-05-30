#include <Arduino.h>
#include <string.h>
#include "pilots.h"
#include "config.h"

PilotState pilots[MAX_PILOTS];

void initPilots() {
    for (int i = 0; i < MAX_PILOTS; i++) {
        memset(pilots[i].uid, 0, 6);
        pilots[i].hasUid         = false;
        pilots[i].name[0]        = '\0';
        pilots[i].emaRssi        = -120.0f;
        pilots[i].rawRssi        = -120;
        pilots[i].crossing       = false;
        pilots[i].peakRssi       = -120;
        pilots[i].peakTime       = 0;
        pilots[i].lastPeakTime   = 0;
        pilots[i].lastLapTime    = 0;
        pilots[i].lapCount       = 0;
        pilots[i].entryThreshold = DEFAULT_ENTRY_THR;
        pilots[i].exitThreshold  = DEFAULT_EXIT_THR;
    }
}

void resetPilots() {
    for (int i = 0; i < MAX_PILOTS; i++) {
        pilots[i].crossing      = false;
        pilots[i].peakRssi      = -120;
        pilots[i].peakTime      = 0;
        pilots[i].lastPeakTime  = 0;
        pilots[i].lastLapTime   = 0;
        pilots[i].lapCount      = 0;
        pilots[i].emaRssi       = -120.0f;
        pilots[i].rawRssi       = -120;
    }
    Serial.println("[Gate] Pilot state reset");
}

int findPilotByUID(const uint8_t* uid) {
    // Full 6-byte match (normal provisioned case).
    for (int i = 0; i < MAX_PILOTS; i++) {
        if (pilots[i].hasUid && memcmp(pilots[i].uid, uid, 6) == 0) return i;
    }
    // Auto-discovered EP1 sets uid[0:1]=0x00 (unused by FHSS seed).
    // Fall back to matching on uid[2:5] (the bytes that determine FHSS).
    if (uid[0] == 0 && uid[1] == 0) {
        for (int i = 0; i < MAX_PILOTS; i++) {
            if (pilots[i].hasUid && memcmp(pilots[i].uid + 2, uid + 2, 4) == 0) return i;
        }
    }
    return -1;
}

bool anyPilotRegistered() {
    for (int i = 0; i < MAX_PILOTS; i++) if (pilots[i].hasUid) return true;
    return false;
}

void uidToStr(const uint8_t* uid, char* buf) {
    snprintf(buf, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
}

// Unknown UIDs (sniffers not yet registered as pilots): rate-limit and forward
// to web_node as {"type":"scan","mac":"..."} — "mac" key kept for web_node compat.
static struct ScanEntry { uint8_t uid[6]; uint32_t lastSent; } scanTable[MAX_SCAN_MACS];
static int scanCount = 0;

void reportScanUid(const uint8_t* uid, int8_t rssi) {
    uint32_t now = millis();
    int slot = -1;
    for (int k = 0; k < scanCount; k++) {
        if (memcmp(scanTable[k].uid, uid, 6) == 0) { slot = k; break; }
    }
    if (slot < 0) {
        if (scanCount >= MAX_SCAN_MACS) return;
        slot = scanCount++;
        memcpy(scanTable[slot].uid, uid, 6);
        scanTable[slot].lastSent = 0;
    }
    if (now - scanTable[slot].lastSent < SCAN_INTERVAL_MS) return;
    scanTable[slot].lastSent = now;

    char uidStr[18];
    uidToStr(uid, uidStr);
    char buf[96];
    // Field is still "mac" because web_node gate_comm.cpp reads doc["mac"] for
    // scan messages — do not change until Step 8 (web UI UID rename).
    snprintf(buf, sizeof(buf),
             R"({"type":"scan","mac":"%s","rssi":%d,"ts":%lu})",
             uidStr, (int)rssi, (unsigned long)now);
    Serial1.println(buf);
    Serial.printf("[Gate] SCAN uid=%s rssi=%d\n", uidStr, (int)rssi);
}

void resetScanTimers() {
    for (int k = 0; k < scanCount; k++) scanTable[k].lastSent = 0;
}
