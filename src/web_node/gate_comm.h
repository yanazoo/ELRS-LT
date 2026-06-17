#pragma once
#include <Arduino.h>

void sendGateCmd(const char* action);
void sendGateCooldown();
void sendGateSdLogMode();
void sendGatePilot(int slot);
void sendGateThreshold(int slot);
void sendAllPilots();
void sendAllThresholds();
void processGateLine(const String& line);
void updateScanMac(const char* mac, int rssi);
void sendEp1ProvisionForSlot(int slot);
void sendAllEp1Provisions();

// Deferred (non-blocking) gate dumps: HTTP handlers set these and return; the
// actual UART streaming runs from loop() via runDeferredGateTasks().
extern volatile bool gReqRaceSave;
extern volatile bool gReqPilotsBackup;
void runDeferredGateTasks();
