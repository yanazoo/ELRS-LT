// sx1280_sniffer.h - minimal SX1280 control for passive RSSI sniffing
#pragma once
#include <stdint.h>

// Bring up SPI to the EP1's onboard SX1280 and put it in a known LoRa config
// matching ELRS (bandwidth, spreading factor, coding rate for the packet rate
// in use). Returns true on successful chip ID read.
bool sxBegin();

// Park the radio on a specific RF frequency (Hz) and enter continuous RX.
void sxSetFrequencyHz(uint32_t freqHz);

// Read instantaneous RSSI (dBm, negative). Returns the value cached by the
// last sxReadRssiNow() call.
int8_t sxReadRssi();

// Read the RSSI (dBm) of the most recently received packet directly from the
// radio (GetPacketStatus).  sxPacketReceived() no longer reads RSSI itself, to
// keep per-packet handling short, so call this for packets you actually report
// (i.e. telemetry uplink), right after sxReadPayload().
int8_t sxReadRssiNow();

// True if a packet was received since the last check (sync detection).
bool sxPacketReceived();

// Read the payload of the last received packet into buf (up to maxLen bytes).
// Returns the number of bytes actually copied.  Call after sxPacketReceived().
uint8_t sxReadPayload(uint8_t *buf, uint8_t maxLen);

// True when the SX1280 has been unresponsive (BUSY stuck HIGH) for long enough
// that it has hung and needs a hardware-reset recovery. Poll from the main loop.
bool sxNeedsRecovery();

// Hardware-reset the SX1280 and re-apply the LoRa configuration. Call when
// sxNeedsRecovery() returns true, then re-enter SCAN to re-acquire the link.
void sxRecover();
