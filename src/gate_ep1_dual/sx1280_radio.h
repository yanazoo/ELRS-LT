// sx1280_radio.h - instance-based SX1280 control for passive ELRS sniffing.
//
// Unlike the single-radio firmware (module-static state, fixed #define pins),
// this driver is an object so two SX1280 radios can share one SPI bus on the
// EP1 Dual.  Each radio carries its own NSS/BUSY/DIO1/RST and cached state; the
// shared SPIClass is passed in at init.  All access is single-threaded from the
// main loop, so per-call SPI transactions need no extra locking.
#pragma once
#include <stdint.h>
#include <SPI.h>

struct SxRadio {
    SPIClass *spi;          // shared bus
    uint8_t   nss, busy, dio1, rst;
    const char *tag;        // "A"/"B" for logs
    uint16_t  busyStuckCount;
};

// Initialise the shared SPI bus.  Call ONCE before sxBegin() on either radio.
void sxBusBegin(SPIClass *spi);

// Reset + configure one radio.  Returns true if its firmware-version register
// reads back healthy.  Pins/tag are stored in r.
bool sxBegin(SxRadio &r, SPIClass *spi,
             uint8_t nss, uint8_t busy, uint8_t dio1, uint8_t rst,
             const char *tag);

// Park this radio on an RF frequency (Hz) and enter continuous RX.
void sxSetFrequencyHz(SxRadio &r, uint32_t freqHz);

// True if a (CRC-valid) packet was received since the last check.
bool sxPacketReceived(SxRadio &r);

// Read the payload of the last received packet (up to maxLen bytes).
uint8_t sxReadPayload(SxRadio &r, uint8_t *buf, uint8_t maxLen);

// Read the RSSI (dBm, negative) of the most recently received packet.
int8_t sxReadRssiNow(SxRadio &r);

// True when this radio has hung (BUSY stuck HIGH long enough).
bool sxNeedsRecovery(SxRadio &r);

// Hardware-reset this radio and re-apply the LoRa configuration.
void sxRecover(SxRadio &r);
