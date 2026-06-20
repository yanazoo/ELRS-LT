// sx1280_radio.cpp - instance-based SX1280 driver (EP1 Dual, 2 radios / 1 SPI bus)
//
// Ported from the proven single-radio src/gate_ep1/sx1280_sniffer.cpp, with the
// module-static state and #define pins replaced by an SxRadio object so two
// chips can coexist on one shared SPI bus (each toggles its own NSS).
//
// LoRa configuration is fixed to 500 Hz ELRS 2.4 GHz: SF5 / BW800 / CR_LI_4_6.
// Verified against ELRS 3.6.3 src/src/common.cpp air-rate table.
//   500Hz: SF5(0x50) CR_LI_4_6(0x06) preamble12  slot 2000us  <-- ACTIVE
//   250Hz: SF6(0x60) CR_LI_4_8(0x07) preamble14  slot 4000us
//   150Hz: SF7(0x70) CR_LI_4_8(0x07) preamble12  slot 6666us

#include "sx1280_radio.h"
#include "config.h"
#include <Arduino.h>

// ---- SX1280 command opcodes ----
#define SX_CMD_GET_RX_BUFFER_STS   0x17
#define SX_CMD_READ_BUFFER         0x1B
#define SX_CMD_SET_STANDBY         0x80
#define SX_CMD_SET_PACKET_TYPE     0x8A
#define SX_CMD_SET_RF_FREQUENCY    0x86
#define SX_CMD_SET_MOD_PARAMS      0x8B
#define SX_CMD_SET_PKT_PARAMS      0x8C
#define SX_CMD_SET_BUFFER_BASE     0x8F
#define SX_CMD_SET_DIO_IRQ         0x8D
#define SX_CMD_SET_RX              0x82
#define SX_CMD_GET_IRQ_STATUS      0x15
#define SX_CMD_CLR_IRQ_STATUS      0x97
#define SX_CMD_GET_PKT_STATUS      0x1D
#define SX_CMD_READ_REGISTER       0x19
#define SX_CMD_WRITE_REGISTER      0x18

// NOTE on TCXO: unlike SX126x there is no DIO3-as-TCXO-supply opcode on SX1280;
// SX1280 boards power the TCXO externally (the proven single-radio EP1 firmware
// never touches it).  The SX_DIO3_TCXO_VOLTAGE config knob is therefore only a
// documented placeholder, guarded off by default.

// ---- SX1280 register addresses ----
#define REG_FIRMWARE_VERSION_MSB   0x0891   // non-zero on healthy chip
#define REG_SF_ADDITIONAL_CONFIG   0x0925   // patched after SetModulationParams

// ---- Packet type ----
#define PKT_TYPE_LORA              0x01
#define STDBY_RC                   0x00

// ---- LoRa modem params for 500 Hz ELRS 2.4 GHz (verified vs ELRS 3.6.3) ----
#define ELRS_LORA_SF               0x50   // SF5  (500Hz)
#define ELRS_LORA_BW               0x18   // 800 kHz
#define ELRS_LORA_CR               0x06   // CR_LI_4_6  (500Hz)
#define ELRS_LORA_PREAMBLE         12     // symbols  (500Hz)
#define ELRS_LORA_PAYLOAD          8      // bytes (OTA4_PACKET_SIZE)
#define ELRS_LORA_HEADER_IMPLICIT  0x80
#define ELRS_LORA_CRC_OFF          0x00
#define ELRS_LORA_IQ_NORMAL        0x40
#define SF5_6_REG_PATCH            0x1E   // reg 0x925 patch for SF5/SF6

// ---- IRQ bit masks ----
#define IRQ_RX_DONE                0x0002
#define IRQ_CRC_ERROR              0x0040

// ---- RF frequency step: 52 MHz / 2^18 ≈ 198.3642578125 Hz ----
#define FREQ_STEP  (52000000.0 / 262144.0)

// ---- SetRx timeout: 0xFFFF periodBaseCount = continuous RX ----
#define RX_TIMEOUT_TICK            0x00
#define RX_TIMEOUT_HI              0xFF
#define RX_TIMEOUT_LO              0xFF

#define SX_BUSY_RECOVER_COUNT  8   // consecutive BUSY timeouts (~400ms) -> recover

static SPISettings s_spiCfg(8000000, MSBFIRST, SPI_MODE0);

// ---- SPI helpers (per-radio, NSS-addressed, wrapped in a bus transaction) ----

// Returns false if BUSY stays HIGH > 50 ms (avoids WDT reset). Counts consecutive
// timeouts so sxNeedsRecovery() can flag a persistent hang.
static bool busyWait(SxRadio &r) {
    uint32_t t0 = millis();
    while (digitalRead(r.busy)) {
        if (millis() - t0 > 50) {
            Serial.printf("[sx%s] BUSY stuck >50ms\n", r.tag);
            r.busyStuckCount++;
            return false;
        }
        yield();
    }
    r.busyStuckCount = 0;
    return true;
}

static void writeCmd(SxRadio &r, uint8_t cmd, const uint8_t *data, uint8_t len) {
    busyWait(r);
    r.spi->beginTransaction(s_spiCfg);
    digitalWrite(r.nss, LOW);
    r.spi->transfer(cmd);
    for (uint8_t i = 0; i < len; i++) r.spi->transfer(data[i]);
    digitalWrite(r.nss, HIGH);
    r.spi->endTransaction();
}

static void readCmd(SxRadio &r, uint8_t cmd, uint8_t *buf, uint8_t len) {
    busyWait(r);
    r.spi->beginTransaction(s_spiCfg);
    digitalWrite(r.nss, LOW);
    r.spi->transfer(cmd);
    r.spi->transfer(0x00);   // status byte (ignored)
    for (uint8_t i = 0; i < len; i++) buf[i] = r.spi->transfer(0x00);
    digitalWrite(r.nss, HIGH);
    r.spi->endTransaction();
}

static void writeReg(SxRadio &r, uint16_t addr, uint8_t value) {
    busyWait(r);
    r.spi->beginTransaction(s_spiCfg);
    digitalWrite(r.nss, LOW);
    r.spi->transfer(SX_CMD_WRITE_REGISTER);
    r.spi->transfer((uint8_t)(addr >> 8));
    r.spi->transfer((uint8_t)(addr & 0xFF));
    r.spi->transfer(value);
    digitalWrite(r.nss, HIGH);
    r.spi->endTransaction();
}

static uint8_t readReg(SxRadio &r, uint16_t addr) {
    busyWait(r);
    r.spi->beginTransaction(s_spiCfg);
    digitalWrite(r.nss, LOW);
    r.spi->transfer(SX_CMD_READ_REGISTER);
    r.spi->transfer((uint8_t)(addr >> 8));
    r.spi->transfer((uint8_t)(addr & 0xFF));
    r.spi->transfer(0x00);   // status byte
    uint8_t val = r.spi->transfer(0x00);
    digitalWrite(r.nss, HIGH);
    r.spi->endTransaction();
    return val;
}

// Apply the fixed LoRa modem configuration. Assumes the chip was just reset and
// is responsive (BUSY low).
static void sxApplyLoRaConfig(SxRadio &r) {
    uint8_t stdby = STDBY_RC;
    writeCmd(r, SX_CMD_SET_STANDBY, &stdby, 1);
    delay(2);

    uint8_t pktType = PKT_TYPE_LORA;
    writeCmd(r, SX_CMD_SET_PACKET_TYPE, &pktType, 1);

    uint8_t modParams[3] = { ELRS_LORA_SF, ELRS_LORA_BW, ELRS_LORA_CR };
    writeCmd(r, SX_CMD_SET_MOD_PARAMS, modParams, 3);
    writeReg(r, REG_SF_ADDITIONAL_CONFIG, SF5_6_REG_PATCH);

    uint8_t pktParams[7] = {
        ELRS_LORA_PREAMBLE, ELRS_LORA_HEADER_IMPLICIT, ELRS_LORA_PAYLOAD,
        ELRS_LORA_CRC_OFF,  ELRS_LORA_IQ_NORMAL,       0x00, 0x00
    };
    writeCmd(r, SX_CMD_SET_PKT_PARAMS, pktParams, 7);

    uint8_t baseAddr[2] = { 0x00, 0x00 };
    writeCmd(r, SX_CMD_SET_BUFFER_BASE, baseAddr, 2);

    uint16_t mask = IRQ_RX_DONE | IRQ_CRC_ERROR;
    uint8_t irqParams[8] = {
        (uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF),
        (uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF),
        0x00, 0x00, 0x00, 0x00
    };
    writeCmd(r, SX_CMD_SET_DIO_IRQ, irqParams, 8);
}

// Hardware reset pulse: hold RST LOW 100us, release, poll BUSY until boot done.
static bool sxResetAndWait(SxRadio &r, uint32_t busyTimeoutMs) {
    digitalWrite(r.rst, LOW);
    delayMicroseconds(100);
    digitalWrite(r.rst, HIGH);
    uint32_t t0 = millis();
    while (digitalRead(r.busy)) {
        if (millis() - t0 > busyTimeoutMs) return false;
        yield();
    }
    return true;
}

// ---- Public API ----

void sxBusBegin(SPIClass *spi) {
    spi->begin(SX_PIN_SCK, SX_PIN_MISO, SX_PIN_MOSI, -1);  // NSS handled per-radio
}

bool sxBegin(SxRadio &r, SPIClass *spi,
             uint8_t nss, uint8_t busy, uint8_t dio1, uint8_t rst,
             const char *tag) {
    r.spi = spi;
    r.nss = nss; r.busy = busy; r.dio1 = dio1; r.rst = rst;
    r.tag = tag;
    r.busyStuckCount = 0;

    pinMode(r.nss,  OUTPUT); digitalWrite(r.nss, HIGH);
    pinMode(r.rst,  OUTPUT); digitalWrite(r.rst, HIGH);
    pinMode(r.busy, INPUT);
    pinMode(r.dio1, INPUT);
    delay(5);

    if (!sxResetAndWait(r, 200)) {
        Serial.printf("[sx%s] BUSY stuck after reset (>200ms)\n", r.tag);
        return false;
    }

    uint8_t stdby = STDBY_RC;
    writeCmd(r, SX_CMD_SET_STANDBY, &stdby, 1);
    delay(2);

#if SX_DIO3_TCXO_VOLTAGE
    // Placeholder: SX1280 boards power the TCXO externally; see note in header.
    // Left intentionally empty so the knob documents intent without misbehaving.
#endif

    uint8_t fwHi = readReg(r, REG_FIRMWARE_VERSION_MSB);
    uint8_t fwLo = readReg(r, REG_FIRMWARE_VERSION_MSB + 1);
    uint16_t fwVer = ((uint16_t)fwHi << 8) | fwLo;
    Serial.printf("[sx%s] FW ver=0x%04X\n", r.tag, fwVer);
    if (fwVer == 0x0000 || fwVer == 0xFFFF) return false;

    sxApplyLoRaConfig(r);
    Serial.printf("[sx%s] config SF=0x%02X BW=0x%02X CR=0x%02X\n",
                  r.tag, ELRS_LORA_SF, ELRS_LORA_BW, ELRS_LORA_CR);
    return true;
}

bool sxNeedsRecovery(SxRadio &r) {
    return r.busyStuckCount >= SX_BUSY_RECOVER_COUNT;
}

void sxRecover(SxRadio &r) {
    Serial.printf("[sx%s] recover: hardware reset + reconfigure\n", r.tag);
    sxResetAndWait(r, 200);
    sxApplyLoRaConfig(r);
    r.busyStuckCount = 0;
}

void sxSetFrequencyHz(SxRadio &r, uint32_t freqHz) {
    // SetRfFrequency is only valid in Standby; SetStandby first or the chip
    // ignores it from RX mode and stays stuck on the previous channel.
    uint8_t stdby = STDBY_RC;
    writeCmd(r, SX_CMD_SET_STANDBY, &stdby, 1);

    uint32_t regFreq = (uint32_t)((double)freqHz / FREQ_STEP);
    uint8_t freqBuf[3] = {
        (uint8_t)((regFreq >> 16) & 0xFF),
        (uint8_t)((regFreq >>  8) & 0xFF),
        (uint8_t)( regFreq        & 0xFF)
    };
    writeCmd(r, SX_CMD_SET_RF_FREQUENCY, freqBuf, 3);

    uint8_t rxParams[3] = { RX_TIMEOUT_TICK, RX_TIMEOUT_HI, RX_TIMEOUT_LO };
    writeCmd(r, SX_CMD_SET_RX, rxParams, 3);
}

int8_t sxReadRssiNow(SxRadio &r) {
    // GetPacketStatus (LoRa): after the discarded status byte the response is
    // [rssiSync, snr, ...].  dBm = -(rssiSync / 2).
    uint8_t ps[2] = {};
    readCmd(r, SX_CMD_GET_PKT_STATUS, ps, 2);
    return -(int8_t)(ps[0] / 2);
}

uint8_t sxReadPayload(SxRadio &r, uint8_t *buf, uint8_t maxLen) {
    // ELRS uses LoRa IMPLICIT-header mode, so the reported payload length is 0;
    // we ignore it and read the fixed configured packet size (maxLen).
    uint8_t sts[2] = {};
    readCmd(r, SX_CMD_GET_RX_BUFFER_STS, sts, 2);
    uint8_t startPtr = sts[1];   // = RX base address (0)
    uint8_t n = maxLen;

    busyWait(r);
    r.spi->beginTransaction(s_spiCfg);
    digitalWrite(r.nss, LOW);
    r.spi->transfer(SX_CMD_READ_BUFFER);
    r.spi->transfer(startPtr);
    r.spi->transfer(0x00);   // NOP status byte
    for (uint8_t i = 0; i < n; i++) buf[i] = r.spi->transfer(0x00);
    digitalWrite(r.nss, HIGH);
    r.spi->endTransaction();
    return n;
}

bool sxPacketReceived(SxRadio &r) {
    uint8_t irqBuf[2] = {};
    readCmd(r, SX_CMD_GET_IRQ_STATUS, irqBuf, 2);
    uint16_t irq = ((uint16_t)irqBuf[0] << 8) | irqBuf[1];

    if (!(irq & IRQ_RX_DONE)) return false;

    // RSSI is intentionally NOT read here (kept short); read it lazily via
    // sxReadRssiNow() only for the telemetry packets we report.
    uint8_t clr[2] = { 0xFF, 0xFF };
    writeCmd(r, SX_CMD_CLR_IRQ_STATUS, clr, 2);

    return !(irq & IRQ_CRC_ERROR);
}
