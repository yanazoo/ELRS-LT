// sx1280_sniffer.cpp - SX1280 control for passive ELRS uplink sniffing.
//
// Targets the EP1/EP2 TCXO hardware: ESP8285 HSPI (SCK=14 MOSI=13 MISO=12 NSS=15)
// + SX1280 (BUSY=5 DIO1=4 RST=2).  Pins are defined in config.h.
//
// LoRa configuration is fixed to 250 Hz ELRS 2.4 GHz: SF7 / BW800 / CR4_5.
// Adjust ELRS_LORA_* defines if using a different packet rate.
//
// Opcode reference: SX1280 datasheet, section 13 "Radio Control Commands".

#include "sx1280_sniffer.h"
#include "config.h"
#include <Arduino.h>
#include <SPI.h>

// ---- SX1280 command opcodes ----
#define SX_CMD_GET_STATUS          0xC0
#define SX_CMD_SET_STANDBY         0x80
#define SX_CMD_SET_PACKET_TYPE     0x8A
#define SX_CMD_SET_RF_FREQUENCY    0x86
#define SX_CMD_SET_MOD_PARAMS      0x8B
#define SX_CMD_SET_PKT_PARAMS      0x8C
#define SX_CMD_SET_DIO_IRQ         0x8D
#define SX_CMD_SET_RX              0x82
#define SX_CMD_GET_IRQ_STATUS      0x15
#define SX_CMD_CLR_IRQ_STATUS      0x97
#define SX_CMD_GET_PKT_STATUS      0x1D
#define SX_CMD_READ_REGISTER       0x19
#define SX_CMD_WRITE_REGISTER      0x18

// ---- SX1280 register addresses ----
#define REG_FIRMWARE_VERSION_MSB   0x0891   // non-zero on healthy chip
#define REG_SF_ADDITIONAL_CONFIG   0x0925   // must be patched after SetModulationParams

// ---- Packet type ----
#define PKT_TYPE_LORA              0x01
#define STDBY_RC                   0x00

// ---- LoRa modem params for 250 Hz ELRS 2.4 GHz ----
#define ELRS_LORA_SF               0x70   // SF7
#define ELRS_LORA_BW               0x18   // 800 kHz
#define ELRS_LORA_CR               0x01   // 4/5
#define ELRS_LORA_PREAMBLE         12     // symbols
#define ELRS_LORA_PAYLOAD          8      // bytes (250 Hz payload)
#define ELRS_LORA_HEADER_IMPLICIT  0x80
#define ELRS_LORA_CRC_OFF          0x00
#define ELRS_LORA_IQ_NORMAL        0x40
// SF7/8 post-command register patch value (per SX1280 datasheet note)
#define SF7_8_REG_PATCH            0x37

// ---- IRQ bit masks ----
#define IRQ_RX_DONE                0x0002
#define IRQ_CRC_ERROR              0x0040

// ---- RF frequency step: 52 MHz / 2^18 ≈ 198.3642578125 Hz ----
// regval = (uint32_t)(freqHz / FREQ_STEP)
#define FREQ_STEP  (52000000.0 / 262144.0)

// ---- SetRx timeout: 15.625 µs tick, 0xFFFF ≈ 1024 ms (longer than any slot) ----
#define RX_TIMEOUT_TICK            0x00   // SX1280_RADIO_TICK_SIZE_0015_US
#define RX_TIMEOUT_HI              0xFF
#define RX_TIMEOUT_LO              0xFF

// ---- Module state ----
static int8_t s_last_rssi = -127;

// ---- SPI helpers ----

// Returns false if BUSY stays HIGH for more than 50 ms (avoids WDT reset).
static bool busyWait() {
    uint32_t t0 = millis();
    while (digitalRead(SX_PIN_BUSY)) {
        if (millis() - t0 > 50) {
            Serial.println("[sx] BUSY stuck >50ms");
            return false;
        }
        yield();
    }
    return true;
}

static void writeCmd(uint8_t cmd, const uint8_t *data, uint8_t len) {
    busyWait();
    digitalWrite(SX_PIN_NSS, LOW);
    SPI.transfer(cmd);
    for (uint8_t i = 0; i < len; i++) SPI.transfer(data[i]);
    digitalWrite(SX_PIN_NSS, HIGH);
}

// Sends cmd, reads status byte (discarded) + len response bytes into buf.
static void readCmd(uint8_t cmd, uint8_t *buf, uint8_t len) {
    busyWait();
    digitalWrite(SX_PIN_NSS, LOW);
    SPI.transfer(cmd);
    SPI.transfer(0x00);   // status byte (ignored)
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(SX_PIN_NSS, HIGH);
}

static void writeReg(uint16_t addr, uint8_t value) {
    busyWait();
    digitalWrite(SX_PIN_NSS, LOW);
    SPI.transfer(SX_CMD_WRITE_REGISTER);
    SPI.transfer((uint8_t)(addr >> 8));
    SPI.transfer((uint8_t)(addr & 0xFF));
    SPI.transfer(value);
    digitalWrite(SX_PIN_NSS, HIGH);
}

static uint8_t readReg(uint16_t addr) {
    busyWait();
    digitalWrite(SX_PIN_NSS, LOW);
    SPI.transfer(SX_CMD_READ_REGISTER);
    SPI.transfer((uint8_t)(addr >> 8));
    SPI.transfer((uint8_t)(addr & 0xFF));
    SPI.transfer(0x00);   // status byte
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(SX_PIN_NSS, HIGH);
    return val;
}

// ---- Public API ----

bool sxBegin() {
    pinMode(SX_PIN_NSS,  OUTPUT); digitalWrite(SX_PIN_NSS, HIGH);
    pinMode(SX_PIN_RST,  OUTPUT); digitalWrite(SX_PIN_RST, HIGH);
    pinMode(SX_PIN_BUSY, INPUT);
    pinMode(SX_PIN_DIO1, INPUT);

    // Initialize SPI BEFORE reset so bus glitches during SPI.begin() do not
    // reach the chip (SX1280 ignores SPI while RST is asserted or while BUSY).
    SPI.begin();
    SPI.setFrequency(8000000);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    delay(5);  // let the SPI hardware settle

    Serial.printf("[sx] SPI init, BUSY=%d RST=%d\n",
                  digitalRead(SX_PIN_BUSY), digitalRead(SX_PIN_RST));

    // Hardware reset: hold RST LOW for 100 µs, then release.
    digitalWrite(SX_PIN_RST, LOW);
    delayMicroseconds(100);
    digitalWrite(SX_PIN_RST, HIGH);

    // Poll BUSY until the chip finishes its internal firmware boot (typically
    // < 3 ms).  200 ms hard timeout avoids a WDT reset if BUSY stays HIGH.
    {
        uint32_t t0 = millis();
        while (digitalRead(SX_PIN_BUSY)) {
            if (millis() - t0 > 200) {
                Serial.println("[sx] BUSY stuck after reset (>200ms)");
                return false;
            }
            yield();
        }
    }
    Serial.printf("[sx] after reset, BUSY=%d\n", digitalRead(SX_PIN_BUSY));

    // SetStandby RC — required before any configuration writes.
    uint8_t stdby = STDBY_RC;
    Serial.println("[sx] SetStandby...");
    writeCmd(SX_CMD_SET_STANDBY, &stdby, 1);
    delay(2);
    Serial.printf("[sx] SetStandby done, BUSY=%d\n", digitalRead(SX_PIN_BUSY));

    // Verify chip is present: firmware version register must be non-zero/non-0xFF.
    Serial.println("[sx] reading FW ver...");
    uint8_t fwHi = readReg(REG_FIRMWARE_VERSION_MSB);
    uint8_t fwLo = readReg(REG_FIRMWARE_VERSION_MSB + 1);
    uint16_t fwVer = ((uint16_t)fwHi << 8) | fwLo;
    Serial.printf("[sx] FW ver=0x%04X\n", fwVer);
    if (fwVer == 0x0000 || fwVer == 0xFFFF) return false;

    // SetPacketType: LoRa
    uint8_t pktType = PKT_TYPE_LORA;
    writeCmd(SX_CMD_SET_PACKET_TYPE, &pktType, 1);

    // SetModulationParams: SF7 / BW800 / CR4_5
    uint8_t modParams[3] = { ELRS_LORA_SF, ELRS_LORA_BW, ELRS_LORA_CR };
    writeCmd(SX_CMD_SET_MOD_PARAMS, modParams, 3);
    // Datasheet-required register patch for SF7/8 after SetModulationParams.
    writeReg(REG_SF_ADDITIONAL_CONFIG, SF7_8_REG_PATCH);

    // SetPacketParams: preamble=12, implicit header, payload=8, CRC off, IQ normal.
    uint8_t pktParams[7] = {
        0x00, ELRS_LORA_PREAMBLE,   // PreambleLength MSB + LSB (16-bit symbol count)
        ELRS_LORA_HEADER_IMPLICIT,  // HeaderType
        ELRS_LORA_PAYLOAD,          // PayloadLength
        ELRS_LORA_CRC_OFF,          // CrcMode
        ELRS_LORA_IQ_NORMAL,        // InvertIQ
        0x00                        // reserved
    };
    writeCmd(SX_CMD_SET_PKT_PARAMS, pktParams, 7);

    // SetDioIrqParams: route RX_DONE + CRC_ERROR to DIO1.
    uint16_t mask = IRQ_RX_DONE | IRQ_CRC_ERROR;
    uint8_t irqParams[8] = {
        (uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF),   // IRQ mask
        (uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF),   // DIO1 mask
        0x00, 0x00,                                       // DIO2 off
        0x00, 0x00                                        // DIO3 off
    };
    writeCmd(SX_CMD_SET_DIO_IRQ, irqParams, 8);

    return true;
}

void sxSetFrequencyHz(uint32_t freqHz) {
    uint32_t regFreq = (uint32_t)((double)freqHz / FREQ_STEP);

    // SetRfFrequency: 3-byte big-endian register value.
    uint8_t freqBuf[3] = {
        (uint8_t)((regFreq >> 16) & 0xFF),
        (uint8_t)((regFreq >>  8) & 0xFF),
        (uint8_t)( regFreq        & 0xFF)
    };
    writeCmd(SX_CMD_SET_RF_FREQUENCY, freqBuf, 3);

    // SetRx: enter continuous RX with ~1 second timeout (longer than any slot).
    uint8_t rxParams[3] = { RX_TIMEOUT_TICK, RX_TIMEOUT_HI, RX_TIMEOUT_LO };
    writeCmd(SX_CMD_SET_RX, rxParams, 3);
}

int8_t sxReadRssi() {
    return s_last_rssi;
}

bool sxPacketReceived() {
    uint8_t irqBuf[2] = {};
    readCmd(SX_CMD_GET_IRQ_STATUS, irqBuf, 2);
    uint16_t irq = ((uint16_t)irqBuf[0] << 8) | irqBuf[1];

    if (!(irq & IRQ_RX_DONE)) return false;

    // Read packet RSSI from GetPacketStatus for accuracy.
    // LoRa response layout (6 bytes): [RFU, rssiSync, snrPkt, RFU, RFU, RFU]
    // rssiSync → dBm = -(raw / 2)
    uint8_t ps[6] = {};
    readCmd(SX_CMD_GET_PKT_STATUS, ps, 6);
    s_last_rssi = -(int8_t)(ps[1] / 2);

    // Clear all IRQ flags.
    uint8_t clr[2] = { 0xFF, 0xFF };
    writeCmd(SX_CMD_CLR_IRQ_STATUS, clr, 2);

    // Return true only if CRC passed (or CRC off — then CRC_ERROR won't be set).
    return !(irq & IRQ_CRC_ERROR);
}
