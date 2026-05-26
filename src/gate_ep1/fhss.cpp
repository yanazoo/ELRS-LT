// fhss.cpp - ELRS FHSS sequence generation for 2.4GHz (SX1280).
//
// Ported from ExpressLRS src/lib/FHSS/FHSS.cpp + src/include/rng.h.
// Must be byte-for-byte compatible with the ELRS TX the sniffer is tracking.
//
// Algorithm summary:
//   seed    = UID[2] | UID[3]<<8 | UID[4]<<16 | UID[5]<<24  (little-endian)
//   rng     = xorshift32(seed) — matches ELRS rng.h exactly
//   sequence built in FHSS_CHANNEL_COUNT-wide blocks:
//     - block slot 0 = sync channel (FHSS_CHANNEL_COUNT/2 = 40)
//     - remaining slots initialized to their index within the block
//     - then shuffled: for each non-sync slot, swap with a random slot
//       in [1, FHSS_CHANNEL_COUNT-1] of the same block

#include "fhss.h"
#include "config.h"

// ---- xorshift32 RNG (matches ExpressLRS rng.h) ----
static uint32_t s_seed = 0;

static void rngSeed(uint32_t seed) { s_seed = seed; }

static uint32_t rngNext() {
    s_seed ^= s_seed << 13;
    s_seed ^= s_seed >> 17;
    s_seed ^= s_seed << 5;
    return s_seed;
}

// Returns value in [0, n-1].  Caller adds 1 to get [1, n-1] for intra-block swap.
static uint8_t rngN(uint8_t n) {
    return (uint8_t)(rngNext() % (uint32_t)n);
}

// ---- Sequence state ----
static uint8_t s_sequence[FHSS_SEQUENCE_LEN];

// 2.4GHz ISM channel table: 80 channels, 1 MHz apart from 2400.4 MHz.
// Matches ELRS domains[] entry for ISM_2400 / CE_LBT:
//   {FREQ_HZ_TO_REG_VAL(2400400000), FREQ_HZ_TO_REG_VAL(2479400000), 80, ...}
static const uint32_t FHSS_FREQ_BASE_HZ = 2400400000UL;
static const uint32_t FHSS_FREQ_STEP_HZ =    1000000UL;

void fhssGenerate(const uint8_t uid[6]) {
    // Seed from UID bytes 2-5, little-endian (matches ELRS FHSSrandomiseFHSSsequence caller)
    uint32_t seed = (uint32_t)uid[2]
                  | ((uint32_t)uid[3] <<  8)
                  | ((uint32_t)uid[4] << 16)
                  | ((uint32_t)uid[5] << 24);
    rngSeed(seed);

    const uint8_t freq_count = FHSS_CHANNEL_COUNT;   // 80
    const uint8_t sync_ch    = freq_count / 2;        // 40

    // Initialise: sync channel at block-start positions, sequential elsewhere.
    // Matches ELRS FHSSrandomiseFHSSsequenceBuild() initialisation loop.
    for (uint16_t i = 0; i < FHSS_SEQUENCE_LEN; i++) {
        if      (i % freq_count == 0)       s_sequence[i] = sync_ch;
        else if (i % freq_count == sync_ch) s_sequence[i] = 0;
        else                                s_sequence[i] = (uint8_t)(i % freq_count);
    }

    // Fisher-Yates shuffle within each block, skipping the sync slot (index 0 in block).
    // rand in [1, freq_count-1] ensures we stay inside the block and never touch slot 0.
    for (uint16_t i = 0; i < FHSS_SEQUENCE_LEN; i++) {
        if (i % freq_count != 0) {
            uint8_t offset = (uint8_t)((i / freq_count) * freq_count);
            uint8_t r      = rngN(freq_count - 1) + 1;   // [1, freq_count-1] = [1, 79]
            uint8_t tmp         = s_sequence[i];
            s_sequence[i]       = s_sequence[offset + r];
            s_sequence[offset + r] = tmp;
        }
    }
}

uint8_t fhssChannelAt(uint16_t hopIndex) {
    return s_sequence[hopIndex % FHSS_SEQUENCE_LEN];
}

uint32_t fhssFreqHz(uint8_t channelIndex) {
    return FHSS_FREQ_BASE_HZ + (uint32_t)channelIndex * FHSS_FREQ_STEP_HZ;
}

uint16_t fhssSequenceLength() { return FHSS_SEQUENCE_LEN; }
