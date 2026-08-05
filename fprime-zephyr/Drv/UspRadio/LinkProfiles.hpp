// ======================================================================
// \title  LinkProfiles.hpp
// \brief  Versioned, append-only Link Profile table.  Free of USP/RAL and
//         Zephyr includes so it compiles host-side as well as on-target.
//         GFSK OBW methodology (Carson rule vs 1.5x bitrate) and the
//         ≤125 kHz IARU band constraint: see ADR 0002.
// ======================================================================

#ifndef ZEPHYR_USP_RADIO_LINK_PROFILES_HPP
#define ZEPHYR_USP_RADIO_LINK_PROFILES_HPP

// When building host-side (unit tests, GRC) F' may not be on the include path.
// The guard LINK_PROFILES_USE_FPRIME_TYPES lets the build inject its own typedefs;
// otherwise we pull in F' as normal (all target builds do this).
#ifndef LINK_PROFILES_USE_HOST_TYPES
#include <Fw/FPrimeBasicTypes.hpp>
#else
#include <cstdint>
using U8  = uint8_t;
using U16 = uint16_t;
using U32 = uint32_t;
using I8  = int8_t;
using I16 = int16_t;
using I32 = int32_t;
#endif

namespace Zephyr {

// ---------------------------------------------------------------------------
// Modulation kind tag
// ---------------------------------------------------------------------------
enum class ModKind : U8 {
    LORA = 0,
    GFSK = 1,
};

// ---------------------------------------------------------------------------
// GFSK pulse-shape / BT selector
// (maps to Semtech RAL GFSK pulse-shape enum values later)
// ---------------------------------------------------------------------------
enum class GfskPulseShape : U8 {
    NONE  = 0,  //!< no filtering
    BT_03 = 1,
    BT_05 = 2,  //!< used by P3 and P4
    BT_07 = 3,
};

// ---------------------------------------------------------------------------
// GFSK CRC type selector
// ---------------------------------------------------------------------------
enum class GfskCrcType : U8 {
    OFF    = 0,
    BYTE_1 = 1,
    BYTE_2 = 2,  //!< used by P3 and P4 (16-bit CRC, CCITT)
};

// ---------------------------------------------------------------------------
// Sub-structs
// ---------------------------------------------------------------------------
struct LoRaParams {
    U8  sf;            //!< Spreading factor (5–12; SF5–SF6 are SX126x-only)
    U32 bw_hz;         //!< Bandwidth in Hz (must be 125000 for all v1 profiles)
    U8  cr;            //!< Coding rate denominator: 5=CR4/5, 6=CR4/6, …
    bool ldro;         //!< Low Data Rate Optimisation (recommended for SF≥10+BW125)
    U16 sync_word;     //!< LoRa sync word, 1-byte convention: 0x12 = private, 0x34 = public.
                       //!< ral_set_lora_sync_word(uint8_t) takes this 1-byte value and
                       //!< computes the SX126x 2-byte register encoding (0x12 → 0x1424).
    U16 preamble_len;  //!< Preamble length in symbols
};

struct GfskParams {
    U32 bitrate_bps;       //!< Bit rate in bits/s
    U32 fdev_hz;           //!< Frequency deviation in Hz
    U32 bw_dsb_hz;         //!< Double-sideband RX bandwidth in Hz
    GfskPulseShape pulse_shape;  //!< Gaussian BT factor (0.5 for all v1 profiles)
    U16 preamble_len_bits; //!< Preamble length in bits
    U8  sync_word[4];      //!< Sync word bytes (MSB first)
    U8  sync_word_len;     //!< Sync word length in bytes (1–4)
    GfskCrcType crc_type;  //!< CRC type
    bool whitening;        //!< Data whitening on/off
};

// ---------------------------------------------------------------------------
// The top-level Link Profile struct
// ---------------------------------------------------------------------------
struct LinkProfile {
    ModKind mod;  //!< Modulation kind (selects which union member is valid)
    union {
        LoRaParams lora;
        GfskParams gfsk;
    };
};

// ---------------------------------------------------------------------------
// Table version and size constants
// ---------------------------------------------------------------------------
constexpr U8 LINK_PROFILE_TABLE_VERSION = 2;
constexpr U8 LINK_PROFILE_COUNT         = 6;
constexpr U8 BOOT_DEFAULT_PROFILE       = 0;  //!< Index of boot-default (P0)

// ---------------------------------------------------------------------------
// The profile table (v1)
// ---------------------------------------------------------------------------
//
// APPEND-ONLY: never renumber existing entries (ADR 0002).
// Indices are part of the ops vocabulary and command sequences.
//
// P0  LoRa SF8/125k/CR4:5  — boot default, parity with today's legacy LoRa
//     link (Zephyr loramac-node: SF8, BW125, CR4/5, preamble 8, private
//     sync word 0x12, LDRO off).
//
// P1  LoRa SF10/125k       — long-range / link-degraded fallback.
//     LDRO recommended (ToA symbol time ≈ 8.2 ms > 16 ms threshold at 125k).
//
// P2  LoRa SF5/125k        — fast LoRa (~15 kbps); SX126x-only SF.
//
// P3  GFSK 38400 bps       — conservative bulk downlink.
//     OBW_carson = 38400 + 2·20000 = 78400 Hz   (< 125000 ✓)
//     OBW_bt05   = 1.5 × 38400   = 57600 Hz     (< 125000 ✓)
//     Sync word 0xC9C9C9C9: 4-byte, alternating-run structure chosen for
//     high autocorrelation peak-to-sidelobe ratio.
//
// P4  GFSK 75000 bps       — max-rate bulk downlink.
//     OBW_carson = 75000 + 2·25000 = 125000 Hz  (== 125000 ✓ — exactly at limit)
//     OBW_bt05   = 1.5 × 75000   = 112500 Hz    (< 125000 ✓)
//     The Carson-rule estimate equals the limit exactly; BT=0.5 filtering brings
//     it within budget.  Document for bench verification in Phase 5.
//     Sync word 0xD391D391: distinct from P3; similar Hamming-weight balance.
//
constexpr LinkProfile LINK_PROFILE_TABLE[LINK_PROFILE_COUNT] = {
    // ------------------------------------------------------------------
    // P0: LoRa SF8/125k/CR4:5 — boot default
    // ------------------------------------------------------------------
    {
        .mod  = ModKind::LORA,
        .lora = {
            .sf           = 8,
            .bw_hz        = 125000,
            .cr           = 5,         // CR4/5
            .ldro         = false,
            .sync_word    = 0x12U,     // private-network (1-byte convention; GRC uses 0x12)
            .preamble_len = 8,         // matches legacy LoRaConfig::PREAMBLE_LENGTH
        },
    },
    // ------------------------------------------------------------------
    // P1: LoRa SF10/125k — long-range / degraded-link fallback
    // ------------------------------------------------------------------
    {
        .mod  = ModKind::LORA,
        .lora = {
            .sf           = 10,
            .bw_hz        = 125000,
            .cr           = 5,         // CR4/5
            .ldro         = true,      // LDRO on: symbol time ~8.2 ms at SF10/125k
            .sync_word    = 0x12U,
            .preamble_len = 8,
        },
    },
    // ------------------------------------------------------------------
    // P2: LoRa SF5/125k — fast LoRa (~15 kbps); SX126x-only
    // ------------------------------------------------------------------
    {
        .mod  = ModKind::LORA,
        .lora = {
            .sf           = 5,
            .bw_hz        = 125000,
            .cr           = 5,         // CR4/5
            .ldro         = false,
            .sync_word    = 0x12U,
            .preamble_len = 8,
        },
    },
    // ------------------------------------------------------------------
    // P3: GFSK 38400 bps — conservative bulk downlink
    // OBW_carson = 38400 + 2*20000 = 78400 Hz  (< 125000)
    // ------------------------------------------------------------------
    {
        .mod  = ModKind::GFSK,
        .gfsk = {
            .bitrate_bps      = 38400,
            .fdev_hz          = 20000,
            .bw_dsb_hz        = 100000,
            .pulse_shape      = GfskPulseShape::BT_05,
            .preamble_len_bits = 32,
            .sync_word        = {0xC9, 0xC9, 0xC9, 0xC9},
            .sync_word_len    = 4,
            .crc_type         = GfskCrcType::BYTE_2,
            .whitening        = true,
        },
    },
    // ------------------------------------------------------------------
    // P4: GFSK 75000 bps — max-rate bulk downlink
    // OBW_carson = 75000 + 2*25000 = 125000 Hz  (== limit; BT=0.5 ok)
    // OBW_bt05   = 1.5 * 75000    = 112500 Hz  (< 125000)
    // ------------------------------------------------------------------
    {
        .mod  = ModKind::GFSK,
        .gfsk = {
            .bitrate_bps      = 75000,
            .fdev_hz          = 25000,
            .bw_dsb_hz        = 125000,
            .pulse_shape      = GfskPulseShape::BT_05,
            .preamble_len_bits = 32,
            .sync_word        = {0xD3, 0x91, 0xD3, 0x91},
            .sync_word_len    = 4,
            .crc_type         = GfskCrcType::BYTE_2,
            .whitening        = true,
        },
    },
    // ------------------------------------------------------------------
    // P5: GMSK 83333 bps — max-rate GMSK candidate (GFSK h=0.5, BT=0.5)
    // h = 2*fdev/br = 2*20833/83333 = 0.49999 (GMSK)
    // OBW_carson = 83333 + 2*20833 = 124999 Hz  (< 125000 ✓, at limit)
    // OBW_bt05   = 1.5 * 83333    = 125000 Hz  (== limit)
    // RX BW: SX126x GFSK DSB steps are 117.3k / 156.2k; 117.3k < OBW, so
    // 156.2k DSB is the minimum legal choice → CFO margin ≈ ±15.6 kHz
    // ((156200-124999)/2) before signal energy leaves the filter.
    // Sync word 0xB27DB27D: distinct from P3/P4, balanced Hamming weight.
    // ------------------------------------------------------------------
    {
        .mod  = ModKind::GFSK,
        .gfsk = {
            .bitrate_bps      = 83333,
            .fdev_hz          = 20833,
            .bw_dsb_hz        = 156200,
            .pulse_shape      = GfskPulseShape::BT_05,
            .preamble_len_bits = 32,
            .sync_word        = {0xB2, 0x7D, 0xB2, 0x7D},
            .sync_word_len    = 4,
            .crc_type         = GfskCrcType::BYTE_2,
            .whitening        = true,
        },
    },
};

// ---------------------------------------------------------------------------
// Compile-time integrity checks
// ---------------------------------------------------------------------------

// 1. Table size must match the declared count.
static_assert(sizeof(LINK_PROFILE_TABLE) / sizeof(LINK_PROFILE_TABLE[0]) ==
              LINK_PROFILE_COUNT,
              "LINK_PROFILE_TABLE size does not match LINK_PROFILE_COUNT");

// 2. Boot default index must be in range.
static_assert(BOOT_DEFAULT_PROFILE < LINK_PROFILE_COUNT,
              "BOOT_DEFAULT_PROFILE index out of range");

// 3. Boot default must be LoRa (spec requirement: boot on the robust LoRa link).
static_assert(LINK_PROFILE_TABLE[BOOT_DEFAULT_PROFILE].mod == ModKind::LORA,
              "Boot default profile must be a LoRa profile");

// 4. All LoRa profiles must use 125 kHz bandwidth (250/500 kHz excluded per
//    IARU band constraint).
static_assert(LINK_PROFILE_TABLE[0].lora.bw_hz == 125000, "P0 LoRa bw must be 125000 Hz");
static_assert(LINK_PROFILE_TABLE[1].lora.bw_hz == 125000, "P1 LoRa bw must be 125000 Hz");
static_assert(LINK_PROFILE_TABLE[2].lora.bw_hz == 125000, "P2 LoRa bw must be 125000 Hz");

// 5. GFSK OBW estimates must be <= 125000 Hz (Carson rule: bitrate + 2*fdev).
//    P3: 38400 + 2*20000 = 78400
static_assert(
    LINK_PROFILE_TABLE[3].gfsk.bitrate_bps + 2 * LINK_PROFILE_TABLE[3].gfsk.fdev_hz <= 125000,
    "P3 GFSK OBW_carson exceeds 125 kHz IARU limit");
//    P4: 75000 + 2*25000 = 125000  (exactly at limit)
static_assert(
    LINK_PROFILE_TABLE[4].gfsk.bitrate_bps + 2 * LINK_PROFILE_TABLE[4].gfsk.fdev_hz <= 125000,
    "P4 GFSK OBW_carson exceeds 125 kHz IARU limit");
//    P5: 83333 + 2*20833 = 124999
static_assert(
    LINK_PROFILE_TABLE[5].gfsk.bitrate_bps + 2 * LINK_PROFILE_TABLE[5].gfsk.fdev_hz <= 125000,
    "P5 GMSK OBW_carson exceeds 125 kHz IARU limit");

// 6. Table version must be non-zero.
static_assert(LINK_PROFILE_TABLE_VERSION > 0, "Table version must be >= 1");

}  // namespace Zephyr

#endif  // ZEPHYR_USP_RADIO_LINK_PROFILES_HPP
