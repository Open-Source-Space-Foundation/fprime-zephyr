// ======================================================================
// test_LinkProfiles.cpp
// Host-side unit tests for the LinkProfiles table (Phase 2, ADR 0002).
//
// Build: included via pcr's CMakeLists.txt host-UT target (same gtest
// infrastructure used by test_DetumbleManager_*, test_RtcManager_*, etc).
//
// These tests supplement the static_asserts in LinkProfiles.hpp — they
// exercise runtime-accessible table properties that the compiler cannot
// always evaluate (e.g. OBW formula for every GFSK profile, not just the
// two we named in static_asserts).
// ======================================================================

// Pull in the table under test (header-only, no USP/Zephyr deps).
// LINK_PROFILES_USE_HOST_TYPES activates the inline cstdint typedefs inside
// LinkProfiles.hpp so the header compiles without the F' framework on the path.
#define LINK_PROFILES_USE_HOST_TYPES
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"

#include <gtest/gtest.h>

using namespace Zephyr;

// ---------------------------------------------------------------------------
// Basic table integrity
// ---------------------------------------------------------------------------

TEST(LinkProfiles, CountMatchesMacro) {
    EXPECT_EQ(static_cast<int>(LINK_PROFILE_COUNT), 6);
}

TEST(LinkProfiles, VersionIsTwo) {
    // v2 added P5 (GMSK 83333 bps)
    EXPECT_EQ(static_cast<int>(LINK_PROFILE_TABLE_VERSION), 2);
}

TEST(LinkProfiles, BootDefaultIsZero) {
    EXPECT_EQ(static_cast<int>(BOOT_DEFAULT_PROFILE), 0);
}

TEST(LinkProfiles, BootDefaultIsLoRa) {
    EXPECT_EQ(LINK_PROFILE_TABLE[BOOT_DEFAULT_PROFILE].mod, ModKind::LORA);
}

// ---------------------------------------------------------------------------
// LoRa profiles: bandwidth constraint
// ---------------------------------------------------------------------------

TEST(LinkProfiles, AllLoRaBandwidthIs125k) {
    for (int i = 0; i < LINK_PROFILE_COUNT; ++i) {
        if (LINK_PROFILE_TABLE[i].mod == ModKind::LORA) {
            EXPECT_EQ(LINK_PROFILE_TABLE[i].lora.bw_hz, 125000u)
                << "Profile " << i << " LoRa bw_hz != 125000";
        }
    }
}

// ---------------------------------------------------------------------------
// P0 mirrors legacy LoRa link parameters
// ---------------------------------------------------------------------------

TEST(LinkProfiles, P0_SF8) {
    EXPECT_EQ(LINK_PROFILE_TABLE[0].lora.sf, 8);
}

TEST(LinkProfiles, P0_CR45) {
    EXPECT_EQ(LINK_PROFILE_TABLE[0].lora.cr, 5);
}

TEST(LinkProfiles, P0_Preamble8) {
    EXPECT_EQ(LINK_PROFILE_TABLE[0].lora.preamble_len, 8);
}

TEST(LinkProfiles, P0_PrivateSyncWord) {
    // 0x12 is the private-network sync word (1-byte convention matching GRC loramac-node).
    // ral_set_lora_sync_word(uint8_t) takes this value; the SX126x chip driver
    // computes the 2-byte register encoding 0x1424 internally.
    EXPECT_EQ(LINK_PROFILE_TABLE[0].lora.sync_word, 0x12u);
}

TEST(LinkProfiles, P0_LdroOff) {
    EXPECT_FALSE(LINK_PROFILE_TABLE[0].lora.ldro);
}

// ---------------------------------------------------------------------------
// P1 long-range profile
// ---------------------------------------------------------------------------

TEST(LinkProfiles, P1_SF10) {
    EXPECT_EQ(LINK_PROFILE_TABLE[1].lora.sf, 10);
}

TEST(LinkProfiles, P1_LdroOn) {
    // LDRO recommended for SF10/BW125 (symbol time ~8.2 ms)
    EXPECT_TRUE(LINK_PROFILE_TABLE[1].lora.ldro);
}

// ---------------------------------------------------------------------------
// P2 fast LoRa (SX126x-only SF5)
// ---------------------------------------------------------------------------

TEST(LinkProfiles, P2_SF5) {
    EXPECT_EQ(LINK_PROFILE_TABLE[2].lora.sf, 5);
}

// ---------------------------------------------------------------------------
// GFSK profiles: OBW constraint (Carson rule: bitrate + 2*fdev <= 125000)
// ---------------------------------------------------------------------------

TEST(LinkProfiles, AllGfskOBW_Carson_Within125k) {
    for (int i = 0; i < LINK_PROFILE_COUNT; ++i) {
        if (LINK_PROFILE_TABLE[i].mod == ModKind::GFSK) {
            const auto& g = LINK_PROFILE_TABLE[i].gfsk;
            const uint32_t obw_carson = g.bitrate_bps + 2u * g.fdev_hz;
            EXPECT_LE(obw_carson, 125000u)
                << "Profile " << i
                << " OBW_carson=" << obw_carson << " exceeds 125 kHz";
        }
    }
}

TEST(LinkProfiles, P3_Bitrate38400) {
    EXPECT_EQ(LINK_PROFILE_TABLE[3].gfsk.bitrate_bps, 38400u);
}

TEST(LinkProfiles, P3_Fdev20k) {
    EXPECT_EQ(LINK_PROFILE_TABLE[3].gfsk.fdev_hz, 20000u);
}

TEST(LinkProfiles, P3_SyncWord_C9C9C9C9) {
    const auto& g = LINK_PROFILE_TABLE[3].gfsk;
    EXPECT_EQ(g.sync_word[0], 0xC9u);
    EXPECT_EQ(g.sync_word[1], 0xC9u);
    EXPECT_EQ(g.sync_word[2], 0xC9u);
    EXPECT_EQ(g.sync_word[3], 0xC9u);
    EXPECT_EQ(g.sync_word_len, 4u);
}

TEST(LinkProfiles, P3_WhiteningOn) {
    EXPECT_TRUE(LINK_PROFILE_TABLE[3].gfsk.whitening);
}

TEST(LinkProfiles, P3_CRC2Byte) {
    EXPECT_EQ(LINK_PROFILE_TABLE[3].gfsk.crc_type, GfskCrcType::BYTE_2);
}

TEST(LinkProfiles, P4_Bitrate75000) {
    EXPECT_EQ(LINK_PROFILE_TABLE[4].gfsk.bitrate_bps, 75000u);
}

TEST(LinkProfiles, P4_Fdev25k) {
    EXPECT_EQ(LINK_PROFILE_TABLE[4].gfsk.fdev_hz, 25000u);
}

TEST(LinkProfiles, P4_OBW_Carson_ExactlyAtLimit) {
    const auto& g = LINK_PROFILE_TABLE[4].gfsk;
    const uint32_t obw_carson = g.bitrate_bps + 2u * g.fdev_hz;
    // 75000 + 50000 = 125000 — exactly at the IARU limit
    EXPECT_EQ(obw_carson, 125000u);
}

TEST(LinkProfiles, P4_OBW_BT05_Within125k) {
    // BT=0.5 estimate: 1.5 * bitrate = 112500 Hz
    const auto& g = LINK_PROFILE_TABLE[4].gfsk;
    const uint32_t obw_bt05 = (g.bitrate_bps * 3u) / 2u;
    EXPECT_LE(obw_bt05, 125000u);
}

TEST(LinkProfiles, P4_SyncWord_D391D391) {
    const auto& g = LINK_PROFILE_TABLE[4].gfsk;
    EXPECT_EQ(g.sync_word[0], 0xD3u);
    EXPECT_EQ(g.sync_word[1], 0x91u);
    EXPECT_EQ(g.sync_word[2], 0xD3u);
    EXPECT_EQ(g.sync_word[3], 0x91u);
    EXPECT_EQ(g.sync_word_len, 4u);
}

// ---------------------------------------------------------------------------
// P5 GMSK (max-rate GMSK candidate: h=0.5, BT=0.5)
// ---------------------------------------------------------------------------

TEST(LinkProfiles, P5_Bitrate83333) {
    EXPECT_EQ(LINK_PROFILE_TABLE[5].gfsk.bitrate_bps, 83333u);
}

TEST(LinkProfiles, P5_Fdev20833) {
    EXPECT_EQ(LINK_PROFILE_TABLE[5].gfsk.fdev_hz, 20833u);
}

TEST(LinkProfiles, P5_ModIndexIsGmsk) {
    // h = 2*fdev/br must be 0.5 within integer-rounding tolerance (GMSK)
    const auto& g = LINK_PROFILE_TABLE[5].gfsk;
    const uint32_t twice_fdev = 2u * g.fdev_hz;   // 41666
    // |2*fdev - 0.5*br| <= 1 Hz of rounding: 0.5*83333 = 41666.5
    EXPECT_LE(twice_fdev, (g.bitrate_bps + 1u) / 2u + 1u);
    EXPECT_GE(twice_fdev, g.bitrate_bps / 2u - 1u);
}

TEST(LinkProfiles, P5_PulseShapeBT05) {
    EXPECT_EQ(LINK_PROFILE_TABLE[5].gfsk.pulse_shape, GfskPulseShape::BT_05);
}

TEST(LinkProfiles, P5_OBW_Carson_OneHzUnderLimit) {
    const auto& g = LINK_PROFILE_TABLE[5].gfsk;
    // 83333 + 2*20833 = 124999 — one hertz under the IARU limit
    EXPECT_EQ(g.bitrate_bps + 2u * g.fdev_hz, 124999u);
}

TEST(LinkProfiles, P5_RxBwMinimumLegal156k) {
    // SX126x GFSK DSB steps are 117.3k / 156.2k; 117.3k < OBW, so 156.2k
    // is the minimum legal RX bandwidth for this profile.
    EXPECT_EQ(LINK_PROFILE_TABLE[5].gfsk.bw_dsb_hz, 156200u);
}

TEST(LinkProfiles, P5_SyncWord_B27DB27D) {
    const auto& g = LINK_PROFILE_TABLE[5].gfsk;
    EXPECT_EQ(g.sync_word[0], 0xB2u);
    EXPECT_EQ(g.sync_word[1], 0x7Du);
    EXPECT_EQ(g.sync_word[2], 0xB2u);
    EXPECT_EQ(g.sync_word[3], 0x7Du);
    EXPECT_EQ(g.sync_word_len, 4u);
}

TEST(LinkProfiles, GfskProfilesDistinctSyncWords) {
    // All GFSK sync words must differ pairwise to avoid cross-detection
    for (int i = 0; i < LINK_PROFILE_COUNT; ++i) {
        if (LINK_PROFILE_TABLE[i].mod != ModKind::GFSK) continue;
        for (int j = i + 1; j < LINK_PROFILE_COUNT; ++j) {
            if (LINK_PROFILE_TABLE[j].mod != ModKind::GFSK) continue;
            const auto& a = LINK_PROFILE_TABLE[i].gfsk;
            const auto& b = LINK_PROFILE_TABLE[j].gfsk;
            bool same = true;
            for (int k = 0; k < 4; ++k) {
                if (a.sync_word[k] != b.sync_word[k]) { same = false; break; }
            }
            EXPECT_FALSE(same) << "GFSK sync words for profiles " << i
                               << " and " << j << " must be distinct";
        }
    }
}
