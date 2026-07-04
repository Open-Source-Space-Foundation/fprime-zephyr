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
    EXPECT_EQ(static_cast<int>(LINK_PROFILE_COUNT), 5);
}

TEST(LinkProfiles, VersionIsOne) {
    EXPECT_EQ(static_cast<int>(LINK_PROFILE_TABLE_VERSION), 1);
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
    // 0x1424 is the SX126x private-network sync word (loramac-node default)
    EXPECT_EQ(LINK_PROFILE_TABLE[0].lora.sync_word, 0x1424u);
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

TEST(LinkProfiles, GfskProfilesDistinctSyncWords) {
    // P3 and P4 sync words must differ to avoid cross-detection
    const auto& p3 = LINK_PROFILE_TABLE[3].gfsk;
    const auto& p4 = LINK_PROFILE_TABLE[4].gfsk;
    bool same = true;
    for (int b = 0; b < 4; ++b) {
        if (p3.sync_word[b] != p4.sync_word[b]) { same = false; break; }
    }
    EXPECT_FALSE(same) << "P3 and P4 GFSK sync words must be distinct";
}
