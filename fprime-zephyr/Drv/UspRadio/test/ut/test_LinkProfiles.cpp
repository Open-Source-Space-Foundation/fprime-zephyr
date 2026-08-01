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
// P0 mirrors legacy LoRa link parameters
// ---------------------------------------------------------------------------

TEST(LinkProfiles, P0_PrivateSyncWord) {
    // 0x12 is the private-network sync word (1-byte convention matching GRC loramac-node).
    // ral_set_lora_sync_word(uint8_t) takes this value; the SX126x chip driver
    // computes the 2-byte register encoding 0x1424 internally.
    EXPECT_EQ(LINK_PROFILE_TABLE[0].lora.sync_word, 0x12u);
}

// ---------------------------------------------------------------------------
// P1 long-range profile
// ---------------------------------------------------------------------------

TEST(LinkProfiles, P1_LdroOn) {
    // LDRO recommended for SF10/BW125 (symbol time ~8.2 ms)
    EXPECT_TRUE(LINK_PROFILE_TABLE[1].lora.ldro);
}

// ---------------------------------------------------------------------------
// GFSK profiles: OBW constraint (Carson rule: bitrate + 2*fdev <= 125000)
// ---------------------------------------------------------------------------

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
