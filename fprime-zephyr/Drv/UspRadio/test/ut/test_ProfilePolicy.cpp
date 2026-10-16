// ======================================================================
// test_ProfilePolicy.cpp
// Host-side unit tests for the ProfilePolicy state machine (Phase 3).
//
// Covers:
//   - TX profile switch (valid + invalid index)
//   - RX profile switch with confirm-by-frame
//   - RX auto-revert expiry
//   - Revert counter increment
//   - Invalid profile index guard
//   - TX profile change while pending RX change does not interfere
//   - SET_RX_PROFILE to current profile is a no-op
//   - SET_RX_PROFILE with revert_s = 0 (no revert armed)
//
// Build: included via pcr's CMakeLists.txt host-UT target.
// LINK_PROFILES_USE_HOST_TYPES is injected via target_compile_definitions in
// the test CMakeLists, activating cstdint typedefs in both headers.
// ======================================================================

#include "fprime-zephyr/Drv/UspRadio/ProfilePolicy.hpp"
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"

#include <gtest/gtest.h>

using namespace Zephyr;

// ---------------------------------------------------------------------------
// Boot state
// ---------------------------------------------------------------------------

TEST(ProfilePolicy, BootStateIsTxP0RxP0) {
    ProfilePolicy p;
    EXPECT_EQ(p.txProfile(), BOOT_DEFAULT_PROFILE);
    EXPECT_EQ(p.rxProfile(), BOOT_DEFAULT_PROFILE);
    EXPECT_FALSE(p.hasPendingRx());
    EXPECT_EQ(p.revertCount(), 0u);
}

// ---------------------------------------------------------------------------
// SET_TX_PROFILE
// ---------------------------------------------------------------------------

TEST(ProfilePolicy, SetTxProfile_ValidIdx_ReturnsApplyTx) {
    ProfilePolicy p;
    EXPECT_EQ(p.setTxProfile(1), ProfilePolicy::Action::kApplyTx);
    EXPECT_EQ(p.txProfile(), 1u);
}

TEST(ProfilePolicy, SetTxProfile_OutOfRange_ReturnsInvalidIndex) {
    ProfilePolicy p;
    EXPECT_EQ(p.setTxProfile(LINK_PROFILE_COUNT), ProfilePolicy::Action::kInvalidIndex);
    EXPECT_EQ(p.txProfile(), BOOT_DEFAULT_PROFILE);  // unchanged
}

TEST(ProfilePolicy, SetTxProfile_DoesNotAffectRxProfile) {
    ProfilePolicy p;
    p.setTxProfile(2);
    EXPECT_EQ(p.rxProfile(), BOOT_DEFAULT_PROFILE);
}

TEST(ProfilePolicy, SetTxProfile_AllValidIndices) {
    ProfilePolicy p;
    for (uint8_t i = 0; i < LINK_PROFILE_COUNT; ++i) {
        EXPECT_EQ(p.setTxProfile(i), ProfilePolicy::Action::kApplyTx) << "idx=" << (int)i;
        EXPECT_EQ(p.txProfile(), i);
    }
}

// ---------------------------------------------------------------------------
// SET_RX_PROFILE
// ---------------------------------------------------------------------------

TEST(ProfilePolicy, SetRxProfile_ValidIdx_ReturnsApplyRx) {
    ProfilePolicy p;
    auto action = p.setRxProfile(1, 30, 1000);
    EXPECT_EQ(action, ProfilePolicy::Action::kApplyRx);
    EXPECT_TRUE(p.hasPendingRx());
    EXPECT_EQ(p.pendingRxProfile(), 1u);
    EXPECT_EQ(p.rxProfile(), BOOT_DEFAULT_PROFILE);  // not committed yet
}

TEST(ProfilePolicy, SetRxProfile_SameAsCurrent_ReturnsNoOp) {
    ProfilePolicy p;
    // Default is P0; setting to P0 again is a no-op
    auto action = p.setRxProfile(BOOT_DEFAULT_PROFILE, 30, 1000);
    EXPECT_EQ(action, ProfilePolicy::Action::kNoOp);
    EXPECT_FALSE(p.hasPendingRx());
}

TEST(ProfilePolicy, SetRxProfile_OutOfRange_ReturnsInvalidIndex) {
    ProfilePolicy p;
    auto action = p.setRxProfile(LINK_PROFILE_COUNT, 10, 0);
    EXPECT_EQ(action, ProfilePolicy::Action::kInvalidIndex);
    EXPECT_FALSE(p.hasPendingRx());
}

TEST(ProfilePolicy, SetRxProfile_NoRevert_ZeroSeconds) {
    ProfilePolicy p;
    p.setRxProfile(2, 0, 5000);  // revert_s = 0 → no deadline
    EXPECT_TRUE(p.hasPendingRx());
    // Tick far in the future: no revert should fire
    EXPECT_EQ(p.tick(999999999ULL), ProfilePolicy::Action::kNoOp);
    EXPECT_TRUE(p.hasPendingRx());
}

// ---------------------------------------------------------------------------
// frameReceived — confirm-by-frame
// ---------------------------------------------------------------------------

TEST(ProfilePolicy, FrameReceived_NoPending_ReturnsNoOp) {
    ProfilePolicy p;
    EXPECT_EQ(p.frameReceived(), ProfilePolicy::Action::kNoOp);
}

TEST(ProfilePolicy, FrameReceived_PendingRx_ReturnsConfirmRx) {
    ProfilePolicy p;
    p.setRxProfile(3, 30, 1000);
    auto action = p.frameReceived();
    EXPECT_EQ(action, ProfilePolicy::Action::kConfirmRx);
    EXPECT_EQ(p.rxProfile(), 3u);
    EXPECT_FALSE(p.hasPendingRx());
}

TEST(ProfilePolicy, FrameReceived_AfterConfirm_CancelsRevertDeadline) {
    ProfilePolicy p;
    p.setRxProfile(3, 5, 1000);   // revert after 5 s (deadline = 6000 ms)
    p.frameReceived();             // confirm
    // Tick past original deadline — must NOT revert
    EXPECT_EQ(p.tick(7000), ProfilePolicy::Action::kNoOp);
    EXPECT_EQ(p.revertCount(), 0u);
}

TEST(ProfilePolicy, FrameReceived_AfterConfirm_NoPending) {
    ProfilePolicy p;
    p.setRxProfile(1, 30, 0);
    p.frameReceived();
    EXPECT_EQ(p.frameReceived(), ProfilePolicy::Action::kNoOp);
}

// ---------------------------------------------------------------------------
// Auto-revert expiry
// ---------------------------------------------------------------------------

TEST(ProfilePolicy, Tick_BeforeDeadline_ReturnsNoOp) {
    ProfilePolicy p;
    p.setRxProfile(2, 10, 1000);   // deadline = 11000 ms
    EXPECT_EQ(p.tick(10999), ProfilePolicy::Action::kNoOp);
    EXPECT_TRUE(p.hasPendingRx());
}

TEST(ProfilePolicy, Tick_AtDeadline_ReturnsRevert) {
    ProfilePolicy p;
    p.setRxProfile(2, 10, 1000);   // deadline = 11000 ms
    auto action = p.tick(11000);
    EXPECT_EQ(action, ProfilePolicy::Action::kRevert);
    EXPECT_EQ(p.rxProfile(), BOOT_DEFAULT_PROFILE);
    EXPECT_FALSE(p.hasPendingRx());
    EXPECT_EQ(p.revertCount(), 1u);
    // pendingRxProfile() must still hold the pre-revert "from" profile (P2=2)
    // so callers can log "ProfileReverted from P2 to P0" correctly.
    EXPECT_EQ(p.pendingRxProfile(), 2u);
}

TEST(ProfilePolicy, Tick_AfterDeadline_ReturnsRevert) {
    ProfilePolicy p;
    p.setRxProfile(4, 5, 0);   // deadline = 5000 ms
    EXPECT_EQ(p.tick(5001), ProfilePolicy::Action::kRevert);
    EXPECT_EQ(p.revertCount(), 1u);
}

TEST(ProfilePolicy, Tick_MultipleReverts_CounterAccumulates) {
    ProfilePolicy p;
    for (int i = 0; i < 3; ++i) {
        // Switch to P1 and let it revert each time
        p.setRxProfile(1, 5, static_cast<uint64_t>(i) * 20000ULL);
        p.tick(static_cast<uint64_t>(i) * 20000ULL + 6000ULL);
    }
    EXPECT_EQ(p.revertCount(), 3u);
}

TEST(ProfilePolicy, Tick_NoPendingRx_AlwaysNoOp) {
    ProfilePolicy p;
    EXPECT_EQ(p.tick(99999), ProfilePolicy::Action::kNoOp);
    EXPECT_EQ(p.tick(0),     ProfilePolicy::Action::kNoOp);
}

// ---------------------------------------------------------------------------
// TX profile change does not interfere with pending RX change
// ---------------------------------------------------------------------------

TEST(ProfilePolicy, TxSwitch_DuringPendingRx_NeitherInterferes) {
    ProfilePolicy p;
    p.setRxProfile(3, 30, 1000);
    p.setTxProfile(2);
    EXPECT_TRUE(p.hasPendingRx());
    EXPECT_EQ(p.txProfile(), 2u);
    EXPECT_EQ(p.rxProfile(), BOOT_DEFAULT_PROFILE);
}
