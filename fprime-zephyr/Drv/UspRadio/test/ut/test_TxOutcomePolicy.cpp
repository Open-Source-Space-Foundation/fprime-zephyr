// ======================================================================
// test_TxOutcomePolicy.cpp
// Host-side unit tests for the deferredTxPacket comStatus decision
// (wedge-fix item 3, PR #21, 2026-07-24/25 — HWIL anomaly B Latch B).
//
// Covers:
//   - Successful TX: transmitted, no SendFailed, throttles cleared, SUCCESS
//   - Failed TX (arbitrary nonzero rc, incl. -ETIMEDOUT): not transmitted,
//     SendFailed logged, throttles NOT cleared, comStatus is still SUCCESS
//   - Regression guard: comStatus is NEVER FAILURE for any rc, positive or
//     negative — this is the exact invariant whose absence caused
//     Svc::ComQueue to park in WAITING forever after one dropped frame
//   - transmitted/logSendFailed/clearThrottles are mutually exclusive and
//     rc-dependent (transmitted iff rc == 0)
//
// TxOutcomePolicy.hpp is header-only and free of F'/USP/Zephyr includes
// (no LINK_PROFILES_USE_HOST_TYPES needed — plain int/bool only), so it
// compiles host-side unmodified.
//
// Build: included via pcr's CMakeLists.txt host-UT target.
// ======================================================================

#include "fprime-zephyr/Drv/UspRadio/TxOutcomePolicy.hpp"

#include <cstdint>
#include <gtest/gtest.h>

using namespace Zephyr;

// ---------------------------------------------------------------------------
// Success path (rc == 0)
// ---------------------------------------------------------------------------

TEST(TxOutcomePolicy, SuccessTransmitsAndClearsThrottles) {
    const auto o = TxOutcomePolicy::evaluate(0);
    EXPECT_TRUE(o.transmitted);
    EXPECT_FALSE(o.logSendFailed);
    EXPECT_TRUE(o.clearThrottles);
    EXPECT_EQ(o.comStatus, TxOutcomePolicy::ComStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// Failure path (rc != 0) — the anomaly-B regression surface
// ---------------------------------------------------------------------------

TEST(TxOutcomePolicy, TimeoutFailureDropsFrameButReportsComStatusSuccess) {
    // -ETIMEDOUT: the exact rc RalSessionImpl::transmitPacket() returns after
    // the TX_DONE timeout / recovery path (HWIL anomaly B trigger).
    const auto o = TxOutcomePolicy::evaluate(-116);
    EXPECT_FALSE(o.transmitted);
    EXPECT_TRUE(o.logSendFailed);
    EXPECT_FALSE(o.clearThrottles);
    // THE regression guard: pre-fix this reported FAILURE here, which parks
    // Svc::ComQueue in WAITING forever (no later SUCCESS event ever arrives).
    EXPECT_EQ(o.comStatus, TxOutcomePolicy::ComStatus::SUCCESS);
}

TEST(TxOutcomePolicy, ComStatusIsNeverFailureRegardlessOfRc) {
    // Sweep a range of plausible negative errno-style rcs plus a couple of
    // positive/edge values — comStatus must be SUCCESS in every case. This
    // is the invariant whose violation caused the permanent downlink park.
    const int rcs[] = {-116 /*ETIMEDOUT*/, -5 /*EIO*/, -1, 1, 2, INT32_MIN, INT32_MAX};
    for (int rc : rcs) {
        const auto o = TxOutcomePolicy::evaluate(rc);
        EXPECT_EQ(o.comStatus, TxOutcomePolicy::ComStatus::SUCCESS) << "rc=" << rc;
    }
}

TEST(TxOutcomePolicy, FailureLogsSendFailedButDoesNotClearThrottles) {
    const auto o = TxOutcomePolicy::evaluate(-5);
    EXPECT_TRUE(o.logSendFailed);
    EXPECT_FALSE(o.clearThrottles);
}

// ---------------------------------------------------------------------------
// transmitted iff rc == 0
// ---------------------------------------------------------------------------

TEST(TxOutcomePolicy, TransmittedIsExactlyRcEqualsZero) {
    EXPECT_TRUE(TxOutcomePolicy::evaluate(0).transmitted);
    EXPECT_FALSE(TxOutcomePolicy::evaluate(1).transmitted);
    EXPECT_FALSE(TxOutcomePolicy::evaluate(-1).transmitted);
}

TEST(TxOutcomePolicy, LogSendFailedAndTransmittedAreMutuallyExclusive) {
    for (int rc : {0, 1, -1, -116}) {
        const auto o = TxOutcomePolicy::evaluate(rc);
        EXPECT_NE(o.transmitted, o.logSendFailed) << "rc=" << rc;
    }
}
