// ======================================================================
// \title  ProfilePolicy.cpp
// \brief  Profile-selection state machine implementation.
// ======================================================================

// LINK_PROFILES_USE_HOST_TYPES is defined by the test runner or host build
// target (see test CMakeLists.txt).  On Zephyr/F' target builds, F'
// FPrimeBasicTypes.hpp is on the path; do not define the macro here.
#include "fprime-zephyr/Drv/UspRadio/ProfilePolicy.hpp"

namespace Zephyr {

ProfilePolicy::ProfilePolicy()
    : m_txProfile(BOOT_DEFAULT_PROFILE),
      m_rxProfile(BOOT_DEFAULT_PROFILE),
      m_pendingRxProfile(BOOT_DEFAULT_PROFILE),
      m_hasPendingRx(false),
      m_revertDeadlineMs(0),
      m_revertCount(0) {}

// ---------------------------------------------------------------------------

ProfilePolicy::Action ProfilePolicy::setTxProfile(U8 idx) {
    if (idx >= LINK_PROFILE_COUNT) {
        return Action::kInvalidIndex;
    }
    m_txProfile = idx;
    return Action::kApplyTx;
}

ProfilePolicy::Action ProfilePolicy::setRxProfile(U8 idx, U16 revert_s, uint64_t nowMs) {
    if (idx >= LINK_PROFILE_COUNT) {
        return Action::kInvalidIndex;
    }
    if (idx == m_rxProfile && !m_hasPendingRx) {
        return Action::kNoOp;
    }
    m_pendingRxProfile = idx;
    m_hasPendingRx = true;

    if (revert_s > 0) {
        m_revertDeadlineMs = nowMs + static_cast<uint64_t>(revert_s) * 1000ULL;
    } else {
        m_revertDeadlineMs = 0;  // no revert armed
    }
    return Action::kApplyRx;
}

ProfilePolicy::Action ProfilePolicy::frameReceived() {
    if (!m_hasPendingRx) {
        return Action::kNoOp;
    }
    // Confirm the pending profile
    m_rxProfile = m_pendingRxProfile;
    m_hasPendingRx = false;
    m_revertDeadlineMs = 0;
    return Action::kConfirmRx;
}

ProfilePolicy::Action ProfilePolicy::tick(uint64_t nowMs) {
    if (!m_hasPendingRx || m_revertDeadlineMs == 0) {
        return Action::kNoOp;
    }
    if (nowMs >= m_revertDeadlineMs) {
        // Revert to boot default.
        // m_pendingRxProfile holds the pending (non-committed) profile (e.g. P1).
        // After revert, m_rxProfile = BOOT_DEFAULT.  We leave m_pendingRxProfile
        // intact so callers can read pendingRxProfile() to get the "from" value
        // for the ProfileReverted log message before clearing state.
        m_rxProfile = BOOT_DEFAULT_PROFILE;
        m_hasPendingRx = false;
        m_revertDeadlineMs = 0;
        ++m_revertCount;
        return Action::kRevert;
    }
    return Action::kNoOp;
}

}  // namespace Zephyr
