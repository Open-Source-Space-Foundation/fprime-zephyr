// ======================================================================
// \title  ProfilePolicy.hpp
// \brief  Profile-selection state machine (host-compilable, no F´/USP deps).
//
// This class holds the current TX/RX profile indices, the pending-RX-change
// plus revert deadline, and the auto-revert counter.  It is intentionally
// free of F´ framework and USP/Zephyr headers so it compiles for host-side
// unit tests without any special guards.
//
// Caller (UspRadio component thread) supplies millisecond timestamps via
// the nowMs argument so the class never reads a clock directly — making
// it straightforwardly testable.
//
// State machine summary (ADR 0002):
//   - Boot: tx = rx = BOOT_DEFAULT_PROFILE (P0)
//   - SET_TX_PROFILE(idx):
//       * if invalid → kInvalidIndex action
//       * else       → kApplyTx action, update m_txProfile
//   - SET_RX_PROFILE(idx, revert_s):
//       * if invalid → kInvalidIndex action
//       * if idx == current rx → kNoOp
//       * else       → kApplyRx action, arm revert deadline, store m_pendingRxProfile
//   - frameReceived() while pending:
//       * if received on pending profile → kConfirmRx action (cancels timer)
//       * (received on any profile confirms the pending RX profile since the
//          component cannot distinguish which profile decoded it — if a frame
//          arrived within the window, the profile works)
//   - tick(nowMs):
//       * if revert deadline passed → kRevert action, increment revert counter
// ======================================================================

#ifndef ZEPHYR_USP_RADIO_PROFILE_POLICY_HPP
#define ZEPHYR_USP_RADIO_PROFILE_POLICY_HPP

// Use minimal types for host builds; F´ types on target.
#ifndef LINK_PROFILES_USE_HOST_TYPES
#include <Fw/FPrimeBasicTypes.hpp>
#else
#include <cstdint>
using U8  = uint8_t;
using U16 = uint16_t;
using U32 = uint32_t;
using I8  = int8_t;
using I16 = int16_t;
#endif

#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"

namespace Zephyr {

class ProfilePolicy {
  public:
    // ------------------------------------------------------------------
    // Action codes returned to the component
    // ------------------------------------------------------------------
    enum class Action : U8 {
        kNoOp,         //!< Nothing to do
        kApplyTx,      //!< Apply m_txProfile to the TX path
        kApplyRx,      //!< Apply m_pendingRxProfile to the RX path (new pending active)
        kConfirmRx,    //!< Pending RX profile confirmed by frame; no further action needed
        kRevert,       //!< Revert RX to m_rxProfile (= boot default after revert)
        kInvalidIndex, //!< Caller supplied an out-of-range profile index
    };

    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------
    ProfilePolicy();

    // ------------------------------------------------------------------
    // Commands (called from component thread)
    // ------------------------------------------------------------------

    //! Set the TX profile.
    //! @param idx   Profile index (must be < LINK_PROFILE_COUNT).
    //! @returns kApplyTx on success, kInvalidIndex if out of range.
    Action setTxProfile(U8 idx);

    //! Set the RX profile with auto-revert guard.
    //! @param idx      Profile index.
    //! @param revert_s Revert timeout in seconds (0 = no revert).
    //! @param nowMs    Current monotonic timestamp in milliseconds.
    //! @returns kApplyRx on success, kInvalidIndex if out of range, kNoOp if same as current.
    Action setRxProfile(U8 idx, U16 revert_s, uint64_t nowMs);

    // ------------------------------------------------------------------
    // Events (called from component thread, triggered by radio events)
    // ------------------------------------------------------------------

    //! Notify that a valid frame was received.  If a pending RX profile
    //! change is waiting for confirmation, this confirms it.
    //! @returns kConfirmRx if a pending profile was confirmed, kNoOp otherwise.
    Action frameReceived();

    //! Rate-group tick — check revert deadline.
    //! @param nowMs   Current monotonic timestamp in milliseconds.
    //! @returns kRevert if deadline passed, kNoOp otherwise.
    Action tick(uint64_t nowMs);

    // ------------------------------------------------------------------
    // Accessors (read-only)
    // ------------------------------------------------------------------
    U8 txProfile()      const { return m_txProfile; }
    U8 rxProfile()      const { return m_rxProfile; }      //!< The committed RX profile
    //!< Profile awaiting confirmation (or, after kRevert, the profile that was
    //!< pending before the revert — valid to read as "from" in ProfileReverted).
    U8 pendingRxProfile() const { return m_pendingRxProfile; }
    bool hasPendingRx() const { return m_hasPendingRx; }
    U32 revertCount()   const { return m_revertCount; }

  private:
    U8      m_txProfile;          //!< Current TX profile index
    U8      m_rxProfile;          //!< Committed (confirmed) RX profile index
    U8      m_pendingRxProfile;   //!< Candidate RX profile pending confirmation
    bool    m_hasPendingRx;       //!< True if awaiting confirmation
    uint64_t m_revertDeadlineMs;  //!< Deadline in ms (0 = no revert armed)
    U32     m_revertCount;        //!< Number of auto-reverts since boot
};

}  // namespace Zephyr

#endif  // ZEPHYR_USP_RADIO_PROFILE_POLICY_HPP
