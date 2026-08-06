// ======================================================================
// \title  ProfilePolicy.hpp
// \brief  Profile-selection state machine (host-compilable, no F´/USP deps).
//         Holds TX/RX profile indices, pending RX change + revert deadline,
//         and the auto-revert counter (ADR 0002).  Callers supply timestamps
//         via nowMs so the class never reads a clock — testable host-side.
//         Any frame received while pending confirms the pending RX profile.
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

    //! The profile the RX hardware should currently be listening on.
    //!
    //! This is the pending candidate while one is awaiting confirmation, and
    //! the committed profile otherwise.  It is NOT rxProfile(): a pending
    //! change is confirmed by receiving a frame on the NEW profile, so the
    //! radio has to actually be armed on it — arming the committed profile
    //! instead makes confirmation impossible (no frame can ever arrive on a
    //! profile the chip isn't listening to), so the change silently never
    //! takes effect and the revert deadline always expires.
    //! Use this for every hardware RX arm; use rxProfile() only to report
    //! what is confirmed.
    U8 armedRxProfile() const { return m_hasPendingRx ? m_pendingRxProfile : m_rxProfile; }
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
