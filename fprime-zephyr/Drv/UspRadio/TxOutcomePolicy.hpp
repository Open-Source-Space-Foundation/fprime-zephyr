// ======================================================================
// \title  TxOutcomePolicy.hpp
// \brief  comStatus decision for a completed (successful or failed) TX
//         attempt (wedge-fix item 3, PR #21, 2026-07-24/25).
//
// This header is intentionally free of USP/RAL, Zephyr, and F´ component
// includes so it compiles host-side (unit tests) as well as on-target,
// mirroring the RadioHeadShim.hpp / ProfilePolicy.hpp host-testability
// pattern already used in this directory.
//
// Why this exists
// ----------------
// UspRadio::deferredTxPacket_internalInterfaceHandler() used to report
// Fw::Success::FAILURE as the dataReturnOut comStatus whenever
// m_session->transmitPacket() returned a nonzero rc.  Svc::ComQueue holds
// the downlink queue in WAITING until a SUCCESS comStatus arrives, and no
// later event ever emits one after a FAILURE — so a single dropped frame
// permanently parked the com pipeline.  This was Latch B of HWIL anomaly B
// (2026-07-24, PROVES v5e + GRC bench, rung-9 soak): one TX_DONE timeout
// froze BytesSent for the rest of the run even after the radio itself had
// already recovered (see quiesceRadio() / transmitPacket() timeout recovery
// in RalSessionImpl.cpp for the chip-level half of the fix).
//
// The fix: ALWAYS report SUCCESS ("ready for the next frame"), regardless of
// whether the frame actually made it on air.  The link layer is lossy by
// design; the SendFailed WARNING_HI event remains the operator-visible
// signal for the dropped frame, and BytesSent/throttle-clear only happen on
// an actual successful transmit.
//
// evaluate() below is the pure decision extracted verbatim from
// UspRadio::deferredTxPacket_internalInterfaceHandler() (see UspRadio.cpp) —
// same branches, same outcomes, no behavior change.  It exists so the
// wedge-fix invariant ("comStatus is never FAILURE") has a host-side
// regression test independent of the fprime autocoder / Zephyr build.
// ======================================================================

#ifndef ZEPHYR_USP_RADIO_TX_OUTCOME_POLICY_HPP
#define ZEPHYR_USP_RADIO_TX_OUTCOME_POLICY_HPP

namespace Zephyr {
namespace TxOutcomePolicy {

//! Mirrors the two values of Fw::Success::T that deferredTxPacket cares
//! about, without pulling in F´ headers for the host build.
enum class ComStatus { SUCCESS, FAILURE };

//! Decision + telemetry/event effects for a completed TX attempt.
struct Outcome {
    bool transmitted;     //!< true iff transmitRc == 0 (payload actually sent on air)
    bool logSendFailed;   //!< true iff a WARNING_HI_SendFailed event should be logged
    bool clearThrottles;  //!< true iff the SendFailed/ConfigurationFailed throttles should be cleared
    ComStatus comStatus;  //!< dataReturnOut comStatus — see rationale above: ALWAYS SUCCESS
};

//! Given the raw return code of RalSession::transmitPacket(), decide the
//! comStatus + logging/telemetry effects.  transmitRc == 0 is success;
//! any nonzero value (e.g. -ETIMEDOUT from a TX_DONE timeout) is a dropped
//! frame, which is reported to the operator via SendFailed but never as a
//! FAILURE comStatus (see file header — that was the anomaly-B latch).
inline Outcome evaluate(int transmitRc) {
    Outcome o{};
    o.transmitted = (transmitRc == 0);
    o.logSendFailed = !o.transmitted;
    o.clearThrottles = o.transmitted;
    o.comStatus = ComStatus::SUCCESS;  // never FAILURE, success or drop alike
    return o;
}

}  // namespace TxOutcomePolicy
}  // namespace Zephyr

#endif  // ZEPHYR_USP_RADIO_TX_OUTCOME_POLICY_HPP
