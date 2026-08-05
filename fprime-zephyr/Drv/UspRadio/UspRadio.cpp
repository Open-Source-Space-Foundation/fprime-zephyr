// ======================================================================
// \title  UspRadio.cpp
// \brief  Active UHF radio component using Semtech USP (RAL) on SX1262.
//         See UspRadio.hpp for the threading model and buffer ownership.
// ======================================================================

#include "fprime-zephyr/Drv/UspRadio/UspRadio.hpp"
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"
#include "fprime-zephyr/Drv/UspRadio/RadioHeadShim.hpp"
#include "fprime-zephyr/Drv/UspRadio/TxOutcomePolicy.hpp"

#include <Fw/Logger/Logger.hpp>
#include <cstring>

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#endif

namespace Zephyr {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

UspRadio::UspRadio(const char* compName)
    : UspRadioComponentBase(compName),
      m_session(nullptr),
      m_policy(),
      m_transmitState(UspTransmitState::DISABLED),
      m_rxHead(0),
      m_rxTail(0),
      m_rxDropped(0),
      m_rxDroppedReported(0),
      m_rxRssi(0),
      m_rxSnr(0),
      m_pendingTxFrames(0),
      m_rearmAfterTx(false),
      m_bytesSent(0),
      m_bytesReceived(0),
      m_rxReverts(0),
      m_revertRearmPending(false),
      m_radioEverOn(false),
      m_comStatusOwed(false) {}

UspRadio::~UspRadio() {}

void UspRadio::configure(RalSession& session) {
    m_session = &session;
}

// ---------------------------------------------------------------------------
// startRadio()
// ---------------------------------------------------------------------------

bool UspRadio::startRadio(UspTransmitState initialTransmitState) {
    m_transmitState = initialTransmitState;

    // Register RX callback (fires on USP thread → posts internal msg)
    m_session->setCallbacks(
        [this](const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr) {
            this->onRxDone(data, size, rssi, snr);
        });

    if (m_session->init() != 0) {
        Fw::Logger::log("[UspRadio] session.init() failed\n");
        return false;
    }

    // Apply boot-default profile to the radio hardware
    if (!applyProfile(BOOT_DEFAULT_PROFILE, UspRadioDirection::RX)) {
        return false;
    }
    if (!applyProfile(BOOT_DEFAULT_PROFILE, UspRadioDirection::TX)) {
        return false;
    }

    if (m_session->startReceive() != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
        return false;
    }

    // If initially enabled, kick off the ping-pong comStatus protocol.
    // Guard with m_comStatusOwed for defense-in-depth (startRadio() only runs
    // once, before any frame could have primed it, so this should always be
    // false here) — see m_comStatusOwed for the one-in-flight invariant.
    if ((initialTransmitState == UspTransmitState::ENABLED) &&
        !m_comStatusOwed.load(std::memory_order_relaxed)) {
        Fw::Success status = Fw::Success::SUCCESS;
        this->comStatusOut_out(0, status);
        m_comStatusOwed.store(true, std::memory_order_relaxed);
        this->signalFirstStart();
    }

    // Boot-flush telemetry so channels have initial values before first activity.
    this->tlmWrite_BytesSent(m_bytesSent);
    this->tlmWrite_BytesReceived(m_bytesReceived);
    this->tlmWrite_LastRssi(m_rxRssi);
    this->tlmWrite_LastSnr(m_rxSnr);
    this->tlmWrite_TxProfile(static_cast<LinkProfileId::T>(m_policy.txProfile()));
    this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
    this->tlmWrite_RxReverts(m_rxReverts);
    this->tlmWrite_RxDropped(m_rxDropped.load(std::memory_order_relaxed));
    return true;
}

// ---------------------------------------------------------------------------
// onRxDone() — called on USP thread
// ---------------------------------------------------------------------------

void UspRadio::onRxDone(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr) {
    // Enqueue into the RX ring (USP thread → component thread boundary).
    // One ring slot + one internal message per frame, so the consumer never
    // re-reads a slot (the single-scratch predecessor delivered the LATEST
    // frame once per queued message at saturation — duplicates + loss).
    const U32 head = m_rxHead.load(std::memory_order_relaxed);
    const U32 tail = m_rxTail.load(std::memory_order_acquire);
    if ((head - tail) >= RX_RING_DEPTH) {
        // Ring full: drop.  No F´ ports from the USP thread — the component
        // thread notices the counter change and emits RxOverrun/RxDropped.
        m_rxDropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    RxSlot& slot = m_rxRing[head % RX_RING_DEPTH];
    const FwSizeType safeSize =
        (size > RX_SCRATCH_SIZE) ? RX_SCRATCH_SIZE : static_cast<FwSizeType>(size);
    (void)::memcpy(slot.data, data, safeSize);
    slot.len  = safeSize;
    slot.rssi = rssi;
    slot.snr  = snr;
    // Publish the slot before the message can be consumed.
    m_rxHead.store(head + 1, std::memory_order_release);

    // Post internal message — runs on component thread
    this->deferredRxDone_internalInterfaceInvoke();
}

// ---------------------------------------------------------------------------
// run_handler — rate-group tick
// ---------------------------------------------------------------------------

void UspRadio::run_handler(FwIndexType /*portNum*/, U32 /*context*/) {
    ProfilePolicy::Action action = m_policy.tick(nowMs());
    if (action == ProfilePolicy::Action::kRevert) {
        U8 fromProfile = m_policy.pendingRxProfile();  // saved "from" before revert
        U8 toProfile   = m_policy.rxProfile();

        ++m_rxReverts;
        this->log_WARNING_LO_ProfileReverted(
            static_cast<LinkProfileId::T>(fromProfile),
            static_cast<LinkProfileId::T>(toProfile));
        this->tlmWrite_RxReverts(m_rxReverts);
        this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(toProfile));

        // tick() has already committed the revert in the policy — only the
        // HARDWARE apply remains.  Mark it pending so a busy radio retries on
        // subsequent ticks instead of dropping the revert.
        m_revertRearmPending = true;
    }

    if (m_revertRearmPending) {
        // stopRadio() must succeed before applyProfile()/startReceive() can
        // (RAC ALREADY_RUNNING guard — see deferredTxPacket).  No external
        // retry exists for a revert: on failure keep the flag and retry next tick.
        if (m_session->stopRadio() != 0) {
            this->log_WARNING_LO_ProfileChangeDeferred(UspRadioDirection::RX);
            return;
        }
        if (!applyProfile(m_policy.rxProfile(), UspRadioDirection::RX)) {
            return;  // ConfigurationFailed logged by applyProfile; retry next tick
        }
        if (m_session->startReceive() != 0) {
            this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
            return;  // retry next tick
        }
        m_revertRearmPending = false;
    }
}

// ---------------------------------------------------------------------------
// dataIn_handler — enqueue TX (component thread does the work)
// ---------------------------------------------------------------------------

void UspRadio::dataIn_handler(FwIndexType /*portNum*/,
                               Fw::Buffer& data,
                               const ComCfg::FrameContext& context) {
    // Count before enqueue: deferredTxPacket sees >0 while more frames are
    // in flight and skips the per-frame RX re-arm (see m_pendingTxFrames).
    m_pendingTxFrames.fetch_add(1, std::memory_order_relaxed);
    // A frame arriving here proves Svc::ComQueue dequeued and sent it, which
    // only happens after it consumed our last outstanding comStatus (i.e. it
    // was READY and is now WAITING again).  Clear the debt so the next
    // comStatusOut_out (emitted at the end of deferredTxPacket, on the
    // component thread) is known-safe.  See m_comStatusOwed.
    m_comStatusOwed.store(false, std::memory_order_relaxed);
    this->deferredTxPacket_internalInterfaceInvoke(data, context);
}

void UspRadio::dataReturnIn_handler(FwIndexType /*portNum*/,
                                    Fw::Buffer& data,
                                    const ComCfg::FrameContext& /*context*/) {
    this->deallocate_out(0, data);
}

// ---------------------------------------------------------------------------
// deferredRxDone — component thread
// ---------------------------------------------------------------------------

void UspRadio::deferredRxDone_internalInterfaceHandler() {
    // Pop one ring slot (the slot carries the frame's data and rssi/snr).
    const U32 tail = m_rxTail.load(std::memory_order_relaxed);
    const U32 head = m_rxHead.load(std::memory_order_acquire);
    if (tail == head) {
        // Message without a slot: can only happen if the producer dropped a
        // frame between our reads — nothing to deliver.
        return;
    }
    const RxSlot& slot = m_rxRing[tail % RX_RING_DEPTH];

    // Notify ProfilePolicy that a frame was received
    ProfilePolicy::Action action = m_policy.frameReceived();
    if (action == ProfilePolicy::Action::kConfirmRx) {
        this->log_ACTIVITY_LO_ProfileConfirmed(
            static_cast<LinkProfileId::T>(m_policy.rxProfile()));
        this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
    }

    m_rxRssi = slot.rssi;
    m_rxSnr  = slot.snr;

    // Locate the deliverable payload within the on-air packet.  In
    // RadioHead-compat mode the first 4 bytes are the peer's RadioHead
    // header (see RadioHeadShim.hpp) — strip them.  A header-only runt
    // carries no payload: consume the slot without delivering (it still
    // confirmed the RX profile above — it was a valid frame on this profile).
    const bool rhCompat = this->radioHeadCompatFor(m_policy.rxProfile());
    U32 payloadOffset = 0;
    U32 payloadLen = 0;
    if (RadioHeadShim::locateRxPayload(static_cast<U32>(slot.len), rhCompat,
                                       payloadOffset, payloadLen)) {
        // Allocate and dispatch the received frame
        const FwSizeType payloadSize = static_cast<FwSizeType>(payloadLen);
        Fw::Buffer buffer = this->allocate_out(0, payloadSize);
        if (buffer.isValid()) {
            (void)::memcpy(buffer.getData(), slot.data + payloadOffset, payloadSize);
            // Slot contents fully copied out — release it back to the producer.
            m_rxTail.store(tail + 1, std::memory_order_release);

            ComCfg::FrameContext frameContext;
            this->dataOut_out(0, buffer, frameContext);

            m_bytesReceived += payloadSize;
            this->tlmWrite_BytesReceived(m_bytesReceived);
        } else {
            m_rxTail.store(tail + 1, std::memory_order_release);
            this->log_WARNING_HI_AllocationFailed(payloadSize);
        }
    } else {
        m_rxTail.store(tail + 1, std::memory_order_release);
    }

    this->tlmWrite_LastRssi(m_rxRssi);
    this->tlmWrite_LastSnr(m_rxSnr);

    // Surface producer-side drops (ring full) from the component thread.
    const U32 dropped = m_rxDropped.load(std::memory_order_relaxed);
    if (dropped != m_rxDroppedReported) {
        m_rxDroppedReported = dropped;
        this->tlmWrite_RxDropped(dropped);
        this->log_WARNING_HI_RxOverrun(dropped);
    }
}

// ---------------------------------------------------------------------------
// deferredTxPacket — component thread
// ---------------------------------------------------------------------------

void UspRadio::deferredTxPacket_internalInterfaceHandler(const Fw::Buffer& data,
                                                          const ComCfg::FrameContext& context) {
    // This frame is now being processed — no longer pending.
    m_pendingTxFrames.fetch_sub(1, std::memory_order_relaxed);

    Fw::Buffer mutableData = data;  // cast away const for dataReturnOut
    Fw::Success returnStatus = Fw::Success::FAILURE;

    if (m_transmitState == UspTransmitState::ENABLED) {
        // RadioHead-compat shim (see RadioHeadShim.hpp): legacy peers expect a
        // 4-byte header on every LoRa frame; it consumes on-air budget, so the
        // payload cap shrinks accordingly (252 → 248 = TmFrameFixedSize).
        const bool rhCompat = this->radioHeadCompatFor(m_policy.txProfile());
        const FwSizeType maxPayload = static_cast<FwSizeType>(
            RadioHeadShim::maxPayload(static_cast<U32>(MAX_PACKET_SIZE), rhCompat));
        FwSizeType size = data.getSize();
        if (size > maxPayload) {
            size = maxPayload;  // truncate
        }

        // Stop continuous RX before TX.  startReceive() holds the RAC lock
        // window open indefinitely (RP_TASK_STATE_RUNNING on this hook_id),
        // so a subsequent smtc_rac_lock_radio_access call for TX would return
        // RP_TASK_STATUS_ALREADY_RUNNING → SMTC_RAC_ERROR → -EIO.
        // stopRadio() acquires its own lock window to put the chip in standby,
        // releasing the continuous-RX lock so transmitPacket() can proceed.
        // (When the previous frame skipped its re-arm, RX is not RUNNING and
        // this hits stopRadio's was-idle fast path — cheap.)
        (void)m_session->stopRadio();

        // Compat mode stages header + payload in m_txScratch; raw mode
        // transmits the frame buffer directly (no copy).
        const U8* txData = data.getData();
        std::size_t txSize = static_cast<std::size_t>(size);
        if (rhCompat) {
            txSize = static_cast<std::size_t>(
                RadioHeadShim::buildTxPacket(data.getData(), static_cast<U32>(size), m_txScratch));
            txData = m_txScratch;
        }

        int rc = m_session->transmitPacket(txData, txSize);
        // Decision extracted to TxOutcomePolicy::evaluate() (host-testable,
        // see its file header for the anomaly-B rationale) — same branches,
        // same outcomes as before the extraction, no behavior change.
        const TxOutcomePolicy::Outcome outcome = TxOutcomePolicy::evaluate(rc);
        if (outcome.logSendFailed) {
            this->log_WARNING_HI_SendFailed(static_cast<I32>(rc));
        }
        if (outcome.transmitted) {
            // Payload bytes only (header excluded) — keeps BytesSent/BytesReceived
            // parity checks meaningful across compat and raw peers.
            m_bytesSent += size;
            this->tlmWrite_BytesSent(m_bytesSent);
        }
        if (outcome.clearThrottles) {
            this->log_WARNING_HI_ConfigurationFailed_ThrottleClear();
            this->log_WARNING_HI_SendFailed_ThrottleClear();
        }
        // ALWAYS SUCCESS, transmitted or not — see TxOutcomePolicy.hpp.
        // Svc::ComQueue parks in WAITING forever on a FAILURE comStatus with
        // no later SUCCESS event; that permanent-drop latch was HWIL
        // anomaly B (2026-07-24).  SendFailed above remains the
        // operator-visible signal for the dropped frame.
        returnStatus =
            (outcome.comStatus == TxOutcomePolicy::ComStatus::SUCCESS) ? Fw::Success::SUCCESS : Fw::Success::FAILURE;

        // Re-arm continuous RX after TX — deferred to the handler tail, AFTER
        // comStatusOut releases the one-in-flight com pipeline (see below).
        m_rearmAfterTx = true;
    } else if (m_transmitState == UspTransmitState::DISABLING) {
        m_transmitState = UspTransmitState::DISABLED;
        // TX episode over — guarantee continuous RX is armed or the board
        // stays deaf after TRANSMIT DISABLED.  Stop first: if the last frame's
        // re-arm succeeded, a bare startReceive() would fail ALREADY_RUNNING.
        if ((m_session->stopRadio() != 0) || (m_session->startReceive() != 0)) {
            this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
        }
    }

    this->dataReturnOut_out(0, mutableData, context);
    // dataIn_handler cleared m_comStatusOwed when this frame arrived (it
    // proved ComQueue consumed the prior status and is WAITING again), so
    // this emission is always the one status ComQueue expects per frame —
    // no guard needed, just re-mark the debt so the next priming/attempt
    // knows a status is now outstanding.  See m_comStatusOwed.
    this->comStatusOut_out(0, returnStatus);
    m_comStatusOwed.store(true, std::memory_order_relaxed);

    if (m_rearmAfterTx) {
        m_rearmAfterTx = false;
        // comStatus is out — if the com pipeline has another frame it reaches
        // dataIn within a few ms.  Give it a short bounded window; if a frame
        // shows up, skip the re-arm (the TX→re-arm→stop→TX dance costs
        // ~50 ms/frame and throttles saturated TX).  Deaf-safe: the episode's
        // final frame exhausts the window and re-arms, and the
        // DISABLING/DISABLED seams re-arm unconditionally.
#ifdef __ZEPHYR__
        for (int i = 0; (i < 5) && (m_pendingTxFrames.load(std::memory_order_relaxed) == 0); i++) {
            k_sleep(K_MSEC(1));
        }
#endif
        if (m_pendingTxFrames.load(std::memory_order_relaxed) == 0) {
            if (m_session->startReceive() != 0) {
                this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// enableTransmit / disableTransmit — StartupManager quiescence contract
//
// Fw.Signal sync input ports: they arrive on the *caller's* thread (the rate
// group driving StartupManager).  They therefore do no radio work inline —
// they enqueue the same internal message the async TRANSMIT command uses, so
// there is exactly one transmit-state machine and one thread that touches the
// RAL.  Semantics are verbatim from Zephyr::LoRa::setTransmitState():
// enable -> ENABLED, disable -> DISABLING.
// ---------------------------------------------------------------------------

void UspRadio::enableTransmit_handler(FwIndexType portNum) {
    (void)portNum;
    this->deferredTransmitCmd_internalInterfaceInvoke(UspTransmitState::ENABLED);
}

void UspRadio::disableTransmit_handler(FwIndexType portNum) {
    (void)portNum;
    this->deferredTransmitCmd_internalInterfaceInvoke(UspTransmitState::DISABLING);
}

void UspRadio::signalFirstStart() {
    if (m_radioEverOn) {
        return;
    }
    m_radioEverOn = true;
    if (this->isConnected_radioFirstStart_OutputPort(0)) {
        this->radioFirstStart_out(0);
    }
}

// ---------------------------------------------------------------------------
// deferredTransmitCmd — component thread
// ---------------------------------------------------------------------------

void UspRadio::deferredTransmitCmd_internalInterfaceHandler(const UspTransmitState& enabled) {
    if (enabled == UspTransmitState::ENABLED) {
        // Only prime ComQueue's pipeline if no comStatus is currently
        // outstanding (ComQueue is WAITING).  Without this guard, a
        // DISABLED->ENABLED transition after a deaf-recovery
        // DISABLED/DISABLED cycle (repeat TRANSMIT DISABLED, a documented
        // operator lever elsewhere in this handler) with no frame ever
        // consumed in between would emit a second SUCCESS into an
        // already-READY ComQueue, which is an FW_ASSERT/crash-loop bug in
        // the pinned Svc::ComQueue.  See m_comStatusOwed.
        if ((m_transmitState == UspTransmitState::DISABLED) &&
            !m_comStatusOwed.load(std::memory_order_relaxed)) {
            Fw::Success comStatus = Fw::Success::SUCCESS;
            this->comStatusOut_out(0, comStatus);
            m_comStatusOwed.store(true, std::memory_order_relaxed);
        }
        m_transmitState = UspTransmitState::ENABLED;
        this->signalFirstStart();
    } else {
        if (m_transmitState == UspTransmitState::ENABLED) {
            // A trailing frame is expected: its DISABLING→DISABLED transition
            // in deferredTxPacket completes the episode and re-arms RX.
            m_transmitState = UspTransmitState::DISABLING;
        } else {
            // DISABLING with no trailing frame in the queue, or already
            // DISABLED: no send cycle will run the re-arm — guarantee
            // continuous RX here.  This also gives operators an explicit
            // recover-RX lever (repeat TRANSMIT DISABLED), since a
            // same-profile SET_RX_PROFILE is a kNoOp and does not re-arm.
            m_transmitState = UspTransmitState::DISABLED;
            if ((m_session->stopRadio() != 0) || (m_session->startReceive() != 0)) {
                this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// deferredSetTxProfile — component thread
// ---------------------------------------------------------------------------

void UspRadio::deferredSetTxProfile_internalInterfaceHandler(const LinkProfileId& profile) {
    U8 idx = static_cast<U8>(profile.e);
    ProfilePolicy::Action action = m_policy.setTxProfile(idx);
    if (action == ProfilePolicy::Action::kInvalidIndex) {
        this->log_WARNING_HI_InvalidProfile(profile);
        return;
    }
    // Stop continuous RX before applying (RAC ALREADY_RUNNING guard — see
    // deferredTxPacket).  On failure skip the apply: the policy already holds
    // m_txProfile, and a later SET_TX_PROFILE retry applies it to hardware.
    if (m_session->stopRadio() != 0) {
        this->log_WARNING_LO_ProfileChangeDeferred(UspRadioDirection::TX);
        return;
    }
    (void)applyProfile(idx, UspRadioDirection::TX);
    // Re-arm continuous RX — otherwise the radio is left deaf in standby.
    if (m_session->startReceive() != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
    }
    this->tlmWrite_TxProfile(profile);
}

// ---------------------------------------------------------------------------
// deferredSetRxProfile — component thread
// ---------------------------------------------------------------------------

void UspRadio::deferredSetRxProfile_internalInterfaceHandler(const LinkProfileId& profile,
                                                              U16 revert_s) {
    U8 idx = static_cast<U8>(profile.e);
    ProfilePolicy::Action action = m_policy.setRxProfile(idx, revert_s, nowMs());
    if (action == ProfilePolicy::Action::kInvalidIndex) {
        this->log_WARNING_HI_InvalidProfile(profile);
        return;
    }
    if (action == ProfilePolicy::Action::kNoOp) {
        // Same-profile SET_RX_PROFILE: no policy/revert change, but treat it as
        // an explicit "ensure RX is armed on this profile" request (operator
        // deaf-recovery lever).  Re-apply too: a preceding TX episode may have
        // left TX-profile modulation params on the chip.
        if (m_session->stopRadio() != 0) {
            this->log_WARNING_LO_ProfileChangeDeferred(UspRadioDirection::RX);
            return;
        }
        (void)applyProfile(idx, UspRadioDirection::RX);
        if (m_session->startReceive() != 0) {
            this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
            return;
        }
        // RX is now armed on an explicitly requested profile — supersedes any
        // pending revert hardware re-arm.
        m_revertRearmPending = false;
        return;
    }
    // kApplyRx — stop RX, apply the new profile, re-arm RX (RAC ALREADY_RUNNING
    // guard — see deferredTxPacket).  On stop failure skip the apply: pending
    // profile + revert deadline stay untouched, so the auto-revert still fires
    // on schedule and a later retry re-attempts with fresh state.
    if (m_session->stopRadio() != 0) {
        this->log_WARNING_LO_ProfileChangeDeferred(UspRadioDirection::RX);
        return;
    }
    if (!applyProfile(idx, UspRadioDirection::RX)) {
        return;
    }
    if (m_session->startReceive() != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
    } else {
        // RX is now armed on an explicitly requested profile — supersedes any
        // pending revert hardware re-arm.
        m_revertRearmPending = false;
    }
    this->log_ACTIVITY_HI_ProfileChanged(UspRadioDirection::RX, profile);
    this->tlmWrite_RxProfile(profile);
}

// ---------------------------------------------------------------------------
// deferredContinuousWave — component thread
// ---------------------------------------------------------------------------

void UspRadio::deferredContinuousWave_internalInterfaceHandler(U16 seconds) {
    // Stop continuous RX before CW (RAC ALREADY_RUNNING guard — see deferredTxPacket).
    (void)m_session->stopRadio();

    // CW uses the session's configured freq/power; pkt-type is forced to LoRa
    // by RalSessionImpl per the RAL CW requirement.
    int rc = m_session->startCw();
    if (rc != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::TX);
        return;
    }

    // Block the component thread for the CW duration (component queue stalls;
    // the USP thread keeps running the RAC).  Acceptable per safety limits
    // for durations <= 10 s.
#ifdef __ZEPHYR__
    k_sleep(K_SECONDS(seconds));
#else
    // Host test stub: no sleep
    (void)seconds;
#endif

    // Stop CW and return to RX
    (void)m_session->stopRadio();

    // Re-apply current RX profile and restart receive
    U8 rxIdx = m_policy.rxProfile();
    (void)applyProfile(rxIdx, UspRadioDirection::RX);
    if (m_session->startReceive() != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
    }
}

// ---------------------------------------------------------------------------
// Command handlers (async → deferred)
// ---------------------------------------------------------------------------

void UspRadio::TRANSMIT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, UspTransmitState enabled) {
    this->deferredTransmitCmd_internalInterfaceInvoke(enabled);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void UspRadio::CONTINUOUS_WAVE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U16 seconds) {
    this->deferredContinuousWave_internalInterfaceInvoke(seconds);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void UspRadio::SET_TX_PROFILE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, LinkProfileId profile) {
    this->deferredSetTxProfile_internalInterfaceInvoke(profile);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void UspRadio::SET_RX_PROFILE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                          LinkProfileId profile, U16 revert_s) {
    this->deferredSetRxProfile_internalInterfaceInvoke(profile, revert_s);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool UspRadio::applyProfile(U8 idx, UspRadioDirection direction) {
    if (idx >= LINK_PROFILE_COUNT) {
        return false;
    }
    const LinkProfile& p = LINK_PROFILE_TABLE[idx];
    int rc = m_session->applyProfile(p);
    if (rc != 0) {
        this->log_WARNING_HI_ConfigurationFailed(direction);
        return false;
    }
    LinkProfileId profileId = static_cast<LinkProfileId::T>(idx);
    this->log_ACTIVITY_HI_ProfileChanged(direction, profileId);
    return true;
}

bool UspRadio::radioHeadCompatFor(U8 profileIdx) {
    // The RadioHead header is a LoRa-ecosystem convention (RadioHead /
    // adafruit_rfm9x / legacy Zephyr::LoRa); GFSK profiles are USP-only
    // links and are always raw regardless of the parameter.
    if ((profileIdx >= LINK_PROFILE_COUNT) ||
        (LINK_PROFILE_TABLE[profileIdx].mod != ModKind::LORA)) {
        return false;
    }
    Fw::ParamValid valid;
    const bool compat = this->paramGet_RADIOHEAD_COMPAT(valid);
    if ((valid != Fw::ParamValid::VALID) && (valid != Fw::ParamValid::DEFAULT)) {
        // Parameter not loaded (shouldn't happen: topology loads parameters
        // before startRadio) — fail toward compat, the fleet-safe default.
        return true;
    }
    return compat;
}

uint64_t UspRadio::nowMs() const {
#ifdef __ZEPHYR__
    return static_cast<uint64_t>(k_uptime_get());
#else
    // Host test stub: callers supply their own timestamps via ProfilePolicy
    return 0;
#endif
}

}  // namespace Zephyr
