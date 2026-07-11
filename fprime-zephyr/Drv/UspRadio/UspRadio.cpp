// ======================================================================
// \title  UspRadio.cpp
// \brief  Active UHF radio component using Semtech USP (RAL) on SX1262.
//
// Threading model (ADR 0001, SBand deferred-handler pattern):
//   1. dataIn fires on the Svc.Com calling thread → enqueues deferredTxPacket
//   2. USP RX callback fires on USP thread → calls onRxDone() → enqueues
//      deferredRxDone (stores rssi/snr in scratch) → component thread handles
//   3. All commands are async → internally invoked on component thread
//   4. run_handler ticks ProfilePolicy for revert deadline
//   All RAL/SPI work happens in RalSession methods (component thread only).
//
// Buffer ownership (identical to LoRa.cpp / SBand.cpp):
//   TX: dataIn gives us a buffer; we return it via dataReturnOut after TX.
//   RX: we allocate via allocate_out, send via dataOut; caller returns via
//       dataReturnIn → we deallocate_out.
// ======================================================================

#include "fprime-zephyr/Drv/UspRadio/UspRadio.hpp"
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"

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
      m_rxScratchLen(0),
      m_rxRssi(0),
      m_rxSnr(0),
      m_bytesSent(0),
      m_bytesReceived(0),
      m_rxReverts(0) {}

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
        },
        []() {}  // TX done: no action needed beyond the sem in RalSessionImpl
    );

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

    // If initially enabled, kick off the ping-pong comStatus protocol
    if (initialTransmitState == UspTransmitState::ENABLED) {
        Fw::Success status = Fw::Success::SUCCESS;
        this->comStatusOut_out(0, status);
    }

    flushTelemetry();
    return true;
}

// ---------------------------------------------------------------------------
// onRxDone() — called on USP thread
// ---------------------------------------------------------------------------

void UspRadio::onRxDone(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr) {
    // Store in scratch (USP thread → component thread boundary).
    // The component thread processes this in deferredRxDone_handler.
    // Note: scratch may be overwritten if two frames arrive before the
    // component thread drains the queue — acceptable in low-rate scenarios;
    // Phase 5 HWIL will verify.
    FwSizeType safeSize = (size > RX_SCRATCH_SIZE) ? RX_SCRATCH_SIZE : static_cast<FwSizeType>(size);
    (void)::memcpy(m_rxScratch, data, safeSize);
    m_rxScratchLen = safeSize;
    m_rxRssi = rssi;
    m_rxSnr  = snr;

    // Post internal message — runs on component thread
    this->deferredRxDone_internalInterfaceInvoke(
        static_cast<I16>(rssi),
        static_cast<I8>(snr));
}

// ---------------------------------------------------------------------------
// run_handler — rate-group tick
// ---------------------------------------------------------------------------

void UspRadio::run_handler(FwIndexType /*portNum*/, U32 /*context*/) {
    ProfilePolicy::Action action = m_policy.tick(nowMs());
    if (action == ProfilePolicy::Action::kRevert) {
        U8 fromProfile = m_policy.pendingRxProfile();  // saved "from" before revert
        U8 toProfile   = m_policy.rxProfile();

        // Apply the reverted profile to the radio hardware
        (void)applyProfile(toProfile, UspRadioDirection::RX);
        if (m_session->startReceive() != 0) {
            this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
        }

        ++m_rxReverts;
        this->log_WARNING_LO_ProfileReverted(
            static_cast<LinkProfileId::T>(fromProfile),
            static_cast<LinkProfileId::T>(toProfile));
        this->tlmWrite_RxReverts(m_rxReverts);
        this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(toProfile));
    }
}

// ---------------------------------------------------------------------------
// dataIn_handler — enqueue TX (component thread does the work)
// ---------------------------------------------------------------------------

void UspRadio::dataIn_handler(FwIndexType /*portNum*/,
                               Fw::Buffer& data,
                               const ComCfg::FrameContext& context) {
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

void UspRadio::deferredRxDone_internalInterfaceHandler(I16 rssi, I8 snr) {
    // Notify ProfilePolicy that a frame was received
    ProfilePolicy::Action action = m_policy.frameReceived();
    if (action == ProfilePolicy::Action::kConfirmRx) {
        this->log_ACTIVITY_LO_ProfileConfirmed(
            static_cast<LinkProfileId::T>(m_policy.rxProfile()));
        this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
    }

    // Allocate and dispatch the received frame
    const FwSizeType payloadSize = m_rxScratchLen;
    Fw::Buffer buffer = this->allocate_out(0, payloadSize);
    if (buffer.isValid()) {
        (void)::memcpy(buffer.getData(), m_rxScratch, payloadSize);
        ComCfg::FrameContext frameContext;
        this->dataOut_out(0, buffer, frameContext);

        m_bytesReceived += payloadSize;
        this->tlmWrite_BytesReceived(m_bytesReceived);
    } else {
        this->log_WARNING_HI_AllocationFailed(static_cast<FwSizeType>(payloadSize));
    }

    this->tlmWrite_LastRssi(rssi);
    this->tlmWrite_LastSnr(snr);
}

// ---------------------------------------------------------------------------
// deferredTxPacket — component thread
// ---------------------------------------------------------------------------

void UspRadio::deferredTxPacket_internalInterfaceHandler(const Fw::Buffer& data,
                                                          const ComCfg::FrameContext& context) {
    Fw::Buffer mutableData = data;  // cast away const for dataReturnOut
    Fw::Success returnStatus = Fw::Success::FAILURE;

    if (m_transmitState == UspTransmitState::ENABLED) {
        FwSizeType size = data.getSize();
        if (size > MAX_PACKET_SIZE) {
            size = MAX_PACKET_SIZE;  // truncate; Phase 5 will validate framing
        }

        // Stop continuous RX before TX.  startReceive() holds the RAC lock
        // window open indefinitely (RP_TASK_STATE_RUNNING on this hook_id),
        // so a subsequent smtc_rac_lock_radio_access call for TX would return
        // RP_TASK_STATUS_ALREADY_RUNNING → SMTC_RAC_ERROR → -EIO.
        // stopRadio() acquires its own lock window to put the chip in standby,
        // releasing the continuous-RX lock so transmitPacket() can proceed.
        (void)m_session->stopRadio();

        int rc = m_session->transmitPacket(data.getData(), static_cast<std::size_t>(size));
        if (rc != 0) {
            this->log_WARNING_HI_SendFailed(static_cast<I32>(rc));
            returnStatus = Fw::Success::FAILURE;
        } else {
            m_bytesSent += size;
            this->tlmWrite_BytesSent(m_bytesSent);
            returnStatus = Fw::Success::SUCCESS;
            this->log_WARNING_HI_ConfigurationFailed_ThrottleClear();
            this->log_WARNING_HI_SendFailed_ThrottleClear();
        }

        // Re-arm continuous RX after TX
        if (m_session->startReceive() != 0) {
            this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
        }
    } else if (m_transmitState == UspTransmitState::DISABLING) {
        m_transmitState = UspTransmitState::DISABLED;
        // TX episode over — guarantee continuous RX is armed (HWIL 2026-07-11:
        // without this the board stays deaf after TRANSMIT DISABLED).  Stop
        // first: if the last ENABLED frame's re-arm succeeded, RX is already
        // RUNNING and a bare startReceive() would fail ALREADY_RUNNING.
        if ((m_session->stopRadio() != 0) || (m_session->startReceive() != 0)) {
            this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
        }
    }

    this->dataReturnOut_out(0, mutableData, context);
    this->comStatusOut_out(0, returnStatus);
}

// ---------------------------------------------------------------------------
// deferredTransmitCmd — component thread
// ---------------------------------------------------------------------------

void UspRadio::deferredTransmitCmd_internalInterfaceHandler(const UspTransmitState& enabled) {
    if (enabled == UspTransmitState::ENABLED) {
        if (m_transmitState == UspTransmitState::DISABLED) {
            m_transmitState = UspTransmitState::ENABLED;
            Fw::Success comStatus = Fw::Success::SUCCESS;
            this->comStatusOut_out(0, comStatus);
        }
        m_transmitState = UspTransmitState::ENABLED;
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
    // Stop continuous RX before applying a new TX profile (same ALREADY_RUNNING
    // guard as CW and deferredSetRxProfile).  If TX is actively saturating the
    // RAC hook (rate group feeding frames back-to-back), stopRadio() cannot
    // abort it within its deadline and returns non-zero; applyProfile() would
    // then be guaranteed to fail the same way (ALREADY_RUNNING), so skip the
    // attempt rather than spam ConfigurationFailed every cycle.  m_txProfile
    // is already recorded in the policy above; a later SET_TX_PROFILE retry
    // (e.g. once TX is disabled or quiesces) will apply it to hardware.
    if (m_session->stopRadio() != 0) {
        this->log_WARNING_LO_ProfileChangeDeferred(UspRadioDirection::TX);
        return;
    }
    (void)applyProfile(idx, UspRadioDirection::TX);
    // Re-arm continuous RX (mirrors deferredSetRxProfile).  Without this the
    // radio is left in standby after the TX-profile apply — deaf until a
    // different-profile SET_RX_PROFILE or reboot (HWIL 2026-07-11).
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
        // Same-profile SET_RX_PROFILE: no policy/revert change, but treat it
        // as an explicit "ensure RX is armed on this profile" request (HWIL
        // 2026-07-11: deaf boards could only be recovered with a different-
        // profile cycle because this path short-circuited without re-arming).
        // Re-apply too: a preceding TX episode may have left TX-profile
        // modulation params on the chip.
        if (m_session->stopRadio() != 0) {
            this->log_WARNING_LO_ProfileChangeDeferred(UspRadioDirection::RX);
            return;
        }
        (void)applyProfile(idx, UspRadioDirection::RX);
        if (m_session->startReceive() != 0) {
            this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
        }
        return;
    }
    // kApplyRx — stop continuous RX (which holds the RAC lock open in RUNNING
    // state), apply the new profile, then re-arm RX.  Without stopRadio(),
    // applyProfile() → smtc_rac_lock_radio_access returns ALREADY_RUNNING →
    // SMTC_RAC_ERROR → failure, silently leaving the profile unchanged.
    //
    // If TX is actively saturating the RAC hook (rate group feeding frames
    // back-to-back), stopRadio() cannot abort it within its deadline and
    // returns non-zero; applyProfile() would then be guaranteed to fail the
    // same way.  Previously this fell through to applyProfile() anyway, which
    // failed, logged ConfigurationFailed, and left the caller to retry
    // indefinitely — every retry repeating the same failed stopRadio() +
    // applyProfile() pair once every ~25 s until TRANSMIT was disabled.
    // Skip the attempt instead: the pending profile + revert deadline armed
    // by setRxProfile() above are left completely untouched, so (a) the
    // auto-revert timer still fires on schedule if the change never lands
    // before revert_s elapses, and (b) a later SET_RX_PROFILE retry (or the
    // next tick once TX quiesces) re-attempts the apply with fresh state —
    // no revert bookkeeping is corrupted either way.
    if (m_session->stopRadio() != 0) {
        this->log_WARNING_LO_ProfileChangeDeferred(UspRadioDirection::RX);
        return;
    }
    if (!applyProfile(idx, UspRadioDirection::RX)) {
        return;
    }
    if (m_session->startReceive() != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
    }
    this->log_ACTIVITY_HI_ProfileChanged(UspRadioDirection::RX, profile);
    this->tlmWrite_RxProfile(profile);
}

// ---------------------------------------------------------------------------
// deferredContinuousWave — component thread
// ---------------------------------------------------------------------------

void UspRadio::deferredContinuousWave_internalInterfaceHandler(U16 seconds) {
    // Use the current TX profile for CW (freq/power; pkt-type overridden to LoRa by RalSessionImpl)
    U8 txIdx = m_policy.txProfile();
    const LinkProfile& cwProfile = LINK_PROFILE_TABLE[txIdx];

    // Stop continuous RX before CW.  startReceive() holds the RAC lock window
    // open indefinitely (RP_TASK_STATE_RUNNING on this hook_id), so a subsequent
    // smtc_rac_lock_radio_access call for CW would return
    // RP_TASK_STATUS_ALREADY_RUNNING → SMTC_RAC_ERROR → -EIO.
    // stopRadio() aborts the running task via smtc_rac_unlock_radio_access and
    // waits for the post-callback to confirm the hook_id is FINISHED.
    (void)m_session->stopRadio();

    int rc = m_session->startCw(cwProfile);
    if (rc != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::TX);
        return;
    }

    // Block the component thread for the requested CW duration.
    // The component queue is stalled during this time (matching legacy LoRa.cpp
    // behaviour: lora_test_cw() blocks).  The USP thread continues to run the RAC.
    // For durations ≤ 10 s this is acceptable per safety limits; a k_timer
    // approach would be cleaner but adds complexity deferred to Phase 4.
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

uint64_t UspRadio::nowMs() const {
#ifdef __ZEPHYR__
    return static_cast<uint64_t>(k_uptime_get());
#else
    // Host test stub: callers supply their own timestamps via ProfilePolicy
    return 0;
#endif
}

void UspRadio::flushTelemetry() {
    this->tlmWrite_BytesSent(m_bytesSent);
    this->tlmWrite_BytesReceived(m_bytesReceived);
    this->tlmWrite_LastRssi(m_rxRssi);
    this->tlmWrite_LastSnr(m_rxSnr);
    this->tlmWrite_TxProfile(static_cast<LinkProfileId::T>(m_policy.txProfile()));
    this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
    this->tlmWrite_ProfileTableVersion(LINK_PROFILE_TABLE_VERSION);
    this->tlmWrite_RxReverts(m_rxReverts);
}

}  // namespace Zephyr
