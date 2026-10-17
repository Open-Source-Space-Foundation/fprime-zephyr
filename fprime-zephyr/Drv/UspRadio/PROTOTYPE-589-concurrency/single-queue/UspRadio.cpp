// PROTOTYPE #589 (throwaway, not built). Option 2: single-queue UspRadio.
//
// Every handler below runs on the component thread, and so does the RAC
// engine (RalSessionSt::pump), so all UspRadio state is plain fields: no
// std::atomic, no SPSC ring, no k_sleep polls.  Only the handlers that change
// relative to fix/uspradio-integration-c123 are shown.  Unchanged:
// constructor, configure, signalFirstStart, radioHeadCompatFor, nowMs, the
// command handlers (each still enqueues its deferred* message), the
// RadioHead-shim bodies of the TX and RX paths.

namespace Zephyr {

// ---------------------------------------------------------------------------
// The com link: one field replaces m_comStatusOwed + m_pendingTxFrames +
// m_rearmAfterTx.  comStatusOut is emitted only on the two edges INTO
// CREDIT_OUT, and neither edge starts from CREDIT_OUT, so a second status
// into a READY ComQueue (the #14 FW_ASSERT) has no code path.
//
//   NO_CREDIT  --prime (TRANSMIT ENABLED)-->  CREDIT_OUT      emit SUCCESS
//   CREDIT_OUT --frame dequeued------------>  FRAME_HELD
//   FRAME_HELD --txDone / frame dropped---->  CREDIT_OUT      emit SUCCESS
// ---------------------------------------------------------------------------

void UspRadio::emitCredit() {
    FW_ASSERT(m_link != Link::CREDIT_OUT);
    Fw::Success ok = Fw::Success::SUCCESS;
    this->comStatusOut_out(0, ok);
    m_link = Link::CREDIT_OUT;
}

bool UspRadio::txInFlight() const {
    return m_link == Link::FRAME_HELD;
}

//! Re-arm RX on the policy's armed profile, unless a TX is in flight: then
//! txDone() re-arms when it finishes.  Every profile change, revert and
//! recovery lever goes through here, so none of them can abort a TX.
void UspRadio::rearmRxWhenIdle() {
    if (txInFlight()) {
        return;
    }
    const U8 idx = m_policy.armedRxProfile();
    m_rxArmFailed = (m_session->armRx(LINK_PROFILE_TABLE[idx]) != 0);
    if (m_rxArmFailed) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);  // run() retries
    }
}

// ---------------------------------------------------------------------------
// startRadio: failure is fatal to the radio and says so (today it returns
// false and the topology ignores it).
// ---------------------------------------------------------------------------

bool UspRadio::startRadio(UspTransmitState initialTransmitState) {
    m_transmitState = initialTransmitState;
    if (m_session->init(*this) != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
        return false;  // every later op returns -ENODEV from RalSessionSt::stop()
    }
    rearmRxWhenIdle();
    if (initialTransmitState == UspTransmitState::ENABLED) {
        emitCredit();
        this->signalFirstStart();
    }
    // boot telemetry flush: unchanged
    return true;
}

// ---------------------------------------------------------------------------
// Engine and RX
// ---------------------------------------------------------------------------

void UspRadio::engineKick_internalInterfaceHandler() {
    m_session->onEngineKick();  // may call rxFrame() and txDone() below
}

void UspRadio::postEngineKick() {
    this->engineKick_internalInterfaceInvoke();  // kick pump thread
}

//! From inside the engine pass.  The frame was read from the FIFO while the
//! current RX arm was live, and arms only change on this thread, so the frame
//! was received on m_policy.armedRxProfile(): a confirm from a frame of the
//! previous profile cannot happen.
void UspRadio::rxFrame(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr) {
    if (m_policy.frameReceived() == ProfilePolicy::Action::kConfirmRx) {
        this->log_ACTIVITY_LO_ProfileConfirmed(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
        this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
    }
    m_rxRssi = rssi;
    m_rxSnr = snr;
    // RadioHead strip + allocate + dataOut: UspRadio.cpp:232-256, unchanged,
    // minus the ring-slot bookkeeping.
    this->deliverRx(data, size);
}

// ---------------------------------------------------------------------------
// TX
// ---------------------------------------------------------------------------

void UspRadio::dataIn_handler(FwIndexType, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    this->deferredTxPacket_internalInterfaceInvoke(data, context);  // nothing else
}

void UspRadio::deferredTxPacket_internalInterfaceHandler(const Fw::Buffer& data,
                                                          const ComCfg::FrameContext& context) {
    FW_ASSERT(m_link == Link::CREDIT_OUT);  // ComQueue sends only after a SUCCESS
    m_link = Link::FRAME_HELD;
    m_held = data;
    m_heldContext = context;

    if (m_transmitState != UspTransmitState::ENABLED) {
        if (m_transmitState == UspTransmitState::DISABLING) {
            m_transmitState = UspTransmitState::DISABLED;
        }
        finishFrame();  // dropped, pipeline stays alive (SUCCESS, as in #26)
        return;
    }
    // RadioHead staging + truncate: UspRadio.cpp:293-319, unchanged.
    const LinkProfile& p = LINK_PROFILE_TABLE[m_policy.txProfile()];
    const int rc = m_session->startTx(p, m_txData, m_txSize, nowMs());
    if (rc != 0) {
        this->log_WARNING_HI_SendFailed(static_cast<I32>(rc));
        finishFrame();
    }
    // else: txDone() finishes the frame from a later engineKick.
}

void UspRadio::txDone(int rc) {
    const TxOutcomePolicy::Outcome outcome = TxOutcomePolicy::evaluate(rc);
    // SendFailed / BytesSent / throttle clears: UspRadio.cpp:337-349, unchanged.
    (void)outcome;
    finishFrame();
}

void UspRadio::finishFrame() {
    FW_ASSERT(m_link == Link::FRAME_HELD);
    this->dataReturnOut_out(0, m_held, m_heldContext);
    emitCredit();
    // Always re-arm.  #26 skipped the re-arm while more frames were queued
    // because stop -> re-arm -> stop cost ~50 ms per frame in cross-thread
    // waits.  Here stop() and armRx() are synchronous SPI on this thread, so the
    // skip (and its 5 x 1 ms k_sleep poll) is dropped.  Bench: P3 saturation.
    rearmRxWhenIdle();
}

// ---------------------------------------------------------------------------
// Transmit state, profiles, CW, rate group
// ---------------------------------------------------------------------------

void UspRadio::deferredTransmitCmd_internalInterfaceHandler(const UspTransmitState& enabled) {
    if (enabled == UspTransmitState::ENABLED) {
        if (m_link == Link::NO_CREDIT) {
            emitCredit();
        }
        m_transmitState = UspTransmitState::ENABLED;
        this->signalFirstStart();
        return;
    }
    // DISABLING with a frame in flight: finishFrame() of the next frame lands
    // DISABLED.  Otherwise DISABLED now, and re-arm (the recover-RX lever).
    m_transmitState = txInFlight() ? UspTransmitState::DISABLING : UspTransmitState::DISABLED;
    rearmRxWhenIdle();
}

void UspRadio::deferredSetTxProfile_internalInterfaceHandler(const LinkProfileId& profile) {
    if (m_policy.setTxProfile(static_cast<U8>(profile.e)) == ProfilePolicy::Action::kInvalidIndex) {
        this->log_WARNING_HI_InvalidProfile(profile);
        return;
    }
    this->tlmWrite_TxProfile(profile);
    this->log_ACTIVITY_HI_ProfileChanged(UspRadioDirection::TX, profile);
    // Nothing touches the chip: startTx() applies the TX profile in its own
    // lock window, and RX was never disturbed.
}

void UspRadio::deferredSetRxProfile_internalInterfaceHandler(const LinkProfileId& profile, U16 revert_s) {
    const ProfilePolicy::Action action = m_policy.setRxProfile(static_cast<U8>(profile.e), revert_s, nowMs());
    if (action == ProfilePolicy::Action::kInvalidIndex) {
        this->log_WARNING_HI_InvalidProfile(profile);
        return;
    }
    rearmRxWhenIdle();  // kNoOp too: same-profile SET_RX_PROFILE stays a recover-RX lever
    this->log_ACTIVITY_HI_ProfileChanged(UspRadioDirection::RX, profile);
    this->tlmWrite_RxProfile(profile);
}

void UspRadio::deferredContinuousWave_internalInterfaceHandler(U16 seconds) {
    if (txInFlight()) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::TX);
        return;
    }
    if (m_session->startCw() != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::TX);
        return;
    }
    // No k_sleep: the queue keeps draining during CW; run() ends it.
    m_cwUntilMs = nowMs() + (static_cast<uint64_t>(seconds) * 1000U);
}

void UspRadio::run_handler(FwIndexType, U32) {
    const uint64_t now = nowMs();
    m_session->checkTxDeadline(now);  // anomaly-B watchdog -> txDone(-ETIMEDOUT)
    if ((m_cwUntilMs != 0) && (now >= m_cwUntilMs)) {
        m_cwUntilMs = 0;
        rearmRxWhenIdle();  // armRx() stops CW first
    }
    if (m_policy.tick(now) == ProfilePolicy::Action::kRevert) {
        // ProfileReverted event + telemetry: unchanged.
        rearmRxWhenIdle();
    } else if (m_rxArmFailed) {
        rearmRxWhenIdle();  // any failed arm (revert, profile change, TX tail)
                            // retries at 1 Hz; replaces m_revertRearmPending,
                            // which covered only the revert path
    }
}

}  // namespace Zephyr
