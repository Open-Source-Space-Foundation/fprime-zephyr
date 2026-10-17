// OPTION 2 (single queue) UspRadio: the sketch's handlers verbatim where given; the parts the sketch elides
// ("unchanged: ... RadioHead-shim bodies of the TX and RX paths", constructor, signalFirstStart, radioHeadCompatFor,
// the command handlers) are copied from UspRadio.cpp.  Deviations marked "ARENA-c3:".
#include "UspRadioSt.hpp"
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"
#include "fprime-zephyr/Drv/UspRadio/RadioHeadShim.hpp"
#include "fprime-zephyr/Drv/UspRadio/TxOutcomePolicy.hpp"
#include <Fw/Logger/Logger.hpp>
#include <cstring>

namespace Zephyr {

void UspRadioSt::emitCredit() {
    FW_ASSERT(m_link != Link::CREDIT_OUT);
    Fw::Success ok = Fw::Success::SUCCESS;
    this->comStatusOut_out(0, ok);
    m_link = Link::CREDIT_OUT;
}

void UspRadioSt::rearmRxWhenIdle() {
    if (txInFlight()) {
        return;
    }
    const U8 idx = m_policy.armedRxProfile();
    m_rxArmFailed = (m_session->armRx(LINK_PROFILE_TABLE[idx]) != 0);
    if (m_rxArmFailed) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
    }
}

bool UspRadioSt::startRadio(RalSessionSt* session, UspTransmitState initialTransmitState) {
    m_session = session;
    m_transmitState = initialTransmitState;
    if (m_session->init(*this) != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
        return false;
    }
    rearmRxWhenIdle();
    if (initialTransmitState == UspTransmitState::ENABLED) {
        emitCredit();
        this->signalFirstStart();
    }
    this->tlmWrite_BytesSent(m_bytesSent);
    this->tlmWrite_BytesReceived(m_bytesReceived);
    this->tlmWrite_LastRssi(m_rxRssi);
    this->tlmWrite_LastSnr(m_rxSnr);
    this->tlmWrite_TxProfile(static_cast<LinkProfileId::T>(m_policy.txProfile()));
    this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
    this->tlmWrite_RxReverts(m_rxReverts);
    return true;
}

void UspRadioSt::engineKick_internalInterfaceHandler() { m_session->onEngineKick(); }
void UspRadioSt::postEngineKick() { this->engineKick_internalInterfaceInvoke(); }

void UspRadioSt::rxFrame(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr) {
    if (m_policy.frameReceived() == ProfilePolicy::Action::kConfirmRx) {
        this->log_ACTIVITY_LO_ProfileConfirmed(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
        this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
    }
    m_rxRssi = rssi;
    m_rxSnr = snr;
    this->deliverRx(data, size);
}

// UspRadio.cpp deferredRxDone body minus the ring slot bookkeeping
void UspRadioSt::deliverRx(const uint8_t* data, std::size_t size) {
    const bool rhCompat = this->radioHeadCompatFor(m_policy.rxProfile());
    U32 payloadOffset = 0;
    U32 payloadLen = 0;
    if (RadioHeadShim::locateRxPayload(static_cast<U32>(size), rhCompat, payloadOffset, payloadLen)) {
        const FwSizeType payloadSize = static_cast<FwSizeType>(payloadLen);
        Fw::Buffer buffer = this->allocate_out(0, payloadSize);
        if (buffer.isValid()) {
            (void)::memcpy(buffer.getData(), data + payloadOffset, payloadSize);
            ComCfg::FrameContext frameContext;
            this->dataOut_out(0, buffer, frameContext);
            m_bytesReceived += payloadSize;
            this->tlmWrite_BytesReceived(m_bytesReceived);
        } else {
            this->log_WARNING_HI_AllocationFailed(payloadSize);
        }
    }
    this->tlmWrite_LastRssi(m_rxRssi);
    this->tlmWrite_LastSnr(m_rxSnr);
}

void UspRadioSt::dataIn_handler(FwIndexType, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    this->deferredTxPacket_internalInterfaceInvoke(data, context);
}
void UspRadioSt::dataReturnIn_handler(FwIndexType, Fw::Buffer& data, const ComCfg::FrameContext&) {
    this->deallocate_out(0, data);
}
void UspRadioSt::enableTransmit_handler(FwIndexType) { this->deferredTransmitCmd_internalInterfaceInvoke(UspTransmitState::ENABLED); }
void UspRadioSt::disableTransmit_handler(FwIndexType) { this->deferredTransmitCmd_internalInterfaceInvoke(UspTransmitState::DISABLING); }

void UspRadioSt::deferredTxPacket_internalInterfaceHandler(const Fw::Buffer& data, const ComCfg::FrameContext& context) {
    FW_ASSERT(m_link == Link::CREDIT_OUT);
    m_link = Link::FRAME_HELD;
    m_held = data;
    m_heldContext = context;

    if (m_transmitState != UspTransmitState::ENABLED) {
        if (m_transmitState == UspTransmitState::DISABLING) {
            m_transmitState = UspTransmitState::DISABLED;
        }
        finishFrame();
        return;
    }
    // RadioHead staging + truncate: UspRadio.cpp deferredTxPacket (unchanged)
    const bool rhCompat = this->radioHeadCompatFor(m_policy.txProfile());
    const FwSizeType maxPayload = static_cast<FwSizeType>(RadioHeadShim::maxPayload(static_cast<U32>(MAX_PACKET_SIZE), rhCompat));
    FwSizeType size = data.getSize();
    if (size > maxPayload) size = maxPayload;
    m_txData = data.getData();
    m_txSize = static_cast<std::size_t>(size);
    if (rhCompat) {
        m_txSize = static_cast<std::size_t>(RadioHeadShim::buildTxPacket(data.getData(), static_cast<U32>(size), m_txScratch));
        m_txData = m_txScratch;
    }
    const LinkProfile& p = LINK_PROFILE_TABLE[m_policy.txProfile()];
    const int rc = m_session->startTx(p, m_txData, m_txSize, nowMs());
    if (rc != 0) {
        this->log_WARNING_HI_SendFailed(static_cast<I32>(rc));
        finishFrame();
    }
}

void UspRadioSt::txDone(int rc) {
    const TxOutcomePolicy::Outcome outcome = TxOutcomePolicy::evaluate(rc);
    if (outcome.logSendFailed) this->log_WARNING_HI_SendFailed(static_cast<I32>(rc));
    if (outcome.transmitted) {
        m_bytesSent += m_txSize;
        this->tlmWrite_BytesSent(m_bytesSent);
    }
    if (outcome.clearThrottles) {
        this->log_WARNING_HI_ConfigurationFailed_ThrottleClear();
        this->log_WARNING_HI_SendFailed_ThrottleClear();
    }
    finishFrame();
}

void UspRadioSt::finishFrame() {
    FW_ASSERT(m_link == Link::FRAME_HELD);
    this->dataReturnOut_out(0, m_held, m_heldContext);
    emitCredit();
    // ARENA-c3: sketch comment "Always re-arm" kept: the #26 skip optimisation is dropped.
    // If the transmit state moved to DISABLING during the TX the old code latched DISABLED on the next frame;
    // unchanged here.
    rearmRxWhenIdle();
}

void UspRadioSt::signalFirstStart() {
    if (m_radioEverOn) return;
    m_radioEverOn = true;
    if (this->isConnected_radioFirstStart_OutputPort(0)) this->radioFirstStart_out(0);
}

void UspRadioSt::deferredTransmitCmd_internalInterfaceHandler(const UspTransmitState& enabled) {
    if (enabled == UspTransmitState::ENABLED) {
        if (m_link == Link::NO_CREDIT) emitCredit();
        m_transmitState = UspTransmitState::ENABLED;
        this->signalFirstStart();
        return;
    }
    m_transmitState = txInFlight() ? UspTransmitState::DISABLING : UspTransmitState::DISABLED;
    rearmRxWhenIdle();
}

void UspRadioSt::deferredSetTxProfile_internalInterfaceHandler(const LinkProfileId& profile) {
    if (m_policy.setTxProfile(static_cast<U8>(profile.e)) == ProfilePolicy::Action::kInvalidIndex) {
        this->log_WARNING_HI_InvalidProfile(profile);
        return;
    }
    this->tlmWrite_TxProfile(profile);
    this->log_ACTIVITY_HI_ProfileChanged(UspRadioDirection::TX, profile);
}

void UspRadioSt::deferredSetRxProfile_internalInterfaceHandler(const LinkProfileId& profile, U16 revert_s) {
    const ProfilePolicy::Action action = m_policy.setRxProfile(static_cast<U8>(profile.e), revert_s, nowMs());
    if (action == ProfilePolicy::Action::kInvalidIndex) {
        this->log_WARNING_HI_InvalidProfile(profile);
        return;
    }
    rearmRxWhenIdle();
    this->log_ACTIVITY_HI_ProfileChanged(UspRadioDirection::RX, profile);
    this->tlmWrite_RxProfile(profile);
}

void UspRadioSt::deferredContinuousWave_internalInterfaceHandler(U16 seconds) {
    if (txInFlight()) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::TX);
        return;
    }
    if (m_session->startCw() != 0) {
        this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::TX);
        return;
    }
    m_cwUntilMs = nowMs() + (static_cast<uint64_t>(seconds) * 1000U);
}

void UspRadioSt::run_handler(FwIndexType, U32) {
    const uint64_t now = nowMs();
    m_session->checkTxDeadline(now);
    if ((m_cwUntilMs != 0) && (now >= m_cwUntilMs)) {
        m_cwUntilMs = 0;
        rearmRxWhenIdle();
    }
    if (m_policy.tick(now) == ProfilePolicy::Action::kRevert) {
        const U8 fromProfile = m_policy.pendingRxProfile();
        const U8 toProfile = m_policy.rxProfile();
        ++m_rxReverts;
        this->log_WARNING_LO_ProfileReverted(static_cast<LinkProfileId::T>(fromProfile), static_cast<LinkProfileId::T>(toProfile));
        this->tlmWrite_RxReverts(m_rxReverts);
        this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(toProfile));
        rearmRxWhenIdle();
    } else if (m_rxArmFailed) {
        rearmRxWhenIdle();
    }
}

void UspRadioSt::TRANSMIT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, UspTransmitState enabled) {
    this->deferredTransmitCmd_internalInterfaceInvoke(enabled);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}
void UspRadioSt::CONTINUOUS_WAVE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U16 seconds) {
    this->deferredContinuousWave_internalInterfaceInvoke(seconds);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}
void UspRadioSt::SET_TX_PROFILE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, LinkProfileId profile) {
    this->deferredSetTxProfile_internalInterfaceInvoke(profile);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}
void UspRadioSt::SET_RX_PROFILE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, LinkProfileId profile, U16 revert_s) {
    this->deferredSetRxProfile_internalInterfaceInvoke(profile, revert_s);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

bool UspRadioSt::radioHeadCompatFor(U8 profileIdx) {
    if ((profileIdx >= LINK_PROFILE_COUNT) || (LINK_PROFILE_TABLE[profileIdx].mod != ModKind::LORA)) return false;
    Fw::ParamValid valid;
    const bool compat = this->paramGet_RADIOHEAD_COMPAT(valid);
    if ((valid != Fw::ParamValid::VALID) && (valid != Fw::ParamValid::DEFAULT)) return true;
    return compat;
}

}  // namespace Zephyr
