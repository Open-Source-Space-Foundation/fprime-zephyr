// OPTION 2 (single queue) UspRadio, completed from PROTOTYPE-589-concurrency/single-queue/UspRadio.cpp.
#pragma once
#include "fprime-zephyr/Drv/UspRadio/UspRadioComponentAc.hpp"
#include "fprime-zephyr/Drv/UspRadio/ProfilePolicy.hpp"
#include "RalSessionSt.hpp"

namespace Zephyr {

class UspRadioSt final : public UspRadioComponentBase, public RadioSink {
  public:
    static constexpr FwSizeType MAX_PACKET_SIZE = 252;
    explicit UspRadioSt(const char* n) : UspRadioComponentBase(n) {}
    bool startRadio(RalSessionSt* session, UspTransmitState initialTransmitState);

    // RadioSink
    void rxFrame(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr) override;
    void txDone(int rc) override;
    void postEngineKick() override;

    // sim observation
    RalSessionSt* session() { return m_session; }
    long falseConfirmsNone = 0;

  private:
    enum class Link : U8 { NO_CREDIT, CREDIT_OUT, FRAME_HELD };
    void run_handler(FwIndexType, U32) override;
    void dataIn_handler(FwIndexType, Fw::Buffer&, const ComCfg::FrameContext&) override;
    void dataReturnIn_handler(FwIndexType, Fw::Buffer&, const ComCfg::FrameContext&) override;
    void enableTransmit_handler(FwIndexType) override;
    void disableTransmit_handler(FwIndexType) override;
    void deferredTxPacket_internalInterfaceHandler(const Fw::Buffer&, const ComCfg::FrameContext&) override;
    void deferredTransmitCmd_internalInterfaceHandler(const UspTransmitState&) override;
    void deferredSetTxProfile_internalInterfaceHandler(const LinkProfileId&) override;
    void deferredSetRxProfile_internalInterfaceHandler(const LinkProfileId&, U16) override;
    void deferredContinuousWave_internalInterfaceHandler(U16) override;
    void engineKick_internalInterfaceHandler() override;
    void TRANSMIT_cmdHandler(FwOpcodeType, U32, UspTransmitState) override;
    void CONTINUOUS_WAVE_cmdHandler(FwOpcodeType, U32, U16) override;
    void SET_TX_PROFILE_cmdHandler(FwOpcodeType, U32, LinkProfileId) override;
    void SET_RX_PROFILE_cmdHandler(FwOpcodeType, U32, LinkProfileId, U16) override;

    void emitCredit();
    bool txInFlight() const { return m_link == Link::FRAME_HELD; }
    void rearmRxWhenIdle();
    void finishFrame();
    void deliverRx(const uint8_t* data, std::size_t size);
    bool radioHeadCompatFor(U8 idx);
    uint64_t nowMs() const { return static_cast<uint64_t>(k_uptime_get()); }
    void signalFirstStart();

    RalSessionSt* m_session = nullptr;
    ProfilePolicy m_policy;
    UspTransmitState m_transmitState = UspTransmitState::DISABLED;
    Link m_link = Link::NO_CREDIT;
    Fw::Buffer m_held;
    ComCfg::FrameContext m_heldContext;
    const U8* m_txData = nullptr;
    std::size_t m_txSize = 0;
    U8 m_txScratch[MAX_PACKET_SIZE];
    bool m_rxArmFailed = false;
    uint64_t m_cwUntilMs = 0;
    bool m_radioEverOn = false;
    FwSizeType m_bytesSent = 0, m_bytesReceived = 0;
    I16 m_rxRssi = 0; I8 m_rxSnr = 0;
    U32 m_rxReverts = 0;
};

}  // namespace Zephyr
