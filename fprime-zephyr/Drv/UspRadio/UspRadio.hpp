// ======================================================================
// \title  UspRadio.hpp
// \brief  Active UHF radio component using Semtech USP (RAL) on SX1262.
//
// Execution model (ADR 0001, SBand deferred-handler pattern):
//   - dataIn_handler: enqueues deferredTxPacket (component thread does TX)
//   - USP RX callback: enqueues deferredRxDone (component thread handles RX)
//   - TRANSMIT/CONTINUOUS_WAVE/SET_*_PROFILE commands: async → enqueued
//   - run_handler: ticks ProfilePolicy for revert deadline
//   - ALL RAL/SPI work runs on the component thread via RalSession methods
//
// Buffer ownership (matches LoRa.cpp / SBand.cpp contract):
//   - dataIn buffer: returned via dataReturnOut in the deferred TX handler
//   - RX buffers: allocated via allocate_out, sent via dataOut, returned
//     via dataReturnIn_handler → deallocate_out
// ======================================================================

#ifndef ZEPHYR_USP_RADIO_HPP
#define ZEPHYR_USP_RADIO_HPP

#include "fprime-zephyr/Drv/UspRadio/UspRadioComponentAc.hpp"
#include "fprime-zephyr/Drv/UspRadio/ProfilePolicy.hpp"
#include "fprime-zephyr/Drv/UspRadio/RalSession.hpp"

namespace Zephyr {

class UspRadio final : public UspRadioComponentBase {
  public:
    //! Maximum payload size (bytes).  252 = LoRa hardware limit; GFSK supports
    //! up to 255 but we keep a single constant for CCSDS framing compatibility
    //! (phase 5 note: per-profile limit may become a table field if needed).
    static constexpr FwSizeType MAX_PACKET_SIZE = 252;

    // ----------------------------------------------------------------------
    // Construction / destruction
    // ----------------------------------------------------------------------

    //! Construct with compName only.  Call configure() before start().
    explicit UspRadio(const char* compName);
    ~UspRadio() override;

    //! Inject the RalSession implementation (must be called before start()).
    //! Allows the FPP autocoder to instantiate the component without knowing
    //! the concrete session type (which is on-target only).
    void configure(RalSession& session);

    //! Initialize and start the radio.  Must be called after configure() and
    //! after component wiring is complete.
    //! Calls session.init(), applyProfile(boot default), session.startReceive().
    //! @returns true on success.
    bool start(UspTransmitState initialTransmitState = UspTransmitState::DISABLED);

    //! Called by the RalSession RX callback (runs on USP thread).
    //! Posts a deferredRxDone internal message; does NOT touch F´ ports directly.
    void onRxDone(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr);

  private:
    // ----------------------------------------------------------------------
    // Port handlers
    // ----------------------------------------------------------------------

    //! Rate-group tick: revert deadline poll + periodic telemetry
    void run_handler(FwIndexType portNum, U32 context) override;

    //! Incoming frame to transmit
    void dataIn_handler(FwIndexType portNum,
                        Fw::Buffer& data,
                        const ComCfg::FrameContext& context) override;

    //! Buffer returned by downstream after dataOut
    void dataReturnIn_handler(FwIndexType portNum,
                              Fw::Buffer& data,
                              const ComCfg::FrameContext& context) override;

    // ----------------------------------------------------------------------
    // Internal interface handlers (run on component thread)
    // ----------------------------------------------------------------------

    void deferredRxDone_internalInterfaceHandler(I16 rssi, I8 snr) override;

    void deferredTxPacket_internalInterfaceHandler(const Fw::Buffer& data,
                                                   const ComCfg::FrameContext& context) override;

    void deferredTransmitCmd_internalInterfaceHandler(const UspTransmitState& enabled) override;

    void deferredSetTxProfile_internalInterfaceHandler(const LinkProfileId& profile) override;

    void deferredSetRxProfile_internalInterfaceHandler(const LinkProfileId& profile,
                                                       U16 revert_s) override;

    void deferredContinuousWave_internalInterfaceHandler(U16 seconds) override;

    // ----------------------------------------------------------------------
    // Command handlers — all async (deferred to component thread)
    // ----------------------------------------------------------------------

    void TRANSMIT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                             UspTransmitState enabled) override;

    void CONTINUOUS_WAVE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                    U16 seconds) override;

    void SET_TX_PROFILE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                   LinkProfileId profile) override;

    void SET_RX_PROFILE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                   LinkProfileId profile, U16 revert_s) override;

    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    //! Apply a profile via RalSession and log events.
    //! @param idx profile index (must be valid).
    //! @param direction TX or RX (for event logging only).
    //! @returns true on success.
    bool applyProfile(U8 idx, UspRadioDirection direction);

    //! Monotonic millisecond timestamp (wraps Os::Queue / Zephyr k_uptime_get)
    uint64_t nowMs() const;

    //! Flush all telemetry channels once.
    void flushTelemetry();

    // ----------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------
    RalSession*       m_session;  //!< Injected via configure(); never null after start()
    ProfilePolicy     m_policy;
    UspTransmitState  m_transmitState;

    // Receive scratchpad (filled by deferredRxDone handler on component thread)
    static constexpr FwSizeType RX_SCRATCH_SIZE = MAX_PACKET_SIZE;
    uint8_t           m_rxScratch[RX_SCRATCH_SIZE];
    FwSizeType        m_rxScratchLen;
    I16               m_rxRssi;
    I8                m_rxSnr;

    // Telemetry accumulators
    FwSizeType m_bytesSent;
    FwSizeType m_bytesReceived;
    U32        m_rxReverts;
};

}  // namespace Zephyr

#endif  // ZEPHYR_USP_RADIO_HPP
