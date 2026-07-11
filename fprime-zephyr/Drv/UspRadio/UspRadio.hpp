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

#include <atomic>

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
    //! after component wiring is complete (and after the F' task start() call).
    //! Calls session.init(), applyProfile(boot default), session.startReceive().
    //! Named startRadio() to avoid clashing with ActiveComponentBase::start().
    //! @returns true on success.
    bool startRadio(UspTransmitState initialTransmitState = UspTransmitState::DISABLED);

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

    // Receive ring (SPSC: USP-thread producer via onRxDone, component-thread
    // consumer via deferredRxDone).  Replaces the single scratch slot, which
    // HWIL-failed at saturation (2026-07-11 session 4): a second frame arriving
    // before the component thread drained the queue overwrote the first AND
    // made every queued handler invocation re-read (and re-count) the latest
    // frame — duplicate delivery upstream + BytesReceived overcount.
    // Indexes increment monotonically (unsigned wrap is safe: DEPTH is a power
    // of two, and head - tail stays correct across wrap).  Producer owns
    // m_rxHead, consumer owns m_rxTail; release/acquire pairs order the slot
    // payload copies against index publication.
    static constexpr FwSizeType RX_SCRATCH_SIZE = MAX_PACKET_SIZE;
    static constexpr U32 RX_RING_DEPTH = 4;  // must be a power of two
    struct RxSlot {
        uint8_t    data[RX_SCRATCH_SIZE];
        FwSizeType len;
        I16        rssi;
        I8         snr;
    };
    RxSlot            m_rxRing[RX_RING_DEPTH];
    std::atomic<U32>  m_rxHead;      //!< next slot to write (producer-owned)
    std::atomic<U32>  m_rxTail;      //!< next slot to read (consumer-owned)
    std::atomic<U32>  m_rxDropped;   //!< frames dropped: ring full at onRxDone
    U32               m_rxDroppedReported;  //!< last value emitted (component thread)
    I16               m_rxRssi;      //!< last received RSSI (component thread)
    I8                m_rxSnr;       //!< last received SNR (component thread)

    //! TX frames accepted by dataIn_handler but not yet processed by
    //! deferredTxPacket (incremented on the Svc.Com caller thread, decremented
    //! at deferred-handler entry).  When > 0 after a transmit, more TX work is
    //! already queued behind us, so the per-frame continuous-RX re-arm is
    //! skipped: the TX→re-arm→stop→TX dance costs ~50 ms/frame and throttled
    //! saturated P3 TX to ~19 kbps vs the 33.5 kbps airtime ceiling (HWIL
    //! 2026-07-11 session 4).  Every other radio-touching path still ends
    //! re-armed, and the DISABLING/DISABLED seams re-arm unconditionally, so
    //! the radio is never left deaf once the TX queue drains.
    std::atomic<U32>  m_pendingTxFrames;

    //! Set by the ENABLED TX path: re-arm continuous RX at the handler tail,
    //! after comStatusOut has released the one-in-flight com pipeline (so a
    //! queued next frame has a chance to reach dataIn and suppress the re-arm).
    bool              m_rearmAfterTx;

    // Telemetry accumulators
    FwSizeType m_bytesSent;
    FwSizeType m_bytesReceived;
    U32        m_rxReverts;

    //! True when ProfilePolicy has committed an RX auto-revert but the
    //! hardware apply (stopRadio → applyProfile → startReceive) has not yet
    //! succeeded — retried on each run tick.  tick() commits the revert
    //! irreversibly before returning kRevert, so this flag is the only record
    //! that the chip still needs re-programming.  Cleared by a successful
    //! explicit SET_RX_PROFILE apply (which supersedes the revert re-arm).
    bool m_revertRearmPending;
};

}  // namespace Zephyr

#endif  // ZEPHYR_USP_RADIO_HPP
