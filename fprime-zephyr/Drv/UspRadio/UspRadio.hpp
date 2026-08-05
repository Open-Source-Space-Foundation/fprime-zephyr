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
    //! up to 255 but a single constant keeps CCSDS framing compatibility.
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

    //! Rate-group tick: revert deadline poll
    void run_handler(FwIndexType portNum, U32 context) override;

    //! Incoming frame to transmit
    void dataIn_handler(FwIndexType portNum,
                        Fw::Buffer& data,
                        const ComCfg::FrameContext& context) override;

    //! Buffer returned by downstream after dataOut
    void dataReturnIn_handler(FwIndexType portNum,
                              Fw::Buffer& data,
                              const ComCfg::FrameContext& context) override;

    //! StartupManager transmit-enable signal.  Sync port on the caller thread:
    //! only enqueues deferredTransmitCmd(ENABLED) — all radio work stays on the
    //! component thread, reusing the TRANSMIT command's state machine verbatim.
    void enableTransmit_handler(FwIndexType portNum) override;

    //! StartupManager transmit-disable signal.  Enqueues
    //! deferredTransmitCmd(DISABLING), identical to TRANSMIT(DISABLING).
    void disableTransmit_handler(FwIndexType portNum) override;

    // ----------------------------------------------------------------------
    // Internal interface handlers (run on component thread)
    // ----------------------------------------------------------------------

    void deferredRxDone_internalInterfaceHandler() override;

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

    //! True when the RadioHead-compat shim applies for the given profile:
    //! the RADIOHEAD_COMPAT parameter is set AND the profile is LoRa (GFSK
    //! never carried the RadioHead header on any legacy path).  An unloaded
    //! parameter reads as compat-enabled — the safe default for the fleet.
    bool radioHeadCompatFor(U8 profileIdx);

    //! Monotonic millisecond timestamp (wraps Os::Queue / Zephyr k_uptime_get)
    uint64_t nowMs() const;

    //! Emit radioFirstStart once per boot, on the first transmit-enable.
    //! Idempotent; a no-op if the port is unconnected.  Called only from
    //! startRadio() (before the component task runs) and from the component
    //! thread, so the latch needs no synchronization.
    void signalFirstStart();

    // ----------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------
    RalSession*       m_session;  //!< Injected via configure(); never null after start()
    ProfilePolicy     m_policy;
    UspTransmitState  m_transmitState;

    // Receive ring (SPSC: USP-thread producer onRxDone, component-thread
    // consumer deferredRxDone).  Producer owns m_rxHead, consumer owns m_rxTail;
    // release/acquire pairs order slot payload copies against index publication.
    // Indexes increment monotonically (unsigned wrap safe: DEPTH is a power of two).
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
    //! at deferred-handler entry).  When > 0 the per-frame continuous-RX re-arm
    //! is skipped (throughput); the DISABLING/DISABLED seams re-arm
    //! unconditionally, so the radio is never left deaf once the queue drains.
    std::atomic<U32>  m_pendingTxFrames;

    //! Set by the ENABLED TX path: re-arm continuous RX at the handler tail,
    //! after comStatusOut has released the one-in-flight com pipeline.
    bool              m_rearmAfterTx;

    //! TX staging buffer for RadioHead-compat mode (header + payload).
    //! Sized to the on-air cap: compat mode limits the payload to
    //! MAX_PACKET_SIZE - 4 so header + payload always fits.  Component-thread
    //! only (written and consumed inside deferredTxPacket).
    U8                m_txScratch[MAX_PACKET_SIZE];

    // Telemetry accumulators
    FwSizeType m_bytesSent;
    FwSizeType m_bytesReceived;
    U32        m_rxReverts;

    //! True when ProfilePolicy has committed an RX auto-revert but the hardware
    //! apply has not yet succeeded — retried each run tick (this flag is the only
    //! record).  Cleared by a successful explicit SET_RX_PROFILE apply.
    bool m_revertRearmPending;

    //! Latched true after transmit is first enabled; gates radioFirstStart to a
    //! single emission per boot (mirrors Zephyr::LoRa::m_lora_ever_on).
    bool m_radioEverOn;

    //! One-in-flight comStatus invariant.  The pinned Svc::ComQueue asserts if
    //! it receives a comStatus while it is not in WAITING (i.e. a previously
    //! emitted status has not yet been "spent" by ComQueue dequeuing and
    //! sending us a frame).  True means we have emitted a comStatus that
    //! dataIn_handler has not yet observed a frame for — i.e. it is NOT safe
    //! to emit another one (ComQueue is presumed READY, not WAITING).
    //! Cleared in dataIn_handler (a frame arriving proves ComQueue consumed
    //! the outstanding status and is WAITING again); set on every
    //! comStatusOut_out emission (priming or per-frame).  Guards the
    //! DISABLED->ENABLED priming path so a repeat TRANSMIT ENABLED after a
    //! deaf-recovery DISABLED/DISABLED cycle — with no frame ever consumed —
    //! cannot double-prime into an already-READY ComQueue.
    //! Atomic: written from dataIn_handler (caller thread, per the class
    //! header's threading model) and from the component thread
    //! (startRadio()/deferredTransmitCmd/deferredTxPacket) — same cross-thread
    //! reasoning as m_pendingTxFrames.
    std::atomic<bool> m_comStatusOwed;
};

}  // namespace Zephyr

#endif  // ZEPHYR_USP_RADIO_HPP
