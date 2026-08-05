module Zephyr {

    @ Transmit-enable state
    enum UspTransmitState : U8 {
        ENABLED
        DISABLED
        DISABLING
    }

    @ Direction selector for profile-change events
    enum UspRadioDirection : U8 {
        TX = 0
        RX = 1
    }

    @ UHF radio component using Semtech USP (RAL layer) on SX1262 (FCB v5e+).
    @ Active component - all RAL/SPI work runs on the component thread.
    @ External surface (Svc.Com + Svc.BufferAllocation) is identical to
    @ Zephyr::LoRa so the topology layer is board-agnostic.
    active component UspRadio {

        @ Import the communication interface (dataIn/dataOut/dataReturnIn/dataReturnOut/comStatusOut)
        import Svc.Com

        @ Import the buffer allocation interface (allocate/deallocate)
        import Svc.BufferAllocation

        @ Rate-group servicing: revert-deadline tick.  Async: a sync port
        @ would run run_handler on the CALLING (rate-group) thread, but
        @ run_handler does real RAL/SPI work (stopRadio/applyProfile/
        @ startReceive) on a revert.  Async routes it through the component's
        @ own message queue like every other RAL-touching handler in this
        @ component, so all RAL/SPI access stays single-threaded.
        async input port run: Svc.Sched

        @ Internal port - RX done (fires from USP callback; runs on component thread).
        @ Frame data (incl. rssi/snr) travels in the RX ring; this message only wakes the consumer.
        internal port deferredRxDone priority 10

        @ Internal port - TX packet (deferred from dataIn; runs on component thread)
        internal port deferredTxPacket(
            data:    Fw.Buffer
            context: ComCfg.FrameContext
        ) priority 10

        @ Internal port - TRANSMIT command state change (deferred to component thread)
        internal port deferredTransmitCmd(
            enabled: UspTransmitState
        ) priority 10

        @ Internal port - SET_TX_PROFILE command (deferred to component thread)
        internal port deferredSetTxProfile(
            profile: LinkProfileId
        ) priority 10

        @ Internal port - SET_RX_PROFILE command (deferred to component thread)
        internal port deferredSetRxProfile(
            profile:  LinkProfileId
            revert_s: U16
        ) priority 10

        @ Internal port - CONTINUOUS_WAVE command (deferred to component thread)
        internal port deferredContinuousWave(
            seconds: U16
        ) priority 10

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------

        @ RadioHead-header backwards compatibility for the UHF LoRa link.
        @ When true (default), TX prepends the 4-byte RadioHead-style header
        @ ({0,0,0,0}, byte-identical to legacy Zephyr::LoRa / LoRaConfig::HEADER)
        @ and RX strips the first 4 bytes of every received LoRa frame — required
        @ to interoperate with legacy Zephyr::LoRa peers and RadioHead-based
        @ ground radios (e.g. the CI adafruit_rfm9x passthrough board).
        @ Set false for raw F´ frames on the air (USP<->USP links only; both ends
        @ must agree).  Applies to LoRa profiles only; GFSK profiles are always raw.
        param RADIOHEAD_COMPAT: bool default true

        # ------------------------------------------------------------------
        # Commands
        # ------------------------------------------------------------------

        @ Start/stop transmission on the UHF USP radio
        async command TRANSMIT(enabled: UspTransmitState)

        @ Continuous-wave transmission for N seconds then return to RX
        async command CONTINUOUS_WAVE(seconds: U16)

        @ Select the active TX link profile
        async command SET_TX_PROFILE(profile: LinkProfileId)

        @ Select the active RX link profile; auto-reverts to P0 after revert_s if no frame received
        async command SET_RX_PROFILE(profile: LinkProfileId, revert_s: U16)

        # ------------------------------------------------------------------
        # Events
        # ------------------------------------------------------------------

        @ Active TX or RX profile has been switched
        event ProfileChanged(
            direction: UspRadioDirection
            profile:   LinkProfileId
        ) severity activity high \
            format "USP radio {} profile changed to {}"

        @ RX profile confirmed by reception of a valid frame
        event ProfileConfirmed(
            profile: LinkProfileId
        ) severity activity low \
            format "USP RX profile {} confirmed by received frame"

        @ RX profile auto-reverted to boot default (no frame received in time)
        event ProfileReverted(
            from_profile: LinkProfileId
            to_profile:   LinkProfileId
        ) severity warning low \
            format "USP RX profile reverted from {} to {} (revert timeout)"

        @ Radio hardware configuration failed for the given direction
        event ConfigurationFailed(
            mode: UspRadioDirection
        ) severity warning high \
            format "USP radio configuration failed for direction: {}" throttle 2

        @ Deferred profile change (SET_TX_PROFILE/SET_RX_PROFILE) could not be
        @ applied because the radio is busy (e.g. TX actively saturating the
        @ RAC hook) and stopRadio() could not abort it before its deadline.
        @ The change is left pending; retry once the radio quiesces (e.g.
        @ after TRANSMIT DISABLED) rather than resending immediately.
        event ProfileChangeDeferred(
            direction: UspRadioDirection
        ) severity warning low \
            format "USP radio {} profile change deferred: radio busy" throttle 2

        @ Packet transmit failed with RAL status code
        event SendFailed(
            status: I32
        ) severity warning high \
            format "USP radio send failed: {}" throttle 2

        @ Buffer allocation failed for received packet
        event AllocationFailed(
            allocation_size: FwSizeType
        ) severity warning high \
            format "USP radio failed to allocate buffer of: {} bytes" throttle 2

        @ SET_RX_PROFILE or SET_TX_PROFILE specified an out-of-range profile index
        event InvalidProfile(
            profile: LinkProfileId
        ) severity warning high \
            format "USP radio invalid profile index: {}" throttle 2

        @ RX ring full when a frame arrived — frame dropped (receiver-limited
        @ regime: frames arriving faster than the component thread drains them)
        event RxOverrun(
            total_dropped: U32
        ) severity warning high \
            format "USP radio RX ring overrun; {} frames dropped since boot" throttle 5

        # ------------------------------------------------------------------
        # Telemetry
        # ------------------------------------------------------------------

        @ Cumulative bytes sent
        telemetry BytesSent: FwSizeType update on change

        @ Cumulative bytes received
        telemetry BytesReceived: FwSizeType update on change

        @ RSSI of last received frame (dBm)
        telemetry LastRssi: I16 update on change

        @ SNR of last received frame (dB, integer)
        telemetry LastSnr: I8 update on change

        @ Currently active TX profile index
        telemetry TxProfile: LinkProfileId update on change

        @ Currently active RX profile index
        telemetry RxProfile: LinkProfileId update on change

        @ Number of RX auto-reverts since boot
        telemetry RxReverts: U32 update on change

        @ Received frames dropped because the RX ring was full
        telemetry RxDropped: U32 update on change

        # ------------------------------------------------------------------
        # Startup transmit-quiescence contract (parity with Zephyr::LoRa)
        # ------------------------------------------------------------------

        @ Enable radio transmission.  Signal-only equivalent of
        @ TRANSMIT(ENABLED); driven by StartupManager once the launch
        @ quiescence period has elapsed.  Sync port: the handler only enqueues
        @ onto the component queue, so no RAL/SPI work runs on the caller thread.
        sync input port enableTransmit: Fw.Signal

        @ Disable radio transmission.  Signal-only equivalent of
        @ TRANSMIT(DISABLING); same deferral rules as enableTransmit.
        sync input port disableTransmit: Fw.Signal

        @ Emitted exactly once per boot, the first time radio transmit becomes
        @ enabled (whether via startRadio(ENABLED), the TRANSMIT command, or the
        @ enableTransmit port).  Consumed by StartupManager to suppress its
        @ hard-coded fallback enable.  Equivalent to Zephyr::LoRa.loraFirstStart.
        output port radioFirstStart: Fw.Signal

        # ------------------------------------------------------------------
        # Standard AC Ports
        # ------------------------------------------------------------------

        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Port to return the value of a parameter
        param get port prmGetOut

        @ Port to set the value of a parameter
        param set port prmSetOut

    }

}
