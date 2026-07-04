module Zephyr {

    @ Transmit-enable state - identical to LoRa.fpp (TRANSMIT verbatim)
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

        @ Rate-group servicing: revert-deadline tick + periodic telemetry flush
        sync input port run: Svc.Sched

        @ Internal port - RX done (fires from USP callback; runs on component thread)
        internal port deferredRxDone(
            rssi: I16
            snr:  I8
        ) priority 10

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
        # Commands - verbatim from LoRa.fpp + profile commands
        # ------------------------------------------------------------------

        @ Start/stop transmission on the UHF USP radio (verbatim from LoRa)
        async command TRANSMIT(enabled: UspTransmitState)

        @ Continuous-wave transmission for N seconds then return to RX (verbatim from LoRa)
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

        @ Profile table version (from LinkProfiles.hpp)
        telemetry ProfileTableVersion: U8 update on change

        @ Number of RX auto-reverts since boot
        telemetry RxReverts: U32 update on change

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
