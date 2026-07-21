module Zephyr {

  passive component ZephyrUartDriver {
    import Drv.ByteStreamDriver

    @ Polled sched-in for reading UART
    guarded input port schedIn: Svc.Sched

    @Allocate new buffer
    output port allocate: Fw.BufferGet

    @return the allocated buffer
    output port deallocate: Fw.BufferSend

    ###############################################################################
    # Standard AC Ports: Required for Events and Telemetry                       #
    ###############################################################################
    @ Port for requesting the current time
    time get port timeCaller

    @ Port for sending textual representation of events
    text event port logTextOut

    @ Port for sending events to downlink
    event port logOut

    @ Port for sending telemetry channels to downlink
    telemetry port tlmOut

    @ issue #457: the RX ISR (serial_cb) drops a byte on the floor whenever the
    # software ring buffer is full (ring_buf_put fails). This used to be a bare
    # printk with no visibility to the rest of the system. Surface it as a
    # throttled warning event plus a running count telemetry channel so RX
    # overruns are diagnosable without a debugger attached.
    @ A byte was dropped because the RX ring buffer was full
    event RxRingBufferOverrun(totalDropped: U32) severity warning low \
        format "UART RX ring buffer overrun, {} byte(s) dropped total" throttle 10

    @ Running count of bytes dropped due to RX ring buffer overrun
    telemetry RxOverrunCount: U32 update on change
  }
}
