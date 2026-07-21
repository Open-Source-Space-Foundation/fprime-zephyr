// ======================================================================
// \title  ZephyrUartDriver.hpp
// \author ethanchee
// \brief  hpp file for ZephyrUartDriver component implementation class
// ======================================================================

#ifndef ZephyrUartDriver_HPP
#define ZephyrUartDriver_HPP

#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriverComponentAc.hpp"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>

#define RING_BUF_SIZE 1024

namespace Zephyr {

  class ZephyrUartDriver :
    public ZephyrUartDriverComponentBase
  {

    // issue #457: was 64. RAM-neutral bump -- commsBufferManager's pool bin is
    // already sized at 1024B per buffer (see ComCcsdsConfig::BuffMgr::commsBuffSize),
    // so requesting up to 248B per allocate_out() call still comes out of the same
    // existing 1024B-bucket pool; this does not add any static/pool RAM.
    static constexpr FwSizeType SERIAL_BUFFER_SIZE = 248;

    // issue #457: schedIn_handler now drains the ring buffer in a loop instead of
    // taking a single SERIAL_BUFFER_SIZE-sized bite per tick, so that a burst that
    // fully fills the (unchanged, RAM-neutral) 1024B ring buffer can be fully
    // drained in one scheduler tick rather than dribbling out 1 read's worth per
    // tick while more bytes queue up and eventually overrun. Capped at the number
    // of reads needed to drain a completely full ring buffer, so worst-case work
    // per tick is bounded (no unbounded loop / no new allocation growth).
    static constexpr FwSizeType MAX_DRAIN_ITERATIONS_PER_TICK = (RING_BUF_SIZE / SERIAL_BUFFER_SIZE) + 1;

    public:

        // ----------------------------------------------------------------------
        // Construction, initialization, and destruction
        // ----------------------------------------------------------------------

        //! Construct object ZephyrUartDriver
        //!
        ZephyrUartDriver(
            const char *const compName /*!< The component name*/
        );

        //! Destroy object ZephyrUartDriver
        //!
        ~ZephyrUartDriver();

        void configure(const struct device *dev, U32 baud_rate);

    public:

        static void serial_cb(const struct device *dev, void *user_data);

        // ----------------------------------------------------------------------
        // Handler implementations for user-defined typed input ports
        // ----------------------------------------------------------------------

        //! Handler implementation for schedIn
        //!
        void schedIn_handler(
            const FwIndexType portNum, /*!< The port number*/
            U32 context /*!< 
        The call order
        */
        );


        //! Handler implementation for send
        //!
        Drv::ByteStreamStatus send_handler(
            const FwIndexType portNum, /*!< The port number*/
            Fw::Buffer &sendBuffer 
        );

        void recvReturnIn_handler(
            const FwIndexType portNum, /*!< The port number*/
            Fw::Buffer &returnBuffer
        );

        const struct device *m_dev;

        U8 m_ring_buf_data[RING_BUF_SIZE];
        struct ring_buf m_ring_buf;

        // issue #457: incremented (from ISR context, via serial_cb) every time
        // ring_buf_put() fails because the ring buffer is full, i.e. a received
        // byte was dropped. atomic_t so the ISR-side increment and the
        // schedIn_handler-side read (normal task context) don't race. No new
        // buffers/pools -- this is a single word.
        atomic_t m_rx_overrun_count;

        // Last value of m_rx_overrun_count that was reported via event/telemetry,
        // so schedIn_handler only emits when the count has actually changed.
        U32 m_rx_overrun_count_reported;

        //! Report any new RX ring-buffer overruns since the last report (called
        //! from schedIn_handler, i.e. normal task context -- never from the ISR).
        void reportRxOverrunsIfAny();

        // issue #457: ISR-level RX back-pressure (GRC-proven pattern, RAM-neutral
        // -- ring stays 1024B, no growth). Previously the RX ISR would keep
        // pulling bytes out of the UART/USB-CDC hardware FIFO and silently drop
        // them on the floor once m_ring_buf was full (see the old
        // RxRingBufferOverrun path). Now, once the ring is full, the ISR
        // disables its own RX interrupt instead of reading further -- the
        // USB-CDC stack then NAKs the host's bulk-OUT endpoint, so unread bytes
        // queue up in the HOST's USB stack (true end-to-end flow control)
        // rather than being lost. schedIn_handler re-enables RX once it has
        // freed ring space. This means even a multi-hundred-ms stall elsewhere
        // in the system (e.g. a flash erase/program stalling XIP) is survivable
        // -- the host simply waits longer, nothing is dropped.
        //
        // volatile: written by the ISR (serial_cb), read/cleared by
        // schedIn_handler (normal task context). Only ever transitions
        // false->true in the ISR and true->false in schedIn_handler, so there is
        // no read-modify-write race -- a plain volatile bool is sufficient
        // (no atomic needed for a single-writer-per-direction flag like this).
        volatile bool m_rx_paused;
    };

} // end namespace Zephyr

#endif
