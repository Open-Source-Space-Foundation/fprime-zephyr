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

#include <Os/Task.hpp>

#define RING_BUF_SIZE 1024

// TX staging ring buffer used to decouple send_handler() (called synchronously on the caller's
// thread, e.g. the ComAggregator consumer thread) from the actual UART write, which can block
// for a long time (uart_poll_out() on a CDC-ACM backed UART spins until the host reads or DTR
// drops - see cdc_acm_poll_out() in usbd_cdc_acm.c). Frames are stored as a 4-byte length prefix
// followed by the raw bytes. Sized to comfortably hold roughly two max-size ComCcsds frames
// (BuffMgr.commsBuffSize == 2048 bytes) plus their headers.
#define TX_RING_BUF_SIZE 4096

// Re-log the TX-backpressure warning only every Nth drop, to avoid flooding the log while a CDC
// host session is attached-but-stalled for an extended period (can last from ~25s to minutes).
#define TX_DROP_LOG_INTERVAL 50

// How long the TX writer thread waits on its semaphore before re-checking the ring buffer for
// any data that may have been staged without a wakeup being observed (defensive; the thread
// runs for the lifetime of the deployment, so this is just a periodic safety-net poll).
#define TX_WRITER_WAKE_MS 100

// Stack for the TX writer thread. Must not exceed CONFIG_DYNAMIC_THREAD_STACK_SIZE (4096) —
// Os::Task::TASK_DEFAULT is numeric_limits<FwSizeType>::max() and cannot be satisfied by
// k_thread_stack_alloc, so an explicit size is mandatory here.
#define TX_WRITER_STACK_SIZE 2048

namespace Zephyr {

  class ZephyrUartDriver :
    public ZephyrUartDriverComponentBase
  {

    const FwSizeType SERIAL_BUFFER_SIZE = 64;

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

        //! Entry point for the dedicated TX writer thread. Drains m_tx_ring_buf and performs
        //! the (potentially long-blocking) uart_poll_out() calls, fully decoupled from
        //! whichever thread calls send_handler.
        static void txWriterTaskEntry(void *ptr);

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

        // ---- TX backpressure handling ----

        //! Staging ring buffer for outgoing frames: single producer (send_handler, called
        //! synchronously on the caller's thread) / single consumer (m_txWriterTask), so no
        //! additional locking is required (mirrors the m_ring_buf RX pattern above).
        U8 m_tx_ring_buf_data[TX_RING_BUF_SIZE];
        struct ring_buf m_tx_ring_buf;

        //! Signals the TX writer thread that new data has been staged.
        struct k_sem m_tx_data_sem;

        //! Dedicated task that drains m_tx_ring_buf via the (blocking) uart_poll_out() path,
        //! isolated from the com thread that calls send_handler.
        Os::Task m_txWriterTask;

        //! Count of frames dropped because m_tx_ring_buf did not have room for them (sustained
        //! TX backpressure, e.g. a CDC host session attached-but-stalled).
        U32 m_txDropCount;

        //! Last m_txDropCount value reported via the TX writer thread's post-drain log.
        U32 m_txDropsReported;

        //! True once m_txWriterTask started successfully; if false, send_handler degrades to
        //! the legacy synchronous uart_poll_out path instead of staging (never asserts).
        bool m_txWriterRunning;

        //! Set when send_handler rejected a frame (ring full → upstream paused via
        //! OTHER_ERROR/comStatus FAILURE); cleared by the TX writer thread when the ring
        //! drains below half, at which point it re-signals ready to resume the upstream.
        bool m_txPaused;
    };

} // end namespace Zephyr

#endif
