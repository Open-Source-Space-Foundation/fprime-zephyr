// ======================================================================
// \title  ZephyrUartDriver.cpp
// \author ethanchee
// \brief  cpp file for ZephyrUartDriver component implementation class
// ======================================================================


#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriver.hpp"
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/Assert.hpp"
#include <Fw/FPrimeBasicTypes.hpp>
#include <Fw/Logger/Logger.hpp>

#include <cstring>

namespace Zephyr {

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    ZephyrUartDriver ::
        ZephyrUartDriver(
            const char *const compName
        ) : ZephyrUartDriverComponentBase(compName), m_txDropCount(0), m_txDropsReported(0), m_txWriterRunning(false), m_txPaused(false)
    {
    }

    ZephyrUartDriver ::
        ~ZephyrUartDriver()
    {

    }

    void ZephyrUartDriver::configure(const struct device *dev, U32 baud_rate) {
        FW_ASSERT(dev != nullptr);
        m_dev = dev;

        if (!device_is_ready(this->m_dev)) {
            return;
        }

        struct uart_config uart_cfg = {
            .baudrate = baud_rate,
            .parity = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1,
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        };
        uart_configure(this->m_dev, &uart_cfg);

        ring_buf_init(&this->m_ring_buf, RING_BUF_SIZE, this->m_ring_buf_data);
        uart_irq_callback_user_data_set(this->m_dev, serial_cb, &this->m_ring_buf);

        uart_irq_rx_enable(this->m_dev);
	    uart_irq_tx_disable(this->m_dev);

        // Set up the TX staging ring buffer and its dedicated writer thread (see the
        // TX_RING_BUF_SIZE rationale in the header).
        ring_buf_init(&this->m_tx_ring_buf, TX_RING_BUF_SIZE, this->m_tx_ring_buf_data);
        k_sem_init(&this->m_tx_data_sem, 0, 1);

        // Explicit priority/stack: Os::Task::TASK_DEFAULT is numeric_limits max, which
        // k_thread_stack_alloc cannot satisfy (dynamic-thread pool stacks are
        // CONFIG_DYNAMIC_THREAD_STACK_SIZE=4096). Lowest preemptible priority is fine —
        // this thread only drains the staging ring into uart_poll_out.
        Os::TaskString taskName("UartTxWriter");
        Os::Task::Arguments txArgs(taskName, txWriterTaskEntry, this,
                                   static_cast<FwTaskPriorityType>(14),
                                   static_cast<FwSizeType>(TX_WRITER_STACK_SIZE));
        Os::Task::Status txStat = this->m_txWriterTask.start(txArgs);
        this->m_txWriterRunning = (txStat == Os::Task::OP_OK);
        if (!this->m_txWriterRunning) {
            // Degrade to the legacy synchronous-write path rather than asserting:
            // a boot-time FW_ASSERT here becomes a reboot loop now that FatalHandler
            // hard-resets on FATAL.
            Fw::Logger::log("ZephyrUartDriver: TX writer task failed to start (%d); "
                            "falling back to blocking writes\n",
                            static_cast<int>(txStat));
        }

        if (this->isConnected_ready_OutputPort(0)) {
            this->ready_out(0);
        }
    }

    void ZephyrUartDriver::serial_cb(const struct device *dev, void *user_data)
    {
        struct ring_buf *ring_buf = reinterpret_cast<struct ring_buf *>(user_data);

        if (!uart_irq_update(dev)) {
            return;
        }

        if (!uart_irq_rx_ready(dev)) {
            return;
        }

        U8 c;
        // TODO: Get rid of the endless loop (in an IRQ handler!).
        while (uart_fifo_read(dev, &c, 1) == 1) {
            if (ring_buf_put(ring_buf, &c, 1) != 1) {
                // TODO: Handle properly.
                printk("UART buffer overrun\n");
            }
        }
    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void ZephyrUartDriver ::
        schedIn_handler(
            const FwIndexType portNum,
            U32 context
        )
    {
        Fw::Buffer recv_buffer = this->allocate_out(0, SERIAL_BUFFER_SIZE);

        U32 recv_size = ring_buf_get(&this->m_ring_buf, recv_buffer.getData(), recv_buffer.getSize());
        if (recv_size == 0) {
            // No data received, deallocate buffer
            this->deallocate_out(0, recv_buffer);
        } else {
            recv_buffer.setSize(recv_size);
            recv_out(0, recv_buffer, Drv::ByteStreamStatus::OP_OK);
        }
    }

    Drv::ByteStreamStatus ZephyrUartDriver ::
        send_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        // NOTE: must never block — stage a length-prefixed copy for the TX writer thread;
        // if the ring is full, drop rather than block (see header rationale).
        const U32 frameSize = static_cast<U32>(sendBuffer.getSize());
        const U32 framedSize = static_cast<U32>(sizeof(frameSize)) + frameSize;

        if (!this->m_txWriterRunning) {
            // Writer thread unavailable (task start failed at configure time):
            // legacy synchronous path — may block under CDC stall, but never asserts.
            for (U32 i = 0; i < frameSize; i++) {
                uart_poll_out(this->m_dev, sendBuffer.getData()[i]);
            }
            return Drv::ByteStreamStatus::OP_OK;
        }

        bool enqueued = false;
        if ((frameSize > 0) && (framedSize <= sizeof(this->m_tx_ring_buf_data)) &&
            (ring_buf_space_get(&this->m_tx_ring_buf) >= framedSize)) {
            U8 lenPrefix[sizeof(frameSize)];
            (void) memcpy(lenPrefix, &frameSize, sizeof(frameSize));

            U32 wrote = ring_buf_put(&this->m_tx_ring_buf, lenPrefix, sizeof(lenPrefix));
            wrote += ring_buf_put(&this->m_tx_ring_buf, sendBuffer.getData(), frameSize);
            // Space was already confirmed above, so this should always fully succeed.
            enqueued = (wrote == framedSize);
        }

        if (enqueued) {
            (void) k_sem_give(&this->m_tx_data_sem);
            return Drv::ByteStreamStatus::OP_OK;
        }

        // Ring full: reject with OTHER_ERROR to restore END-TO-END FLOW CONTROL.
        // Returning OP_OK on a drop (earlier design) turns a CDC stall into an
        // unbounded frame pump — comStatus SUCCESS lets ComQueue immediately send the
        // next frame, saturating every queue in the pipeline until the aggregator's
        // 10 Hz state-machine signal hits a full queue and FW_ASSERTs (HWIL soaks 1-3,
        // 2026-07-10). OTHER_ERROR → ComStub emits comStatus FAILURE and sets
        // m_reinitialize; ComQueue pauses (holding/aging frames with its own WARNING
        // on overflow) until the TX writer thread drains and re-signals ready
        // (→ drvConnected → comStatus SUCCESS → resume).
        // Count silently — NEVER log from this path (console shares the stalled CDC;
        // an unbounded console write here wedges the calling com thread).
        this->m_txDropCount++;
        this->m_txPaused = true;
        return Drv::ByteStreamStatus::OTHER_ERROR;
    }

    void ZephyrUartDriver ::recvReturnIn_handler(const FwIndexType portNum, Fw::Buffer &returnBuffer) {
        this->deallocate_out(0, returnBuffer);
    }

    void ZephyrUartDriver ::txWriterTaskEntry(void *ptr)
    {
        FW_ASSERT(ptr != nullptr);
        ZephyrUartDriver *self = reinterpret_cast<ZephyrUartDriver *>(ptr);

        while (true) {
            // Wake on new data (k_sem_give from send_handler), or on the periodic timeout as a
            // defensive safety-net poll; either way, re-check the ring buffer for anything
            // staged since the last drain.
            (void) k_sem_take(&self->m_tx_data_sem, K_MSEC(TX_WRITER_WAKE_MS));

            bool drained = false;
            while (ring_buf_size_get(&self->m_tx_ring_buf) >= sizeof(U32)) {
                drained = true;
                U32 frameSize = 0;
                U8 lenPrefix[sizeof(frameSize)];
                U32 got = ring_buf_get(&self->m_tx_ring_buf, lenPrefix, sizeof(lenPrefix));
                if (got != sizeof(lenPrefix)) {
                    // Unexpected partial header; staging buffer is single-producer/single-
                    // consumer so this should not happen. Bail out of this drain pass.
                    break;
                }
                (void) memcpy(&frameSize, lenPrefix, sizeof(frameSize));
                if (frameSize > sizeof(self->m_tx_ring_buf_data)) {
                    // Corrupt/garbage length prefix - drop the rest of this drain pass rather
                    // than risk reading forever.
                    break;
                }

                U32 remaining = frameSize;
                while (remaining > 0) {
                    U8 chunk[64];
                    U32 want = (remaining < sizeof(chunk)) ? remaining : static_cast<U32>(sizeof(chunk));
                    U32 chunkGot = ring_buf_get(&self->m_tx_ring_buf, chunk, want);
                    if (chunkGot == 0) {
                        // Should not happen given the length prefix accounting above.
                        break;
                    }
                    for (U32 i = 0; i < chunkGot; i++) {
                        // May block under CDC backpressure — fine, this thread is decoupled.
                        uart_poll_out(self->m_dev, chunk[i]);
                    }
                    remaining -= chunkGot;
                }
            }

            // Safe reporting point: a drain just completed, so uart_poll_out (and therefore
            // the shared CDC console) is flowing. Report any drops accumulated during the
            // preceding stall window.
            if (drained && (self->m_txDropCount != self->m_txDropsReported)) {
                Fw::Logger::log("ZephyrUartDriver: rejected %d frame(s) during TX backpressure\n",
                                static_cast<int>(self->m_txDropCount));
                self->m_txDropsReported = self->m_txDropCount;
            }

            // Flow-control resume: if send_handler paused the upstream com stack
            // (ring was full) and the ring has drained below half, signal ready →
            // ComStub drvConnected → comStatus SUCCESS → ComQueue resumes sending.
            if (self->m_txPaused && drained &&
                (ring_buf_space_get(&self->m_tx_ring_buf) > (TX_RING_BUF_SIZE / 2))) {
                self->m_txPaused = false;
                if (self->isConnected_ready_OutputPort(0)) {
                    self->ready_out(0);
                }
            }
        }
    }

} // end namespace Zephyr
