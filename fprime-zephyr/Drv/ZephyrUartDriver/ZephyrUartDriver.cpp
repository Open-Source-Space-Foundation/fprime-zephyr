// ======================================================================
// \title  ZephyrUartDriver.cpp
// \author ethanchee
// \brief  cpp file for ZephyrUartDriver component implementation class
// ======================================================================


#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriver.hpp"
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/Assert.hpp"
#include <Fw/FPrimeBasicTypes.hpp>

namespace Zephyr {

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    ZephyrUartDriver ::
        ZephyrUartDriver(
            const char *const compName
        ) : ZephyrUartDriverComponentBase(compName)
    {
        atomic_set(&this->m_rx_overrun_count, 0);
        this->m_rx_overrun_count_reported = 0;
        this->m_rx_paused = false;
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
        // issue #457: pass `this` (rather than just &m_ring_buf) so the ISR can
        // also bump the overrun counter on drop.
        uart_irq_callback_user_data_set(this->m_dev, serial_cb, this);

        uart_irq_rx_enable(this->m_dev);
	    uart_irq_tx_disable(this->m_dev);

        if (this->isConnected_ready_OutputPort(0)) {
            this->ready_out(0);
        }
    }

    void ZephyrUartDriver::serial_cb(const struct device *dev, void *user_data)
    {
        ZephyrUartDriver *self = reinterpret_cast<ZephyrUartDriver *>(user_data);

        if (!uart_irq_update(dev)) {
            return;
        }

        if (!uart_irq_rx_ready(dev)) {
            return;
        }

        // issue #457: ISR-level RX back-pressure (GRC-proven pattern). Only pull
        // a byte out of the hardware FIFO if the ring buffer actually has room
        // for it. Once the ring is full, stop reading and disable the RX
        // interrupt entirely -- for a USB-CDC-backed UART this makes the
        // underlying stack NAK the host's bulk-OUT endpoint, so unread bytes
        // queue up in the HOST's USB stack instead of being dropped here. This
        // is true end-to-end flow control: previously, once the ring filled,
        // this loop kept draining the hardware FIFO anyway and threw the bytes
        // away (see the old RxRingBufferOverrun counter below), which is what
        // silently corrupted uplinks. schedIn_handler re-enables RX once it has
        // freed ring space.
        U8 c;
        while (ring_buf_space_get(&self->m_ring_buf) > 0) {
            if (uart_fifo_read(dev, &c, 1) != 1) {
                break;  // Hardware FIFO drained; nothing more to read right now.
            }
            // Space was just confirmed above, so this put cannot fail -- but
            // keep the old counting path too as a should-never-fire-now
            // tripwire in case of a future regression.
            if (ring_buf_put(&self->m_ring_buf, &c, 1) != 1) {
                atomic_inc(&self->m_rx_overrun_count);
            }
        }
        if (ring_buf_space_get(&self->m_ring_buf) == 0) {
            uart_irq_rx_disable(dev);
            self->m_rx_paused = true;
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
        // issue #457: drain the ring buffer in a loop instead of taking a single
        // SERIAL_BUFFER_SIZE-sized bite per tick. Previously, if the ISR filled
        // the ring buffer faster than one bite per tick could drain it (e.g. a
        // rate-group cycle slip delaying this handler), bytes would back up and
        // get silently dropped by the ISR's ring_buf_put() failure path even
        // though there was no new data still arriving -- this schedIn call
        // simply never got back around to draining the backlog before the next
        // burst added more. The iteration cap bounds worst-case per-tick work to
        // fully draining a completely full (unchanged, 1024B) ring buffer, so
        // this cannot spin unboundedly.
        for (FwSizeType i = 0; i < MAX_DRAIN_ITERATIONS_PER_TICK; i++) {
            Fw::Buffer recv_buffer = this->allocate_out(0, SERIAL_BUFFER_SIZE);

            U32 recv_size = ring_buf_get(&this->m_ring_buf, recv_buffer.getData(), recv_buffer.getSize());
            if (recv_size == 0) {
                // Ring buffer empty -- nothing more to drain this tick.
                this->deallocate_out(0, recv_buffer);
                break;
            }
            recv_buffer.setSize(recv_size);
            recv_out(0, recv_buffer, Drv::ByteStreamStatus::OP_OK);
        }

        // issue #457: if the RX ISR paused itself for back-pressure (ring was
        // full) and draining above has since freed room, resume RX. If the
        // ring is still full (e.g. commsBufferManager's pool is exhausted so
        // allocate_out() keeps failing and the drain loop can't make progress),
        // RX correctly stays paused/back-pressured until room frees up --
        // that's the flow control working as intended, not a bug.
        if (this->m_rx_paused && (ring_buf_space_get(&this->m_ring_buf) > 0)) {
            this->m_rx_paused = false;
            uart_irq_rx_enable(this->m_dev);
        }

        this->reportRxOverrunsIfAny();
    }

    void ZephyrUartDriver::reportRxOverrunsIfAny() {
        // Called from schedIn_handler (normal task context) only -- never from
        // serial_cb (ISR context). atomic_get is safe to race against the ISR's
        // atomic_inc; worst case we report on the next tick instead.
        U32 current = static_cast<U32>(atomic_get(&this->m_rx_overrun_count));
        if (current != this->m_rx_overrun_count_reported) {
            this->m_rx_overrun_count_reported = current;
            if (this->isConnected_logOut_OutputPort(0)) {
                this->log_WARNING_LO_RxRingBufferOverrun(current);
            }
            if (this->isConnected_tlmOut_OutputPort(0)) {
                this->tlmWrite_RxOverrunCount(current);
            }
        }
    }

    Drv::ByteStreamStatus ZephyrUartDriver ::
        send_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        for (U32 i = 0; i < sendBuffer.getSize(); i++) {
            uart_poll_out(this->m_dev, sendBuffer.getData()[i]);
        }
        return Drv::ByteStreamStatus::OP_OK;
    }

    void ZephyrUartDriver ::recvReturnIn_handler(const FwIndexType portNum, Fw::Buffer &returnBuffer) {
        this->deallocate_out(0, returnBuffer);
    }

} // end namespace Zephyr
