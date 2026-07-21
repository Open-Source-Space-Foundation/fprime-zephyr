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

        U8 c;
        // TODO: Get rid of the endless loop (in an IRQ handler!).
        while (uart_fifo_read(dev, &c, 1) == 1) {
            if (ring_buf_put(&self->m_ring_buf, &c, 1) != 1) {
                // issue #457: this used to be a bare printk with no visibility
                // to the rest of the system -- a dropped byte here silently
                // desyncs whatever frame was in flight (deframer/uplink mode
                // machine confusion downstream). Count it; schedIn_handler
                // (normal task context, not ISR) turns this into an F' event +
                // telemetry channel. atomic_inc is ISR-safe and RAM-neutral
                // (single word).
                atomic_inc(&self->m_rx_overrun_count);
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
