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
        this->m_rx_paused = false;
        // Pass the driver instance (not just the ring) so the RX ISR can pause
        // itself for flow control by touching m_rx_paused / disabling the RX IRQ.
        uart_irq_callback_user_data_set(this->m_dev, serial_cb, this);

        uart_irq_rx_enable(this->m_dev);
	    uart_irq_tx_disable(this->m_dev);

        if (this->isConnected_ready_OutputPort(0)) {
            this->ready_out(0);
        }
    }

    void ZephyrUartDriver::serial_cb(const struct device *dev, void *user_data)
    {
        ZephyrUartDriver *drv = reinterpret_cast<ZephyrUartDriver *>(user_data);

        if (!uart_irq_update(dev)) {
            return;
        }

        if (!uart_irq_rx_ready(dev)) {
            return;
        }

        // Drain the hardware FIFO into the ring, but only while the ring has
        // room. When it fills, stop reading and disable the RX IRQ so the
        // USB-CDC/UART layer back-pressures the host (NAKs the bulk-OUT
        // endpoint) instead of us dropping bytes. schedIn re-enables RX once it
        // drains the ring. This is the flow control that bounds a fast host
        // (GDS) to the slow LoRa egress: LoRa slow -> pool full -> ring full ->
        // RX paused -> host blocks. Without it, a multi-KB host burst overruns
        // the ring and the dropped bytes corrupt the raw byte-stream
        // passthrough to LoRa (every downstream frame then fails flight auth).
        U8 c;
        while (ring_buf_space_get(&drv->m_ring_buf) > 0) {
            if (uart_fifo_read(dev, &c, 1) != 1) {
                break;  // FIFO drained; nothing more to read right now
            }
            // Space was just confirmed, so this put cannot fail.
            (void)ring_buf_put(&drv->m_ring_buf, &c, 1);
        }
        if (ring_buf_space_get(&drv->m_ring_buf) == 0) {
            uart_irq_rx_disable(dev);
            drv->m_rx_paused = true;
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

        // If the RX ISR paused itself for back-pressure (ring was full) and we
        // have since freed room, resume reading. Note pool exhaustion can leave
        // the ring full across ticks (allocate_out returns empty -> recv_size 0);
        // RX simply stays paused until LoRa drains the pool, which is exactly the
        // back-pressure we want.
        if (this->m_rx_paused && ring_buf_space_get(&this->m_ring_buf) > 0) {
            this->m_rx_paused = false;
            uart_irq_rx_enable(this->m_dev);
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
