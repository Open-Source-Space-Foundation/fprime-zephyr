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

// The RX ISR drains the UART/USB-CDC FIFO into this ring; schedIn empties it
// into the buffer pool. A fast host burst (e.g. a file-uplink chunk sequence)
// can deliver more bytes than one schedIn tick drains, so the ring must absorb
// the burst -- on overflow the ISR back-pressures the host (see serial_cb).
// 16384 covers the observed worst-case GDS burst with margin.
#define RING_BUF_SIZE 16384

namespace Zephyr {

  class ZephyrUartDriver :
    public ZephyrUartDriverComponentBase
  {

    // Bytes pulled from the RX ring per schedIn tick. On the data path each pull
    // becomes exactly one LoRa frame, so this also sets the LoRa payload size.
    // Over-air loss is strongly inter-frame-spacing dependent (bench-measured:
    // ~1 s gap = 0% loss, ~130 ms = ~16%); at 64 each cooldown-paced GDS chunk
    // was chopped into ~4 back-to-back LoRa frames, landing in the lossy regime.
    // Sizing this to the LoRa max payload (252 - 4 B header = 248) makes each
    // well-spaced chunk a single frame.
    const FwSizeType SERIAL_BUFFER_SIZE = 248;

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

        // Set by the RX ISR when it disables itself because m_ring_buf is full
        // (flow-control back-pressure); cleared by schedIn when it re-enables RX
        // after freeing ring space. volatile: written in ISR, read/written in the
        // schedIn thread (the RX-IRQ-disabled invariant serializes the handoff).
        volatile bool m_rx_paused = false;
    };

} // end namespace Zephyr

#endif
