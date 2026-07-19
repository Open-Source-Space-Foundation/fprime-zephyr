// ======================================================================
// \title  LoRa.hpp
// \author lestarch
// \brief  hpp file for LoRa component implementation class
// ======================================================================

#ifndef Zephyr_LoRa_HPP
#define Zephyr_LoRa_HPP

#include "Os/Mutex.hpp"
#include "fprime-zephyr/Drv/LoRa/LoRaComponentAc.hpp"
#include <zephyr/drivers/lora.h>

namespace Zephyr {

class LoRa final : public LoRaComponentBase {
  public:
    //! Maximum payload size of the LoRa radio. This is a hardware property.
    static constexpr FwSizeType MAX_PACKET_SIZE = 252;
    // Status returned from various LoRa operations
    enum Status { NOT_READY, NOT_INITIALIZED, ERROR, SUCCESS };
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct LoRa object
    LoRa(const char* const compName  //!< The component name
    );

    //! Destroy LoRa object
    ~LoRa();

    //! LoRa receive callback
    static void receiveCallback(const struct device* dev, U8* data, U16 size, I16 rssi, I8 snr, void* user_data);

    //! Configure LoRa radio the supplied device and start it
    //!
    //! Applies runtime transmit state from the TRANSMIT_ENABLE parameter. Prefer loading
    //! parameters before calling start().
    Status start(const struct device* lora_device);

    //! Configure LoRa radio and set TRANSMIT_ENABLE (updates the local parameter)
    Status start(const struct device* lora_device, TransmitEnable transmit_enabled);

    //! Enable tx
    Status enableTx();

    //! Enable rx
    Status enableRx(bool initial = false);

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for dataIn
    //!
    //! Data to be sent on the wire (coming in to the component)
    void dataIn_handler(FwIndexType portNum,  //!< The port number
                        Fw::Buffer& data,
                        const ComCfg::FrameContext& context) override;

    //! Handler implementation for dataReturnIn
    //!
    //! Port receiving back ownership of buffer sent out on dataOut
    void dataReturnIn_handler(FwIndexType portNum,  //!< The port number
                              Fw::Buffer& data,
                              const ComCfg::FrameContext& context) override;

    //! Handler implementation for enableTransmit
    void enableTransmit_handler(FwIndexType portNum  //!< The port number
                                ) override;

    //! Handler implementation for disableTransmit
    void disableTransmit_handler(FwIndexType portNum  //!< The port number
                                 ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command CONTINUOUS_WAVE
    //!
    //! No-op command
    void CONTINUOUS_WAVE_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                    U32 cmdSeq,           //!< The command sequence number
                                    U16 seconds) override;

    // ----------------------------------------------------------------------
    // Parameter handlers
    // ----------------------------------------------------------------------

    //! Called when a parameter is updated (e.g. via PRM_SET)
    void parameterUpdated(FwPrmIdType id  //!< The parameter ID
                          ) override;

    //! Set the runtime transmit state of the LoRa component
    //! Used by parameter updates and the enable/disable transmit port handlers
    void setTransmitState(TransmitState state);

    //! Sync TRANSMIT_ENABLE parameter and apply runtime transmit state
    void setTransmitEnable(TransmitEnable enable);

  private:
    U8 m_send_buffer[LoRa::MAX_PACKET_SIZE];  //!< Buffer for sending data (max LoRa packet size)
    //! Process received data
    //!
    void receive(U8* data,  //!< Data to process
                 U16 size,  //!< Size of the data
                 I16 rssi,  //!< RSSI value for telemetry
                 I8 snr     //!< SNR value for telemetry
    );

    //! Pointer to the LoRa device
    const struct device* m_lora_device;
    Zephyr::TransmitState m_transmit_enabled;  //!< Runtime transmit state (includes DISABLING)
    Os::Mutex m_mutex;                         //!< Mutex for thread safety

    FwSizeType m_bytes_sent = 0;      //!< Total bytes sent telemetry
    FwSizeType m_bytes_received = 0;  //!< Total bytes received telemetry
};

}  // namespace Zephyr

#endif
