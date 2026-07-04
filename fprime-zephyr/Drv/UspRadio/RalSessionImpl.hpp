// ======================================================================
// \title  RalSessionImpl.hpp
// \brief  On-target implementation of RalSession using Semtech USP RAL/RAC.
//
// COMPILE GUARD: this header (and its .cpp) must only be compiled when the
// USP Kconfig options are enabled.  The guard matches the v5e defconfig:
//   CONFIG_LORA_BASICS_MODEM_DRIVERS=y  (selects usp_zephyr module)
//   CONFIG_USP=y
//
// v5c / v5d / host builds must NEVER include this header directly.
// UspRadio.cpp receives the RalSession* via constructor injection.
//
// Threading: see REPORT-bench-phase0.md "RSSI-delta closure" — ALL RAL
// calls happen in pre_radio_transaction callbacks while the RAC holds the
// TCXO / radio lock.  This class never calls bare RAL functions from the
// F' component thread.
// ======================================================================

#ifndef ZEPHYR_USP_RADIO_RAL_SESSION_IMPL_HPP
#define ZEPHYR_USP_RADIO_RAL_SESSION_IMPL_HPP

#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS

#include "fprime-zephyr/Drv/UspRadio/RalSession.hpp"
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"

// USP / Zephyr headers — only visible in Zephyr builds
#include <zephyr/usp/smtc_zephyr_usp_api.h>
extern "C" {
#include <smtc_rac_api/smtc_rac_api.h>
#include <smtc_ralf/src/ralf.h>
#include <smtc_ralf/src/ralf_sx126x.h>
#include <smtc_ral/src/ral.h>
#include <smtc_ral/src/ral_defs.h>
}

#include <zephyr/kernel.h>

namespace Zephyr {

class RalSessionImpl final : public RalSession {
  public:
    //! Construct.  freq_hz / tx_power_dbm are the hardware-level radio
    //! parameters shared across all profiles (carrier frequency and PA power).
    //! Profile table supplies all modulation parameters; these two come from
    //! the board-level LoRaCfg / zephyr-config (kept consistent with the
    //! legacy LoRa component via zephyr-config/LoRaCfg.hpp).
    explicit RalSessionImpl(uint32_t freq_hz, int8_t tx_power_dbm);
    ~RalSessionImpl() override = default;

    int init() override;
    int applyProfile(const LinkProfile& profile) override;
    int startReceive() override;
    int transmitPacket(const uint8_t* data, std::size_t size) override;
    int startCw(const LinkProfile& cwProfile) override;
    int stopRadio() override;
    void setCallbacks(RxDoneCallback rxDone, TxDoneCallback txDone) override;

  private:
    // ------------------------------------------------------------------
    // RAC transaction context (pre-tx callback payload)
    // ------------------------------------------------------------------

    //! Context passed to RAC lock/transaction callbacks.
    struct TxContext {
        RalSessionImpl* self;
        const uint8_t*  data;
        std::size_t     size;
        int             result;  //!< Set by callback; read by transmitPacket()
    };

    struct CwContext {
        RalSessionImpl* self;
    };

    struct RxContext {
        RalSessionImpl* self;
    };

    // Static RAC callback trampolines
    static void onPreTx(void* ctx);
    static void onPreCw(void* ctx);
    static void onPreRx(void* ctx);
    static void onRxDone(void* ctx);  //!< post-transaction callback for received frame

    // Internal helpers (called from within RAC lock window)
    int applyLoRa(const LoRaParams& p);
    int applyGfsk(const GfskParams& p);

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    uint32_t        m_freq_hz;
    int8_t          m_tx_power_dbm;
    ral_t*          m_ral;         //!< obtained via ral_from_ralf after init
    ralf_t*         m_ralf;        //!< obtained from smtc_rac_get_radio after init
    bool            m_initialized;

    RxDoneCallback  m_rxDone;
    TxDoneCallback  m_txDone;

    // Receive scratchpad (filled by the USP thread in RX IRQ callback)
    static constexpr std::size_t MAX_RX_BUF = 255;
    uint8_t  m_rxBuf[MAX_RX_BUF];
    std::size_t m_rxLen;
    int16_t  m_rxRssi;
    int8_t   m_rxSnr;

    // Semaphore: transmitPacket() blocks until the RAC lock window fires
    struct k_sem m_txDoneSem;
    struct k_sem m_cwDoneSem;
};

}  // namespace Zephyr

#endif  // CONFIG_LORA_BASICS_MODEM_DRIVERS
#endif  // ZEPHYR_USP_RADIO_RAL_SESSION_IMPL_HPP
