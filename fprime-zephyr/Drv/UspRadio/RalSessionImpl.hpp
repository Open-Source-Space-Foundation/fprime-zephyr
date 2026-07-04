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
// UspRadio.cpp receives the RalSession* via configure() injection.
//
// Threading: see REPORT-bench-phase0.md "RSSI-delta closure" — ALL RAL
// calls happen in pre_radio_transaction callbacks while the RAC holds the
// TCXO / radio lock.  This class never calls bare RAL functions from the
// F' component thread.
//
// Callback design: smtc_rac_scheduler_config_t callbacks take no context
// pointer (2025 RAC API).  A file-scope static RalSessionImpl* s_instance
// (set during init()) lets the static trampolines reach instance state.
// Only one RalSessionImpl may exist at a time — guaranteed because there
// is one UspRadio component instance per topology.
// ======================================================================

#ifndef ZEPHYR_USP_RADIO_RAL_SESSION_IMPL_HPP
#define ZEPHYR_USP_RADIO_RAL_SESSION_IMPL_HPP

#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS

#include "fprime-zephyr/Drv/UspRadio/RalSession.hpp"
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"

// USP / Zephyr headers — only visible in Zephyr builds
#include <zephyr/usp/smtc_zephyr_usp_api.h>
extern "C" {
// Include paths (from Zephyr module CMakeLists):
//   smtc_rac_lib/smtc_rac_api  -> include as <smtc_rac_api.h>
//   smtc_rac_lib/smtc_ralf/src -> include as <ralf.h> / <ralf_sx126x.h>
//   smtc_rac_lib/smtc_ral/src  -> include as <ral.h>  / <ral_defs.h>
#include <smtc_rac_api.h>
#include <ralf.h>
#include <ralf_sx126x.h>
#include <ral.h>
#include <ral_defs.h>
}

#include <zephyr/kernel.h>

namespace Zephyr {

class RalSessionImpl final : public RalSession {
  public:
    //! Construct.  freq_hz / tx_power_dbm are the hardware-level radio
    //! parameters shared across all profiles (carrier frequency and PA power).
    //! Profile table supplies all modulation parameters; these two come from
    //! the board-level configuration (passed in from ReferenceDeploymentTopology.cpp).
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
    // RAC transaction context (pre-tx callback payload, stored in instance)
    // ------------------------------------------------------------------

    //! Transmit scratch: set before smtc_rac_lock_radio_access, read after.
    struct TxScratch {
        const uint8_t*  data;
        std::size_t     size;
        int             result;  //!< Set by pre-callback; read by transmitPacket()
    };

    //! Apply-profile scratch
    struct ApplyScratch {
        const LinkProfile* profile;
        int                result;
    };

    // ------------------------------------------------------------------
    // Static RAC callback trampolines (2025 RAC API: no context param).
    // Access instance state via s_instance.
    // ------------------------------------------------------------------

    // applyProfile lock window
    static void onPreApply(void);
    static void onPostApply(rp_status_t status);

    // startReceive lock window (pre opens RX; post fires on each frame)
    static void onPreRx(void);
    static void onPostRx(rp_status_t status);

    // transmitPacket lock window
    static void onPreTx(void);
    static void onPostTx(rp_status_t status);

    // startCw lock window
    static void onPreCw(void);

    // stopRadio lock window
    static void onPreStop(void);

    // Internal helpers (called from within RAC lock window)
    bool applyLoRa_or_Gfsk(const LinkProfile& p);
    int applyLoRa(const LoRaParams& p);
    int applyGfsk(const GfskParams& p);

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    uint32_t        m_freq_hz;
    int8_t          m_tx_power_dbm;
    uint8_t         m_radio_id;    //!< obtained from smtc_rac_open_radio() in init()
    ral_t*          m_ral;         //!< obtained via ral_from_ralf after init
    ralf_t*         m_ralf;        //!< obtained from smtc_rac_get_radio after init
    bool            m_initialized;

    RxDoneCallback  m_rxDone;
    TxDoneCallback  m_txDone;

    // Per-operation scratch (valid only while an operation is in-flight)
    TxScratch    m_txScratch;
    ApplyScratch m_applyScratch;

    // Receive scratchpad (filled by the USP thread in RX IRQ callback)
    static constexpr std::size_t MAX_RX_BUF = 255;
    uint8_t     m_rxBuf[MAX_RX_BUF];
    std::size_t m_rxLen;
    int16_t     m_rxRssi;
    int8_t      m_rxSnr;

    // Semaphore: transmitPacket() blocks until the RAC lock window fires
    struct k_sem m_txDoneSem;
    struct k_sem m_cwDoneSem;

    // ------------------------------------------------------------------
    // Singleton pointer — set in init(), read by static trampolines.
    // Safe because the topology only creates one UspRadio / RalSessionImpl.
    // ------------------------------------------------------------------
    static RalSessionImpl* s_instance;
};

}  // namespace Zephyr

#endif  // CONFIG_LORA_BASICS_MODEM_DRIVERS
#endif  // ZEPHYR_USP_RADIO_RAL_SESSION_IMPL_HPP
