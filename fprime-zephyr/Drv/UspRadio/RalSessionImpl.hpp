// ======================================================================
// \title  RalSessionImpl.hpp
// \brief  On-target implementation of RalSession using Semtech USP RAL/RAC.
//         Compiled only when CONFIG_LORA_BASICS_MODEM_DRIVERS=y; host builds
//         must never include this directly (UspRadio gets a RalSession* via
//         configure() injection).  See RalSessionImpl.cpp for threading and
//         callback design.
// ======================================================================

#ifndef ZEPHYR_USP_RADIO_RAL_SESSION_IMPL_HPP
#define ZEPHYR_USP_RADIO_RAL_SESSION_IMPL_HPP

#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS

#include "fprime-zephyr/Drv/UspRadio/RalSession.hpp"
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"

// USP / Zephyr headers — only visible in Zephyr builds
#include <zephyr/usp/smtc_zephyr_usp_api.h>
extern "C" {
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
    int startCw() override;
    int stopRadio() override;
    void setCallbacks(RxDoneCallback rxDone) override;

  private:
    //! Force the transceiver to STDBY_XOSC with IRQs cleared (chip-level
    //! "radio stopped" postcondition; see the .cpp comment for the
    //! anomaly-B rationale).
    void quiesceRadio();
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
    static void onPostCw(rp_status_t status);

    // No pre/post callbacks for stopRadio: stop is achieved via
    // smtc_rac_unlock_radio_access (abort), which fires the running task's
    // post-callback with RP_STATUS_TASK_ABORTED.  See stopRadio() + onPostRx/Tx/Cw.

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

    // Per-operation scratch (valid only while an operation is in-flight)
    TxScratch    m_txScratch;
    ApplyScratch m_applyScratch;

    // Last-applied packet params, kept so onPreTx can re-issue them with the
    // TRUE payload length before each transmit and onPreRx can restore the
    // profile's max-length value before each RX arm.  The SX126x takes its
    // transmit length (and the explicit-header / GFSK length-byte value) from
    // SetPacketParams — ral_set_pkt_payload only writes the FIFO — and in
    // GFSK variable-length RX pld_len_in_bytes acts as the max-accepted
    // payload filter.
    ral_pkt_type_t        m_pktType;
    ral_lora_pkt_params_t m_loraPktParams;
    ral_gfsk_pkt_params_t m_gfskPktParams;

    // Semaphore: transmitPacket() blocks until the RAC lock window fires
    struct k_sem m_txDoneSem;
    struct k_sem m_cwDoneSem;
    // Semaphore: applyProfile() blocks until onPreApply has run and set the result.
    // Without this, applyProfile() reads m_applyScratch.result before the RAC
    // pre-callback has executed (smtc_rac_lock_radio_access enqueues async).
    struct k_sem m_applyDoneSem;
    // Semaphore: stopRadio() blocks until onPreStop has run (chip in standby).
    // Required so transmitPacket() doesn't try to enqueue while RX lock is
    // still running (RP_TASK_STATUS_ALREADY_RUNNING → SMTC_RAC_ERROR).
    struct k_sem m_stopDoneSem;

    // ------------------------------------------------------------------
    // Singleton pointer — set in init(), read by static trampolines.
    // Safe because the topology only creates one UspRadio / RalSessionImpl.
    // ------------------------------------------------------------------
    static RalSessionImpl* s_instance;
};

}  // namespace Zephyr

#endif  // CONFIG_LORA_BASICS_MODEM_DRIVERS
#endif  // ZEPHYR_USP_RADIO_RAL_SESSION_IMPL_HPP
