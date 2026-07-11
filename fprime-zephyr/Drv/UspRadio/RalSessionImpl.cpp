// ======================================================================
// \title  RalSessionImpl.cpp
// \brief  On-target RalSession implementation (Semtech USP RAL/RAC).
//
// Only compiled when CONFIG_LORA_BASICS_MODEM_DRIVERS=y (see CMakeLists.txt).
//
// KEY THREADING RULE (from REPORT-bench-phase0.md + REPORT-ral-architecture):
//   ALL ral_* calls must run on the USP thread inside a RAC lock window
//   (smtc_rac_lock_radio_access pre_radio_transaction callback) or inside
//   a RAC transaction's pre_tx_callback.
//   The F' component thread calls into this class but NEVER touches
//   smtc_rac_* directly — all RAC calls happen here, inside the lock.
//   Continuous RX + RAL_IRQ_NONE never fires the post-callback (bench lesson);
//   the RSSI/payload read happens in the DIO1 IRQ path, not a post-callback.
//
// CW sequencing (from REPORT-ral-architecture gotchas):
//   1. ral_set_pkt_type → ral_set_rf_freq → ral_set_tx_cfg
//   2. ral_set_tx_cw()
//   3. k_timer fires after N seconds → stopRadio() → back to RX
//   CW runs as a single RAC lock window; the k_timer stop is scheduled
//   from outside that window (from the F' component thread) and calls
//   stopRadio() which acquires its own lock window.
//
// Callback design (2025 RAC API):
//   smtc_rac_scheduler_config_t callbacks carry NO context pointer.
//   All state is accessed via the file-scope s_instance singleton.
//   Only one RalSessionImpl may be active at a time (enforced by topology).
// ======================================================================

#ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS

#include "fprime-zephyr/Drv/UspRadio/RalSessionImpl.hpp"
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"
// NOTE: LoRaCfg.hpp is intentionally NOT included here.  RalSessionImpl
// receives freq_hz / tx_power_dbm through its constructor; it does not use
// the Zephyr LoRa driver header (which requires CONFIG_LORA=y — absent on v5e).

#include <Fw/Logger/Logger.hpp>
#include <zephyr/kernel.h>

// Pull in USP RAL pulse-shape / CRC enums
// (smtc_rac_lib/smtc_ral/src is in the include path; include as <ral_defs.h>)
extern "C" {
#include <ral_defs.h>
}

// smtc_modem_hal_get_time_in_ms is implemented in the usp_zephyr module as
// k_uptime_get_32().  Its header is not on the public include path, so we
// either forward-declare it or use k_uptime_get_32() directly.  Using
// k_uptime_get_32() is simpler and avoids a private-header dependency.
//
// The RAC scheduler_config.start_time_ms comment says:
//   "set smtc_modem_hal_get_time_in_ms() if you want NOW"
// k_uptime_get_32() is precisely that — same implementation.

namespace Zephyr {

// ---------------------------------------------------------------------------
// Singleton pointer — set in init(), read by static trampolines
// ---------------------------------------------------------------------------

RalSessionImpl* RalSessionImpl::s_instance = nullptr;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

RalSessionImpl::RalSessionImpl(uint32_t freq_hz, int8_t tx_power_dbm)
    : m_freq_hz(freq_hz),
      m_tx_power_dbm(tx_power_dbm),
      m_radio_id(RAC_INVALID_RADIO_ID),
      m_ral(nullptr),
      m_ralf(nullptr),
      m_initialized(false),
      m_rxLen(0),
      m_rxRssi(0),
      m_rxSnr(0) {
    m_txScratch = {};
    m_applyScratch = {};
    k_sem_init(&m_txDoneSem, 0, 1);
    k_sem_init(&m_cwDoneSem, 0, 1);
    k_sem_init(&m_applyDoneSem, 0, 1);
    k_sem_init(&m_stopDoneSem, 0, 1);
}

// ---------------------------------------------------------------------------
// Callback registration
// ---------------------------------------------------------------------------

void RalSessionImpl::setCallbacks(RxDoneCallback rxDone, TxDoneCallback txDone) {
    m_rxDone = rxDone;
    m_txDone = txDone;
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

int RalSessionImpl::init() {
    // Wait for the USP thread to complete its boot sequence (void return).
    zephyr_usp_initialization_wait();

    // zephyr_usp_thread.c intentionally does NOT call smtc_rac_init() (it is
    // commented out in the upstream source).  We call smtc_rac_init() directly
    // from the RAC public API.  Note: zephyr_smtc_rac_init() is declared in the
    // USP Zephyr header but has no implementation — call smtc_rac_init() directly.
    smtc_rac_init();

    // Open a persistent radio access session at high priority.
    m_radio_id = smtc_rac_open_radio(RAC_HIGH_PRIORITY);
    if (m_radio_id == RAC_INVALID_RADIO_ID) {
        Fw::Logger::log("[UspRadio] smtc_rac_open_radio() returned INVALID_ID\n");
        return -ENODEV;
    }

    // Obtain the ral_t* via the RALF abstraction that USP provides.
    m_ralf = smtc_rac_get_radio();
    if (m_ralf == nullptr) {
        Fw::Logger::log("[UspRadio] smtc_rac_get_radio() returned null\n");
        return -EIO;
    }
    m_ral = ral_from_ralf(m_ralf);

    // Hardware reset + init via RAL
    ral_reset(m_ral);
    ral_init(m_ral);

    // Register singleton for static callback trampolines
    s_instance = this;

    m_initialized = true;
    Fw::Logger::log("[UspRadio] RAL session initialized (freq=%" PRIu32 " Hz, pwr=%" PRId8 " dBm)\n",
                    m_freq_hz, m_tx_power_dbm);
    return 0;
}

// ---------------------------------------------------------------------------
// applyProfile() — translates LinkProfile → RAL calls inside a lock window
// ---------------------------------------------------------------------------

// Helper called from within the RAC lock window (static trampoline lands here)
bool RalSessionImpl::applyLoRa_or_Gfsk(const LinkProfile& p) {
    if (p.mod == ModKind::LORA) {
        return applyLoRa(p.lora) == 0;
    } else {
        return applyGfsk(p.gfsk) == 0;
    }
}

void RalSessionImpl::onPreApply(void) {
    RalSessionImpl* self = s_instance;
    if (self == nullptr) return;

    if (self->applyLoRa_or_Gfsk(*self->m_applyScratch.profile)) {
        self->m_applyScratch.result = 0;
    } else {
        self->m_applyScratch.result = -EIO;
    }
    // Unlock immediately — we don't need to hold through a TX/RX.
    // The unlock fires the post-callback (onPostApply) after the radio planner
    // transitions the task to FINISHED.  We signal from onPostApply (not here)
    // so that applyProfile() does not attempt a new smtc_rac_lock_radio_access
    // before the task has reached FINISHED state.
    smtc_rac_unlock_radio_access(self->m_radio_id);
}

void RalSessionImpl::onPostApply(rp_status_t /*status*/) {
    // Fires on the USP thread after the radio planner has fully processed the
    // unlock (task state → FINISHED).  Signal applyProfile() that the result
    // is ready AND the hook_id is available for the next enqueue.
    RalSessionImpl* self = s_instance;
    if (self == nullptr) return;
    k_sem_give(&self->m_applyDoneSem);
}

int RalSessionImpl::applyProfile(const LinkProfile& profile) {
    m_applyScratch.profile = &profile;
    m_applyScratch.result  = -ENODEV;

    smtc_rac_scheduler_config_t cfg = {};
    cfg.start_time_ms                  = k_uptime_get_32();
    cfg.scheduling                     = SMTC_RAC_ASAP_TRANSACTION;
    cfg.callback_pre_radio_transaction  = onPreApply;
    cfg.callback_post_radio_transaction = onPostApply;

    smtc_rac_return_code_t rc = smtc_rac_lock_radio_access(m_radio_id, cfg);
    if (rc != SMTC_RAC_SUCCESS) {
        return -EIO;
    }
    // Block until onPreApply has run and set m_applyScratch.result.
    // Timeout: 2 s (RAC should schedule ASAP on an idle radio).
    int ret = k_sem_take(&m_applyDoneSem, K_SECONDS(2));
    if (ret != 0) {
        Fw::Logger::log("[UspRadio] applyProfile: RAC pre-callback timeout\n");
        return -ETIMEDOUT;
    }
    return m_applyScratch.result;
}

int RalSessionImpl::applyLoRa(const LoRaParams& p) {
    // 1. Packet type
    ral_set_pkt_type(m_ral, RAL_PKT_TYPE_LORA);

    // 2. Modulation params
    ral_lora_mod_params_t mod = {};
    mod.sf   = static_cast<ral_lora_sf_t>(p.sf);  // RAL_LORA_SF5..SF12
    mod.bw   = RAL_LORA_BW_125_KHZ;               // all v1 profiles
    mod.cr   = static_cast<ral_lora_cr_t>(p.cr - 4);  // 5→CR4/5=0, etc.
    mod.ldro = p.ldro ? 1 : 0;
    if (ral_set_lora_mod_params(m_ral, &mod) != RAL_STATUS_OK) return -EIO;

    // 3. Packet params
    ral_lora_pkt_params_t pkt = {};
    pkt.preamble_len_in_symb = p.preamble_len;
    pkt.header_type          = RAL_LORA_PKT_EXPLICIT;
    pkt.pld_len_in_bytes     = 0xFF;   // max; actual payload set per TX
    pkt.crc_is_on            = true;
    pkt.invert_iq_is_on      = false;
    if (ral_set_lora_pkt_params(m_ral, &pkt) != RAL_STATUS_OK) return -EIO;

    // 4. Sync word (1-byte convention: 0x12 = private, 0x34 = public;
    //    ral_set_lora_sync_word takes uint8_t; LinkProfiles stores the value in U16)
    if (ral_set_lora_sync_word(m_ral, static_cast<uint8_t>(p.sync_word)) != RAL_STATUS_OK) return -EIO;

    // 5. Frequency + PA config
    if (ral_set_rf_freq(m_ral, m_freq_hz) != RAL_STATUS_OK) return -EIO;
    if (ral_set_tx_cfg(m_ral, m_tx_power_dbm, 0) != RAL_STATUS_OK) return -EIO;

    return 0;
}

int RalSessionImpl::applyGfsk(const GfskParams& p) {
    // 1. Packet type
    ral_set_pkt_type(m_ral, RAL_PKT_TYPE_GFSK);

    // 2. Modulation params
    ral_gfsk_mod_params_t mod = {};
    mod.br_in_bps    = p.bitrate_bps;
    mod.fdev_in_hz   = p.fdev_hz;
    // Map GfskPulseShape enum to RAL pulse shape
    switch (p.pulse_shape) {
        case GfskPulseShape::NONE:  mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_OFF;    break;
        case GfskPulseShape::BT_03: mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_03;  break;
        case GfskPulseShape::BT_05: mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_05;  break;
        case GfskPulseShape::BT_07: mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_07;  break;
        default:                    mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_05;  break;
    }
    mod.bw_dsb_in_hz = p.bw_dsb_hz;
    if (ral_set_gfsk_mod_params(m_ral, &mod) != RAL_STATUS_OK) return -EIO;

    // 3. Packet params
    ral_gfsk_pkt_params_t pkt = {};
    pkt.preamble_len_in_bits     = p.preamble_len_bits;
    pkt.preamble_detector        = RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
    pkt.sync_word_len_in_bits    = p.sync_word_len * 8;
    pkt.header_type              = RAL_GFSK_PKT_VAR_LEN;
    pkt.pld_len_in_bytes         = 0xFF;
    switch (p.crc_type) {
        case GfskCrcType::OFF:    pkt.crc_type = RAL_GFSK_CRC_OFF;    break;
        case GfskCrcType::BYTE_1: pkt.crc_type = RAL_GFSK_CRC_1_BYTE; break;
        case GfskCrcType::BYTE_2: pkt.crc_type = RAL_GFSK_CRC_2_BYTES; break;
        default:                  pkt.crc_type = RAL_GFSK_CRC_2_BYTES; break;
    }
    pkt.dc_free = p.whitening ? RAL_GFSK_DC_FREE_WHITENING : RAL_GFSK_DC_FREE_OFF;
    if (ral_set_gfsk_pkt_params(m_ral, &pkt) != RAL_STATUS_OK) return -EIO;

    // 4. Sync word
    if (ral_set_gfsk_sync_word(m_ral, p.sync_word, p.sync_word_len) != RAL_STATUS_OK) return -EIO;

    // 4b. CRC seed/polynomial (must be set after pkt_params; not set by ral_set_gfsk_pkt_params).
    //     Standard CCITT values — matches Phase-0 known-good reference (app_per_fsk.h defaults).
    //     seed=0x1D0F, polynomial=0x1021 for RAL_GFSK_CRC_2_BYTES (non-inverted).
    if (pkt.crc_type != RAL_GFSK_CRC_OFF) {
        if (ral_set_gfsk_crc_params(m_ral, 0x1D0FU, 0x1021U) != RAL_STATUS_OK) return -EIO;
    }

    // 4c. Whitening seed (must be set after pkt_params; not set by ral_set_gfsk_pkt_params).
    //     Standard value 0x01FF — matches Phase-0 known-good reference.
    if (p.whitening) {
        if (ral_set_gfsk_whitening_seed(m_ral, 0x01FFU) != RAL_STATUS_OK) return -EIO;
    }

    // 5. Frequency + PA config
    if (ral_set_rf_freq(m_ral, m_freq_hz) != RAL_STATUS_OK) return -EIO;
    if (ral_set_tx_cfg(m_ral, m_tx_power_dbm, 0) != RAL_STATUS_OK) return -EIO;

    return 0;
}

// ---------------------------------------------------------------------------
// startReceive()
// ---------------------------------------------------------------------------

void RalSessionImpl::onPreRx(void) {
    RalSessionImpl* self = s_instance;
    if (self == nullptr) return;

    // PRE callback fires once when the LOCK task is first launched (ASAP → running).
    // Set up IRQ mask and start continuous RX.  Packet reading happens in
    // onPostRx (called on each DIO1 with RP_STATUS_RADIO_LOCKED while RUNNING).
    ral_set_dio_irq_params(self->m_ral,
        RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_CRC_ERROR);
    ral_cfg_rx_boosted(self->m_ral, true);
    ral_set_rx(self->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
    // Lock stays held — the USP thread will service DIO1 edges via onPostRx.
    // stopRadio() aborts via smtc_rac_unlock_radio_access → onPostRx(RADIO_UNLOCKED).
}

void RalSessionImpl::onPostRx(rp_status_t status) {
    RalSessionImpl* self = s_instance;
    if (self == nullptr) return;

    // POST callback fires on two paths:
    //   (A) DIO1 fires while LOCK task is RUNNING → RP_STATUS_RADIO_LOCKED.
    //       Read the IRQ register; if RX_DONE and no CRC_ERROR, harvest the
    //       packet payload and invoke the rxDone callback.  Then re-arm
    //       continuous RX to await the next frame (lock stays held).
    //   (B) Task freed via abort (stopRadio()) → RP_STATUS_RADIO_UNLOCKED or
    //       RP_STATUS_TASK_ABORTED.  Signal the stop waiter.
    //       NOTE: do NOT call ral_set_standby() here — rp_callback already
    //       called ral_set_sleep(retain=true) before invoking this callback.

    if (status == RP_STATUS_RADIO_LOCKED) {
        // (A) DIO1 fired while RX lock is running — read IRQ and harvest packet.
        ral_irq_t irq = RAL_IRQ_NONE;
        ral_get_and_clear_irq_status(self->m_ral, &irq);

        if ((irq & RAL_IRQ_RX_DONE) && !(irq & RAL_IRQ_RX_CRC_ERROR)) {
            uint8_t  buf[MAX_RX_BUF];
            uint16_t len = 0;

            ral_handle_rx_done(self->m_ral);
            ral_get_pkt_payload(self->m_ral, MAX_RX_BUF, buf, &len);

            ral_lora_rx_pkt_status_t lora_status = {};
            ral_gfsk_rx_pkt_status_t gfsk_status = {};
            int16_t rssi = 0;
            int8_t  snr  = 0;

            if (ral_get_lora_rx_pkt_status(self->m_ral, &lora_status) == RAL_STATUS_OK) {
                rssi = static_cast<int16_t>(lora_status.rssi_pkt_in_dbm);
                snr  = static_cast<int8_t>(lora_status.snr_pkt_in_db);
            } else if (ral_get_gfsk_rx_pkt_status(self->m_ral, &gfsk_status) == RAL_STATUS_OK) {
                rssi = static_cast<int16_t>(gfsk_status.rssi_avg_in_dbm);
                snr  = 0;
            }

            if (self->m_rxDone) {
                self->m_rxDone(buf, len, rssi, snr);
            }
        }

        // Re-arm continuous RX for the next frame.
        ral_set_rx(self->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
        return;
    }

    if (status == RP_STATUS_TASK_ABORTED || status == RP_STATUS_RADIO_UNLOCKED) {
        // (B) Task freed via stopRadio().
        k_sem_give(&self->m_stopDoneSem);
        return;
    }

}


int RalSessionImpl::startReceive() {
    smtc_rac_scheduler_config_t cfg = {};
    cfg.start_time_ms                  = k_uptime_get_32();
    cfg.scheduling                     = SMTC_RAC_ASAP_TRANSACTION;
    cfg.callback_pre_radio_transaction  = onPreRx;
    cfg.callback_post_radio_transaction = onPostRx;

    smtc_rac_return_code_t rc = smtc_rac_lock_radio_access(m_radio_id, cfg);
    if (rc != SMTC_RAC_SUCCESS) {
        Fw::Logger::log("[UspRadio] startReceive: smtc_rac_lock_radio_access failed rc=%d\n",
                        static_cast<int>(rc));
        return -EIO;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// transmitPacket()
// ---------------------------------------------------------------------------

void RalSessionImpl::onPreTx(void) {
    RalSessionImpl* self = s_instance;
    if (self == nullptr) return;

    // PRE callback fires once when the LOCK task is first launched.
    // Set up payload, configure TX_DONE IRQ, and start the transmitter.
    // TX_DONE handling happens in onPostTx(RP_STATUS_RADIO_LOCKED).

    // Set payload
    if (ral_set_pkt_payload(self->m_ral,
                            self->m_txScratch.data,
                            static_cast<uint16_t>(self->m_txScratch.size)) != RAL_STATUS_OK) {
        self->m_txScratch.result = -EIO;
        smtc_rac_unlock_radio_access(self->m_radio_id);
        return;
    }
    // IRQ: TX_DONE only
    ral_set_dio_irq_params(self->m_ral, RAL_IRQ_TX_DONE);
    // Start TX
    if (ral_set_tx(self->m_ral) != RAL_STATUS_OK) {
        self->m_txScratch.result = -EIO;
        smtc_rac_unlock_radio_access(self->m_radio_id);
        return;
    }
    // Lock stays held until TX_DONE DIO1 fires → onPostTx(RP_STATUS_RADIO_LOCKED).
}

void RalSessionImpl::onPostTx(rp_status_t status) {
    RalSessionImpl* self = s_instance;
    if (self == nullptr) return;

    // POST callback fires on three paths:
    //   (A) DIO1 fires with TX_DONE while LOCK task RUNNING → RP_STATUS_RADIO_LOCKED.
    //       Read IRQ, call txDone callback, set result=0, unlock to free the task.
    //       The unlock will fire this callback again with RP_STATUS_RADIO_UNLOCKED (B).
    //   (B) Task freed after our own unlock (from path A) → RP_STATUS_RADIO_UNLOCKED.
    //       Signal the TX waiter.
    //   (C) Mid-flight abort (stopRadio()) → RP_STATUS_TASK_ABORTED.
    //       Signal both the TX waiter and the stop waiter.
    //
    // NOTE: do NOT call ral_set_standby() on paths B/C — rp_callback already
    // called ral_set_sleep(retain=true) before invoking this callback.

    if (status == RP_STATUS_RADIO_LOCKED) {
        // (A) TX_DONE IRQ
        ral_irq_t irq = RAL_IRQ_NONE;
        ral_get_and_clear_irq_status(self->m_ral, &irq);
        if (irq & RAL_IRQ_TX_DONE) {
            ral_handle_tx_done(self->m_ral);
            if (self->m_txDone) self->m_txDone();
        }
        self->m_txScratch.result = 0;
        // Unlock → rp_callback processes UNLOCK_RADIO_ACCESS → rp_task_free →
        // ral_set_sleep → fires this callback again with RP_STATUS_RADIO_UNLOCKED.
        smtc_rac_unlock_radio_access(self->m_radio_id);
        return;
    }

    if (status == RP_STATUS_RADIO_UNLOCKED) {
        // (B) Task freed after our own TX_DONE-triggered unlock.
        // m_txScratch.result was set in path (A).
        k_sem_give(&self->m_txDoneSem);
        return;
    }

    if (status == RP_STATUS_TASK_ABORTED) {
        // (C) Mid-flight abort from stopRadio()
        self->m_txScratch.result = -ECANCELED;
        k_sem_give(&self->m_txDoneSem);
        k_sem_give(&self->m_stopDoneSem);
        return;
    }

    k_sem_give(&self->m_txDoneSem);
}

int RalSessionImpl::transmitPacket(const uint8_t* data, std::size_t size) {
    m_txScratch.data   = data;
    m_txScratch.size   = size;
    m_txScratch.result = 0;

    smtc_rac_scheduler_config_t cfg = {};
    cfg.start_time_ms                  = k_uptime_get_32();
    cfg.scheduling                     = SMTC_RAC_ASAP_TRANSACTION;
    cfg.callback_pre_radio_transaction  = onPreTx;
    cfg.callback_post_radio_transaction = onPostTx;

    smtc_rac_return_code_t rc = smtc_rac_lock_radio_access(m_radio_id, cfg);
    if (rc != SMTC_RAC_SUCCESS) return -EIO;

    // Block the component thread until TX_DONE IRQ (signalled via semaphore
    // in onPostTx).  Timeout: conservatively 10 s (longest LoRa symbol airtime).
    int ret = k_sem_take(&m_txDoneSem, K_SECONDS(10));
    return (ret == 0) ? m_txScratch.result : -ETIMEDOUT;
}

// ---------------------------------------------------------------------------
// startCw()
// ---------------------------------------------------------------------------

// CW requires: pkt_type + freq + power configured FIRST, then ral_set_tx_cw().
// (REPORT-ral-architecture gotcha #4)

void RalSessionImpl::onPreCw(void) {
    RalSessionImpl* self = s_instance;
    if (self == nullptr) return;

    // CW always uses LoRa packet type regardless of profile modulation
    ral_set_pkt_type(self->m_ral, RAL_PKT_TYPE_LORA);
    ral_set_rf_freq(self->m_ral, self->m_freq_hz);
    ral_set_tx_cfg(self->m_ral, self->m_tx_power_dbm, 0);
    ral_set_tx_cw(self->m_ral);
    // Signal the component thread that CW is running
    k_sem_give(&self->m_cwDoneSem);
    // Lock stays held for the CW duration; stopRadio() acquires a new lock
    // to call ral_set_standby and release.
}

// CW post-callback: fires when the CW task is aborted via stopRadio().
// Signals m_stopDoneSem so stopRadio() can unblock.
void RalSessionImpl::onPostCw(rp_status_t /*status*/) {
    RalSessionImpl* self = s_instance;
    if (self == nullptr) return;
    // Signal stop waiter regardless of status (abort or unlock).
    k_sem_give(&self->m_stopDoneSem);
}

int RalSessionImpl::startCw(const LinkProfile& /*cwProfile*/) {
    smtc_rac_scheduler_config_t cfg = {};
    cfg.start_time_ms                  = k_uptime_get_32();
    cfg.scheduling                     = SMTC_RAC_ASAP_TRANSACTION;
    cfg.callback_pre_radio_transaction  = onPreCw;
    // CW holds the lock until stopRadio() calls smtc_rac_unlock_radio_access,
    // which triggers this post-callback (RP_STATUS_TASK_ABORTED / RADIO_UNLOCKED).
    cfg.callback_post_radio_transaction = onPostCw;

    smtc_rac_return_code_t rc = smtc_rac_lock_radio_access(m_radio_id, cfg);
    if (rc != SMTC_RAC_SUCCESS) return -EIO;
    // Wait until the CW pre-callback has run (and CW is actually transmitting)
    int ret = k_sem_take(&m_cwDoneSem, K_SECONDS(2));
    return (ret == 0) ? 0 : -ETIMEDOUT;
}

// ---------------------------------------------------------------------------
// stopRadio()
//
// Design: smtc_rac_lock_radio_access cannot be called while another task on
// the same hook_id is RUNNING (rp_task_enqueue returns
// RP_TASK_STATUS_ALREADY_RUNNING).  Instead, we call
// smtc_rac_unlock_radio_access() directly to abort whatever is running, then
// wait for the running task's post-callback to fire with RP_STATUS_TASK_ABORTED.
// The post-callbacks (onPostRx, onPostTx, onPostCw_trampoline) detect the abort
// and give m_stopDoneSem.  After that the hook_id is FINISHED and
// transmitPacket() can safely call smtc_rac_lock_radio_access.
//
// Completion detection: the authoritative "hook is free" signal is the radio
// planner's task state — rp_task_enqueue() only accepts a new lock once the
// hook has returned to RP_TASK_STATE_FINISHED (idle hooks already sit there,
// see rp_task_free() / radio planner init). The post-callback semaphore is
// used as a fast wake-up only: if the hook was already idle no callback ever
// fires, and for an aborted task rp_task_call_aborted() marks the hook
// FINISHED via rp_task_free() immediately before running the callback.
//
// The previous implementation used one fixed 50 ms semaphore wait and treated
// a timeout with unlock_rc == SUCCESS as "already idle". That misclassified
// the common busy case: aborting a RUNNING task is only processed when the
// USP thread next runs the RAC engine (rp_task_abort() just raises a fake
// soft IRQ), and under load that regularly exceeded 50 ms. stopRadio() then
// returned success while the hook was still RUNNING, so the caller's
// immediate smtc_rac_lock_radio_access() failed — a transient
// ConfigurationFailed on roughly 1 in 10 profile switches. unlock_rc cannot
// disambiguate this: rp_task_abort() returns OK for idle, pending, and
// running hooks alike.
// ---------------------------------------------------------------------------

namespace {
constexpr int64_t STOP_DEADLINE_MS = 1000;  //!< generous bound on USP-thread abort processing
constexpr int32_t STOP_POLL_MS = 5;         //!< hook-state poll interval while waiting
}  // namespace

int RalSessionImpl::stopRadio() {
    // Drain any stale semaphore count from a previous un-awaited stop.
    (void)k_sem_take(&m_stopDoneSem, K_NO_WAIT);

    // Abort whatever is running on this hook (RX, CW, or idle). A non-SUCCESS
    // return means the abort could not even be submitted (rp_task_abort()
    // failed) — the hook may be wedged RUNNING, and every later
    // smtc_rac_lock_radio_access() on it would return SMTC_RAC_ERROR.
    // Surface it distinctly rather than swallowing it.
    smtc_rac_return_code_t unlock_rc = smtc_rac_unlock_radio_access(m_radio_id);
    if (unlock_rc != SMTC_RAC_SUCCESS) {
        Fw::Logger::log(
            "[UspRadio] stopRadio: abort FAILED on radio_id=%" PRIu8
            " (unlock_rc=%d) - RAC hook likely wedged RUNNING\n",
            m_radio_id, static_cast<int>(unlock_rc));
        return -EIO;
    }

    // Wait until the hook is actually free: fast path is the abort
    // post-callback giving m_stopDoneSem; truth is the hook state reaching
    // RP_TASK_STATE_FINISHED (covers the already-idle hook, where no
    // callback is coming, without a race on pre-abort state snapshots).
    radio_planner_t* rp = smtc_rac_get_rp();
    const int64_t deadline = k_uptime_get() + STOP_DEADLINE_MS;
    for (;;) {
        if (k_sem_take(&m_stopDoneSem, K_MSEC(STOP_POLL_MS)) == 0) {
            return 0;
        }
        if (rp->tasks[m_radio_id].state == RP_TASK_STATE_FINISHED) {
            return 0;
        }
        if (k_uptime_get() >= deadline) {
            break;
        }
    }

    Fw::Logger::log(
        "[UspRadio] stopRadio: hook not FINISHED %" PRId64 " ms after abort on radio_id=%" PRIu8
        " (state=%d) - RAC hook likely wedged RUNNING\n",
        STOP_DEADLINE_MS, m_radio_id, static_cast<int>(rp->tasks[m_radio_id].state));
    return -ETIMEDOUT;
}

}  // namespace Zephyr

#endif  // CONFIG_LORA_BASICS_MODEM_DRIVERS
