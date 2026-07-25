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

// k_uptime_get_32() == smtc_modem_hal_get_time_in_ms() (same implementation);
// used directly for scheduler start_time_ms to avoid a private-header dependency.

namespace Zephyr {

namespace {
//! Max on-air packet size read into the onPostRx local buffer.
constexpr std::size_t MAX_RX_BUF = 255;
}  // namespace

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
      m_pktType(RAL_PKT_TYPE_LORA) {
    m_txScratch = {};
    m_applyScratch = {};
    m_loraPktParams = {};
    m_gfskPktParams = {};
    k_sem_init(&m_txDoneSem, 0, 1);
    k_sem_init(&m_cwDoneSem, 0, 1);
    k_sem_init(&m_applyDoneSem, 0, 1);
    k_sem_init(&m_stopDoneSem, 0, 1);
}

// ---------------------------------------------------------------------------
// Callback registration
// ---------------------------------------------------------------------------

void RalSessionImpl::setCallbacks(RxDoneCallback rxDone) {
    m_rxDone = rxDone;
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
    RalSessionImpl* self = s_instance;  // registered in init(), never cleared

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
    RalSessionImpl* self = s_instance;  // registered in init(), never cleared
    k_sem_give(&self->m_applyDoneSem);
}

int RalSessionImpl::applyProfile(const LinkProfile& profile) {
    // Drain any stale count from a previous timed-out apply whose onPostApply
    // straggled in after the 2 s wait expired (same discipline as stopRadio()).
    (void)k_sem_take(&m_applyDoneSem, K_NO_WAIT);

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
    m_pktType       = RAL_PKT_TYPE_LORA;
    m_loraPktParams = pkt;

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
    m_pktType       = RAL_PKT_TYPE_GFSK;
    m_gfskPktParams = pkt;

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
    RalSessionImpl* self = s_instance;  // registered in init(), never cleared

    // PRE callback fires once when the LOCK task is first launched (ASAP → running).
    // Set up IRQ mask and start continuous RX.  Packet reading happens in
    // onPostRx (called on each DIO1 with RP_STATUS_RADIO_LOCKED while RUNNING).

    // Restore the profile's packet params before arming: onPreTx re-issues
    // them with the (short) true TX length, and in GFSK variable-length RX
    // pld_len_in_bytes is the max-ACCEPTED payload — a stale short TX value
    // would silently reject longer incoming frames.
    if (self->m_pktType == RAL_PKT_TYPE_GFSK) {
        ral_set_gfsk_pkt_params(self->m_ral, &self->m_gfskPktParams);
    } else {
        ral_set_lora_pkt_params(self->m_ral, &self->m_loraPktParams);
    }
    ral_set_dio_irq_params(self->m_ral,
        RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_CRC_ERROR);
    ral_cfg_rx_boosted(self->m_ral, true);
    ral_set_rx(self->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
    // Lock stays held — the USP thread will service DIO1 edges via onPostRx.
    // stopRadio() aborts via smtc_rac_unlock_radio_access → onPostRx(RADIO_UNLOCKED).
}

void RalSessionImpl::onPostRx(rp_status_t status) {
    RalSessionImpl* self = s_instance;  // registered in init(), never cleared

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


namespace {
//! Bound on waiting for the RAC hook to reach FINISHED before a new enqueue.
//! The USP thread (higher priority than the component thread) normally frees
//! the hook within one engine pass; 100 ms is a generous margin.  Hooks
//! initialize to RP_TASK_STATE_FINISHED (rp_init), so the boot path never waits.
constexpr int32_t ENQUEUE_READY_DEADLINE_MS = 100;
}  // namespace

int RalSessionImpl::startReceive() {
    // smtc_rac_lock_radio_access → rp_task_enqueue returns
    // RP_TASK_STATUS_ALREADY_RUNNING (→ SMTC_RAC_ERROR) if the hook is still
    // RP_TASK_STATE_RUNNING.  Completion semaphores (m_txDoneSem etc.) are
    // given from INSIDE the radio planner's unlock processing (rp_hook_callback
    // runs before rp_callback finishes freeing state), so this thread can
    // legally observe a hook the USP thread has not yet finished with.  Wait
    // (bounded) for FINISHED — the same completion criterion stopRadio() uses —
    // instead of failing the enqueue and leaving the radio deaf.
    radio_planner_t* rp = smtc_rac_get_rp();
    int32_t waited_ms = 0;
    while ((rp->tasks[m_radio_id].state != RP_TASK_STATE_FINISHED) &&
           (waited_ms < ENQUEUE_READY_DEADLINE_MS)) {
        k_sleep(K_MSEC(1));
        ++waited_ms;
    }
    if (rp->tasks[m_radio_id].state != RP_TASK_STATE_FINISHED) {
        Fw::Logger::log(
            "[UspRadio] startReceive: RAC hook not FINISHED after %" PRId32
            " ms (state=%d) - radio busy\n",
            ENQUEUE_READY_DEADLINE_MS, static_cast<int>(rp->tasks[m_radio_id].state));
        return -EBUSY;
    }

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
    RalSessionImpl* self = s_instance;  // registered in init(), never cleared

    // PRE callback fires once when the LOCK task is first launched.
    // Set up payload, configure TX_DONE IRQ, and start the transmitter.
    // TX_DONE handling happens in onPostTx(RP_STATUS_RADIO_LOCKED).

    // Program the TRUE payload length via SetPacketParams: ral_set_pkt_payload
    // only writes the FIFO, and applyProfile left pld_len at the 0xFF
    // placeholder — without this every frame radiates 255 B (payload + stale
    // FIFO tail).
    ral_status_t lenStatus;
    if (self->m_pktType == RAL_PKT_TYPE_GFSK) {
        ral_gfsk_pkt_params_t pkt = self->m_gfskPktParams;
        pkt.pld_len_in_bytes = static_cast<uint16_t>(self->m_txScratch.size);
        lenStatus = ral_set_gfsk_pkt_params(self->m_ral, &pkt);
    } else {
        ral_lora_pkt_params_t pkt = self->m_loraPktParams;
        pkt.pld_len_in_bytes = static_cast<uint8_t>(self->m_txScratch.size);
        lenStatus = ral_set_lora_pkt_params(self->m_ral, &pkt);
    }
    if (lenStatus != RAL_STATUS_OK) {
        self->m_txScratch.result = -EIO;
        smtc_rac_unlock_radio_access(self->m_radio_id);
        return;
    }

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
    RalSessionImpl* self = s_instance;  // registered in init(), never cleared

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
    // Drain any stale semaphore count from a previous un-awaited completion
    // (same discipline as stopRadio()).  Invariant: a stale count must never
    // let transmitPacket() return "success" while its own TX is still in
    // flight — that makes every subsequent startReceive() fail ALREADY_RUNNING.
    (void)k_sem_take(&m_txDoneSem, K_NO_WAIT);

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
    if (ret != 0) {
        // TX_DONE never arrived (chip hung in TX — see quiesceRadio()).  The
        // LOCK task is still RUNNING; without recovery it stays RUNNING
        // forever and every subsequent smtc_rac_lock_radio_access on this
        // hook fails (ALREADY_RUNNING) — the permanent TX-dead latch of
        // HWIL anomaly B.  Abort the stuck task, wait for the engine to
        // free it, then force the chip back to standby.
        (void)smtc_rac_unlock_radio_access(m_radio_id);
        radio_planner_t* rp = smtc_rac_get_rp();
        int32_t waited_ms = 0;
        while ((rp->tasks[m_radio_id].state != RP_TASK_STATE_FINISHED) &&
               (waited_ms < ENQUEUE_READY_DEADLINE_MS)) {
            k_sleep(K_MSEC(1));
            ++waited_ms;
        }
        this->quiesceRadio();
        Fw::Logger::log("[UspRadio] transmitPacket: TX_DONE timeout - task aborted, radio quiesced (hook state=%d)\n",
                        static_cast<int>(rp->tasks[m_radio_id].state));
        return -ETIMEDOUT;
    }
    return m_txScratch.result;
}

// ---------------------------------------------------------------------------
// startCw()
// ---------------------------------------------------------------------------

// CW requires: pkt_type + freq + power configured FIRST, then ral_set_tx_cw().
// (REPORT-ral-architecture gotcha #4)

void RalSessionImpl::onPreCw(void) {
    RalSessionImpl* self = s_instance;  // registered in init(), never cleared

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
    RalSessionImpl* self = s_instance;  // registered in init(), never cleared
    // Signal stop waiter regardless of status (abort or unlock).
    k_sem_give(&self->m_stopDoneSem);
}

int RalSessionImpl::startCw() {
    // Drain any stale count from a previous timed-out CW start (same
    // discipline as stopRadio()).
    (void)k_sem_take(&m_cwDoneSem, K_NO_WAIT);

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
// Completion detection: the hook's task state (RP_TASK_STATE_FINISHED) tells
// us whether a post-callback is coming; the post-callback itself is the
// authoritative completion signal for an aborted task.  Do not treat an
// observed FINISHED as completion while a callback is pending —
// rp_task_call_aborted() frees the task BEFORE running the callback, and
// enqueuing a new lock in that window races the radio planner.
// ---------------------------------------------------------------------------

namespace {
constexpr int32_t STOP_DEADLINE_MS = 1000;  //!< generous bound on USP-thread abort processing
}  // namespace

// ---------------------------------------------------------------------------
// quiesceRadio() — enforce the "radio stopped" postcondition at chip level.
//
// The radio planner's abort/unlock processing frees the RP task WITHOUT
// bringing the transceiver out of an active mode: the legacy engine path
// issues SetSleep directly from a running RX, and the lazy-sleep path leaves
// the chip running in RX outright.  Commanding an SX126x out of an ACTIVE
// GFSK continuous RX with anything but SetStandby first is out of spec, and
// HWIL capture 2026-07-24 (anomaly B) shows the failure mode: the first
// SetTx after a profile switch — issued while the chip was still in GFSK RX
// (GetStatus chip_mode=RX immediately before SetTx) — hangs the chip in TX
// forever (chip_mode=TX at +1 s, IRQ status 0, device error XOSC_START_ERR,
// TX_DONE never fires).  LoRa RX tolerates the same sequence, which is why
// P0 never wedged while P4/P5 (GFSK/GMSK) did.
//
// Fix: stopRadio() guarantees the chip is in STDBY_XOSC with IRQs cleared
// before returning, so every follow-on config/SetTx/SetRx starts from
// standby, per datasheet.  STDBY_XOSC (not RC) keeps the TCXO running so
// the saturation path pays no oscillator restart.  Cost: two short SPI
// commands per stop (~0.1 ms at 4 MHz) — noise against the 27+ ms frame
// airtime.
//
// Concurrency: called with the hook FINISHED and the engine idle for this
// hook.  A concurrent lazy-sleep expiry on the USP thread could interleave
// its SetSleep with this SetStandby; both orders leave the chip in a state
// the next task launch handles (wake from sleep, or standby), and the SPI
// bus itself is serialized by the Zephyr driver.
// ---------------------------------------------------------------------------
void RalSessionImpl::quiesceRadio() {
    (void)ral_set_standby(m_ral, RAL_STANDBY_CFG_XOSC);
    (void)ral_clear_irq_status(m_ral, RAL_IRQ_ALL);
}

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

    // If the hook was already FINISHED before the abort, rp_task_abort() was
    // a no-op and no post-callback is coming — nothing to wait for. (Snapshot
    // taken after the abort call: the abort itself never moves a hook TO
    // FINISHED synchronously, so FINISHED here still means "was idle".)
    radio_planner_t* rp = smtc_rac_get_rp();
    if (rp->tasks[m_radio_id].state == RP_TASK_STATE_FINISHED) {
        this->quiesceRadio();
        return 0;
    }

    // A task was scheduled or running: wait for its post-callback
    // (m_stopDoneSem). The callback is the authoritative completion signal —
    // do NOT return early on observing state == FINISHED, because
    // rp_task_call_aborted() frees the task (state -> FINISHED) immediately
    // BEFORE running the callback; enqueuing a new lock in that window races
    // the still-executing abort callback inside the radio planner. Only at
    // the deadline is the state re-checked, as a fallback for the rare
    // task-finished-between-snapshot-and-abort case (by then any callback
    // has long since run).
    if (k_sem_take(&m_stopDoneSem, K_MSEC(STOP_DEADLINE_MS)) == 0) {
        this->quiesceRadio();
        return 0;
    }
    if (rp->tasks[m_radio_id].state == RP_TASK_STATE_FINISHED) {
        this->quiesceRadio();
        return 0;
    }

    Fw::Logger::log(
        "[UspRadio] stopRadio: hook not FINISHED %" PRId32 " ms after abort on radio_id=%" PRIu8
        " (state=%d) - RAC hook likely wedged RUNNING\n",
        STOP_DEADLINE_MS, m_radio_id, static_cast<int>(rp->tasks[m_radio_id].state));
    return -ETIMEDOUT;
}

}  // namespace Zephyr

#endif  // CONFIG_LORA_BASICS_MODEM_DRIVERS
