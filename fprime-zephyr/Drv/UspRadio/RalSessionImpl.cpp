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
extern "C" {
#include <smtc_ral/src/ral_defs.h>
}

namespace Zephyr {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

RalSessionImpl::RalSessionImpl(uint32_t freq_hz, int8_t tx_power_dbm)
    : m_freq_hz(freq_hz),
      m_tx_power_dbm(tx_power_dbm),
      m_ral(nullptr),
      m_ralf(nullptr),
      m_initialized(false),
      m_rxLen(0),
      m_rxRssi(0),
      m_rxSnr(0) {
    k_sem_init(&m_txDoneSem, 0, 1);
    k_sem_init(&m_cwDoneSem, 0, 1);
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
    // Wait for the USP thread to complete its boot sequence.
    int rc = zephyr_usp_initialization_wait();
    if (rc != 0) {
        Fw::Logger::log("[UspRadio] USP init wait failed: %d\n", rc);
        return rc;
    }

    // zephyr_usp_thread.c intentionally does NOT call smtc_rac_init().
    // We must call it here via the Zephyr wrapper (REPORT-ral-architecture §Init).
    zephyr_smtc_rac_init();

    // Obtain the ral_t* via the RALF abstraction that USP provides.
    // smtc_rac_get_radio() returns the ralf_t*; ral_from_ralf() gives ral_t*.
    m_ralf = smtc_rac_get_radio();
    if (m_ralf == nullptr) {
        Fw::Logger::log("[UspRadio] smtc_rac_get_radio() returned null\n");
        return -EIO;
    }
    m_ral = ral_from_ralf(m_ralf);

    // Hardware reset + init via RAL
    ral_reset(m_ral);
    ral_init(m_ral);

    m_initialized = true;
    Fw::Logger::log("[UspRadio] RAL session initialized (freq=%" PRIu32 " Hz, pwr=%" PRId8 " dBm)\n",
                    m_freq_hz, m_tx_power_dbm);
    return 0;
}

// ---------------------------------------------------------------------------
// applyProfile() — translates LinkProfile → RAL calls inside a lock window
// ---------------------------------------------------------------------------

namespace {
// Trampoline payload for applyProfile lock window
struct ApplyCtx {
    RalSessionImpl* self;
    const LinkProfile* profile;
    int result;
};

static void onPreApply(void* raw) {
    auto* ctx = static_cast<ApplyCtx*>(raw);
    if (ctx->self->applyLoRa_or_Gfsk(*ctx->profile)) {
        ctx->result = 0;
    } else {
        ctx->result = -EIO;
    }
    // Unlock immediately — we don't need to hold through a TX/RX
    smtc_rac_unlock_radio_access();
}
}  // namespace

// Helper called from within the RAC lock window
bool RalSessionImpl::applyLoRa_or_Gfsk(const LinkProfile& p) {
    if (p.mod == ModKind::LORA) {
        return applyLoRa(p.lora) == 0;
    } else {
        return applyGfsk(p.gfsk) == 0;
    }
}

int RalSessionImpl::applyProfile(const LinkProfile& profile) {
    ApplyCtx ctx{this, &profile, -ENODEV};

    smtc_rac_lock_radio_access_t lock_cfg = {
        .callback_pre_radio_transaction  = onPreApply,
        .callback_post_radio_transaction = nullptr,
        .context                         = &ctx,
        .transaction_type                = SMTC_RAC_ASAP_TRANSACTION,
    };
    int rc = smtc_rac_lock_radio_access(&lock_cfg);
    if (rc != 0) {
        return rc;
    }
    return ctx.result;
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

    // 4. Sync word
    if (ral_set_lora_sync_word(m_ral, p.sync_word) != RAL_STATUS_OK) return -EIO;

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

    // 5. Frequency + PA config
    if (ral_set_rf_freq(m_ral, m_freq_hz) != RAL_STATUS_OK) return -EIO;
    if (ral_set_tx_cfg(m_ral, m_tx_power_dbm, 0) != RAL_STATUS_OK) return -EIO;

    return 0;
}

// ---------------------------------------------------------------------------
// startReceive()
// ---------------------------------------------------------------------------

void RalSessionImpl::onPreRx(void* raw) {
    auto* ctx = static_cast<RxContext*>(raw);
    RalSessionImpl* self = ctx->self;

    ral_set_dio_irq_params(self->m_ral,
        RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_CRC_ERROR);
    ral_cfg_rx_boosted(self->m_ral, true);
    ral_set_rx(self->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
    // Lock stays held — the USP thread will service the DIO1 edge and call
    // the post-transaction callback when a frame arrives.
    // (We do NOT unlock here; continuous RX holds the RAC transaction open
    //  until stopRadio() is called or a frame arrives.)
}

void RalSessionImpl::onRxDone(void* raw) {
    auto* ctx = static_cast<RxContext*>(raw);
    RalSessionImpl* self = ctx->self;

    // Read IRQ status
    ral_irq_t irq = RAL_IRQ_NONE;
    ral_get_and_clear_irq_status(self->m_ral, &irq);

    if ((irq & RAL_IRQ_RX_DONE) && !(irq & RAL_IRQ_RX_CRC_ERROR)) {
        uint8_t buf[MAX_RX_BUF];
        uint16_t len = 0;

        ral_handle_rx_done(self->m_ral);
        ral_get_pkt_payload(self->m_ral, MAX_RX_BUF, buf, &len);

        // Get link quality stats
        ral_lora_rx_pkt_status_t lora_status = {};
        ral_gfsk_rx_pkt_status_t gfsk_status = {};
        int16_t rssi = 0;
        int8_t  snr  = 0;

        // Determine pkt type from current mod state (stored in self)
        // We try LoRa first; if that fails try GFSK
        if (ral_get_lora_rx_pkt_status(self->m_ral, &lora_status) == RAL_STATUS_OK) {
            rssi = static_cast<int16_t>(lora_status.rssi_pkt_in_dbm);
            snr  = static_cast<int8_t>(lora_status.snr_pkt_in_db);
        } else if (ral_get_gfsk_rx_pkt_status(self->m_ral, &gfsk_status) == RAL_STATUS_OK) {
            rssi = static_cast<int16_t>(gfsk_status.rssi_avg_in_dbm);
            snr  = 0;  // GFSK has no SNR
        }

        // Invoke the registered callback (fires on USP thread — must be quick)
        if (self->m_rxDone) {
            self->m_rxDone(buf, len, rssi, snr);
        }

        // Re-arm continuous RX
        ral_set_rx(self->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
    }
}

int RalSessionImpl::startReceive() {
    static RxContext ctx;  // static: lifetime >= the RAC transaction
    ctx.self = this;

    smtc_rac_lock_radio_access_t lock_cfg = {
        .callback_pre_radio_transaction  = onPreRx,
        .callback_post_radio_transaction = onRxDone,
        .context                         = &ctx,
        .transaction_type                = SMTC_RAC_ASAP_TRANSACTION,
    };
    return smtc_rac_lock_radio_access(&lock_cfg);
}

// ---------------------------------------------------------------------------
// transmitPacket()
// ---------------------------------------------------------------------------

void RalSessionImpl::onPreTx(void* raw) {
    auto* ctx = static_cast<TxContext*>(raw);
    RalSessionImpl* self = ctx->self;

    // Set payload
    if (ral_set_pkt_payload(self->m_ral, ctx->data,
                            static_cast<uint16_t>(ctx->size)) != RAL_STATUS_OK) {
        ctx->result = -EIO;
        smtc_rac_unlock_radio_access();
        return;
    }
    // IRQ: TX_DONE only
    ral_set_dio_irq_params(self->m_ral, RAL_IRQ_TX_DONE);
    // Start TX
    if (ral_set_tx(self->m_ral) != RAL_STATUS_OK) {
        ctx->result = -EIO;
        smtc_rac_unlock_radio_access();
        return;
    }
    ctx->result = 0;
    // Lock stays held until TX_DONE IRQ fires → post-callback runs
}

static void onPostTx(void* raw) {
    auto* ctx = static_cast<RalSessionImpl::TxContext*>(raw);
    RalSessionImpl* self = ctx->self;

    ral_irq_t irq = RAL_IRQ_NONE;
    ral_get_and_clear_irq_status(self->m_ral, &irq);
    if (irq & RAL_IRQ_TX_DONE) {
        ral_handle_tx_done(self->m_ral);
        if (self->m_txDone) self->m_txDone();
    }
    k_sem_give(&self->m_txDoneSem);
}

int RalSessionImpl::transmitPacket(const uint8_t* data, std::size_t size) {
    TxContext ctx{this, data, size, 0};

    smtc_rac_lock_radio_access_t lock_cfg = {
        .callback_pre_radio_transaction  = onPreTx,
        .callback_post_radio_transaction = onPostTx,
        .context                         = &ctx,
        .transaction_type                = SMTC_RAC_ASAP_TRANSACTION,
    };
    int rc = smtc_rac_lock_radio_access(&lock_cfg);
    if (rc != 0) return rc;

    // Block the component thread until TX_DONE IRQ (signalled via semaphore
    // in onPostTx).  Timeout: conservatively 10 s (longest LoRa symbol airtime).
    rc = k_sem_take(&m_txDoneSem, K_SECONDS(10));
    return (rc == 0) ? ctx.result : -ETIMEDOUT;
}

// ---------------------------------------------------------------------------
// startCw()
// ---------------------------------------------------------------------------

// CW requires: pkt_type + freq + power configured FIRST, then ral_set_tx_cw().
// (REPORT-ral-architecture gotcha #4)

void RalSessionImpl::onPreCw(void* raw) {
    auto* ctx = static_cast<CwContext*>(raw);
    RalSessionImpl* self = ctx->self;

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

int RalSessionImpl::startCw(const LinkProfile& /*cwProfile*/) {
    CwContext ctx{this};

    smtc_rac_lock_radio_access_t lock_cfg = {
        .callback_pre_radio_transaction  = onPreCw,
        .callback_post_radio_transaction = nullptr,
        .context                         = &ctx,
        .transaction_type                = SMTC_RAC_ASAP_TRANSACTION,
    };
    int rc = smtc_rac_lock_radio_access(&lock_cfg);
    if (rc != 0) return rc;
    // Wait until the CW pre-callback has run (and CW is actually transmitting)
    rc = k_sem_take(&m_cwDoneSem, K_SECONDS(2));
    return (rc == 0) ? 0 : -ETIMEDOUT;
}

// ---------------------------------------------------------------------------
// stopRadio()
// ---------------------------------------------------------------------------

namespace {
struct StopCtx {
    RalSessionImpl* self;
};

static void onPreStop(void* raw) {
    auto* ctx = static_cast<StopCtx*>(raw);
    ral_set_standby(ctx->self->m_ral, RAL_STANDBY_CFG_RC);
    smtc_rac_unlock_radio_access();
}
}  // namespace

int RalSessionImpl::stopRadio() {
    StopCtx ctx{this};
    smtc_rac_lock_radio_access_t lock_cfg = {
        .callback_pre_radio_transaction  = onPreStop,
        .callback_post_radio_transaction = nullptr,
        .context                         = &ctx,
        .transaction_type                = SMTC_RAC_ASAP_TRANSACTION,
    };
    return smtc_rac_lock_radio_access(&lock_cfg);
}

}  // namespace Zephyr

#endif  // CONFIG_LORA_BASICS_MODEM_DRIVERS
