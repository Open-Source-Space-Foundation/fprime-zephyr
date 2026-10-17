// Option 2 session (CONFIG_USP_MAIN_THREAD=n), completed from the sketch.
// Every method runs on the component thread, and so does the engine.  The
// only other contexts are the DIO1 "workq" and the RAC timer, which set
// planner flags and give the HAL event semaphore; the kick pump turns that
// into one coalesced engineKick message.
//
// applyLoRa/applyGfsk are the bodies of RalSessionImpl.cpp:200-292 (#26).
// harvestRx is RalSessionImpl.cpp:337-363.  programTx is :437-472.
#include "RalSessionSt.hpp"
#include "../sim_chip.h"

extern "C" {
#include <ral_defs.h>
#include <radio_planner.h>
struct k_sem* smtc_modem_hal_get_event_sem(void);
}
#include <cerrno>
#include <cstdio>

extern std::atomic<uint64_t> g_opt2_passes;
extern std::atomic<uint64_t> g_opt2_bounded_exits;

namespace Zephyr {

RalSessionSt* RalSessionSt::s_instance = nullptr;

namespace {
constexpr std::size_t MAX_RX_BUF = 255;
K_THREAD_STACK_DEFINE(s_pumpStack, 512);

void kickPump(void* self, void*, void*) {
    sim_thread_name("kickPump");
    auto* s = static_cast<RalSessionSt*>(self);
    struct k_sem* ev = smtc_modem_hal_get_event_sem();
    for (;;) {
        (void)k_sem_take(ev, K_FOREVER);
        if (!s->m_kickQueued.exchange(true)) {
            s->m_sink->postEngineKick();
        }
    }
}
}  // namespace

RalSessionSt::RalSessionSt(uint32_t freq_hz, int8_t tx_power_dbm) : m_freq_hz(freq_hz), m_tx_power_dbm(tx_power_dbm) {}

// SKETCH: `rp_disable_failsafe(smtc_rac_get_rp())` has the wrong arity; the
// real signature is (rp, bool).  Kept with `true` as the sketch intends (#586).
int RalSessionSt::init(RadioSink& sink) {
    m_sink = &sink;
    smtc_rac_init();
    rp_disable_failsafe(smtc_rac_get_rp(), true);
    const uint8_t id = smtc_rac_open_radio(RAC_HIGH_PRIORITY);
    if (id == RAC_INVALID_RADIO_ID) {
        return -ENODEV;
    }
    m_ral = ral_from_ralf(smtc_rac_get_radio());
    ral_reset(m_ral);
    ral_init(m_ral);
    s_instance = this;
    m_radio = RadioId{id};
    k_thread_create(&m_pump, s_pumpStack, K_THREAD_STACK_SIZEOF(s_pumpStack), kickPump, this, nullptr, nullptr,
                    KICK_PUMP_PRIO, 0, K_NO_WAIT);
    return 0;
}

// Bounded: on usp v1.2.1 rp_get_irq_flag reports pending for as long as a
// LOCK is held (#587), so an unbounded loop would spin through continuous RX.
void RalSessionSt::pump() {
    for (int i = 0; i < ENGINE_PASSES_MAX; ++i) {
        smtc_rac_run_engine();
        m_passes++;
        g_opt2_passes++;
        if (!smtc_rac_is_irq_flag_pending() || (m_state == State::RX)) {
            return;
        }
    }
    m_boundedExits++;
    g_opt2_bounded_exits++;
}

void RalSessionSt::onEngineKick() {
    m_kickQueued.store(false);
    pump();
}

int RalSessionSt::stop() {
    if (!ready()) {
        return -ENODEV;
    }
    if (m_state == State::IDLE) {
        quiesce();
        return 0;
    }
    (void)smtc_rac_unlock_radio_access(m_radio.id);
    pump();  // abort -> onPost(ABORTED/UNLOCKED) -> m_state = IDLE
    if (m_state != State::IDLE) {
        return -EIO;
    }
    quiesce();
    return 0;
}

int RalSessionSt::armRx(const LinkProfile& profile) {
    if (stop() != 0) return -EIO;
    m_profile = &profile;
    return lock(State::RX, onPreRx);
}

int RalSessionSt::startTx(const LinkProfile& profile, const uint8_t* data, std::size_t size, uint64_t nowMs) {
    if (stop() != 0) return -EIO;
    m_profile = &profile;
    m_tx = {data, size};
    m_txTimedOut = false;
    m_txDeadlineMs = nowMs + TX_DONE_DEADLINE_MS;
    return lock(State::TX, onPreTx);
}

int RalSessionSt::startCw() {
    if (stop() != 0) return -EIO;
    return lock(State::CW, onPreCw);
}

bool RalSessionSt::applyInWindow() {
    const LinkProfile& p = *m_profile;
    const int rc = (p.mod == ModKind::LORA) ? applyLoRa(p.lora) : applyGfsk(p.gfsk);
    if (rc != 0) {
        m_lastError = rc;
        smtc_rac_unlock_radio_access(m_radio.id);
        return false;
    }
    return true;
}

int RalSessionSt::lock(State next, void (*pre)(void)) {
    m_lastError = 0;
    smtc_rac_scheduler_config_t cfg = {};
    cfg.start_time_ms = k_uptime_get_32();
    cfg.scheduling = SMTC_RAC_ASAP_TRANSACTION;
    cfg.callback_pre_radio_transaction = pre;
    cfg.callback_post_radio_transaction = onPost;
    // SKETCH sets m_state after the lock call; the pre-callback runs INSIDE
    // smtc_rac_lock_radio_access (rp_task_enqueue -> arbiter -> launch), and a
    // failing applyInWindow() unlocks from there, so the state must be set
    // before the call or the abort's onPost sees IDLE and the hook is lost.
    m_state = next;
    if (smtc_rac_lock_radio_access(m_radio.id, cfg) != SMTC_RAC_SUCCESS) {
        m_state = State::IDLE;
        return -EIO;
    }
    pump();  // the launch happened inline; this drains any flag it raised
    return (m_lastError == 0) ? 0 : m_lastError;
}

void RalSessionSt::quiesce() {
    (void)ral_set_standby(m_ral, RAL_STANDBY_CFG_XOSC);
    (void)ral_clear_irq_status(m_ral, RAL_IRQ_ALL);
}

void RalSessionSt::restorePktParams() {
    if (m_pktType == RAL_PKT_TYPE_GFSK) {
        ral_set_gfsk_pkt_params(m_ral, &m_gfskPktParams);
    } else {
        ral_set_lora_pkt_params(m_ral, &m_loraPktParams);
    }
}

void RalSessionSt::programTx() {
    ral_status_t st;
    if (m_pktType == RAL_PKT_TYPE_GFSK) {
        ral_gfsk_pkt_params_t pkt = m_gfskPktParams;
        pkt.pld_len_in_bytes = static_cast<uint16_t>(m_tx.size);
        st = ral_set_gfsk_pkt_params(m_ral, &pkt);
    } else {
        ral_lora_pkt_params_t pkt = m_loraPktParams;
        pkt.pld_len_in_bytes = static_cast<uint8_t>(m_tx.size);
        st = ral_set_lora_pkt_params(m_ral, &pkt);
    }
    if (st != RAL_STATUS_OK || ral_set_pkt_payload(m_ral, m_tx.data, static_cast<uint16_t>(m_tx.size)) != RAL_STATUS_OK) {
        m_lastError = -EIO;
        smtc_rac_unlock_radio_access(m_radio.id);
        return;
    }
    ral_set_dio_irq_params(m_ral, RAL_IRQ_TX_DONE);
    if (ral_set_tx(m_ral) != RAL_STATUS_OK) {
        m_lastError = -EIO;
        smtc_rac_unlock_radio_access(m_radio.id);
    }
}

void RalSessionSt::harvestRx() {
    uint8_t buf[MAX_RX_BUF];
    uint16_t len = 0;
    ral_handle_rx_done(m_ral);
    ral_get_pkt_payload(m_ral, MAX_RX_BUF, buf, &len);
    ral_lora_rx_pkt_status_t lora_status = {};
    ral_gfsk_rx_pkt_status_t gfsk_status = {};
    int16_t rssi = 0;
    int8_t snr = 0;
    if (ral_get_lora_rx_pkt_status(m_ral, &lora_status) == RAL_STATUS_OK) {
        rssi = static_cast<int16_t>(lora_status.rssi_pkt_in_dbm);
        snr = static_cast<int8_t>(lora_status.snr_pkt_in_db);
    } else if (ral_get_gfsk_rx_pkt_status(m_ral, &gfsk_status) == RAL_STATUS_OK) {
        rssi = static_cast<int16_t>(gfsk_status.rssi_avg_in_dbm);
    }
    m_sink->rxFrame(buf, len, rssi, snr);
}

// ---------------------------------------------------------------------------
// RAC callbacks: all on the component thread.
// ---------------------------------------------------------------------------

void RalSessionSt::onPreRx(void) {
    RalSessionSt* s = s_instance;
    if (!s->applyInWindow()) return;
    s->restorePktParams();
    ral_set_dio_irq_params(s->m_ral, RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_CRC_ERROR);
    ral_cfg_rx_boosted(s->m_ral, true);
    ral_set_rx(s->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
}

void RalSessionSt::onPreTx(void) {
    RalSessionSt* s = s_instance;
    if (!s->applyInWindow()) return;
    s->programTx();
}

void RalSessionSt::onPreCw(void) {
    RalSessionSt* s = s_instance;
    ral_set_pkt_type(s->m_ral, RAL_PKT_TYPE_LORA);
    ral_set_rf_freq(s->m_ral, s->m_freq_hz);
    ral_set_tx_cfg(s->m_ral, s->m_tx_power_dbm, 0);
    ral_set_tx_cw(s->m_ral);
}

void RalSessionSt::onPost(rp_status_t status) {
    RalSessionSt* s = s_instance;
    if (status == RP_STATUS_RADIO_LOCKED) {
        ral_irq_t irq = RAL_IRQ_NONE;
        ral_get_and_clear_irq_status(s->m_ral, &irq);
        if (s->m_state == State::RX) {
            if ((irq & RAL_IRQ_RX_DONE) && !(irq & RAL_IRQ_RX_CRC_ERROR)) {
                s->harvestRx();
            }
            ral_set_rx(s->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
            return;
        }
        if (s->m_state == State::TX) {
            if (irq & RAL_IRQ_TX_DONE) {
                ral_handle_tx_done(s->m_ral);
            }
            smtc_rac_unlock_radio_access(s->m_radio.id);  // -> onPost(UNLOCKED) on a later pass
            return;
        }
        return;
    }
    const State was = s->m_state;
    s->m_state = State::IDLE;
    if (was == State::TX) {
        const int rc = s->m_txTimedOut ? -ETIMEDOUT
                     : (status == RP_STATUS_RADIO_UNLOCKED) ? s->m_lastError
                     : -ECANCELED;
        s->m_sink->txDone(rc);
    }
}

void RalSessionSt::checkTxDeadline(uint64_t nowMs) {
    if ((m_state == State::TX) && (nowMs >= m_txDeadlineMs)) {
        m_txTimedOut = true;
        (void)stop();
    }
}

// ---------------------------------------------------------------------------
// RalSessionImpl.cpp:200-292 verbatim (member names adapted)
// ---------------------------------------------------------------------------

int RalSessionSt::applyLoRa(const LoRaParams& p) {
    ral_set_pkt_type(m_ral, RAL_PKT_TYPE_LORA);
    ral_lora_mod_params_t mod = {};
    mod.sf = static_cast<ral_lora_sf_t>(p.sf);
    mod.bw = RAL_LORA_BW_125_KHZ;
    mod.cr = static_cast<ral_lora_cr_t>(p.cr - 4);
    mod.ldro = p.ldro ? 1 : 0;
    if (ral_set_lora_mod_params(m_ral, &mod) != RAL_STATUS_OK) return -EIO;
    ral_lora_pkt_params_t pkt = {};
    pkt.preamble_len_in_symb = p.preamble_len;
    pkt.header_type = RAL_LORA_PKT_EXPLICIT;
    pkt.pld_len_in_bytes = 0xFF;
    pkt.crc_is_on = true;
    pkt.invert_iq_is_on = false;
    if (ral_set_lora_pkt_params(m_ral, &pkt) != RAL_STATUS_OK) return -EIO;
    m_pktType = RAL_PKT_TYPE_LORA;
    m_loraPktParams = pkt;
    if (ral_set_lora_sync_word(m_ral, static_cast<uint8_t>(p.sync_word)) != RAL_STATUS_OK) return -EIO;
    if (ral_set_rf_freq(m_ral, m_freq_hz) != RAL_STATUS_OK) return -EIO;
    if (ral_set_tx_cfg(m_ral, m_tx_power_dbm, 0) != RAL_STATUS_OK) return -EIO;
    return 0;
}

int RalSessionSt::applyGfsk(const GfskParams& p) {
    ral_set_pkt_type(m_ral, RAL_PKT_TYPE_GFSK);
    ral_gfsk_mod_params_t mod = {};
    mod.br_in_bps = p.bitrate_bps;
    mod.fdev_in_hz = p.fdev_hz;
    switch (p.pulse_shape) {
        case GfskPulseShape::NONE:  mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_OFF;   break;
        case GfskPulseShape::BT_03: mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_03; break;
        case GfskPulseShape::BT_05: mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_05; break;
        case GfskPulseShape::BT_07: mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_07; break;
        default:                    mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_05; break;
    }
    mod.bw_dsb_in_hz = p.bw_dsb_hz;
    if (ral_set_gfsk_mod_params(m_ral, &mod) != RAL_STATUS_OK) return -EIO;
    ral_gfsk_pkt_params_t pkt = {};
    pkt.preamble_len_in_bits = p.preamble_len_bits;
    pkt.preamble_detector = RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
    pkt.sync_word_len_in_bits = p.sync_word_len * 8;
    pkt.header_type = RAL_GFSK_PKT_VAR_LEN;
    pkt.pld_len_in_bytes = 0xFF;
    switch (p.crc_type) {
        case GfskCrcType::OFF:    pkt.crc_type = RAL_GFSK_CRC_OFF;     break;
        case GfskCrcType::BYTE_1: pkt.crc_type = RAL_GFSK_CRC_1_BYTE;  break;
        case GfskCrcType::BYTE_2: pkt.crc_type = RAL_GFSK_CRC_2_BYTES; break;
        default:                  pkt.crc_type = RAL_GFSK_CRC_2_BYTES; break;
    }
    pkt.dc_free = p.whitening ? RAL_GFSK_DC_FREE_WHITENING : RAL_GFSK_DC_FREE_OFF;
    if (ral_set_gfsk_pkt_params(m_ral, &pkt) != RAL_STATUS_OK) return -EIO;
    m_pktType = RAL_PKT_TYPE_GFSK;
    m_gfskPktParams = pkt;
    if (ral_set_gfsk_sync_word(m_ral, p.sync_word, p.sync_word_len) != RAL_STATUS_OK) return -EIO;
    if (pkt.crc_type != RAL_GFSK_CRC_OFF) {
        if (ral_set_gfsk_crc_params(m_ral, 0x1D0FU, 0x1021U) != RAL_STATUS_OK) return -EIO;
    }
    if (p.whitening) {
        if (ral_set_gfsk_whitening_seed(m_ral, 0x01FFU) != RAL_STATUS_OK) return -EIO;
    }
    if (ral_set_rf_freq(m_ral, m_freq_hz) != RAL_STATUS_OK) return -EIO;
    if (ral_set_tx_cfg(m_ral, m_tx_power_dbm, 0) != RAL_STATUS_OK) return -EIO;
    return 0;
}

}  // namespace Zephyr
