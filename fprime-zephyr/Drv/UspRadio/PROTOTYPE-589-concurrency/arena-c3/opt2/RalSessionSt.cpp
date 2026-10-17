// OPTION 2 (single queue): RalSessionSt completed from PROTOTYPE-589-concurrency/single-queue/RalSessionSt.cpp.
// Everything in the sketch is kept; additions needed to compile and run are marked "ARENA-c3:".
#include "RalSessionSt.hpp"
#include <Fw/Logger/Logger.hpp>
#include <cstring>

namespace Zephyr {

RalSessionSt* RalSessionSt::s_instance = nullptr;
namespace { constexpr std::size_t MAX_RX_BUF = 255; }

RalSessionSt::RalSessionSt(uint32_t freq_hz, int8_t tx_power_dbm)
    : m_freq_hz(freq_hz), m_tx_power_dbm(tx_power_dbm), m_pktType(RAL_PKT_TYPE_LORA) {
    m_loraPktParams = {};
    m_gfskPktParams = {};
}

void kickPump(void* selfp, void*, void*) {
    auto* s = static_cast<RalSessionSt*>(selfp);
    struct k_sem* ev = smtc_modem_hal_get_event_sem();
    while (true) {
        (void)k_sem_take(ev, K_FOREVER);
        if (!s->m_kickQueued.exchange(true)) {
            s->m_kicks++;
            s->m_sink->postEngineKick();
        } else {
            s->m_kicksCoalesced++;
        }
    }
}

int RalSessionSt::init(RadioSink& sink) {
    m_sink = &sink;
    lorawan_smtc_modem_hal_init(DEVICE_DT_GET(DT_CHOSEN(zephyr_lorawan_transceiver)));  // SMTC_SW_PLATFORM_INIT() with USP_MAIN_THREAD=n
    smtc_rac_init();
    rp_disable_failsafe(smtc_rac_get_rp(), true);  // ARENA-c3: sketch called it with one argument; signature is (rp, bool)
    const uint8_t id = smtc_rac_open_radio(RAC_HIGH_PRIORITY);
    if (id == RAC_INVALID_RADIO_ID) {
        return -ENODEV;
    }
    m_ral = ral_from_ralf(smtc_rac_get_radio());
    ral_reset(m_ral);
    ral_init(m_ral);
    s_instance = this;
    m_radio = RadioId{id};
    k_thread_create(&m_pump, nullptr, 0, kickPump, this, nullptr, nullptr, KICK_PUMP_PRIO, 0, K_NO_WAIT);
    return 0;
}

void RalSessionSt::pump() {
    for (int i = 0; i < ENGINE_PASSES_MAX; ++i) {
        const int64_t t0 = zs_now_us();
        smtc_rac_run_engine();
        const int64_t d = zs_now_us() - t0;
        m_enginePasses++;
        if (d > m_maxPassUs) m_maxPassUs = d;
        if (!smtc_rac_is_irq_flag_pending() || (m_state == State::RX)) {
            return;
        }
    }
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
    pump();
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
    m_txResult = 0;
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
    // ARENA-c3: the sketch set m_state = next AFTER smtc_rac_lock_radio_access returned.  The planner may run the
    // pre-callback (and a failing pre-callback's unlock -> onPost(UNLOCKED) -> IDLE) inside that call, so setting
    // the state afterwards would leave m_state = RX/TX over an idle hook.  Set it first, undo on enqueue failure.
    m_state = next;
    if (smtc_rac_lock_radio_access(m_radio.id, cfg) != SMTC_RAC_SUCCESS) {
        m_state = State::IDLE;
        return -EIO;
    }
    pump();
    return 0;
}

void RalSessionSt::quiesce() {
    (void)ral_set_standby(m_ral, RAL_STANDBY_CFG_XOSC);
    (void)ral_clear_irq_status(m_ral, RAL_IRQ_ALL);
}

// ARENA-c3: sketch comments "RalSessionImpl.cpp:309-313, unchanged"
void RalSessionSt::restorePktParams() {
    if (m_pktType == RAL_PKT_TYPE_GFSK) {
        ral_set_gfsk_pkt_params(m_ral, &m_gfskPktParams);
    } else {
        ral_set_lora_pkt_params(m_ral, &m_loraPktParams);
    }
}

void RalSessionSt::onPreRx(void) {
    RalSessionSt* s = s_instance;
    if (!s->applyInWindow()) return;
    s->restorePktParams();
    ral_set_dio_irq_params(s->m_ral, RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_CRC_ERROR);
    ral_cfg_rx_boosted(s->m_ral, true);
    ral_set_rx(s->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
}

// ARENA-c3: "RalSessionImpl.cpp:441-472 unchanged (true length, payload, TX_DONE, SetTx)"
void RalSessionSt::programTx() {
    ral_status_t lenStatus;
    if (m_pktType == RAL_PKT_TYPE_GFSK) {
        ral_gfsk_pkt_params_t pkt = m_gfskPktParams;
        pkt.pld_len_in_bytes = static_cast<uint16_t>(m_tx.size);
        lenStatus = ral_set_gfsk_pkt_params(m_ral, &pkt);
    } else {
        ral_lora_pkt_params_t pkt = m_loraPktParams;
        pkt.pld_len_in_bytes = static_cast<uint8_t>(m_tx.size);
        lenStatus = ral_set_lora_pkt_params(m_ral, &pkt);
    }
    if (lenStatus != RAL_STATUS_OK ||
        ral_set_pkt_payload(m_ral, m_tx.data, static_cast<uint16_t>(m_tx.size)) != RAL_STATUS_OK) {
        m_txResult = -EIO;
        smtc_rac_unlock_radio_access(m_radio.id);
        return;
    }
    ral_set_dio_irq_params(m_ral, RAL_IRQ_TX_DONE);
    if (ral_set_tx(m_ral) != RAL_STATUS_OK) {
        m_txResult = -EIO;
        smtc_rac_unlock_radio_access(m_radio.id);
    }
}

void RalSessionSt::onPreTx(void) {
    RalSessionSt* s = s_instance;
    if (!s->applyInWindow()) { s->m_txResult = s->m_lastError; return; }
    s->programTx();
}

void RalSessionSt::onPreCw(void) {
    RalSessionSt* s = s_instance;
    ral_set_pkt_type(s->m_ral, RAL_PKT_TYPE_LORA);
    ral_set_rf_freq(s->m_ral, s->m_freq_hz);
    ral_set_tx_cfg(s->m_ral, s->m_tx_power_dbm, 0);
    ral_set_tx_cw(s->m_ral);
}

// ARENA-c3: "RalSessionImpl.cpp:341-358 -> m_sink->rxFrame(...)"
void RalSessionSt::harvestRx() {
    uint8_t  buf[MAX_RX_BUF];
    uint16_t len = 0;
    ral_handle_rx_done(m_ral);
    ral_get_pkt_payload(m_ral, MAX_RX_BUF, buf, &len);
    ral_lora_rx_pkt_status_t lora_status = {};
    ral_gfsk_rx_pkt_status_t gfsk_status = {};
    int16_t rssi = 0;
    int8_t  snr  = 0;
    if (ral_get_lora_rx_pkt_status(m_ral, &lora_status) == RAL_STATUS_OK) {
        rssi = static_cast<int16_t>(lora_status.rssi_pkt_in_dbm);
        snr  = static_cast<int8_t>(lora_status.snr_pkt_in_db);
    } else if (ral_get_gfsk_rx_pkt_status(m_ral, &gfsk_status) == RAL_STATUS_OK) {
        rssi = static_cast<int16_t>(gfsk_status.rssi_avg_in_dbm);
    }
    m_sink->rxFrame(buf, len, rssi, snr);
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
            smtc_rac_unlock_radio_access(s->m_radio.id);
            return;
        }
        return;
    }
    const State was = s->m_state;
    s->m_state = State::IDLE;
    if (was == State::TX) {
        const int rc = s->m_txTimedOut ? -ETIMEDOUT
                     : (status == RP_STATUS_RADIO_UNLOCKED) ? s->m_txResult
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

#include "apply_params.inc"

}  // namespace Zephyr
