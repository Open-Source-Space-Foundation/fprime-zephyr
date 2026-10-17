// PROTOTYPE #589 (throwaway, not built). Option 2: single-queue UspRadio.
//
// RalSession for CONFIG_USP_MAIN_THREAD=n.  Every method runs on UspRadio's
// component thread, and so does smtc_rac_run_engine(), so every RAC pre- and
// post-callback runs on that thread too.  One thread touches the RAC, the RAL
// and this object, so this file needs no semaphores, no mutex, no polling of
// rp->tasks[].state with k_sleep, and no cross-thread scratch.
//
// Only two other contexts exist, and neither touches radio state:
//   - DIO1 edge (system workq) -> rp_radio_irq_callback: sets planner flags,
//     gives the USP event semaphore (radio_planner.c:597-602).
//   - RAC timer (k_timer)       -> rp_timer_irq_callback: same.
// The kick pump turns that semaphore into one queued engineKick message.
//
// applyLoRa()/applyGfsk() are unchanged from RalSessionImpl.cpp:200-292 (93
// lines) and are not repeated here.

#include "RalSessionSt.hpp"

namespace Zephyr {

// ---------------------------------------------------------------------------
// Kick pump: the one extra thread.  Waits on the HAL's event semaphore and posts
// engineKick.  m_kickQueued coalesces kicks so a DIO1 storm can never fill the
// component queue (the handler clears it before running the engine, so an edge
// that lands during the pass still gets its own kick).
// ---------------------------------------------------------------------------

static void kickPump(void* self, void*, void*) {
    auto* s = static_cast<RalSessionSt*>(self);
    struct k_sem* ev = smtc_modem_hal_get_event_sem();
    while (true) {
        (void)k_sem_take(ev, K_FOREVER);
        if (!s->m_kickQueued.exchange(true)) {
            s->m_sink->postEngineKick();  // engineKick_internalInterfaceInvoke()
        }
    }
}
K_THREAD_STACK_DEFINE(s_pumpStack, 512);

// ---------------------------------------------------------------------------
// init: RadioId exists only after a successful open, so nothing can index
// rp->tasks[] with RAC_INVALID_RADIO_ID (0xFF).
// ---------------------------------------------------------------------------

int RalSessionSt::init(RadioSink& sink) {
    m_sink = &sink;
    SMTC_SW_PLATFORM_INIT();  // lorawan_smtc_modem_hal_init(transceiver)
    smtc_rac_init();
    rp_disable_failsafe(smtc_rac_get_rp());  // #586: replaces the usp fork
    const uint8_t id = smtc_rac_open_radio(RAC_HIGH_PRIORITY);
    if (id == RAC_INVALID_RADIO_ID) {
        return -ENODEV;
    }
    m_ral = ral_from_ralf(smtc_rac_get_radio());
    ral_reset(m_ral);
    ral_init(m_ral);
    s_instance = this;
    m_radio = RadioId{id};
    k_thread_create(&m_pump, s_pumpStack, K_THREAD_STACK_SIZEOF(s_pumpStack),
                    kickPump, this, nullptr, nullptr, KICK_PUMP_PRIO, 0, K_NO_WAIT);
    return 0;
}

// ---------------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------------

//! Run the engine until it has nothing pending.  Bounded: on usp v1.2.1
//! rp_get_irq_flag reports pending for as long as a LOCK is held (#587), so an
//! unbounded loop would spin through a continuous-RX window.
void RalSessionSt::pump() {
    for (int i = 0; i < ENGINE_PASSES_MAX; ++i) {
        smtc_rac_run_engine();
        if (!smtc_rac_is_irq_flag_pending() || (m_state == State::RX)) {
            return;
        }
    }
}

void RalSessionSt::onEngineKick() {
    m_kickQueued.store(false);
    pump();
}

// ---------------------------------------------------------------------------
// One entry per radio state.  Each leaves the chip in a known state before it
// returns: stop() is synchronous because the abort's post-callback runs inside
// the pump() below, on this thread.
// ---------------------------------------------------------------------------

int RalSessionSt::stop() {
    if (!ready()) {
        return -ENODEV;  // init failed: every op fails here, nothing reads tasks[0xFF]
    }
    if (m_state == State::IDLE) {
        quiesce();
        return 0;
    }
    (void)smtc_rac_unlock_radio_access(m_radio.id);
    pump();  // abort -> onPost(ABORTED) -> m_state = IDLE
    if (m_state != State::IDLE) {
        return -EIO;  // hook wedged: the post-callback never ran
    }
    quiesce();
    return 0;
}

//! Arm continuous RX on `profile`.  The profile is an argument, so no caller
//! can re-arm RX on whatever modulation the chip last had (#14 defect 2).
int RalSessionSt::armRx(const LinkProfile& profile) {
    if (stop() != 0) return -EIO;
    m_profile = &profile;  // applied inside onPreRx, in the same lock window
    return lock(State::RX, onPreRx);
}

//! Start TX on `profile`.  Returns once the chip is transmitting; completion
//! arrives later as RadioSink::txDone() from inside a pump().
int RalSessionSt::startTx(const LinkProfile& profile, const uint8_t* data, std::size_t size,
                          uint64_t nowMs) {
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

//! Apply m_profile.  Called only from a pre-callback, so profile and arm share
//! one lock window: the chip can never be armed on a profile other than the
//! one the caller passed.  #14 spent a separate RAC transaction plus a 2 s
//! semaphore wait on this.
bool RalSessionSt::applyInWindow() {
    const LinkProfile& p = *m_profile;
    const int rc = (p.mod == ModKind::LORA) ? applyLoRa(p.lora) : applyGfsk(p.gfsk);
    if (rc != 0) {
        m_lastError = rc;
        smtc_rac_unlock_radio_access(m_radio.id);  // -> onPost(UNLOCKED): state IDLE
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
    if (smtc_rac_lock_radio_access(m_radio.id, cfg) != SMTC_RAC_SUCCESS) {
        return -EIO;
    }
    m_state = next;
    pump();  // launch: the pre-callback runs here if it did not run inline
    return 0;
}

void RalSessionSt::quiesce() {
    // Same two SPI commands as RalSessionImpl::quiesceRadio, but no engine can
    // run concurrently: the engine runs only from pump(), on this thread.
    (void)ral_set_standby(m_ral, RAL_STANDBY_CFG_XOSC);
    (void)ral_clear_irq_status(m_ral, RAL_IRQ_ALL);
}

// ---------------------------------------------------------------------------
// RAC callbacks: all on the component thread, inside pump().
// ---------------------------------------------------------------------------

void RalSessionSt::onPreRx(void) {
    RalSessionSt* s = s_instance;
    if (!s->applyInWindow()) return;
    s->restorePktParams();  // RalSessionImpl.cpp:309-313, unchanged
    ral_set_dio_irq_params(s->m_ral, RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_CRC_ERROR);
    ral_cfg_rx_boosted(s->m_ral, true);
    ral_set_rx(s->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
}

void RalSessionSt::onPreTx(void) {
    RalSessionSt* s = s_instance;
    if (!s->applyInWindow()) return;
    // RalSessionImpl.cpp:441-472 unchanged (true length, payload, TX_DONE, SetTx);
    // on any failure: s->m_txResult = -EIO; smtc_rac_unlock_radio_access(id).
    s->programTx();
}

void RalSessionSt::onPreCw(void) {
    RalSessionSt* s = s_instance;
    ral_set_pkt_type(s->m_ral, RAL_PKT_TYPE_LORA);
    ral_set_rf_freq(s->m_ral, s->m_freq_hz);
    ral_set_tx_cfg(s->m_ral, s->m_tx_power_dbm, 0);
    ral_set_tx_cw(s->m_ral);
}

//! One post-callback for every state: the session's own m_state says what was
//! running, so RX/TX/CW need no separate trampolines.
void RalSessionSt::onPost(rp_status_t status) {
    RalSessionSt* s = s_instance;
    if (status == RP_STATUS_RADIO_LOCKED) {
        ral_irq_t irq = RAL_IRQ_NONE;
        ral_get_and_clear_irq_status(s->m_ral, &irq);
        if (s->m_state == State::RX) {
            if ((irq & RAL_IRQ_RX_DONE) && !(irq & RAL_IRQ_RX_CRC_ERROR)) {
                s->harvestRx();  // RalSessionImpl.cpp:341-358 -> m_sink->rxFrame(...)
            }
            ral_set_rx(s->m_ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
            return;
        }
        if (s->m_state == State::TX) {
            if (irq & RAL_IRQ_TX_DONE) {
                ral_handle_tx_done(s->m_ral);
            }
            smtc_rac_unlock_radio_access(s->m_radio.id);  // -> onPost(UNLOCKED), next pass
            return;
        }
        return;
    }
    // UNLOCKED or ABORTED: the hook is free.
    const State was = s->m_state;
    s->m_state = State::IDLE;
    if (was == State::TX) {
        const int rc = s->m_txTimedOut ? -ETIMEDOUT
                     : (status == RP_STATUS_RADIO_UNLOCKED) ? s->m_lastError
                     : -ECANCELED;
        s->m_sink->txDone(rc);
    }
}

//! TX_DONE watchdog, polled from UspRadio::run (1 Hz): a chip hung in TX
//! (HWIL anomaly B) never fires TX_DONE.  Replaces the 10 s k_sem_take.
void RalSessionSt::checkTxDeadline(uint64_t nowMs) {
    if ((m_state == State::TX) && (nowMs >= m_txDeadlineMs)) {
        m_txTimedOut = true;
        (void)stop();  // abort -> onPost(ABORTED) -> txDone(-ETIMEDOUT); then quiesce
    }
}

}  // namespace Zephyr
