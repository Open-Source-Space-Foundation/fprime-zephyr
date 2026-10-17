// Option 1 (#14 + #26) and option 3 (same component code; RalSessionImpl
// built with the upstream-mutex diff) component model.
//
// The handler bodies are lifted from fprime-zephyr/Drv/UspRadio/UspRadio.cpp
// @ 25d5cd2 with the F´ autocoded surface replaced:
//   log_*/tlmWrite_*          -> counters
//   *_internalInterfaceInvoke -> MsgQueue::send(priority 10)
//   comStatusOut_out          -> ComQueue stand-in thread (delivers next frame to dataIn)
//   dataOut_out / allocate    -> counters + stale-confirm check
// RadioHead compat is off (raw frames) to keep the payload path short; it does
// not touch the concurrency structure.  The RalSession is the REAL
// RalSessionImpl.cpp, the policy is the REAL ProfilePolicy.cpp.
#include "../component.hpp"
#include "../sim_chip.h"
#include "../usp_thread.h"

#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"
#include "fprime-zephyr/Drv/UspRadio/ProfilePolicy.hpp"
#include "fprime-zephyr/Drv/UspRadio/RalSessionImpl.hpp"
#include "fprime-zephyr/Drv/UspRadio/TxOutcomePolicy.hpp"

#include <zephyr/kernel.h>
#include <cstdio>

using namespace Zephyr;

namespace {

enum class TxState { DISABLED, ENABLED, DISABLING };

class UspRadioSim1 final : public RadioSim {
  public:
    UspRadioSim1(const char* name, uint32_t freq, int8_t pwr)
        : m_name(name), m_sessionImpl(freq, pwr), m_session(&m_sessionImpl), m_queue(20) {
        m_rxHead = 0;
        m_rxTail = 0;
        m_thread = std::thread([this] { componentMain(); });
        m_comq = std::thread([this] { comQueueMain(); });
    }
    ~UspRadioSim1() override = default;

    const char* name() const override { return m_name; }
    const SimStats& stats() const override { return m_stats; }
    uint8_t policyRxProfile() const override { return m_policy.rxProfile(); }
    bool policyHasPending() const override { return m_policy.hasPendingRx(); }
    uint64_t queueOverflow() const override { return m_queue.overflow(); }

    // ------------------------------------------------------------------
    // startRadio()  (UspRadio.cpp:55-104)
    // ------------------------------------------------------------------
    bool startRadio(bool txEnabled) override {
        m_transmitState = txEnabled ? TxState::ENABLED : TxState::DISABLED;
        m_session->setCallbacks([this](const uint8_t* d, std::size_t n, int16_t rssi, int8_t snr) {
            this->onRxDone(d, n, rssi, snr);
        });
        usp_thread_start();
        if (m_session->init() != 0) {
            std::fprintf(stderr, "[%s] session.init() failed\n", m_name);
            return false;
        }
        if (!applyProfile(BOOT_DEFAULT_PROFILE, true)) return false;
        if (!applyProfile(BOOT_DEFAULT_PROFILE, true)) return false;
        if (m_session->startReceive() != 0) {
            m_stats.cfg_failed++;
            return false;
        }
        if (txEnabled && !m_comStatusOwed.load(std::memory_order_relaxed)) {
            comStatusOut(true);
            m_comStatusOwed.store(true, std::memory_order_relaxed);
        }
        return true;
    }

    // ------------------------------------------------------------------
    // Command / port surface: enqueue only (UspRadio.cpp:603-622, 180-193, 142)
    // ------------------------------------------------------------------
    void cmdSetRxProfile(uint8_t idx, uint16_t revert_s) override {
        m_queue.send(10, [this, idx, revert_s] { deferredSetRxProfile(idx, revert_s); });
    }
    void cmdSetTxProfile(uint8_t idx) override {
        m_queue.send(10, [this, idx] { deferredSetTxProfile(idx); });
    }
    void cmdTransmit(bool enabled) override {
        m_queue.send(10, [this, enabled] { deferredTransmitCmd(enabled); });
    }
    void cmdContinuousWave(uint16_t seconds) override {
        m_queue.send(10, [this, seconds] { deferredContinuousWave(seconds); });
    }
    void portRun() override {
        m_queue.send(0, [this] { run_handler(); }, "run");
    }
    void comQueuePush(const std::vector<uint8_t>& frame) override {
        std::lock_guard<std::mutex> lk(m_comqM);
        m_comqFrames.push_back(frame);
        m_comqCv.notify_one();
    }
    size_t comQueueDepth() override {
        std::lock_guard<std::mutex> lk(m_comqM);
        return m_comqFrames.size();
    }
    void drain() override {
        for (;;) {
            if (m_queue.size() == 0 && !m_busy.load()) {
                k_sleep(K_MSEC(2));
                if (m_queue.size() == 0 && !m_busy.load()) return;
            }
            k_sleep(K_MSEC(1));
        }
    }

  private:
    // ------------------------------------------------------------------
    // onRxDone() -- USP thread  (UspRadio.cpp:110-136)
    // ------------------------------------------------------------------
    static constexpr uint32_t RX_RING_DEPTH = 4;
    struct RxSlot {
        uint8_t data[252];
        size_t len;
        int16_t rssi;
        int8_t snr;
        uint8_t profileTag;  // sim-only: which profile the chip received this on
    };
    void onRxDone(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr) {
        const uint32_t head = m_rxHead.load(std::memory_order_relaxed);
        const uint32_t tail = m_rxTail.load(std::memory_order_acquire);
        if ((head - tail) >= RX_RING_DEPTH) {
            m_stats.rx_dropped_ring++;
            return;
        }
        RxSlot& slot = m_rxRing[head % RX_RING_DEPTH];
        const size_t safe = size > sizeof(slot.data) ? sizeof(slot.data) : size;
        std::memcpy(slot.data, data, safe);
        slot.len = safe;
        slot.rssi = rssi;
        slot.snr = snr;
        slot.profileTag = sim_chip_rx_frame_profile();
        m_rxHead.store(head + 1, std::memory_order_release);
        m_queue.send(10, [this] { deferredRxDone(); }, "rxDone");
    }

    // ------------------------------------------------------------------
    // run_handler  (UspRadio.cpp:142-174)
    // ------------------------------------------------------------------
    void run_handler() {
        ProfilePolicy::Action action = m_policy.tick(nowMs());
        if (action == ProfilePolicy::Action::kRevert) {
            m_stats.reverts++;
            m_revertRearmPending = true;
        }
        if (m_revertRearmPending) {
            if (m_session->stopRadio() != 0) {
                m_stats.change_deferred++;
                return;
            }
            if (!rearmRx()) return;
            m_revertRearmPending = false;
        }
    }

    // ------------------------------------------------------------------
    // dataIn_handler -- ComQueue thread  (UspRadio.cpp:180-193)
    // ------------------------------------------------------------------
    void dataIn_handler(std::vector<uint8_t> frame) {
        if (std::getenv("SIM_VERBOSE")) {
            std::fprintf(stderr, "[%s] dataIn %lld ms after comStatus; queue=%zu\n", m_name,
                         (long long)((MsgQueue::now_us() - m_lastComStatusUs) / 1000), m_queue.size());
        }
        m_pendingTxFrames.fetch_add(1, std::memory_order_relaxed);
        m_comStatusOwed.store(false, std::memory_order_relaxed);
        const int64_t t0 = MsgQueue::now_us();
        m_queue.send(10, [this, frame = std::move(frame), t0] { deferredTxPacket(frame, t0); }, "txPacket");
    }

    // ------------------------------------------------------------------
    // deferredRxDone  (UspRadio.cpp:205-268)
    // ------------------------------------------------------------------
    void deferredRxDone() {
        const uint32_t tail = m_rxTail.load(std::memory_order_relaxed);
        const uint32_t head = m_rxHead.load(std::memory_order_acquire);
        if (tail == head) return;
        const RxSlot& slot = m_rxRing[tail % RX_RING_DEPTH];

        ProfilePolicy::Action action = m_policy.frameReceived();
        if (action == ProfilePolicy::Action::kConfirmRx) {
            m_stats.rx_confirms++;
            if (slot.profileTag != m_policy.rxProfile()) {
                m_stats.rx_false_confirms++;
                if (std::getenv("SIM_VERBOSE")) {
                    std::fprintf(stderr, "[%s] FALSE CONFIRM: frame on P%u confirmed P%u\n", m_name, slot.profileTag,
                                 m_policy.rxProfile());
                }
            }
        }
        deliverRx(slot.data, slot.len);
        m_rxTail.store(tail + 1, std::memory_order_release);
    }

    void deliverRx(const uint8_t* data, size_t len) {
        m_stats.rx_delivered++;
        if (len >= 8) {
            int64_t t0;
            std::memcpy(&t0, data, 8);
            stat_sample(m_stats.rx_service_us_max, m_stats.rx_service_us_sum, &m_stats.rx_service_n,
                        static_cast<uint64_t>(MsgQueue::now_us() - t0));
        }
    }

    // ------------------------------------------------------------------
    // deferredTxPacket  (UspRadio.cpp:274-407)
    // ------------------------------------------------------------------
    void deferredTxPacket(const std::vector<uint8_t>& data, int64_t t0) {
        m_pendingTxFrames.fetch_sub(1, std::memory_order_relaxed);
        bool returnSuccess = true;
        if (m_transmitState == TxState::ENABLED) {
            size_t size = data.size() > 252 ? 252 : data.size();
            if (m_session->stopRadio() != 0) m_stats.stop_errors++;  // UspRadio.cpp:309 ignores this rc
            int rc = -1;
            if (applyProfile(m_policy.txProfile(), false)) {
                rc = m_session->transmitPacket(data.data(), size);
            }
            const TxOutcomePolicy::Outcome outcome = TxOutcomePolicy::evaluate(rc);
            if (outcome.transmitted) m_stats.tx_done_ok++; else m_stats.tx_failed++;
            returnSuccess = (outcome.comStatus == TxOutcomePolicy::ComStatus::SUCCESS);
            m_rearmAfterTx = true;
        } else if (m_transmitState == TxState::DISABLING) {
            m_transmitState = TxState::DISABLED;
            if (m_session->stopRadio() != 0) {
                m_stats.cfg_failed++;
            } else {
                (void)rearmRx();
            }
        }
        stat_sample(m_stats.tx_episode_us_max, m_stats.tx_episode_us_sum, &m_stats.tx_episode_n,
                    static_cast<uint64_t>(MsgQueue::now_us() - t0));
        comStatusOut(returnSuccess);
        m_comStatusOwed.store(returnSuccess, std::memory_order_relaxed);

        if (m_rearmAfterTx) {
            m_rearmAfterTx = false;
            for (int i = 0; (i < 5) && (m_pendingTxFrames.load(std::memory_order_relaxed) == 0); i++) {
                k_sleep(K_MSEC(1));
            }
            if (m_pendingTxFrames.load(std::memory_order_relaxed) == 0) {
                (void)rearmRx();
            }
        }
    }

    // ------------------------------------------------------------------
    // deferredTransmitCmd  (UspRadio.cpp:444-481)
    // ------------------------------------------------------------------
    void deferredTransmitCmd(bool enabled) {
        if (enabled) {
            if ((m_transmitState == TxState::DISABLED) && !m_comStatusOwed.load(std::memory_order_relaxed)) {
                comStatusOut(true);
                m_comStatusOwed.store(true, std::memory_order_relaxed);
            }
            m_transmitState = TxState::ENABLED;
        } else {
            if (m_transmitState == TxState::ENABLED) {
                m_transmitState = TxState::DISABLING;
            } else {
                m_transmitState = TxState::DISABLED;
                if (m_session->stopRadio() != 0) {
                    m_stats.cfg_failed++;
                } else {
                    (void)rearmRx();
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // deferredSetTxProfile  (UspRadio.cpp:487-515)
    // ------------------------------------------------------------------
    void deferredSetTxProfile(uint8_t idx) {
        if (m_policy.setTxProfile(idx) == ProfilePolicy::Action::kInvalidIndex) return;
        if (m_session->stopRadio() != 0) {
            m_stats.change_deferred++;
            return;
        }
        (void)rearmRx();
    }

    // ------------------------------------------------------------------
    // deferredSetRxProfile  (UspRadio.cpp:521-564)
    // ------------------------------------------------------------------
    void deferredSetRxProfile(uint8_t idx, uint16_t revert_s) {
        ProfilePolicy::Action action = m_policy.setRxProfile(idx, revert_s, nowMs());
        if (action == ProfilePolicy::Action::kInvalidIndex) return;
        if (action == ProfilePolicy::Action::kNoOp) {
            if (m_session->stopRadio() != 0) {
                m_stats.change_deferred++;
                return;
            }
            if (!rearmRx()) return;
            m_revertRearmPending = false;
            return;
        }
        if (m_session->stopRadio() != 0) {
            m_stats.change_deferred++;
            return;
        }
        if (rearmRx()) m_revertRearmPending = false;
    }

    // ------------------------------------------------------------------
    // deferredContinuousWave  (UspRadio.cpp:570-597)
    // ------------------------------------------------------------------
    void deferredContinuousWave(uint16_t seconds) {
        (void)m_session->stopRadio();
        if (m_session->startCw() != 0) {
            m_stats.cfg_failed++;
            return;
        }
        k_sleep(K_SECONDS(seconds));
        (void)m_session->stopRadio();
        (void)rearmRx();
    }

    // ------------------------------------------------------------------
    // Helpers  (UspRadio.cpp:628-661)
    // ------------------------------------------------------------------
    bool applyProfile(uint8_t idx, bool) {
        if (idx >= LINK_PROFILE_COUNT) return false;
        sim_chip_note_profile_being_applied(idx);
        if (m_session->applyProfile(LINK_PROFILE_TABLE[idx]) != 0) {
            m_stats.cfg_failed++;
            return false;
        }
        return true;
    }
    bool rearmRx() {
        const int64_t t0 = MsgQueue::now_us();
        if (!applyProfile(m_policy.armedRxProfile(), false)) return false;
        if (m_session->startReceive() != 0) {
            m_stats.cfg_failed++;
            return false;
        }
        stat_sample(m_stats.rearm_us_max, m_stats.rearm_us_sum, &m_stats.rearm_n,
                    static_cast<uint64_t>(MsgQueue::now_us() - t0));
        return true;
    }
    uint64_t nowMs() const { return static_cast<uint64_t>(k_uptime_get()); }

    // ------------------------------------------------------------------
    // F´ stand-ins
    // ------------------------------------------------------------------
    void comStatusOut(bool success) {
        if (!success) return;
        m_lastComStatusUs = MsgQueue::now_us();
        std::lock_guard<std::mutex> lk(m_comqM);
        m_comqCredit = true;
        m_comqCv.notify_one();
    }
    // Svc::ComQueue stand-in: one frame per SUCCESS comStatus, on its own thread.
    void comQueueMain() {
        sim_thread_name("comQueue");
        for (;;) {
            std::vector<uint8_t> frame;
            {
                std::unique_lock<std::mutex> lk(m_comqM);
                m_comqCv.wait(lk, [&] { return m_comqCredit && !m_comqFrames.empty(); });
                m_comqCredit = false;
                frame = std::move(m_comqFrames.front());
                m_comqFrames.pop_front();
            }
            dataIn_handler(std::move(frame));
        }
    }
    void componentMain() {
        sim_thread_name("uspRadio");
        for (;;) {
            Msg m = m_queue.receive();
            m_busy = true;
            const int64_t t0 = MsgQueue::now_us();
            stat_sample(m_stats.queue_wait_us_max, m_stats.queue_wait_us_sum, nullptr,
                        static_cast<uint64_t>(t0 - m.posted_us));
            m.fn();
            const uint64_t dur = static_cast<uint64_t>(MsgQueue::now_us() - t0);
            if (std::getenv("SIM_VERBOSE") && (t0 - m.posted_us > 200000 || dur > 100000 || m.kind[0] == 't')) {
                std::fprintf(stderr, "[%s] handler %s waited %lld ms ran %llu ms queue=%zu\n", m_name, m.kind,
                             (long long)((t0 - m.posted_us) / 1000), (unsigned long long)(dur / 1000), m_queue.size());
            }
            stat_sample(m_stats.handler_us_max, m_stats.handler_us_sum, &m_stats.handlers, dur);
            m_busy = false;
        }
    }

    const char* m_name;
    Zephyr::RalSessionImpl m_sessionImpl;
    Zephyr::RalSession* m_session;
    Zephyr::ProfilePolicy m_policy;
    MsgQueue m_queue;
    std::thread m_thread;
    std::thread m_comq;
    std::atomic<bool> m_busy{false};
    SimStats m_stats;

    TxState m_transmitState = TxState::DISABLED;
    RxSlot m_rxRing[RX_RING_DEPTH];
    std::atomic<uint32_t> m_rxHead;
    std::atomic<uint32_t> m_rxTail;
    std::atomic<uint32_t> m_pendingTxFrames{0};
    bool m_rearmAfterTx = false;
    bool m_revertRearmPending = false;
    std::atomic<bool> m_comStatusOwed{false};

    std::mutex m_comqM;
    std::condition_variable m_comqCv;
    std::deque<std::vector<uint8_t>> m_comqFrames;
    bool m_comqCredit = false;
    std::atomic<int64_t> m_lastComStatusUs{0};
};

}  // namespace

RadioSim* make_radio_sim(const char* name) { return new UspRadioSim1(name, 437000000U, 22); }
