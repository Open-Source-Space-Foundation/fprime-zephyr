// Option 2 (single queue) component model, completed from the prototype
// sketch PROTOTYPE-589-concurrency/single-queue/UspRadio.cpp.  Same F´
// stand-ins as UspRadioSim1.cpp.  The engine runs from engineKick on the
// component thread; no atomics, no ring, no k_sleep polls.
#include "../component.hpp"
#include "../sim_chip.h"
#include "RalSessionSt.hpp"

#include "fprime-zephyr/Drv/UspRadio/ProfilePolicy.hpp"
#include "fprime-zephyr/Drv/UspRadio/TxOutcomePolicy.hpp"

#include <zephyr/kernel.h>
#include <cstdio>

using namespace Zephyr;

namespace {

enum class TxState { DISABLED, ENABLED, DISABLING };
enum class Link { NO_CREDIT, CREDIT_OUT, FRAME_HELD };

class UspRadioSim2 final : public RadioSim, public RadioSink {
  public:
    UspRadioSim2(const char* name, uint32_t freq, int8_t pwr) : m_name(name), m_session(freq, pwr), m_queue(20) {
        m_thread = std::thread([this] { componentMain(); });
        m_comq = std::thread([this] { comQueueMain(); });
    }
    const char* name() const override { return m_name; }
    const SimStats& stats() const override { return m_stats; }
    uint8_t policyRxProfile() const override { return m_policy.rxProfile(); }
    bool policyHasPending() const override { return m_policy.hasPendingRx(); }
    uint64_t queueOverflow() const override { return m_queue.overflow(); }

    bool startRadio(bool txEnabled) override {
        m_transmitState = txEnabled ? TxState::ENABLED : TxState::DISABLED;
        if (m_session.init(*this) != 0) {
            m_stats.cfg_failed++;
            return false;
        }
        rearmRxWhenIdle();
        if (txEnabled) emitCredit();
        return true;
    }
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
        m_queue.send(0, [this] { run_handler(); });
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

    // ---- RadioSink (component thread, inside pump()) ----
    void postEngineKick() override {
        m_queue.send(10, [this] { m_session.onEngineKick(); });
    }
    void rxFrame(const uint8_t* data, std::size_t size, int16_t, int8_t) override {
        const uint8_t tag = sim_chip_rx_frame_profile();
        if (m_policy.frameReceived() == ProfilePolicy::Action::kConfirmRx) {
            m_stats.rx_confirms++;
            if (tag != m_policy.rxProfile()) {
                m_stats.rx_false_confirms++;
                if (std::getenv("SIM_VERBOSE")) {
                    std::fprintf(stderr, "[%s] FALSE CONFIRM: frame on P%u confirmed P%u\n", m_name, tag,
                                 m_policy.rxProfile());
                }
            }
        }
        m_stats.rx_delivered++;
        if (size >= 8) {
            int64_t t0;
            std::memcpy(&t0, data, 8);
            stat_sample(m_stats.rx_service_us_max, m_stats.rx_service_us_sum, &m_stats.rx_service_n,
                        static_cast<uint64_t>(MsgQueue::now_us() - t0));
        }
    }
    void txDone(int rc) override {
        const TxOutcomePolicy::Outcome outcome = TxOutcomePolicy::evaluate(rc);
        if (outcome.transmitted) m_stats.tx_done_ok++; else m_stats.tx_failed++;
        finishFrame();
    }

  private:
    void emitCredit() {
        if (m_link == Link::CREDIT_OUT) {
            std::fprintf(stderr, "[%s] FW_ASSERT: credit emitted twice\n", m_name);
            std::abort();
        }
        m_link = Link::CREDIT_OUT;
        std::lock_guard<std::mutex> lk(m_comqM);
        m_comqCredit = true;
        m_comqCv.notify_one();
    }
    bool txInFlight() const { return m_link == Link::FRAME_HELD; }

    void rearmRxWhenIdle() {
        if (txInFlight()) return;
        const uint8_t idx = m_policy.armedRxProfile();
        const int64_t t0 = MsgQueue::now_us();
        sim_chip_note_profile_being_applied(idx);
        m_rxArmFailed = (m_session.armRx(LINK_PROFILE_TABLE[idx]) != 0);
        if (m_rxArmFailed) {
            m_stats.cfg_failed++;
        } else {
            stat_sample(m_stats.rearm_us_max, m_stats.rearm_us_sum, &m_stats.rearm_n,
                        static_cast<uint64_t>(MsgQueue::now_us() - t0));
        }
    }

    void dataIn_handler(std::vector<uint8_t> frame) {
        const int64_t t0 = MsgQueue::now_us();
        m_queue.send(10, [this, frame = std::move(frame), t0] { deferredTxPacket(frame, t0); });
    }

    void deferredTxPacket(const std::vector<uint8_t>& data, int64_t t0) {
        if (m_link != Link::CREDIT_OUT) {
            std::fprintf(stderr, "[%s] FW_ASSERT: frame without credit\n", m_name);
            std::abort();
        }
        m_link = Link::FRAME_HELD;
        m_held = data;
        m_heldT0 = t0;
        if (m_transmitState != TxState::ENABLED) {
            if (m_transmitState == TxState::DISABLING) m_transmitState = TxState::DISABLED;
            finishFrame();
            return;
        }
        const size_t size = m_held.size() > 252 ? 252 : m_held.size();
        const uint8_t tx = m_policy.txProfile();
        sim_chip_note_profile_being_applied(tx);
        const int rc = m_session.startTx(LINK_PROFILE_TABLE[tx], m_held.data(), size, nowMs());
        if (rc != 0) {
            m_stats.tx_failed++;
            finishFrame();
        }
    }

    void finishFrame() {
        if (m_link != Link::FRAME_HELD) {
            std::fprintf(stderr, "[%s] FW_ASSERT: finishFrame without frame\n", m_name);
            std::abort();
        }
        stat_sample(m_stats.tx_episode_us_max, m_stats.tx_episode_us_sum, &m_stats.tx_episode_n,
                    static_cast<uint64_t>(MsgQueue::now_us() - m_heldT0));
        emitCredit();
        rearmRxWhenIdle();
    }

    void deferredTransmitCmd(bool enabled) {
        if (enabled) {
            if (m_link == Link::NO_CREDIT) emitCredit();
            m_transmitState = TxState::ENABLED;
            return;
        }
        m_transmitState = txInFlight() ? TxState::DISABLING : TxState::DISABLED;
        rearmRxWhenIdle();
    }

    void deferredSetTxProfile(uint8_t idx) {
        if (m_policy.setTxProfile(idx) == ProfilePolicy::Action::kInvalidIndex) return;
    }

    void deferredSetRxProfile(uint8_t idx, uint16_t revert_s) {
        const ProfilePolicy::Action action = m_policy.setRxProfile(idx, revert_s, nowMs());
        if (action == ProfilePolicy::Action::kInvalidIndex) return;
        rearmRxWhenIdle();
    }

    void deferredContinuousWave(uint16_t seconds) {
        if (txInFlight()) {
            m_stats.cfg_failed++;
            return;
        }
        if (m_session.startCw() != 0) {
            m_stats.cfg_failed++;
            return;
        }
        m_cwUntilMs = nowMs() + static_cast<uint64_t>(seconds) * 1000U;
        if (seconds == 0) m_cwUntilMs = 1;  // ends on the next run tick
    }

    void run_handler() {
        const uint64_t now = nowMs();
        m_session.checkTxDeadline(now);
        if ((m_cwUntilMs != 0) && (now >= m_cwUntilMs)) {
            m_cwUntilMs = 0;
            rearmRxWhenIdle();
        }
        if (m_policy.tick(now) == ProfilePolicy::Action::kRevert) {
            m_stats.reverts++;
            rearmRxWhenIdle();
        } else if (m_rxArmFailed) {
            rearmRxWhenIdle();
        }
    }

    uint64_t nowMs() const { return static_cast<uint64_t>(k_uptime_get()); }

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
            stat_sample(m_stats.handler_us_max, m_stats.handler_us_sum, &m_stats.handlers,
                        static_cast<uint64_t>(MsgQueue::now_us() - t0));
            m_busy = false;
        }
    }

    const char* m_name;
    RalSessionSt m_session;
    ProfilePolicy m_policy;
    MsgQueue m_queue;
    std::thread m_thread;
    std::thread m_comq;
    std::atomic<bool> m_busy{false};
    SimStats m_stats;

    TxState m_transmitState = TxState::DISABLED;
    Link m_link = Link::NO_CREDIT;
    std::vector<uint8_t> m_held;
    int64_t m_heldT0 = 0;
    bool m_rxArmFailed = false;
    uint64_t m_cwUntilMs = 0;

    std::mutex m_comqM;
    std::condition_variable m_comqCv;
    std::deque<std::vector<uint8_t>> m_comqFrames;
    bool m_comqCredit = false;
};

}  // namespace

RadioSim* make_radio_sim(const char* name) { return new UspRadioSim2(name, 437000000U, 22); }
