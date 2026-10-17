// Minimal stand-in for the F´ active component machinery: a priority message
// queue drained by one thread (the "component thread"), plus the interface the
// scenarios drive and the counters they read.  Both option implementations
// derive from RadioSim.
#ifndef ARENA_C2_COMPONENT_HPP
#define ARENA_C2_COMPONENT_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct Msg {
    const char* kind;
    int priority;
    uint64_t seq;
    std::function<void()> fn;
    int64_t posted_us;
};

// F´ Os::Generic::PriorityMemQueue semantics: higher priority first, FIFO within a priority.
class MsgQueue {
  public:
    explicit MsgQueue(size_t depth) : m_depth(depth) {}
    bool send(int priority, std::function<void()> fn, const char* kind = "?") {
        std::lock_guard<std::mutex> lk(m_m);
        if (m_q.size() >= m_depth) {
            m_overflow++;
            return false;  // F´: FW_ASSERT on internal ports (assert policy); counted here
        }
        auto it = m_q.begin();
        while (it != m_q.end() && it->priority >= priority) ++it;
        m_q.insert(it, Msg{kind, priority, m_seq++, std::move(fn), now_us()});
        m_cv.notify_one();
        return true;
    }
    Msg receive() {
        std::unique_lock<std::mutex> lk(m_m);
        m_cv.wait(lk, [&] { return !m_q.empty(); });
        Msg m = std::move(m_q.front());
        m_q.pop_front();
        return m;
    }
    size_t size() {
        std::lock_guard<std::mutex> lk(m_m);
        return m_q.size();
    }
    uint64_t overflow() const { return m_overflow; }
    static int64_t now_us() {
        static const auto start = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
    }

  private:
    std::mutex m_m;
    std::condition_variable m_cv;
    std::deque<Msg> m_q;
    size_t m_depth;
    uint64_t m_seq = 0;
    std::atomic<uint64_t> m_overflow{0};
};

// Statistics the scenarios read.  All written on the component thread except
// where noted.
struct SimStats {
    std::atomic<uint64_t> rx_delivered{0};       // dataOut_out calls
    std::atomic<uint64_t> rx_confirms{0};        // ProfilePolicy kConfirmRx
    std::atomic<uint64_t> rx_false_confirms{0};  // confirm by a frame tagged with another profile
    std::atomic<uint64_t> rx_dropped_ring{0};
    std::atomic<uint64_t> tx_done_ok{0};
    std::atomic<uint64_t> tx_failed{0};
    std::atomic<uint64_t> cfg_failed{0};          // ConfigurationFailed events
    std::atomic<uint64_t> change_deferred{0};     // ProfileChangeDeferred events
    std::atomic<uint64_t> reverts{0};
    std::atomic<uint64_t> stop_errors{0};
    std::atomic<uint64_t> handler_us_max{0};
    std::atomic<uint64_t> handler_us_sum{0};
    std::atomic<uint64_t> handlers{0};
    std::atomic<uint64_t> queue_wait_us_max{0};   // posted -> handler start
    std::atomic<uint64_t> queue_wait_us_sum{0};
    // kick / rx service latency: DIO1 raise -> frame harvested from the chip
    std::atomic<uint64_t> rx_service_us_max{0};
    std::atomic<uint64_t> rx_service_us_sum{0};
    std::atomic<uint64_t> rx_service_n{0};
    // TX episode: dataIn -> comStatus SUCCESS back out (one frame)
    std::atomic<uint64_t> tx_episode_us_max{0};
    std::atomic<uint64_t> tx_episode_us_sum{0};
    std::atomic<uint64_t> tx_episode_n{0};
    // re-arm: stop+apply+startReceive sequence on the component thread
    std::atomic<uint64_t> rearm_us_max{0};
    std::atomic<uint64_t> rearm_us_sum{0};
    std::atomic<uint64_t> rearm_n{0};
};

inline void stat_sample(std::atomic<uint64_t>& mx, std::atomic<uint64_t>& sum, std::atomic<uint64_t>* n, uint64_t v) {
    uint64_t cur = mx.load();
    while (v > cur && !mx.compare_exchange_weak(cur, v)) {
    }
    sum += v;
    if (n) (*n)++;
}

class RadioSim {
  public:
    virtual ~RadioSim() = default;
    // Boot path (main thread in the real deployment).
    virtual bool startRadio(bool txEnabled) = 0;
    // Command / port entry points.  Each only enqueues, like the real component.
    virtual void cmdSetRxProfile(uint8_t idx, uint16_t revert_s) = 0;
    virtual void cmdSetTxProfile(uint8_t idx) = 0;
    virtual void cmdTransmit(bool enabled) = 0;
    virtual void cmdContinuousWave(uint16_t seconds) = 0;
    virtual void portRun() = 0;  // rate-group tick (async)
    // ComQueue stand-in: queue a frame for downlink.  Delivered to dataIn when
    // the component emits comStatus SUCCESS (one-in-flight protocol).
    virtual void comQueuePush(const std::vector<uint8_t>& frame) = 0;
    virtual size_t comQueueDepth() = 0;
    // Wait until the component queue is empty and the last handler returned.
    virtual void drain() = 0;
    virtual const SimStats& stats() const = 0;
    virtual const char* name() const = 0;
    virtual uint8_t policyRxProfile() const = 0;
    virtual bool policyHasPending() const = 0;
    virtual uint64_t queueOverflow() const = 0;
};

#endif
