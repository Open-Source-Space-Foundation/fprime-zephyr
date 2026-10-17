// Fake UspRadioComponentBase queue + dispatcher (see the header).
#include "fprime-zephyr/Drv/UspRadio/UspRadioComponentAc.hpp"
#include <zephyr/kernel.h>

namespace Zephyr {

void UspRadioComponentBase::rec(const char* w, long a, long b) {
    std::lock_guard<std::mutex> g(traceMx);
    trace.push_back({k_uptime_us(), w, a, b});
}

int UspRadioComponentBase::count(const char* w) {
    std::lock_guard<std::mutex> g(traceMx);
    int n = 0;
    for (auto& r : trace) n += (r.what == w);
    return n;
}

void UspRadioComponentBase::post(const char* name, int prio, std::function<void()> fn) {
    {
        std::lock_guard<std::mutex> g(m_qm);
        if (static_cast<int>(m_q.size()) >= QUEUE_DEPTH) {
            // Generated code: send NONBLOCKING, FW_ASSERT(qStatus == OP_OK).
            std::fprintf(stderr, "FW_ASSERT queue full posting %s; component thread in '%s' for %llu us; queue:",
                         name, m_current ? m_current : "-",
                         static_cast<unsigned long long>(k_uptime_us() - m_currentSince));
            for (auto& q : m_q) std::fprintf(stderr, " %s", q.name);
            std::fprintf(stderr, "\n");
            std::abort();
        }
        Msg m{prio, m_order++, name, std::move(fn)};
        // stable max-priority insert
        auto it = m_q.begin();
        while (it != m_q.end() && it->prio >= prio) ++it;
        m_q.insert(it, std::move(m));
        if (static_cast<int>(m_q.size()) > queueHighWater) queueHighWater = static_cast<int>(m_q.size());
    }
    m_qc.notify_one();
    if (afterPost) afterPost(name);
}

void UspRadioComponentBase::loop() {
    pthread_setname_np("uspRadio(comp)");
    for (;;) {
        Msg m;
        {
            std::unique_lock<std::mutex> g(m_qm);
            m_idle.notify_all();
            m_qc.wait(g, [this] { return !m_q.empty() || !m_run; });
            if (!m_run && m_q.empty()) return;
            m = std::move(m_q.front());
            m_q.pop_front();
            m_busy = true;
            m_current = m.name;
            m_currentSince = k_uptime_us();
        }
        if (beforeDispatch) beforeDispatch(m.name);
        m.fn();
        {
            std::lock_guard<std::mutex> g(m_qm);
            m_busy = false;
            dispatched++;
        }
        m_idle.notify_all();
    }
}

}  // namespace Zephyr
