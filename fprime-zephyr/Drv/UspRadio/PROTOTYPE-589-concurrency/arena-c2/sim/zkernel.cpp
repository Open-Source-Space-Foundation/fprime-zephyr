// Implementation of the <zephyr/kernel.h> host shim.  See the header.
#include <zephyr/kernel.h>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <mutex>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
const Clock::time_point g_start = Clock::now();

struct SemImpl {
    std::mutex m;
    std::condition_variable cv;
    unsigned count;
    unsigned limit;
};
struct MutexImpl {
    std::recursive_mutex m;
};
struct TimerImpl {
    std::mutex m;
    std::condition_variable cv;
    std::thread thread;
    uint64_t generation = 0;   // bumped on every start/stop; the worker only fires if unchanged
    bool armed = false;
    Clock::time_point deadline;
    bool quit = false;
};
thread_local const char* t_name = "?";
const bool g_debug = std::getenv("SIM_DEBUG") != nullptr;
#define DBG(...) do { if (g_debug) { std::fprintf(stderr, "[%s] ", t_name); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } } while (0)

int64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - g_start).count();
}
}  // namespace

extern "C" {

void k_sem_init(struct k_sem* sem, unsigned int initial, unsigned int limit) {
    auto* s = new SemImpl;
    s->count = initial;
    s->limit = limit;
    sem->impl = s;
}
void k_sem_give(struct k_sem* sem) {
    auto* s = static_cast<SemImpl*>(sem->impl);
    DBG("sem_give %p", (void*)sem);
    std::lock_guard<std::mutex> lk(s->m);
    if (s->count < s->limit) {
        s->count++;
    }
    s->cv.notify_one();
}
int k_sem_take(struct k_sem* sem, k_timeout_t timeout) {
    auto* s = static_cast<SemImpl*>(sem->impl);
    DBG("sem_take %p timeout_us=%lld", (void*)sem, (long long)timeout.ticks_us);
    std::unique_lock<std::mutex> lk(s->m);
    if (s->count > 0) {
        s->count--;
        return 0;
    }
    if (timeout.ticks_us == 0) {
        return -EBUSY;
    }
    if (timeout.ticks_us < 0) {
        s->cv.wait(lk, [&] { return s->count > 0; });
    } else {
        if (!s->cv.wait_for(lk, std::chrono::microseconds(timeout.ticks_us), [&] { return s->count > 0; })) {
            return -EAGAIN;
        }
    }
    s->count--;
    return 0;
}

void k_mutex_init(struct k_mutex* m) { m->impl = new MutexImpl; }
int k_mutex_lock(struct k_mutex* m, k_timeout_t) {
    DBG("mutex_lock %p", (void*)m);
    static_cast<MutexImpl*>(m->impl)->m.lock();
    return 0;
}
int k_mutex_unlock(struct k_mutex* m) {
    DBG("mutex_unlock %p", (void*)m);
    static_cast<MutexImpl*>(m->impl)->m.unlock();
    return 0;
}

// The worker thread is created lazily on the first start: creating threads
// from a global constructor hangs under ThreadSanitizer.
void k_timer_init(struct k_timer* t, k_timer_expiry_t expiry, void*) {
    auto* ti = new TimerImpl;
    t->expiry = expiry;
    t->impl = ti;
}
static void timer_thread_ensure(struct k_timer* t, TimerImpl* ti) {
    if (ti->thread.joinable()) return;
    ti->thread = std::thread([t, ti] {
        sim_thread_name("k_timer");
        std::unique_lock<std::mutex> lk(ti->m);
        while (!ti->quit) {
            if (!ti->armed) {
                ti->cv.wait(lk);
                continue;
            }
            const uint64_t gen = ti->generation;
            if (ti->cv.wait_until(lk, ti->deadline) == std::cv_status::timeout && ti->generation == gen && ti->armed) {
                ti->armed = false;
                lk.unlock();
                t->expiry(t);   // "ISR" context in Zephyr; a plain thread here
                lk.lock();
            }
        }
    });
}
void k_timer_start(struct k_timer* t, k_timeout_t duration, k_timeout_t) {
    auto* ti = static_cast<TimerImpl*>(t->impl);
    DBG("timer_start %lld us", (long long)duration.ticks_us);
    timer_thread_ensure(t, ti);
    std::lock_guard<std::mutex> lk(ti->m);
    ti->generation++;
    ti->armed = true;
    ti->deadline = Clock::now() + std::chrono::microseconds(duration.ticks_us);
    ti->cv.notify_all();
}
void k_timer_stop(struct k_timer* t) {
    auto* ti = static_cast<TimerImpl*>(t->impl);
    std::lock_guard<std::mutex> lk(ti->m);
    ti->generation++;
    ti->armed = false;
    ti->cv.notify_all();
}

int64_t k_uptime_get(void) { return now_us() / 1000; }
uint32_t k_uptime_get_32(void) { return static_cast<uint32_t>(now_us() / 1000); }
uint32_t k_uptime_seconds(void) { return static_cast<uint32_t>(now_us() / 1000000); }
int32_t k_sleep(k_timeout_t t) {
    if (t.ticks_us > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(t.ticks_us));
    } else {
        std::this_thread::yield();
    }
    return 0;
}
int32_t k_msleep(int32_t ms) { return k_sleep(K_MSEC(ms)); }
int32_t k_usleep(int32_t us) { return k_sleep(K_USEC(us)); }
void k_busy_wait(uint32_t us) {
    const int64_t end = now_us() + us;
    while (now_us() < end) {
    }
}
void k_yield(void) { std::this_thread::yield(); }
bool k_is_in_isr(void) { return false; }

k_tid_t k_thread_create(struct k_thread* t, k_thread_stack_t*, size_t, k_thread_entry_t entry, void* p1, void* p2,
                        void* p3, int, uint32_t, k_timeout_t) {
    auto* th = new std::thread([entry, p1, p2, p3] { entry(p1, p2, p3); });
    th->detach();
    t->impl = th;
    return t;
}
void k_thread_name_set(k_tid_t, const char*) {}
void sim_thread_name(const char* name) { t_name = name; }
const char* sim_thread_name_get(void) { return t_name; }

void sim_assert_fail(const char* cond, const char* file, int line) {
    std::fprintf(stderr, "ASSERT %s at %s:%d\n", cond, file, line);
    std::abort();
}

}  // extern "C"
