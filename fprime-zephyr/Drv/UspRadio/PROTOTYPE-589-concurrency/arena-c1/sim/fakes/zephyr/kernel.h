// Host fake of the slice of <zephyr/kernel.h> that UspRadio, RalSessionImpl and
// the usp HAL use.  pthreads underneath.  Not a scheduler model: host threads
// run truly in parallel on several cores, which is a superset of the
// interleavings a single-core preemptive RP2350 can produce (good for race
// detection, useless for priority/latency numbers).
#pragma once
#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t ms;  // <0 = forever
} k_timeout_t;

static inline k_timeout_t k__ms(int64_t ms) {
    k_timeout_t t;
    t.ms = ms;
    return t;
}
#define K_MSEC(ms) k__ms((int64_t)(ms))
#define K_SECONDS(s) k__ms((int64_t)(s) * 1000)
#define K_NO_WAIT k__ms(0)
#define K_FOREVER k__ms(-1)

struct k_sem {
    pthread_mutex_t m;
    pthread_cond_t c;
    unsigned count;
    unsigned limit;
};
void k_sem_init(struct k_sem* s, unsigned initial, unsigned limit);
int k_sem_take(struct k_sem* s, k_timeout_t t);
void k_sem_give(struct k_sem* s);

// Zephyr k_mutex is recursive for its owner.  Initialised lazily with a
// recursive attribute (TSan does not honour the static recursive initializer).
struct k_mutex {
    pthread_mutex_t m;
    pthread_once_t once;
};
#define K_MUTEX_DEFINE(name) struct k_mutex name = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_ONCE_INIT}
int k_mutex_lock(struct k_mutex* mx, k_timeout_t t);
int k_mutex_unlock(struct k_mutex* mx);

int32_t k_sleep(k_timeout_t t);
static inline int32_t k_msleep(int32_t ms) { return k_sleep(K_MSEC(ms)); }
void k_busy_wait(uint32_t us);
int64_t k_uptime_get(void);
uint32_t k_uptime_get_32(void);
uint64_t k_uptime_us(void);  // host-only helper
static inline bool k_is_in_isr(void) { return false; }

// threads (option 2's kick pump)
struct k_thread {
    pthread_t t;
};
typedef struct k_thread* k_tid_t;
typedef void (*k_thread_entry_t)(void*, void*, void*);
#define K_THREAD_STACK_DEFINE(name, size) char name[size]
#define K_THREAD_STACK_SIZEOF(s) sizeof(s)
k_tid_t k_thread_create(struct k_thread* th, char* stack, size_t sz, k_thread_entry_t fn, void* p1, void* p2,
                        void* p3, int prio, uint32_t options, k_timeout_t delay);

// ---- instrumentation (host-only) -------------------------------------------
// Per-thread blocking accounting so a scenario can say how long the
// component thread spent waiting on another thread vs. sleeping.
typedef struct {
    uint64_t sem_waits;      // k_sem_take calls that actually blocked
    uint64_t sem_wait_us;    // total time blocked in k_sem_take
    uint64_t sleeps;         // k_sleep calls
    uint64_t sleep_us;       // total time in k_sleep
} k_thread_stats_t;
k_thread_stats_t* k_thread_stats_self(void);
// k_mutex hold accounting (outermost lock -> unlock), per thread.  The probe
// returns a monotonically increasing per-thread work counter (RAL calls).
extern uint64_t (*k_hold_probe)(void);
typedef struct {
    uint64_t holds, max_hold_us, max_hold_work, total_hold_us;
} k_hold_stats_t;
k_hold_stats_t* k_hold_stats_self(void);
void k_thread_stats_reset_self(void);

#ifdef __cplusplus
}
#endif
