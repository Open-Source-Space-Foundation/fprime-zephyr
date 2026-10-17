/* zsim: a deterministic, virtual-time, single-core, fixed-priority Zephyr-like kernel for the host.
 *
 * Purpose: run the REAL usp / UspRadio / RalSessionImpl code with Zephyr semantics (strict priority
 * preemption, priority-inheriting mutexes, equal-priority time slicing, tick-rounded sleeps) under a
 * virtual clock, so interleavings and latencies are repeatable.  Exactly one sim thread runs at a time
 * (baton passing between host pthreads).  Time only advances in zs_burn() (CPU work), zs_block_us()
 * (device wait with the CPU released) and when every thread is blocked (jump to next event).
 * Preemption points: every k_* call, zs_burn(), zs_block_us(), event delivery.
 *
 * Model limits (say so in RESULTS.md): single core; no cache/bus effects; CPU costs are inputs.
 */
#ifndef ZSIM_H
#define ZSIM_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int64_t us; } k_timeout_t;  /* us>=0 relative, -1 forever */
static inline k_timeout_t zs_timeout(int64_t us) { k_timeout_t t; t.us = us; return t; }
#define K_NO_WAIT zs_timeout(0)
#define K_FOREVER zs_timeout(-1)
#define K_USEC(n) zs_timeout((int64_t)(n))
#define K_MSEC(n) zs_timeout((int64_t)(n) * 1000LL)
#define K_SECONDS(n) zs_timeout((int64_t)(n) * 1000000LL)

#define ZS_MAXW 24
struct zs_thread;
struct k_sem {
    unsigned count, limit;
    struct zs_thread* wq[ZS_MAXW];
    int nwq;
};
#define K_SEM_MAX_LIMIT 0xFFFFFFFFu
#define K_SEM_DEFINE(name, init, lim) struct k_sem name = {(init), (lim), {0}, 0}
int k_sem_init(struct k_sem* s, unsigned init, unsigned limit);
int k_sem_take(struct k_sem* s, k_timeout_t to);
void k_sem_give(struct k_sem* s);
void k_sem_reset(struct k_sem* s);
unsigned k_sem_count_get(struct k_sem* s);

struct k_mutex {
    struct zs_thread* owner;
    unsigned lock_count;
    struct zs_thread* wq[ZS_MAXW];
    int nwq;
};
#define K_MUTEX_DEFINE(name) struct k_mutex name = {0, 0, {0}, 0}
int k_mutex_init(struct k_mutex* m);
int k_mutex_lock(struct k_mutex* m, k_timeout_t to);
int k_mutex_unlock(struct k_mutex* m);

struct k_condvar {
    struct zs_thread* wq[ZS_MAXW];
    int nwq;
};
#define K_CONDVAR_DEFINE(name) struct k_condvar name = {{0}, 0}
int k_condvar_wait(struct k_condvar* cv, struct k_mutex* m, k_timeout_t to);
int k_condvar_broadcast(struct k_condvar* cv);
int k_condvar_signal(struct k_condvar* cv);

typedef void (*k_thread_entry_t)(void*, void*, void*);
struct k_thread { struct zs_thread* t; };
typedef struct zs_thread* k_tid_t;
typedef char k_thread_stack_t;
#define K_THREAD_STACK_DEFINE(name, size) k_thread_stack_t name[1]
#define K_THREAD_STACK_SIZEOF(name) 4096
k_tid_t k_thread_create(struct k_thread* th, k_thread_stack_t* stack, size_t sz, k_thread_entry_t entry,
                        void* p1, void* p2, void* p3, int prio, uint32_t opts, k_timeout_t delay);
int k_thread_name_set(k_tid_t t, const char* n);
k_tid_t k_current_get(void);
/* K_THREAD_DEFINE: registered, started by zs_run(). */
void zs_register_thread(const char* name, k_thread_entry_t entry, void* p1, void* p2, void* p3, int prio);
#define K_THREAD_DEFINE(name, stack_size, entry, p1, p2, p3, prio, opts, delay)                             \
    static void __attribute__((constructor)) zs_reg_##name(void) { zs_register_thread(#name, entry, p1, p2, p3, prio); } \
    int zs_dummy_##name

int32_t k_sleep(k_timeout_t t);
int32_t k_msleep(int32_t ms);
int32_t k_usleep(int32_t us);
void k_busy_wait(uint32_t us);
void k_yield(void);
int64_t k_uptime_get(void);
uint32_t k_uptime_get_32(void);
uint32_t k_uptime_seconds(void);
bool k_is_in_isr(void);
void k_oops(void);

struct k_work;
typedef void (*k_work_handler_t)(struct k_work*);
struct k_work { k_work_handler_t handler; int pending; struct k_work* next; };
int k_work_submit(struct k_work* w);

struct k_timer;
typedef void (*k_timer_expiry_t)(struct k_timer*);
struct k_timer { k_timer_expiry_t expiry; void* user; uint64_t gen; };
#define K_TIMER_DEFINE(name, exp, stop) struct k_timer name = {(exp), 0, 0}
void k_timer_start(struct k_timer* t, k_timeout_t dur, k_timeout_t period);
void k_timer_stop(struct k_timer* t);

/* ---- sim control ---- */
void zs_init(void);
/* Spawn a sim thread (before or during run).  Lower prio number = higher priority. */
k_tid_t zs_spawn(const char* name, int prio, k_thread_entry_t entry, void* p1, void* p2, void* p3);
/* Start all registered threads whose name is not in the NULL-terminated skip list, run until t_end_us. */
void zs_run(int64_t t_end_us, const char* const* skip);
int64_t zs_now_us(void);
/* CPU work for us microseconds on the current thread (preemptible). */
void zs_burn(int64_t us);
/* Wait with the CPU released (device busy); not tick-rounded. */
void zs_block_us(int64_t us);
/* Schedule fn(arg) in ISR context at absolute time at_us (or now). */
void zs_event_at(int64_t at_us, void (*fn)(void*), void* arg);
const char* zs_thread_name(void);
int zs_thread_prio(void);
int64_t zs_thread_cpu_us(const char* name);
int64_t zs_idle_us(void);
/* trace to stderr or file when enabled */
void zs_trace_enable(const char* path);
void zs_trace(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
/* slice (equal-priority round robin), us; 0 disables.  Default 20000 (CONFIG_TIMESLICE_SIZE=20). */
void zs_set_timeslice(int64_t us, int min_prio);
/* randomness for scenarios (xorshift, seeded) */
uint64_t zs_rand(void);
void zs_seed(uint64_t s);
/* end the simulation now (from any context) */
void zs_stop(void);
void zs_set_kcall_cost(int64_t us);
/* report: per-thread CPU */
void zs_report(void* file);
void zs_dump_threads(void* file);
void zs_name_mutex(const void* m, const char* name);
void zs_set_no_pi(int v);
void zs_report_mutex(void* file);
void zs_reset_mutex_stats(void);
double zs_mutex_wait_max_ms(const char* waiter_prefix);
double zs_mutex_hold_max_ms(const char* holder_prefix);
long zs_mutex_wait_count(const char* waiter_prefix);

#ifdef __cplusplus
}
#endif
#endif
