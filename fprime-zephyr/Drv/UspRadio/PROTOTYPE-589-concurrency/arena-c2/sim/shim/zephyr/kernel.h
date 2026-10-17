// Host shim for the subset of <zephyr/kernel.h> that RalSessionImpl.cpp, the
// option-2 session, the usp thread replica and the fake HAL use.  pthreads
// underneath; time is the host steady clock in ms since process start.
// Scheduling is NOT Zephyr's: every thread has the same host priority.
#ifndef ARENA_C2_ZEPHYR_KERNEL_SHIM_H
#define ARENA_C2_ZEPHYR_KERNEL_SHIM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int64_t ticks_us; } k_timeout_t;   // negative = forever
#define K_NO_WAIT   ((k_timeout_t){0})
#define K_FOREVER   ((k_timeout_t){-1})
#define K_USEC(us)  ((k_timeout_t){(int64_t)(us)})
#define K_MSEC(ms)  ((k_timeout_t){(int64_t)(ms) * 1000})
#define K_SECONDS(s) ((k_timeout_t){(int64_t)(s) * 1000000})

struct k_sem { void* impl; };
void k_sem_init(struct k_sem* sem, unsigned int initial, unsigned int limit);
void k_sem_give(struct k_sem* sem);
int  k_sem_take(struct k_sem* sem, k_timeout_t timeout);   // 0 ok, -EAGAIN/-EBUSY on timeout
#define K_SEM_DEFINE(name, init, limit) struct k_sem name = { 0 }; \
    __attribute__((constructor)) static void name##_ctor(void) { k_sem_init(&name, init, limit); }

struct k_mutex { void* impl; };
void k_mutex_init(struct k_mutex* m);
int  k_mutex_lock(struct k_mutex* m, k_timeout_t timeout);
int  k_mutex_unlock(struct k_mutex* m);
#define K_MUTEX_DEFINE(name) struct k_mutex name = { 0 }; \
    __attribute__((constructor)) static void name##_ctor(void) { k_mutex_init(&name); }

struct k_timer;
typedef void (*k_timer_expiry_t)(struct k_timer* timer);
struct k_timer { void* impl; k_timer_expiry_t expiry; };
void k_timer_init(struct k_timer* t, k_timer_expiry_t expiry, void* stop);
void k_timer_start(struct k_timer* t, k_timeout_t duration, k_timeout_t period);
void k_timer_stop(struct k_timer* t);
#define K_TIMER_DEFINE(name, expiry, stop) struct k_timer name = { 0, expiry }; \
    __attribute__((constructor)) static void name##_ctor(void) { k_timer_init(&name, expiry, NULL); }

int64_t  k_uptime_get(void);
uint32_t k_uptime_get_32(void);
uint32_t k_uptime_seconds(void);
int32_t  k_sleep(k_timeout_t t);
int32_t  k_msleep(int32_t ms);
int32_t  k_usleep(int32_t us);
void     k_busy_wait(uint32_t us);
void     k_yield(void);
bool     k_is_in_isr(void);

typedef void (*k_thread_entry_t)(void*, void*, void*);
struct k_thread { void* impl; };
typedef struct k_thread* k_tid_t;
typedef uint8_t k_thread_stack_t;
#define K_THREAD_STACK_DEFINE(name, size) static k_thread_stack_t name[size]
#define K_THREAD_STACK_SIZEOF(name) sizeof(name)
k_tid_t k_thread_create(struct k_thread* t, k_thread_stack_t* stack, size_t stack_size, k_thread_entry_t entry,
                        void* p1, void* p2, void* p3, int prio, uint32_t options, k_timeout_t delay);
void k_thread_name_set(k_tid_t t, const char* name);
// Host-only helper: the sim can name the calling thread for log lines.
void sim_thread_name(const char* name);
const char* sim_thread_name_get(void);

#define __ASSERT(cond, ...) do { if (!(cond)) { sim_assert_fail(#cond, __FILE__, __LINE__); } } while (0)
#define __ASSERT_NO_MSG(cond) __ASSERT(cond, "")
void sim_assert_fail(const char* cond, const char* file, int line);
#define ARG_UNUSED(x) (void)(x)
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifdef __cplusplus
}
#endif
#endif
