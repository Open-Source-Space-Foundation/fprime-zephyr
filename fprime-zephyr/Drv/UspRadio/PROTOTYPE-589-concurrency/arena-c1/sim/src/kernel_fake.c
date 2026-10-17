// pthread implementation of fakes/zephyr/kernel.h
#include <zephyr/kernel.h>

#include <string.h>
#include <time.h>
#include <unistd.h>

static uint64_t now_us_raw(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}
static uint64_t s_t0;
__attribute__((constructor)) static void k_init_clock(void) { s_t0 = now_us_raw(); }

uint64_t k_uptime_us(void) { return now_us_raw() - s_t0; }
int64_t k_uptime_get(void) { return (int64_t)(k_uptime_us() / 1000ULL); }
uint32_t k_uptime_get_32(void) { return (uint32_t)k_uptime_get(); }

static __thread k_thread_stats_t t_stats;
k_thread_stats_t* k_thread_stats_self(void) { return &t_stats; }
void k_thread_stats_reset_self(void) { memset(&t_stats, 0, sizeof(t_stats)); }

void k_sem_init(struct k_sem* s, unsigned initial, unsigned limit) {
    pthread_mutex_init(&s->m, NULL);
    pthread_cond_init(&s->c, NULL);
    s->count = initial;
    s->limit = limit;
}

int k_sem_take(struct k_sem* s, k_timeout_t t) {
    pthread_mutex_lock(&s->m);
    int rc = 0;
    if (s->count == 0) {
        if (t.ms == 0) {
            pthread_mutex_unlock(&s->m);
            return -EBUSY;
        }
        const uint64_t start = k_uptime_us();
        t_stats.sem_waits++;
        struct timespec dl;
        if (t.ms > 0) {
            clock_gettime(CLOCK_REALTIME, &dl);
            uint64_t ns = (uint64_t)dl.tv_nsec + (uint64_t)t.ms * 1000000ULL;
            dl.tv_sec += (time_t)(ns / 1000000000ULL);
            dl.tv_nsec = (long)(ns % 1000000000ULL);
        }
        while (s->count == 0) {
            if (t.ms < 0) {
                pthread_cond_wait(&s->c, &s->m);
            } else if (pthread_cond_timedwait(&s->c, &s->m, &dl) != 0 && s->count == 0) {
                rc = -EAGAIN;
                break;
            }
        }
        t_stats.sem_wait_us += k_uptime_us() - start;
    }
    if (rc == 0) {
        s->count--;
    }
    pthread_mutex_unlock(&s->m);
    return rc;
}

void k_sem_give(struct k_sem* s) {
    pthread_mutex_lock(&s->m);
    if (s->count < s->limit) {
        s->count++;
    }
    pthread_cond_signal(&s->c);
    pthread_mutex_unlock(&s->m);
}

uint64_t (*k_hold_probe)(void);
static __thread int t_hold_depth;
static __thread uint64_t t_hold_t0, t_hold_w0;
static __thread k_hold_stats_t t_hold;
static __thread struct k_mutex* t_init_target;
static void k_mutex_init_recursive(void) {
    pthread_mutexattr_t a;
    pthread_mutexattr_init(&a);
    pthread_mutexattr_settype(&a, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&t_init_target->m, &a);
    pthread_mutexattr_destroy(&a);
}
int k_mutex_lock(struct k_mutex* mx, k_timeout_t t) {
    (void)t;
    t_init_target = mx;
    pthread_once(&mx->once, k_mutex_init_recursive);
    if (pthread_mutex_lock(&mx->m) != 0) return -EINVAL;
    if (t_hold_depth++ == 0) {
        t_hold_t0 = k_uptime_us();
        t_hold_w0 = k_hold_probe ? k_hold_probe() : 0;
    }
    return 0;
}
k_hold_stats_t* k_hold_stats_self(void) { return &t_hold; }
int k_mutex_unlock(struct k_mutex* mx) {
    if (--t_hold_depth == 0) {
        const uint64_t d = k_uptime_us() - t_hold_t0;
        const uint64_t w = k_hold_probe ? k_hold_probe() - t_hold_w0 : 0;
        t_hold.holds++;
        t_hold.total_hold_us += d;
        if (d > t_hold.max_hold_us) t_hold.max_hold_us = d;
        if (w > t_hold.max_hold_work) t_hold.max_hold_work = w;
    }
    return pthread_mutex_unlock(&mx->m) == 0 ? 0 : -EINVAL;
}

int32_t k_sleep(k_timeout_t t) {
    if (t.ms <= 0) {
        return 0;
    }
    const uint64_t start = k_uptime_us();
    t_stats.sleeps++;
    usleep((useconds_t)(t.ms * 1000));
    t_stats.sleep_us += k_uptime_us() - start;
    return 0;
}

void k_busy_wait(uint32_t us) {
    const uint64_t end = k_uptime_us() + us;
    while (k_uptime_us() < end) {
    }
}

struct thread_trampoline {
    k_thread_entry_t fn;
    void *p1, *p2, *p3;
};
static void* k_thread_tramp(void* a) {
    struct thread_trampoline* t = (struct thread_trampoline*)a;
    t->fn(t->p1, t->p2, t->p3);
    return NULL;
}
k_tid_t k_thread_create(struct k_thread* th, char* stack, size_t sz, k_thread_entry_t fn, void* p1, void* p2,
                        void* p3, int prio, uint32_t options, k_timeout_t delay) {
    (void)stack; (void)sz; (void)prio; (void)options; (void)delay;
    static struct thread_trampoline tt[8];
    static int n;
    struct thread_trampoline* t = &tt[n++];
    t->fn = fn; t->p1 = p1; t->p2 = p2; t->p3 = p3;
    pthread_create(&th->t, NULL, k_thread_tramp, t);
    return th;
}
