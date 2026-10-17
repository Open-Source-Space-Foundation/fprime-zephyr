// Host fake of usp_zephyr's smtc_modem_hal.c (+ the usp_main_thread loop of
// subsys/usp/zephyr_usp_thread.c, copied line for line minus LBM/LOG).
//
// Build switches:
//   FORK_RAC_MUTEX           usp_zephyr fork bbdd729: recursive k_mutex in
//                            protect/unprotect_api_call (option 1 today).
//   CONFIG_USP_THREADS_MUTEXES  upstream mutex mode: rac_api_mutex around the
//                            engine pass in the thread loop (option 3).
//   neither                  upstream stubs (protect is a no-op).
#include "sim.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "smtc_modem_hal.h"
#include "smtc_rac_api.h"
#include <zephyr/usp/smtc_zephyr_usp_api.h>

#ifndef CONFIG_USP_MAIN_THREAD_MAX_SLEEP_MS
#define CONFIG_USP_MAIN_THREAD_MAX_SLEEP_MS 60000
#endif

volatile uint64_t g_engine_passes, g_engine_spins, g_engine_sleeps;
k_hold_stats_t g_usp_hold;
volatile int g_panic_count;
char g_panic_where[128];

static struct k_sem lbm_main_loop_sem;
static struct k_sem s_init_sem;
static void (*s_radio_cb)(void*);
static void* s_radio_ctx;
static void (*s_timer_cb)(void*);
static void* s_timer_ctx;
static volatile bool s_irq_enabled = true, s_radio_pending, s_timer_pending;

__attribute__((constructor)) static void hal_init_sems(void) {
    k_sem_init(&lbm_main_loop_sem, 0, 1);
    k_sem_init(&s_init_sem, 0, 1);
}

uint32_t smtc_modem_hal_get_time_in_ms(void) { return k_uptime_get_32(); }
uint32_t smtc_modem_hal_get_time_in_s(void) { return k_uptime_get_32() / 1000; }

static void timer_fire(void* a) {
    (void)a;
    if (s_irq_enabled) s_timer_cb(s_timer_ctx);
    else s_timer_pending = true;
}
void smtc_modem_hal_start_timer(const uint32_t ms, void (*cb)(void*), void* ctx) {
    s_timer_cb = cb;
    s_timer_ctx = ctx;
    sim_timer_start(TIMER_HAL, ms * 1000u, timer_fire, NULL);
}
void smtc_modem_hal_stop_timer(void) { sim_timer_stop(TIMER_HAL); }
void smtc_modem_hal_disable_modem_irq(void) { s_irq_enabled = false; }
void smtc_modem_hal_enable_modem_irq(void) {
    s_irq_enabled = true;
    if (s_radio_pending) { s_radio_pending = false; s_radio_cb(s_radio_ctx); }
    if (s_timer_pending) { s_timer_pending = false; s_timer_cb(s_timer_ctx); }
}
void smtc_modem_hal_irq_config_radio_irq(void (*cb)(void*), void* ctx) { s_radio_cb = cb; s_radio_ctx = ctx; }
void hal_fake_radio_irq(void) {  // sysworkq context
    if (!s_radio_cb) return;
    if (s_irq_enabled) s_radio_cb(s_radio_ctx);
    else s_radio_pending = true;
}
void smtc_modem_hal_radio_irq_clear_pending(void) {}

void smtc_modem_hal_on_panic(uint8_t* func, uint32_t line, const char* fmt, ...) {
    (void)fmt;
    snprintf(g_panic_where, sizeof g_panic_where, "%s:%u", (const char*)func, line);
    __atomic_add_fetch(&g_panic_count, 1, __ATOMIC_SEQ_CST);
    fprintf(stdout, "SMTC_MODEM_HAL_PANIC at %s -> smtc_modem_hal_reset_mcu()\n", g_panic_where);
    fflush(stdout);
    if (getenv("SIM_PANIC_CONTINUE") == NULL) exit(42);  // the MCU resets here
}
void smtc_modem_hal_reset_mcu(void) { exit(43); }
void smtc_modem_hal_reload_wdog(void) {}
uint32_t smtc_modem_hal_get_random_nb_in_range(const uint32_t a, const uint32_t b) { return a + (uint32_t)(rand() % (int)(b - a + 1)); }
bool smtc_modem_external_stack_currently_use_radio(void) { return false; }
void smtc_modem_hal_start_radio_tcxo(void) {}
void smtc_modem_hal_stop_radio_tcxo(void) {}
uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms(void) { return 0; }
void smtc_modem_hal_set_ant_switch(bool on) { (void)on; }
int8_t smtc_modem_hal_get_board_delay_ms(void) { return 0; }
void smtc_modem_hal_print_trace(const char* fmt, ...) {
    if (!getenv("SIM_TRACE")) return;
    va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
}
void smtc_modem_hal_crashlog_store(const uint8_t* s, uint8_t n) { (void)s; (void)n; }

// ---- RAC API serialization -------------------------------------------------
#if defined(FORK_RAC_MUTEX)
K_MUTEX_DEFINE(prv_rac_api_mutex);
void smtc_modem_hal_protect_api_call(void) { (void)k_mutex_lock(&prv_rac_api_mutex, K_FOREVER); }
void smtc_modem_hal_unprotect_api_call(void) { (void)k_mutex_unlock(&prv_rac_api_mutex); }
#else
void smtc_modem_hal_protect_api_call(void) {}
void smtc_modem_hal_unprotect_api_call(void) {}
#endif
#if defined(CONFIG_USP_THREADS_MUTEXES)
K_MUTEX_DEFINE(rac_api_mutex);
#endif

// ---- event semaphore -------------------------------------------------------
void smtc_modem_hal_interruptible_msleep(k_timeout_t t) { k_sem_take(&lbm_main_loop_sem, t); }
void smtc_modem_hal_wake_up(void) { k_sem_give(&lbm_main_loop_sem); }
void smtc_modem_hal_user_lbm_irq(void) { smtc_modem_hal_wake_up(); }
struct k_sem* smtc_modem_hal_get_event_sem(void) { return &lbm_main_loop_sem; }

void zephyr_usp_initialization_wait(void) { k_sem_take(&s_init_sem, K_FOREVER); k_sem_give(&s_init_sem); }
void zephyr_usp_initialization_notify(void) { k_sem_give(&s_init_sem); }

// ---- usp_main_thread (zephyr_usp_thread.c:96-141, LBM branch removed) -------
static void* usp_main_thread(void* unused) {
    (void)unused;
    uint32_t sleep_time_ms = 0;
    pthread_setname_np("usp_main_thread");
    zephyr_usp_initialization_notify();
    while (true) {
#if defined(CONFIG_USP_THREADS_MUTEXES)
        k_mutex_lock(&rac_api_mutex, K_FOREVER);
#endif
        sleep_time_ms = CONFIG_USP_MAIN_THREAD_MAX_SLEEP_MS;
        smtc_rac_run_engine();
        g_engine_passes++;
        g_usp_hold = *k_hold_stats_self();
#if defined(CONFIG_USP_THREADS_MUTEXES)
        k_mutex_unlock(&rac_api_mutex);
#endif
        if (smtc_rac_is_irq_flag_pending()) {
            g_engine_spins++;
            continue;
        }
        g_engine_sleeps++;
        smtc_modem_hal_interruptible_msleep(K_MSEC(sleep_time_ms));
    }
    return NULL;
}
void hal_fake_start_usp_thread(void) {
    pthread_t t;
    pthread_create(&t, NULL, usp_main_thread, NULL);
}
