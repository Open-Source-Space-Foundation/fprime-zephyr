// Replica of usp_zephyr subsys/usp/zephyr_usp_thread.c usp_main_thread()
// (fork 0bf3e20, lines 96-141) for options 1 and 3.  Same loop shape, same
// MAX_SLEEP, same mutex placement.  It counts engine passes and the number of
// times the loop skipped its sleep (the v1.2.1 busy-loop claim F).
#include "usp_thread.h"
#include <zephyr/kernel.h>

extern "C" {
#include <smtc_rac_api.h>
void smtc_modem_hal_interruptible_msleep(k_timeout_t timeout);
}

#include <atomic>
#include <thread>

#ifndef CONFIG_USP_MAIN_THREAD_MAX_SLEEP_MS
#define CONFIG_USP_MAIN_THREAD_MAX_SLEEP_MS 60000
#endif

#if defined(CONFIG_USP_THREADS_MUTEXES)
extern "C" struct k_mutex rac_api_mutex;
#endif

namespace {
std::atomic<uint64_t> g_passes{0};
std::atomic<uint64_t> g_no_sleep{0};
std::atomic<bool> g_started{false};

void usp_main_thread() {
    sim_thread_name("usp_main");
    for (;;) {
#if defined(CONFIG_USP_THREADS_MUTEXES)
        k_mutex_lock(&rac_api_mutex, K_FOREVER);
#endif
        uint32_t sleep_time_ms = CONFIG_USP_MAIN_THREAD_MAX_SLEEP_MS;
        smtc_rac_run_engine();
        g_passes++;
#if defined(CONFIG_USP_THREADS_MUTEXES)
        k_mutex_unlock(&rac_api_mutex);
#endif
        if (smtc_rac_is_irq_flag_pending()) {
            g_no_sleep++;
            continue;
        }
        smtc_modem_hal_interruptible_msleep(K_MSEC(sleep_time_ms));
    }
}
}  // namespace

void usp_thread_start() {
    if (g_started.exchange(true)) return;
    std::thread(usp_main_thread).detach();
}
uint64_t usp_thread_passes() { return g_passes.load(); }
uint64_t usp_thread_no_sleep_iterations() { return g_no_sleep.load(); }
