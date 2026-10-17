// Option 3 priority-inversion risk on rac_api_mutex (and the fork's mutex in option 1): who blocks whom, for how long,
// with a medium-priority CPU hog (prio HOG_PRIO, below the USP thread, above uspRadio) trying to preempt the lock holder.
// NO_PI=1 turns off priority inheritance in the simulated k_mutex (counterfactual; Zephyr k_mutex always inherits).
#include "rig.hpp"
extern "C" struct k_mutex rac_api_mutex;
static void hog(void* period, void* burst, void*) {
    int64_t p = (intptr_t)period, b = (intptr_t)burst;
    int64_t next = zs_now_us() + 7000;
    k_sleep(zs_timeout(7000));
    for (;;) { zs_burn(b); next += p; int64_t now = zs_now_us(); if (next > now) k_sleep(zs_timeout(next - now)); else next = now; }
}
static uint32_t g_tag = 100000;
static int64_t g_spur = 0;
static void spurious(void*) {
    if (zs_now_us() >= (int64_t)env_i64("SPUR_END_US", 1LL << 60)) return;
    chip_raise_irq(0x40 | 0x200);  // CRC error / timeout flags: raise DIO1 whenever the current IRQ mask lets them through
    zs_event_at(zs_now_us() + g_spur + (int64_t)(zs_rand() % g_spur), spurious, nullptr);
}
static int64_t g_end = 0;
static void injector(void*) {
    if (zs_now_us() >= g_end) return;
    chip_inject_frame(zs_now_us() + 1, chip_current_profile(), 20 + (int)(zs_rand() % 200), g_tag++);
    zs_event_at(zs_now_us() + 10000 + (int64_t)(zs_rand() % 60000), injector, nullptr);
}
static void main_fn(void*) {
    zs_set_no_pi((int)env_i64("NO_PI", 0));
#if OPT == 3
    zs_name_mutex(&rac_api_mutex, "rac_api_mutex");
#endif
    g_rig.build();
    BgLoad bg; bg.rg50_us = 300; bg.rg10_us = 1500; bg.rg1_us = 4000; bg.mm_us = 500; bg_start(bg);
    if (env_i64("HOG_BURST_US", 0)) zs_spawn("hog", (int)env_i64("HOG_PRIO", 5), hog, (void*)(intptr_t)env_i64("HOG_PERIOD_US", 50000), (void*)(intptr_t)env_i64("HOG_BURST_US", 0), nullptr);
    g_rig.boot(true);
    k_sleep(K_MSEC(300));
    zs_reset_mutex_stats();
    g_end = zs_now_us() + env_i64("DUR_US", 20000000);
    zs_event_at(zs_now_us() + 1000, injector, nullptr);
    g_spur = env_i64("SPUR_US", 0);
    if (g_spur) zs_event_at(zs_now_us() + 5000, spurious, nullptr);
    long tx = 0;
    while (zs_now_us() < g_end) {
        int r = (int)(zs_rand() % 100);
        if (g_rig.radio->queueLen() < 3) {
            if (r < 40) { if (g_rig.sendTx(100 + (int)(zs_rand() % 150), g_tag++)) tx++; }
            else if (r < 70) g_rig.radio->cmdSetRxProfile((Zephyr::LinkProfileId::T)(zs_rand() % 6), 0);
            else if (r < 85) g_rig.radio->cmdSetTxProfile((Zephyr::LinkProfileId::T)(zs_rand() % 6));
        }
        k_sleep(K_USEC(2000 + (int64_t)(zs_rand() % 30000)));
    }
    k_sleep(K_SECONDS(2));
    printf("RESULT spur_us=%lld hog_burst_us=%lld no_pi=%lld tx_returned=%zu usp_waits=%ld usp_wait_max_ms=%.2f comp_wait_max_ms=%.2f usp_hold_max_ms=%.2f comp_hold_max_ms=%.2f config_failed=%d panics=%d rx_delivered=%zu\n",
           (long long)g_spur, (long long)env_i64("HOG_BURST_US", 0), (long long)env_i64("NO_PI", 0), g_rig.txReturned.size(),
           zs_mutex_wait_count("lbm_main_thread_id"), zs_mutex_wait_max_ms("lbm_main_thread_id"), zs_mutex_wait_max_ms("uspRadio"), zs_mutex_hold_max_ms("lbm_main_thread_id"),
           zs_mutex_hold_max_ms("uspRadio"), g_rig.count("ConfigurationFailed"), g_panic_count, g_rig.rx.size());
    if (getenv("SIM_REPORT")) zs_report_mutex(stdout);
    zs_stop();
}
int main() { rig_threads_and_run(main_fn, 200000000); return 0; }
