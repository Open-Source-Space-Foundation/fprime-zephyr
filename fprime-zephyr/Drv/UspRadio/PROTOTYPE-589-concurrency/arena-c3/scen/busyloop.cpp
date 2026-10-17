// Claim F: usp v1.2.1 rp_get_irq_flag() keeps the USP thread spinning while a LOCK is held.
// Real usp_zephyr zephyr_usp_thread.c loop + real radio_planner.c; thread mix = instances.fpp priorities.
// Measures: CPU share by thread, deployment-thread lateness, and queue latency of a probe command on uspRadio (prio 11).
#include "rig.hpp"
static void sampler(void*, void*, void*) {
    int64_t lu = 0, li = 0, lt = 0;
    for (;;) {
        k_sleep(K_MSEC(250));
        int64_t u = zs_thread_cpu_us("lbm_main_thread_id"), i = zs_idle_us(), t = zs_now_us();
        fprintf(stderr, "[sample] t=%.2fs usp=%.0f%% idle=%.0f%%\n", t / 1e6, 100.0 * (u - lu) / (t - lt), 100.0 * (i - li) / (t - lt));
        lu = u; li = i; lt = t;
    }
}
static void main_fn(void*) {
    if (getenv("SIM_SAMPLE")) zs_spawn("sampler", 0, sampler, nullptr, nullptr, nullptr);
    const int64_t dur = env_i64("DUR_US", 10000000);
    g_rig.build();
    BgLoad bg; bg.rg50_us = (int)env_i64("RG50_US", 300); bg.rg10_us = (int)env_i64("RG10_US", 1500); bg.rg1_us = (int)env_i64("RG1_US", 4000); bg.mm_us = (int)env_i64("MM_US", 500);
    bg_start(bg);
    g_rig.boot(false);
    const int64_t t0 = zs_now_us();
    // a few received frames per second so the radio is genuinely in continuous RX with traffic
    inject_stream(t0 + 100000, env_i64("RX_PERIOD_US", 500000), (int)(dur / env_i64("RX_PERIOD_US", 500000)), 0, 30, 1000);
    // probe: SET_TX_PROFILE every 500 ms; latency = post -> ProfileChanged(TX) event
    std::vector<int64_t> post;
    for (int i = 0; i < (int)(dur / 500000) - 2; i++) {
        post.push_back(zs_now_us());
        if (env_i64("PROBE", 1)) g_rig.radio->cmdSetTxProfile((Zephyr::LinkProfileId::T)(i % 2 ? 2 : 1));
        k_sleep(K_MSEC(500));
    }
    k_sleep(K_MSEC(300));
    std::vector<int64_t> lat;
    size_t pi = 0;
    for (auto& e : g_rig.events) if (e.name == "ProfileChanged" && e.a == 0 && e.t > t0 && pi < post.size()) { lat.push_back(e.t - post[pi]); pi++; }
    int64_t maxlat = 0, sum = 0; for (auto l : lat) { sum += l; if (l > maxlat) maxlat = l; }
    int64_t usp = zs_thread_cpu_us("lbm_main_thread_id"), total = zs_now_us() - t0;
    printf("RESULT dur_s=%.1f usp_thread_cpu_pct=%.1f idle_pct=%.1f probes_posted=%zu probes_run=%zu probe_latency_ms_max=%.1f probe_latency_ms_mean=%.1f rx_delivered=%zu/%lld rg1_cycles=%ld rg1_late_ms_max=%.1f rg10_cycles=%ld rg10_late_ms_max=%.1f rg50_late_ms_max=%.1f mm_cycles=%ld mm_late_ms_max=%.1f uspRadio_cpu_ms=%.1f\n",
           total / 1e6, 100.0 * usp / total, 100.0 * zs_idle_us() / total, post.size(), lat.size(), lat.empty() ? -1.0 : maxlat / 1000.0, lat.empty() ? -1.0 : sum / 1000.0 / lat.size(),
           g_rig.rx.size(), (long long)(dur / env_i64("RX_PERIOD_US", 500000)), g_bgstat[2].cycles, g_bgstat[2].max_late_us / 1000.0, g_bgstat[1].cycles, g_bgstat[1].max_late_us / 1000.0,
           g_bgstat[0].max_late_us / 1000.0, g_bgstat[3].cycles, g_bgstat[3].max_late_us / 1000.0, zs_thread_cpu_us("uspRadio") / 1000.0);
    if (getenv("SIM_REPORT")) zs_report(stdout);
    zs_stop();
}
int main() { rig_threads_and_run(main_fn, 100000000); return 0; }
