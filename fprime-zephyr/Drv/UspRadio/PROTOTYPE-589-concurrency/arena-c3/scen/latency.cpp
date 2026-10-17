// Claim G / RX service: a frame stream on one profile; per-frame chip RX_DONE -> first SPI (kick latency),
// -> FIFO read finished, -> delivered upstream (dataOut).  Thread mix per instances.fpp (assumed CPU bursts).
#include "rig.hpp"
#include <algorithm>
static void hog(void* period, void* burst, void*) {
    int64_t p = (intptr_t)period, b = (intptr_t)burst;
    int64_t next = zs_now_us();
    for (;;) { zs_burn(b); next += p; int64_t now = zs_now_us(); if (next > now) k_sleep(zs_timeout(next - now)); else next = now; }
}
static double pct(std::vector<int64_t>& v, double p) { if (v.empty()) return -1; std::sort(v.begin(), v.end()); return v[(size_t)((v.size() - 1) * p)] / 1000.0; }
static void main_fn(void*) {
    const int prof = (int)env_i64("PROF", 5), len = (int)env_i64("LEN", 252), count = (int)env_i64("COUNT", 100);
    const int64_t gap = env_i64("GAP_US", 5000);
    g_rig.build();
    BgLoad bg; bg.rg50_us = (int)env_i64("RG50_US", 300); bg.rg10_us = (int)env_i64("RG10_US", 1500); bg.rg1_us = (int)env_i64("RG1_US", 4000); bg.mm_us = (int)env_i64("MM_US", 500);
    bg_start(bg);
    if (env_i64("HOG_BURST_US", 0)) zs_spawn("hog", (int)env_i64("HOG_PRIO", 5), hog, (void*)(intptr_t)env_i64("HOG_PERIOD_US", 100000), (void*)(intptr_t)env_i64("HOG_BURST_US", 0), nullptr);
    g_rig.boot(false);
    g_rig.radio->cmdSetRxProfile((Zephyr::LinkProfileId::T)prof, 0);
    k_sleep(K_MSEC(1500));
    const int64_t period = (int64_t)chip_airtime_us(prof, len) + gap;
    const int64_t t0 = zs_now_us() + 100000;
    inject_stream(t0, period, count, prof, len, 7000);
    k_sleep(zs_timeout(t0 - zs_now_us() + count * period + 500000));
    int n; const chip_rxrec_t* rr = chip_rx_records(&n);
    std::vector<int64_t> kick, readend, deliv;
    int delivered = 0;
    for (int i = 0; i < n; i++) {
        if (rr[i].tag < 7000 || rr[i].tag >= 7000 + (uint32_t)count) continue;
        if (rr[i].first_spi_us) kick.push_back(rr[i].first_spi_us - rr[i].done_us);
        if (rr[i].read_end_us) readend.push_back(rr[i].read_end_us - rr[i].done_us);
        for (auto& r : g_rig.rx) if (r.tag == rr[i].tag) { deliv.push_back(r.t - rr[i].done_us); delivered++; break; }
    }
    printf("RESULT prof=%d len=%d gap_us=%lld period_us=%lld airtime_ms=%.1f sent=%d rx_complete=%d delivered=%d kick_ms_p50=%.2f kick_ms_p95=%.2f kick_ms_max=%.2f readend_ms_p50=%.2f deliver_ms_p50=%.2f deliver_ms_p95=%.2f deliver_ms_max=%.2f unread_overwritten=%lld torn=%lld missed_cfg=%lld rg1_late_ms_max=%.1f rg10_late_ms_max=%.1f mm_late_ms_max=%.1f rg1_cycles=%ld mm_cycles=%ld usp_thread_cpu_pct=%.1f\n",
           prof, len, (long long)gap, (long long)period, chip_airtime_us(prof, len) / 1000.0, count, n, delivered,
           pct(kick, .5), pct(kick, .95), pct(kick, 1.0), pct(readend, .5), pct(deliv, .5), pct(deliv, .95), pct(deliv, 1.0),
           (long long)chip_stats.rx_overwritten_unread, (long long)chip_stats.rx_torn_read, (long long)chip_stats.rx_missed_cfg_changed,
           g_bgstat[2].max_late_us / 1000.0, g_bgstat[1].max_late_us / 1000.0, g_bgstat[3].max_late_us / 1000.0, g_bgstat[2].cycles, g_bgstat[3].cycles,
           100.0 * zs_thread_cpu_us("lbm_main_thread_id") / zs_now_us());
    zs_stop();
}
int main() { rig_threads_and_run(main_fn, 300000000); return 0; }
