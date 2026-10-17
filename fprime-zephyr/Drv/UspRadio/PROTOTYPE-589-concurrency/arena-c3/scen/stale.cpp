// Claim A: stale-frame false confirm.  One trial per process; OFFSET_US sweeps the phase of the SET_RX_PROFILE
// command against a continuous stream of frames sent on the OLD profile (P5).  The new profile (P3) is dead
// (nobody transmits on it) unless NEW_LIVE=1, so a ProfileConfirmed(P3) is a false confirm iff the frame it
// consumed was a P5 frame; it disarms the auto-revert and strands the radio on a dead profile.
#include "rig.hpp"
static void main_fn(void*) {
    const int64_t period = env_i64("PERIOD_US", 12000);
    const int64_t offset = env_i64("OFFSET_US", 0);
    const int len = (int)env_i64("LEN", 40);
    const bool newLive = env_i64("NEW_LIVE", 0) != 0;
    g_rig.build();
    BgLoad bg; bg.rg50_us = (int)env_i64("RG50_US", 0); bg.rg10_us = (int)env_i64("RG10_US", 0); bg.rg1_us = (int)env_i64("RG1_US", 0);
    bg_start(bg);
    g_rig.boot(false);
    const int64_t t0 = zs_now_us();
    // phase 1: move to P5 and confirm it with a P5 frame (legit)
    inject_stream(t0 + 250000, period, (int)((9000000) / period), 5, len, 1000);   // P5 stream for 9 s
    k_sleep(K_USEC(200000));
    g_rig.radio->cmdSetRxProfile(Zephyr::LinkProfileId::P5, 0);
    k_sleep(K_USEC(800000 + offset));
    const int64_t tcmd = zs_now_us();
    int64_t ev_before = g_rig.events.size();
    g_rig.radio->cmdSetRxProfile(Zephyr::LinkProfileId::P3, 3);
    if (newLive) inject_stream(tcmd + 1000000, period, (int)(4000000 / period), 3, len, 50000);
    k_sleep(K_USEC(5500000));
    // analysis
    int falseConfirm = 0, confirms3 = 0, staleAfter = 0, reverted = 0, p3deliv = 0;
    for (size_t i = 0; i < g_rig.events.size(); i++) {
        auto& e = g_rig.events[i];
        if (e.t < tcmd) continue;
        if (e.name == "ProfileConfirmed" && e.a == 3) {
            confirms3++;
            for (size_t j = i + 1; j < g_rig.events.size(); j++)
                if (g_rig.events[j].name == "RxDelivered") { if (g_rig.events[j].b != 3) falseConfirm++; break; }
        }
        if (e.name == "RxDelivered" && e.b == 5) staleAfter++;
        if (e.name == "RxDelivered" && e.b == 3) p3deliv++;
        if (e.name == "ProfileReverted") reverted++;
    }
    printf("RESULT offset_us=%lld period_us=%lld false_confirm=%d confirms_p3=%d reverted=%d stale_p5_delivered_after_cmd=%d p3_delivered=%d panics=%d\n",
           (long long)offset, (long long)period, falseConfirm, confirms3, reverted, staleAfter, p3deliv, g_panic_count);
    if (getenv("SIM_DUMP_EVENTS")) {
        printf("  cmd posted at t=%lld\n", (long long)tcmd);
        int n; const chip_rxrec_t* rr = chip_rx_records(&n);
        for (int i = 0; i < n; i++) if (rr[i].done_us > tcmd - 60000 && rr[i].done_us < tcmd + 400000)
            printf("  frame tag=%u prof=%d rx_done=%lld first_spi=%lld read_end=%lld\n", rr[i].tag, rr[i].profile, (long long)rr[i].done_us, (long long)rr[i].first_spi_us, (long long)rr[i].read_end_us);
    }
    if (getenv("SIM_DUMP_EVENTS"))
        for (auto& e : g_rig.events) if (e.t >= tcmd - 100000) printf("  ev t=%lld %s a=%ld b=%ld\n", (long long)e.t, e.name.c_str(), e.a, e.b);
    zs_stop();
}
int main() { rig_threads_and_run(main_fn, 40000000); return 0; }
