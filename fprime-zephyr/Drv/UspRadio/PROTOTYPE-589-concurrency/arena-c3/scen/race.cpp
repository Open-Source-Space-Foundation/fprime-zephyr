// Claim E (and general robustness): a randomized command/TX/RX storm.  Counts failures that a correct
// serialization of the RAC would not produce: planner panics, stuck TX credit, ConfigurationFailed /
// ProfileChangeDeferred / SendFailed events, and a final liveness check (is RX alive on P0 afterwards).
#include "rig.hpp"
static uint32_t g_tag = 100000;
static int64_t g_stormEnd = 0;
static void injector(void*) {
    if (zs_now_us() >= g_stormEnd) return;
    int len = 20 + (int)(zs_rand() % 80);
    int prof = chip_current_profile();
    chip_inject_frame(zs_now_us() + 1, prof, len, g_tag++);
    static int64_t rxgap = env_i64("RX_GAP_US", 30000);
    zs_event_at(zs_now_us() + 3000 + (int64_t)(zs_rand() % rxgap), injector, nullptr);
}
static void main_fn(void*) {
    const int64_t dur = env_i64("DUR_US", 20000000);
    const int gapMax = (int)env_i64("GAP_MAX_MS", 40);
    const int txPct = (int)env_i64("TX_PCT", 35);
    g_rig.build();
    BgLoad bg; bg.rg50_us = (int)env_i64("RG50_US", 0); bg.rg10_us = (int)env_i64("RG10_US", 0); bg.rg1_us = (int)env_i64("RG1_US", 0);
    bg_start(bg);
    g_rig.boot(false);
    g_rig.radio->cmdTransmit(Zephyr::UspTransmitState::ENABLED);
    k_sleep(K_MSEC(100));
    g_stormEnd = zs_now_us() + dur;
    zs_event_at(zs_now_us() + 1000, injector, nullptr);
    long nCmd = 0, nTx = 0;
    std::vector<int64_t> opTimes;
    while (zs_now_us() < g_stormEnd && !g_panic_count) {
        if (g_rig.radio->queueLen() >= 3) { k_sleep(K_MSEC(2)); continue; }
        int r = (int)(zs_rand() % 100);
        if (const char* only = getenv("ONLY")) {  // restrict the action mix: any of tx, setrx, settx, enable joined by '+'
            std::vector<std::string> opts; std::string cur;
            for (const char* c = only;; ++c) { if (*c == '+' || !*c) { opts.push_back(cur); cur.clear(); if (!*c) break; } else cur += *c; }
            const std::string& o = opts[zs_rand() % opts.size()];
            if (o == "tx") r = 0; else if (o == "setrx") r = txPct + 1; else if (o == "settx") r = txPct + 31; else if (o == "enable") r = txPct + 41;
        }
        if (r < txPct) { if (g_rig.sendTx(20 + (int)(zs_rand() % 200), g_tag++)) { nTx++; opTimes.push_back(zs_now_us()); } }
        else if (r < txPct + 30) { g_rig.radio->cmdSetRxProfile((Zephyr::LinkProfileId::T)(zs_rand() % 6), 0); nCmd++; opTimes.push_back(zs_now_us()); }
        else if (r < txPct + 40) { g_rig.radio->cmdSetTxProfile((Zephyr::LinkProfileId::T)(zs_rand() % 6)); nCmd++; opTimes.push_back(zs_now_us()); }
        else if (r < txPct + 45) { g_rig.radio->cmdTransmit((zs_rand() & 1) ? Zephyr::UspTransmitState::ENABLED : Zephyr::UspTransmitState::DISABLED); nCmd++; opTimes.push_back(zs_now_us()); }
        k_sleep(K_USEC(1000 + (int64_t)(zs_rand() % (gapMax * 1000))));
    }
    // liveness: back to P0, let everything drain, then check RX still works
    g_rig.radio->cmdTransmit(Zephyr::UspTransmitState::DISABLED);
    g_rig.radio->cmdSetRxProfile(Zephyr::LinkProfileId::P0, 0);
    k_sleep(K_SECONDS(6));
    size_t before = g_rig.rx.size();
    for (int i = 0; i < 6; i++) { chip_inject_frame(zs_now_us() + 100000 + i * 400000, 0, 30, 900000 + i); }
    k_sleep(K_SECONDS(4));
    int live = 0;
    for (size_t i = before; i < g_rig.rx.size(); i++) if (g_rig.rx[i].tag >= 900000) live++;
    long opsBefore = 0;
    for (auto t : opTimes) if (live == 6 || t <= chip_last_cmd_us()) opsBefore++;
    printf("RESULT seed=%lld ops_posted=%zu ops_before_stall=%ld last_chip_cmd_ms=%.0f panics=%d cmds=%ld tx_sent=%ld tx_returned=%zu config_failed=%d chg_deferred=%d send_failed=%d queue_overflow=%ld live_rx=%d/6 chip_rx=%d lost_asleep=%lld lost=[%s]\n",
           (long long)env_i64("SEED", 1), opTimes.size(), opsBefore, chip_last_cmd_us() / 1000.0, g_panic_count, nCmd, nTx, g_rig.txReturned.size(), g_rig.count("ConfigurationFailed"),
           g_rig.count("ProfileChangeDeferred"), g_rig.count("SendFailed"), g_rig.radio->queueOverflows, live, (int)chip_is_rx(), (long long)chip_stats.cmds_lost_asleep, chip_lost_detail());
    if (live < 6 && getenv("SIM_DUMP_THREADS")) zs_dump_threads(stdout);
    if (g_panic_count) printf("INFO panic: %s\n", g_panic_msg);
    zs_stop();
}
int main() { rig_threads_and_run(main_fn, 200000000); return 0; }
