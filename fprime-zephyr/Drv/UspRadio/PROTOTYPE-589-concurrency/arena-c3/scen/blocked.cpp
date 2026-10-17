// "Queue blocked in TX / CW": how long is a probe command (SET_TX_PROFILE, handler runs immediately) delayed when it is
// posted while the component thread is in the middle of a TX frame / CW / a hung-TX recovery?
//   FLOW=tx    : one TX frame (len LEN on TXPROF), probe posted PROBE_AFTER_MS after the frame
//   FLOW=cw    : CONTINUOUS_WAVE SEC seconds, probe posted 200 ms after
//   FLOW=hang  : CHIP_TX_HANG=1 (anomaly B: SetTx never completes); probe at +200 ms; then is RX alive afterwards?
#include "rig.hpp"
static void main_fn(void*) {
    const char* flow = getenv("FLOW") ? getenv("FLOW") : "tx";
    const int len = (int)env_i64("LEN", 252), txprof = (int)env_i64("TXPROF", 0);
    g_rig.build();
    BgLoad bg; bg.rg50_us = 300; bg.rg10_us = 1500; bg.rg1_us = 4000; bg.mm_us = 500; bg_start(bg);
    g_rig.boot(true);
    g_rig.radio->cmdSetTxProfile((Zephyr::LinkProfileId::T)txprof);
    k_sleep(K_MSEC(800));
    const int64_t t0 = zs_now_us();
    if (!strcmp(flow, "cw")) g_rig.radio->cmdContinuousWave((U16)env_i64("SEC", 3));
    else g_rig.sendTx(len, 1);
    k_sleep(K_MSEC(env_i64("PROBE_AFTER_MS", 200)));
    const int64_t tp = zs_now_us();
    size_t nev = g_rig.events.size();
    g_rig.radio->cmdSetTxProfile(Zephyr::LinkProfileId::P1);
    k_sleep(K_SECONDS(env_i64("WAIT_S", 25)));
    int64_t probe_run = -1;
    for (size_t i = nev; i < g_rig.events.size(); i++) if (g_rig.events[i].name == "ProfileChanged" && g_rig.events[i].a == 0) { probe_run = g_rig.events[i].t; break; }
    int64_t ret = g_rig.txReturned.empty() ? -1 : g_rig.txReturned[0];
    int sendfail = g_rig.count("SendFailed");
    // is RX alive after everything settled?
    size_t before = g_rig.rx.size();
    int rxprof = chip_current_profile();
    chip_inject_frame(zs_now_us() + 50000, rxprof, 30, 900001);
    k_sleep(K_SECONDS(3));
    printf("RESULT flow=%s txprof=%d len=%d airtime_ms=%.1f probe_delay_ms=%.1f tx_returned_ms=%.1f send_failed=%d rx_alive_after=%d chip_rx=%d\n", flow, txprof, len, chip_airtime_us(txprof, len) / 1000.0,
           probe_run < 0 ? -1.0 : (probe_run - tp) / 1000.0, ret < 0 ? -1.0 : (ret - t0) / 1000.0, sendfail, (int)(g_rig.rx.size() > before), (int)chip_is_rx());
    zs_stop();
}
int main() { rig_threads_and_run(main_fn, 400000000); return 0; }
