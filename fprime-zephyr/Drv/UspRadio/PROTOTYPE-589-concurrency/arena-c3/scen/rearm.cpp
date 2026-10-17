// Re-arm cost: how long is the receiver deaf around a profile change / one TX / a TX burst, per option.
#include "rig.hpp"
static const char* codeName(int c) { static const char* n[] = {"?", "RX_LISTEN", "RX_STOP", "TX_START", "TX_DONE"}; return c >= 0 && c < 5 ? n[c] : "?"; }
static void main_fn(void*) {
    const char* flow = getenv("FLOW") ? getenv("FLOW") : "tx1";
    const int len = (int)env_i64("LEN", 252), txprof = (int)env_i64("TXPROF", 5), nframes = (int)env_i64("NFRAMES", 8);
    g_rig.build();
    BgLoad bg; bg.rg50_us = (int)env_i64("RG50_US", 300); bg.rg10_us = (int)env_i64("RG10_US", 1500); bg.rg1_us = (int)env_i64("RG1_US", 4000); bg.mm_us = (int)env_i64("MM_US", 500);
    bg_start(bg);
    g_rig.boot(true);
    g_rig.radio->cmdSetTxProfile((Zephyr::LinkProfileId::T)txprof);
    k_sleep(K_MSEC(800));
    int n0; chip_events(&n0);
    const int64_t t_post = zs_now_us();
    int64_t t_end = 0;
    if (!strcmp(flow, "setrx")) {
        g_rig.radio->cmdSetRxProfile((Zephyr::LinkProfileId::T)env_i64("RXPROF", 2), 0);
        k_sleep(K_MSEC(1500));
    } else if (!strcmp(flow, "tx1")) {
        g_rig.sendTx(len, 1);
        k_sleep(K_MSEC(1500));
    } else if (!strcmp(flow, "txN")) {
        int sent = 0;
        while (sent < nframes) {
            if (g_rig.sendTx(len, 100 + sent)) sent++;
            else k_sleep(K_USEC(200));
            if (zs_now_us() - t_post > 60000000) break;
        }
        // wait for last frame to be returned and the credit to come back
        while (!g_rig.credit && zs_now_us() - t_post < 60000000) k_sleep(K_MSEC(1));
        t_end = zs_now_us();
        k_sleep(K_MSEC(1500));
    }
    int n; const chip_ev_t* ev = chip_events(&n);
    // deaf time = sum over RX_STOP..RX_LISTEN gaps after t_post
    int64_t deaf = 0, last_stop = -1, first_tx = -1, last_txdone = -1, last_listen = -1, txs = 0;
    for (int i = 0; i < n; i++) {
        if (ev[i].t < t_post) continue;
        if (ev[i].code == 2 && last_stop < 0) last_stop = ev[i].t;
        if (ev[i].code == 1 && last_stop >= 0) { deaf += ev[i].t - last_stop; last_stop = -1; last_listen = ev[i].t; }
        if (ev[i].code == 3) { txs++; if (first_tx < 0) first_tx = ev[i].t; }
        if (ev[i].code == 4) last_txdone = ev[i].t;
        if (getenv("SIM_DUMP_EVENTS")) printf("  chip t=%lld (+%.2f ms) %s prof=%d\n", (long long)ev[i].t, (ev[i].t - t_post) / 1000.0, codeName(ev[i].code), ev[i].profile);
    }
    printf("RESULT flow=%s len=%d txprof=%d frames=%lld t_first_tx_start_ms=%.1f t_last_txdone_ms=%.1f t_rx_relisten_ms=%.1f rx_deaf_total_ms=%.1f txstarts=%lld airtime_ms=%.1f burst_total_ms=%.1f returned=%zu\n",
           flow, len, txprof, (long long)(!strcmp(flow, "txN") ? nframes : 1), first_tx < 0 ? -1 : (first_tx - t_post) / 1000.0,
           last_txdone < 0 ? -1 : (last_txdone - t_post) / 1000.0, last_listen < 0 ? -1 : (last_listen - t_post) / 1000.0, deaf / 1000.0,
           (long long)txs, chip_airtime_us(txprof, len) / 1000.0, t_end ? (t_end - t_post) / 1000.0 : -1.0, g_rig.txReturned.size());
    zs_stop();
}
int main() { rig_threads_and_run(main_fn, 300000000); return 0; }
