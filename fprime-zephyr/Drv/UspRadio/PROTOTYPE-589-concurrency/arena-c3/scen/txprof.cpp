// Minimal reproducer search: one TX frame, SET_TX_PROFILE posted DELAY_US after it, then a second TX frame.
// Is the radio alive afterwards (second frame returned, RX listening, a probe RX frame delivered)?
#include "rig.hpp"
static void main_fn(void*) {
    const int64_t delay = env_i64("DELAY_US", 0);
    g_rig.build();
    BgLoad bg; bg.rg50_us = 300; bg.rg10_us = 1500; bg.rg1_us = 4000; bg.mm_us = 500; bg_start(bg);
    g_rig.boot(true);
    k_sleep(K_MSEC(500));
    g_rig.sendTx((int)env_i64("LEN", 100), 1);
    k_sleep(zs_timeout(delay));
    g_rig.radio->cmdSetTxProfile((Zephyr::LinkProfileId::T)env_i64("NEWTX", 2));
    // wait for the first frame to be returned, then offer a second one
    int64_t t = zs_now_us();
    while (!g_rig.credit && zs_now_us() - t < 15000000) k_sleep(K_MSEC(1));
    bool second = g_rig.sendTx(100, 2);
    k_sleep(K_SECONDS(3));
    size_t before = g_rig.rx.size();
    chip_inject_frame(zs_now_us() + 50000, chip_current_profile(), 30, 900001);
    k_sleep(K_SECONDS(1));
    printf("RESULT delay_us=%lld tx_returned=%zu second_offered=%d chip_rx=%d rx_alive=%d last_chip_cmd_ms=%.0f\n", (long long)delay, g_rig.txReturned.size(), (int)second, (int)chip_is_rx(),
           (int)(g_rig.rx.size() > before), chip_last_cmd_us() / 1000.0);
    zs_stop();
}
int main() { rig_threads_and_run(main_fn, 100000000); return 0; }
