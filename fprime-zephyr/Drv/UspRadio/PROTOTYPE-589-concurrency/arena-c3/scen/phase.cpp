// Deterministic race hunt: ONE received frame (P5, LEN bytes) and ONE component-thread operation (CMD) whose start is
// PHASE_US after the frame's RX_DONE (negative = before).  Sweep PHASE_US finely: a phase at which the radio never
// recovers is a reproducer for the engine-vs-caller race (claim E).
#include "rig.hpp"
static void main_fn(void*) {
    const char* cmd = getenv("CMD") ? getenv("CMD") : "settx";
    const int64_t phase = env_i64("PHASE_US", 0);
    const int len = (int)env_i64("LEN", 40);
    g_rig.build();
    BgLoad bg; bg.rg50_us = 300; bg.rg10_us = 1500; bg.rg1_us = 4000; bg.mm_us = 500; bg_start(bg);
    g_rig.boot(true);
    g_rig.radio->cmdSetRxProfile(Zephyr::LinkProfileId::P5, 0);
    g_rig.radio->cmdSetTxProfile(Zephyr::LinkProfileId::P5);
    k_sleep(K_MSEC(600));
    const int64_t air = (int64_t)chip_airtime_us(5, len);
    const int64_t t_done = zs_now_us() + 200000 + air;           // RX_DONE of the single frame
    chip_inject_frame(t_done - air, 5, len, 5000);
    const int64_t t_cmd = t_done + phase;
    k_sleep(zs_timeout(t_cmd - zs_now_us()));
    if (!strcmp(cmd, "settx")) g_rig.radio->cmdSetTxProfile(Zephyr::LinkProfileId::P4);
    else if (!strcmp(cmd, "setrx")) g_rig.radio->cmdSetRxProfile(Zephyr::LinkProfileId::P5, 0);
    else if (!strcmp(cmd, "tx")) g_rig.sendTx(60, 7);
    k_sleep(K_SECONDS(3));
    // liveness: RX alive on its current profile, TX credit returned
    size_t before = g_rig.rx.size();
    chip_inject_frame(zs_now_us() + 50000, chip_current_profile(), 30, 900001);
    k_sleep(K_SECONDS(1));
    printf("RESULT cmd=%s phase_us=%lld frame_delivered=%d rx_alive=%d chip_rx=%d credit=%d last_chip_cmd_ms=%.0f\n", cmd, (long long)phase,
           (int)(!g_rig.rx.empty() && g_rig.rx[0].tag == 5000), (int)(g_rig.rx.size() > before), (int)chip_is_rx(), g_rig.credit, chip_last_cmd_us() / 1000.0);
    zs_stop();
}
int main() { rig_threads_and_run(main_fn, 100000000); return 0; }
