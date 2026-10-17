// Claim B: tasks[0xFF] out-of-bounds read after a failed init.
// MODE=fault_open : the call site sees smtc_rac_open_radio() == RAC_INVALID_RADIO_ID (hal/fault_inject.h), startRadio()
//                   fails, the topology ignores it (ReferenceDeploymentTopology.cpp:141), then the commands arrive.
// MODE=double_open: call the REAL smtc_rac_open_radio() a second time on a live hook: does it return RAC_INVALID_RADIO_ID
//                   or does rp_hook_init panic first?  (decides whether the INVALID branch is reachable on target)
#include "rig.hpp"
extern "C" const char* g_marker;
const char* g_marker = "boot";
static void main_fn(void*) {
    const char* mode = getenv("MODE") ? getenv("MODE") : "fault_open";
    g_rig.build();
    BgLoad bg; bg_start(bg);
    if (!strcmp(mode, "double_open")) {
        bool ok = g_rig.boot(false);
        g_marker = "second smtc_rac_open_radio()";
        uint8_t id = smtc_rac_open_radio(RAC_HIGH_PRIORITY);
        printf("INFO second open returned id=%u (boot ok=%d)\n", id, ok);
        zs_stop();
        return;
    }
    bool ok = g_rig.boot(false);
    printf("INFO startRadio() returned %d (expected 0: init failed)\n", ok);
    const char* seq[] = {"SET_RX_PROFILE", "SET_TX_PROFILE", "TRANSMIT(enable)", "TRANSMIT(disable)", "CONTINUOUS_WAVE", "run(1Hz tick)"};
    for (int i = 0; i < 6; i++) {
        g_marker = seq[i];
        switch (i) {
            case 0: g_rig.radio->cmdSetRxProfile(Zephyr::LinkProfileId::P2, 5); break;
            case 1: g_rig.radio->cmdSetTxProfile(Zephyr::LinkProfileId::P2); break;
            case 2: g_rig.radio->cmdTransmit(Zephyr::UspTransmitState::ENABLED); break;
            case 3: g_rig.radio->cmdTransmit(Zephyr::UspTransmitState::DISABLED); break;
            case 4: g_rig.radio->cmdContinuousWave(1); break;
            case 5: g_rig.radio->run_in(0); break;
        }
        k_sleep(K_MSEC(300));
        printf("INFO after %-18s panics=%d\n", seq[i], g_panic_count);
    }
    zs_stop();
}
int main() {
    rig_threads_and_run(main_fn, 60000000);
    extern const char* g_marker;
    printf("RESULT mode=%s panics=%d first_panic=\"%s\" last_command=\"%s\"\n", getenv("MODE") ? getenv("MODE") : "fault_open", g_panic_count, g_panic_msg, g_marker);
    return 0;
}
