#include "rig.hpp"
#include <cstring>

Rig g_rig;
namespace Fw { namespace Logger { int g_log_count = 0; } }

void Rig::build() {
    session = new SessionT(437400000U, 10);
    radio = new RadioT("uspRadio");
    radio->ports.event = [this](const std::string& n, long a, long b) { events.push_back({zs_now_us(), n, a, b}); };
    radio->ports.comStatusOut = [this](int s) { comStatus.push_back(zs_now_us() * 10 + s); if (s == 0) credit = 1; };
    radio->ports.dataOut = [this](const Fw::Buffer& bf) {
        uint32_t tag = 0;
        if (bf.getSize() >= 5) memcpy(&tag, bf.getData(), 4);
        rx.push_back({zs_now_us(), tag, bf.getSize() >= 5 ? bf.getData()[4] : -1});
        events.push_back({zs_now_us(), "RxDelivered", (long)tag, bf.getSize() >= 5 ? bf.getData()[4] : -1});
    };
    radio->ports.dataReturnOut = [this](const Fw::Buffer&) { txReturned.push_back(zs_now_us()); };
    radio->startThread((int)env_i64("RADIO_PRIO", 11));  // uspRadio: Zephyr priority 11 (RadioInstances_Usp.fppi: "priority 11"); RADIO_PRIO overrides for what-ifs
}

bool Rig::sendTx(int len, uint32_t tag) {
    if (!credit) return false;
    credit = 0;
    txSent++;
    uint8_t* d = (uint8_t*)calloc(1, len);
    memcpy(d, &tag, 4 < len ? 4 : len);
    Fw::Buffer b(d, len);
    ComCfg::FrameContext c;
    radio->dataIn_in(b, c);
    return true;
}

bool Rig::boot(bool txEnabled) {
#if OPT == 2
    return radio->startRadio(session, txEnabled ? Zephyr::UspTransmitState::ENABLED : Zephyr::UspTransmitState::DISABLED);
#else
    radio->configure(*session);
    return radio->startRadio(txEnabled ? Zephyr::UspTransmitState::ENABLED : Zephyr::UspTransmitState::DISABLED);
#endif
}

static BgLoad s_bg;
BgStat g_bgstat[5] = {{"rg50", 0, 0, 0}, {"rg10", 0, 0, 0}, {"rg1", 0, 0, 0}, {"mm", 0, 0, 0}, {"", 0, 0, 0}};
static void rg_thread(void* period_us_p, void* burst_p, void* stat_p) {
    int64_t period = (intptr_t)period_us_p;
    int64_t burst = (intptr_t)burst_p;
    BgStat* st = (BgStat*)stat_p;
    int64_t next = period;
    k_sleep(zs_timeout(period));
    for (;;) {
        int64_t late = zs_now_us() - next;  // scheduled release -> actually running
        if (late < 0) late = 0;
        st->cycles++; st->sum_late_us += late; if (late > st->max_late_us) st->max_late_us = late;
        zs_burn(burst);
        next += period;
        int64_t now = zs_now_us();
        if (next > now) k_sleep(zs_timeout(next - now));
        else next = now;  // overrun: slip
    }
}
static void run_driver(void*, void*, void*) {  // rateGroup1Hz -> uspRadio.run (RadioTopology_Usp.fppi:58)
    int64_t next = 1000000;
    for (;;) {
        int64_t now = zs_now_us();
        if (next > now) k_sleep(zs_timeout(next - now));
        { int64_t late = zs_now_us() - next; if (late < 0) late = 0; BgStat* st = &g_bgstat[2]; st->cycles++; st->sum_late_us += late; if (late > st->max_late_us) st->max_late_us = late; }
        zs_burn(s_bg.rg1_us);
        if (g_rig.radio) g_rig.radio->run_in(0);
        next += 1000000;
    }
}
void bg_start(const BgLoad& l) {
    s_bg = l;
    zs_spawn("rateGroup1Hz", 3, run_driver, nullptr, nullptr, nullptr);
    if (l.rg50_us) zs_spawn("rateGroup50Hz", 1, rg_thread, (void*)(intptr_t)20000, (void*)(intptr_t)l.rg50_us, &g_bgstat[0]);
    if (l.rg10_us) zs_spawn("rateGroup10Hz", 2, rg_thread, (void*)(intptr_t)100000, (void*)(intptr_t)l.rg10_us, &g_bgstat[1]);
    if (l.mm_us) zs_spawn("modeManager", 4, rg_thread, (void*)(intptr_t)500000, (void*)(intptr_t)l.mm_us, &g_bgstat[3]);
}

void rig_threads_and_run(void (*main_fn)(void*), int64_t t_end_us, const char* name) {
    apply_chip_env();
    zs_seed((uint64_t)env_i64("SEED", 1));
    chip_device();
    zs_spawn(name, 0, [](void* f, void*, void*) { ((void (*)(void*))f)(nullptr); for (;;) k_sleep(K_FOREVER); }, (void*)main_fn, nullptr, nullptr);
    zs_run(t_end_us, nullptr);
}

void print_run_summary(FILE* f) {
    fprintf(f, "chip: cmds=%lld spi_ms=%.1f spi_bytes=%lld rx_started=%lld rx_complete=%lld rx_aborted=%lld rx_arms=%lld tx_starts=%lld\n",
            (long long)chip_stats.cmds, chip_stats.spi_us / 1000.0, (long long)chip_stats.spi_bytes, (long long)chip_stats.rx_started,
            (long long)chip_stats.rx_complete, (long long)chip_stats.rx_aborted, (long long)chip_stats.rx_arms, (long long)chip_stats.tx_starts);
    fprintf(f, "chip: missed_not_listening=%lld missed_cfg_changed=%lld overwritten_unread=%lld torn_read=%lld xosc_starts=%lld sleeps=%lld wakes=%lld lost_asleep=%lld\n",
            (long long)chip_stats.rx_missed_not_listening, (long long)chip_stats.rx_missed_cfg_changed, (long long)chip_stats.rx_overwritten_unread,
            (long long)chip_stats.rx_torn_read, (long long)chip_stats.xosc_starts, (long long)chip_stats.sleeps, (long long)chip_stats.wakes, (long long)chip_stats.cmds_lost_asleep);
    fprintf(f, "panics=%d queue_overflows=%ld max_queue=%ld\n", g_panic_count, g_rig.radio ? g_rig.radio->queueOverflows : 0, g_rig.radio ? g_rig.radio->maxQueueLen : 0);
}
