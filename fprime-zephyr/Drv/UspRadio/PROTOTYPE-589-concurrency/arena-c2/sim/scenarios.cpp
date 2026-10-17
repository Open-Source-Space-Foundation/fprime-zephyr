// Scenario driver.  One binary per option/usp-version/sanitizer; the scenario
// is the first argument.  Output: one `key=value` line per measurement on
// stdout, free-form diagnostics on stderr.
#include "component.hpp"
#include "sim_chip.h"
#include "usp_thread.h"

#include <zephyr/kernel.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/resource.h>
#include <thread>

RadioSim* make_radio_sim(const char* name);

namespace {

struct Air {
    // Deliver frames to the chip at a fixed period from a background thread.
    std::thread th;
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> delivered{0}, lost{0};
    void start(uint32_t period_us, uint16_t len) {
        th = std::thread([this, period_us, len] {
            sim_thread_name("fake-air-rx");
            std::vector<uint8_t> f(len, 0xA5);
            while (!stop.load()) {
                const int64_t t = MsgQueue::now_us();
                std::memcpy(f.data(), &t, 8);
                if (sim_air_deliver(f.data(), len)) delivered++; else lost++;
                std::this_thread::sleep_for(std::chrono::microseconds(period_us));
            }
        });
    }
    void join() {
        stop = true;
        if (th.joinable()) th.join();
    }
};

double cpu_seconds() {
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    return ru.ru_utime.tv_sec + ru.ru_utime.tv_usec / 1e6 + ru.ru_stime.tv_sec + ru.ru_stime.tv_usec / 1e6;
}

void print_stats(const RadioSim& r) {
    const SimStats& s = r.stats();
    sim_chip_stats c;
    sim_chip_stats_get(&c);
    std::printf("rx_delivered=%llu rx_confirms=%llu rx_false_confirms=%llu rx_dropped_ring=%llu\n",
                (unsigned long long)s.rx_delivered.load(), (unsigned long long)s.rx_confirms.load(),
                (unsigned long long)s.rx_false_confirms.load(), (unsigned long long)s.rx_dropped_ring.load());
    std::printf("tx_done_ok=%llu tx_failed=%llu cfg_failed=%llu change_deferred=%llu reverts=%llu\n",
                (unsigned long long)s.tx_done_ok.load(), (unsigned long long)s.tx_failed.load(),
                (unsigned long long)s.cfg_failed.load(), (unsigned long long)s.change_deferred.load(),
                (unsigned long long)s.reverts.load());
    std::printf("handlers=%llu handler_us_max=%llu handler_us_avg=%llu queue_wait_us_max=%llu queue_overflow=%llu\n",
                (unsigned long long)s.handlers.load(), (unsigned long long)s.handler_us_max.load(),
                (unsigned long long)(s.handlers ? s.handler_us_sum / s.handlers : 0),
                (unsigned long long)s.queue_wait_us_max.load(), (unsigned long long)r.queueOverflow());
    std::printf("rx_service_n=%llu rx_service_us_max=%llu rx_service_us_avg=%llu\n",
                (unsigned long long)s.rx_service_n.load(), (unsigned long long)s.rx_service_us_max.load(),
                (unsigned long long)(s.rx_service_n ? s.rx_service_us_sum / s.rx_service_n : 0));
    std::printf("fifo_read_n=%llu fifo_read_us_max=%llu fifo_read_us_avg=%llu rx_overwritten=%llu\n",
                (unsigned long long)c.fifo_read_n, (unsigned long long)c.fifo_read_us_max,
                (unsigned long long)(c.fifo_read_n ? c.fifo_read_us_sum / c.fifo_read_n : 0),
                (unsigned long long)c.rx_overwritten);
    std::printf("tx_episode_n=%llu tx_episode_us_max=%llu tx_episode_us_avg=%llu\n",
                (unsigned long long)s.tx_episode_n.load(), (unsigned long long)s.tx_episode_us_max.load(),
                (unsigned long long)(s.tx_episode_n ? s.tx_episode_us_sum / s.tx_episode_n : 0));
    std::printf("rearm_n=%llu rearm_us_max=%llu rearm_us_avg=%llu\n", (unsigned long long)s.rearm_n.load(),
                (unsigned long long)s.rearm_us_max.load(),
                (unsigned long long)(s.rearm_n ? s.rearm_us_sum / s.rearm_n : 0));
    std::printf("spi_transactions=%llu spi_bytes=%llu spi_busy_ms=%llu set_rx=%llu set_tx=%llu set_standby=%llu "
                "set_sleep=%llu set_mod_params=%llu read_buffer=%llu write_buffer=%llu\n",
                (unsigned long long)c.spi_transactions, (unsigned long long)c.spi_bytes,
                (unsigned long long)(c.spi_busy_us / 1000), (unsigned long long)c.set_rx, (unsigned long long)c.set_tx,
                (unsigned long long)c.set_standby, (unsigned long long)c.set_sleep,
                (unsigned long long)c.set_mod_params, (unsigned long long)c.read_buffer,
                (unsigned long long)c.write_buffer);
    uint64_t ec, qc, ov;
    sim_overlap_get(&ec, &qc, &ov);
    std::printf("engine_calls=%llu enqueue_calls=%llu engine_enqueue_overlaps=%llu nested_same_thread=%llu stop_errors=%llu\n",
                (unsigned long long)ec, (unsigned long long)qc, (unsigned long long)ov,
                (unsigned long long)sim_overlap_nested_same_thread(), (unsigned long long)s.stop_errors.load());
    std::printf("air_delivered=%llu air_lost_not_in_rx=%llu dio1=%llu hal_wakeups=%llu usp_passes=%llu "
                "usp_no_sleep=%llu chip_mode=%d\n",
                (unsigned long long)c.frames_delivered, (unsigned long long)c.frames_lost_not_in_rx,
                (unsigned long long)c.dio1_raised, (unsigned long long)sim_hal_wakeups(),
                (unsigned long long)usp_thread_passes(), (unsigned long long)usp_thread_no_sleep_iterations(),
                (int)sim_chip_mode());
}

int arg_int(int argc, char** argv, int i, int def) { return (argc > i) ? std::atoi(argv[i]) : def; }

std::vector<uint8_t> frame_of(size_t n) {
    std::vector<uint8_t> f(n, 0x5A);
    return f;
}

// ---------------------------------------------------------------------------
// smoke: boot, receive, transmit, change profiles.  Everything should work.
// ---------------------------------------------------------------------------
int sc_smoke(RadioSim& r) {
    if (!r.startRadio(true)) return 2;
    Air air;
    air.start(20000, 60);
    k_sleep(K_MSEC(300));
    r.cmdSetRxProfile(1, 5);
    r.drain();
    k_sleep(K_MSEC(100));
    for (int i = 0; i < 5; i++) r.comQueuePush(frame_of(100));
    while (r.comQueueDepth() > 0) k_sleep(K_MSEC(10));
    r.drain();
    k_sleep(K_MSEC(200));
    r.cmdSetTxProfile(3);
    r.cmdSetRxProfile(3, 0);
    r.drain();
    k_sleep(K_MSEC(300));
    for (int i = 0; i < 3; i++) r.portRun();
    r.drain();
    air.join();
    print_stats(r);
    return 0;
}

// ---------------------------------------------------------------------------
// stale (claim A): flood frames; alternate SET_RX_PROFILE P0<->P1 N times.
// Count confirms whose frame was received on the OLD profile.
// ---------------------------------------------------------------------------
int sc_stale(RadioSim& r, int iters, int air_period_us) {
    if (!r.startRadio(false)) return 2;
    Air air;
    air.start(air_period_us, 32);
    k_sleep(K_MSEC(50));
    for (int i = 0; i < iters; i++) {
        const uint8_t target = (i % 2 == 0) ? 1 : 0;
        r.cmdSetRxProfile(target, 30);
        // Wait for the confirm (or give up after 200 ms): pending clears on confirm.
        for (int w = 0; w < 200 && r.policyHasPending(); w++) k_sleep(K_MSEC(1));
        r.drain();
    }
    air.join();
    r.drain();
    std::printf("stale_iters=%d\n", iters);
    print_stats(r);
    return 0;
}

// ---------------------------------------------------------------------------
// initfail (claim B): smtc_rac_open_radio fails; drive every handler.
// Built with -fsanitize=bounds the tasks[0xFF] read is reported on stderr.
// ---------------------------------------------------------------------------
int sc_initfail(RadioSim& r) {
    sim_rac_force_open_failure(true);
    const bool ok = r.startRadio(true);
    std::printf("startRadio_ok=%d\n", ok ? 1 : 0);
    r.cmdSetRxProfile(1, 5);
    r.cmdSetRxProfile(0, 0);
    r.cmdSetTxProfile(2);
    r.cmdTransmit(true);
    r.comQueuePush(frame_of(40));
    k_sleep(K_MSEC(50));
    r.cmdTransmit(false);
    r.cmdTransmit(false);
    r.portRun();
    r.cmdContinuousWave(0);
    r.drain();
    k_sleep(K_MSEC(100));
    r.drain();
    print_stats(r);
    return 0;
}

// ---------------------------------------------------------------------------
// race (claim E): maximum RAC traffic from both sides for `secs` seconds.
// Under TSan the report count is the measurement; otherwise the anomaly
// counters (cfg_failed, change_deferred, tx_failed) are.
// ---------------------------------------------------------------------------
int sc_race(RadioSim& r, int secs) {
    if (!r.startRadio(true)) return 2;
    Air air;
    air.start(700, 24);
    const int64_t end = MsgQueue::now_us() + static_cast<int64_t>(secs) * 1000000;
    int i = 0;
    while (MsgQueue::now_us() < end) {
        r.cmdSetRxProfile(static_cast<uint8_t>(i % 3), 2);
        if (i % 2 == 0) r.comQueuePush(frame_of(20));
        if (i % 5 == 0) r.portRun();
        if (i % 7 == 0) r.cmdSetTxProfile(static_cast<uint8_t>((i / 7) % 6));
        r.drain();
        i++;
    }
    air.join();
    r.drain();
    std::printf("race_rounds=%d\n", i);
    print_stats(r);
    return 0;
}

// ---------------------------------------------------------------------------
// busyloop (claim F): continuous RX armed, nothing else happening for `secs`.
// Engine passes per second and process CPU time tell whether the USP thread
// spins while the LOCK is held.
// ---------------------------------------------------------------------------
int sc_busyloop(RadioSim& r, int secs, int trigger) {
    if (!r.startRadio(false)) return 2;
    r.drain();
    // trigger: 0 = nothing after boot; 1 = one RX frame; 2 = one SET_RX_PROFILE (stop + re-arm)
    if (trigger == 1) {
        std::vector<uint8_t> f(24, 0x11);
        sim_air_deliver(f.data(), 24);
    } else if (trigger == 2) {
        r.cmdSetRxProfile(1, 0);
        r.drain();
    }
    k_sleep(K_MSEC(100));
    const uint64_t p0 = usp_thread_passes();
    const uint64_t n0 = usp_thread_no_sleep_iterations();
    const double c0 = cpu_seconds();
    k_sleep(K_SECONDS(secs));
    const double c1 = cpu_seconds();
    std::printf("busyloop_trigger=%d ", trigger);
    std::printf("busyloop_secs=%d usp_passes_delta=%llu usp_no_sleep_delta=%llu cpu_seconds=%.3f cpu_pct=%.1f\n",
                secs, (unsigned long long)(usp_thread_passes() - p0),
                (unsigned long long)(usp_thread_no_sleep_iterations() - n0), c1 - c0, 100.0 * (c1 - c0) / secs);
    print_stats(r);
    return 0;
}

// ---------------------------------------------------------------------------
// txcost: `n` frames, either one at a time (paced: next frame only after the
// previous episode completes) or saturated (all queued at once).  With
// SIM_SPI_HZ=125000 the SPI cost is modelled.  Measures the TX episode and
// re-arm durations and counts SPI bytes per op.
// ---------------------------------------------------------------------------
int sc_txcost(RadioSim& r, int n, bool saturated) {
    if (!r.startRadio(true)) return 2;
    r.drain();
    k_sleep(K_MSEC(50));
    sim_chip_stats_reset();
    const int64_t t0 = MsgQueue::now_us();
    if (saturated) {
        for (int i = 0; i < n; i++) r.comQueuePush(frame_of(252));
        while (r.comQueueDepth() > 0) k_sleep(K_MSEC(5));
        r.drain();
    } else {
        for (int i = 0; i < n; i++) {
            r.comQueuePush(frame_of(252));
            while (r.comQueueDepth() > 0) k_sleep(K_MSEC(1));
            r.drain();
        }
    }
    k_sleep(K_MSEC(50));
    r.drain();
    const int64_t t1 = MsgQueue::now_us();
    std::printf("txcost_n=%d saturated=%d wall_ms=%lld per_frame_ms=%.2f\n", n, saturated ? 1 : 0,
                (long long)((t1 - t0) / 1000), (t1 - t0) / 1000.0 / n);
    print_stats(r);
    return 0;
}

// ---------------------------------------------------------------------------
// rxlat (claim G): full-size frames every `period_ms` while the component is
// loaded with a TX stream (saturated ComQueue) and profile commands.
// Chip-side fifo_read latency and end-to-end rx_service latency are the
// measurements; rx_overwritten counts frames lost to late service.
// ---------------------------------------------------------------------------
int sc_rxlat(RadioSim& r, int period_ms, int secs, int tx_load) {
    if (!r.startRadio(true)) return 2;
    r.drain();
    k_sleep(K_MSEC(50));
    sim_chip_stats_reset();
    Air air;
    air.start(static_cast<uint32_t>(period_ms) * 1000, 252);
    const int64_t end = MsgQueue::now_us() + static_cast<int64_t>(secs) * 1000000;
    int i = 0;
    while (MsgQueue::now_us() < end) {
        if (tx_load == 1) {
            if (r.comQueueDepth() < 4) r.comQueuePush(frame_of(252));
            if (i % 20 == 0) r.cmdSetTxProfile(static_cast<uint8_t>(3 + (i / 20) % 3));
        } else if (tx_load == 2) {
            if (i % 30 == 0 && r.comQueueDepth() == 0) r.comQueuePush(frame_of(252));
            if (i % 120 == 0) r.cmdSetTxProfile(static_cast<uint8_t>(3 + (i / 120) % 3));
        }
        if (i % 200 == 0) r.portRun();  // 1 Hz rate group, as deployed
        k_sleep(K_MSEC(5));
        i++;
    }
    air.join();
    r.drain();
    k_sleep(K_MSEC(50));
    std::printf("rxlat_period_ms=%d secs=%d tx_load=%d\n", period_ms, secs, tx_load);
    print_stats(r);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <smoke|stale|initfail|race|busyloop|txcost|rxlat> [args]\n", argv[0]);
        return 1;
    }
    sim_thread_name("main");
    const char* hz = std::getenv("SIM_SPI_HZ");
    sim_chip_set_spi_hz(hz ? static_cast<uint32_t>(std::atoi(hz)) : 0);
    const char* air = std::getenv("SIM_TX_AIRTIME_MS");
    sim_chip_set_tx_airtime_ms(air ? static_cast<uint32_t>(std::atoi(air)) : 25);
    const char* pm = std::getenv("SIM_PROTECT_MODE");
    sim_hal_set_protect_mode(pm ? std::atoi(pm) : SIM_DEFAULT_PROTECT_MODE);

    RadioSim* r = make_radio_sim(SIM_OPTION_NAME);
    const std::string sc = argv[1];
    std::printf("option=%s usp=%s scenario=%s spi_hz=%s protect_mode=%d\n", SIM_OPTION_NAME, SIM_USP_VERSION,
                sc.c_str(), hz ? hz : "0", pm ? std::atoi(pm) : SIM_DEFAULT_PROTECT_MODE);
    int rc = 1;
    if (sc == "smoke") rc = sc_smoke(*r);
    else if (sc == "stale") rc = sc_stale(*r, arg_int(argc, argv, 2, 200), arg_int(argc, argv, 3, 1500));
    else if (sc == "initfail") rc = sc_initfail(*r);
    else if (sc == "race") rc = sc_race(*r, arg_int(argc, argv, 2, 5));
    else if (sc == "busyloop") rc = sc_busyloop(*r, arg_int(argc, argv, 2, 3), arg_int(argc, argv, 3, 0));
    else if (sc == "txcost") rc = sc_txcost(*r, arg_int(argc, argv, 2, 20), arg_int(argc, argv, 3, 0) != 0);
    else if (sc == "rxlat") rc = sc_rxlat(*r, arg_int(argc, argv, 2, 30), arg_int(argc, argv, 3, 5), arg_int(argc, argv, 4, 1));
    std::fflush(stdout);
    std::_Exit(rc);  // worker threads never join
}
