// Common rig: builds the real component stack for the selected option and exposes observation hooks.
#pragma once
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "zsim.h"
#include "chip.h"
#include "fprime-zephyr/Drv/UspRadio/UspRadioComponentAc.hpp"
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"

#ifndef OPT
#define OPT 1
#endif
#if OPT == 2
#include "UspRadioSt.hpp"
#include "RalSessionSt.hpp"
using RadioT = Zephyr::UspRadioSt;
using SessionT = Zephyr::RalSessionSt;
#else
#include "fprime-zephyr/Drv/UspRadio/UspRadio.hpp"
#include "fprime-zephyr/Drv/UspRadio/RalSessionImpl.hpp"
using RadioT = Zephyr::UspRadio;
using SessionT = Zephyr::RalSessionImpl;
#endif

extern "C" {
extern int g_panic_count;
extern char g_panic_msg[];
}

struct EvRec { int64_t t; std::string name; long a, b; };
struct RxRec { int64_t t; uint32_t tag; int prof; };
struct Rig {
    SessionT* session = nullptr;
    RadioT* radio = nullptr;
    std::vector<EvRec> events;
    std::vector<RxRec> rx;
    std::vector<int64_t> comStatus;
    std::vector<int64_t> txReturned;
    int credit = 0;               // outstanding comStatus SUCCESS credits (ComQueue sends only after one)
    long txSent = 0;
    bool sendTx(int len, uint32_t tag = 0);  // returns false if no credit
    void build();                 // construct + wire hooks (call from the main sim thread)
    bool boot(bool txEnabled);    // startRadio (call from the main sim thread, after threads started)
    int count(const char* ev) const { int n = 0; for (auto& e : events) if (e.name == ev) n++; return n; }
};
extern Rig g_rig;

// background threads of the real deployment (priorities from instances.fpp)
struct BgLoad {
    int rg50_us = 0, rg10_us = 0, rg1_us = 0;  // CPU burst per cycle
    int mm_us = 0;
};
void bg_start(const BgLoad& l);
struct BgStat { const char* name; long cycles; int64_t max_late_us; int64_t sum_late_us; };
extern BgStat g_bgstat[5];
void rig_threads_and_run(void (*main_fn)(void*), int64_t t_end_us, const char* name = "main");
static inline int64_t env_i64(const char* n, int64_t d);
void print_run_summary(FILE* f);

static inline int64_t env_i64(const char* n, int64_t d) { const char* v = getenv(n); return v ? atoll(v) : d; }
static inline double env_f64(const char* n, double d) { const char* v = getenv(n); return v ? atof(v) : d; }
// Stream of frames from an external transmitter: profile p, len bytes, every period_us, k_from..k_to (tags tag0+k)
static inline void inject_stream(int64_t t0_us, int64_t period_us, int count, int profile, int len, uint32_t tag0) {
    for (int k = 0; k < count; k++) chip_inject_frame(t0_us + k * period_us, profile, len, tag0 + k);
}
// Apply env overrides of the chip model (CHIP_SPI_HZ, CHIP_TCXO_MS, ...)
static inline void apply_chip_env() {
    chip_cfg.spi_hz = (int)env_i64("CHIP_SPI_HZ", chip_cfg.spi_hz);
    chip_cfg.tcxo_ms = (int)env_i64("CHIP_TCXO_MS", chip_cfg.tcxo_ms);
    chip_cfg.irq_latency_us = (int)env_i64("CHIP_IRQ_US", chip_cfg.irq_latency_us);
    chip_cfg.spi_cpu_us = (int)env_i64("CHIP_SPI_CPU_US", chip_cfg.spi_cpu_us);
    chip_cfg.spi_poll = (int)env_i64("CHIP_SPI_POLL", chip_cfg.spi_poll);
    chip_cfg.wake_busy_us = (int)env_i64("CHIP_WAKE_US", chip_cfg.wake_busy_us);
    chip_cfg.mode_busy_us = (int)env_i64("CHIP_MODE_US", chip_cfg.mode_busy_us);
}
