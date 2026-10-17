// #589 scenarios.  One binary per option variant (see Makefile); argv[1]
// picks the scenario.  Every line of output that starts with "RESULT " is
// machine-read by ../run.sh into results/summary.tsv.
#include "sim.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <thread>

#include <sys/resource.h>

#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"
#include "fprime-zephyr/Drv/UspRadio/ProfilePolicy.hpp"

#if defined(VARIANT_opt2) || defined(VARIANT_opt2v121)
#define OPT2 1
#include "RalSessionSt.hpp"
#include "fprime-zephyr/Drv/UspRadio/UspRadioComponentAc.hpp"
#else
#include "fprime-zephyr/Drv/UspRadio/RalSessionImpl.hpp"
#include "fprime-zephyr/Drv/UspRadio/UspRadio.hpp"
#endif

extern "C" {
#include "smtc_rac_api.h"
#include "radio_planner.h"
}

using namespace Zephyr;

#ifndef VARIANT_NAME
#if defined(VARIANT_opt1)
#define VARIANT_NAME "opt1"
#elif defined(VARIANT_opt1nomx)
#define VARIANT_NAME "opt1nomx"
#elif defined(VARIANT_opt3)
#define VARIANT_NAME "opt3"
#elif defined(VARIANT_opt1v121)
#define VARIANT_NAME "opt1v121"
#elif defined(VARIANT_opt2)
#define VARIANT_NAME "opt2"
#elif defined(VARIANT_opt2v121)
#define VARIANT_NAME "opt2v121"
#elif defined(VARIANT_opt1fix)
#define VARIANT_NAME "opt1fix"
#endif
#endif

static const char* g_scn = "";
static void result(const char* key, double v) {
    std::printf("RESULT\t%s\t%s\t%s\t%.3f\n", VARIANT_NAME, g_scn, key, v);
    std::fflush(stdout);
}
static void note(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::printf("NOTE\t%s\t%s\t", VARIANT_NAME, g_scn);
    std::vprintf(fmt, ap);
    std::printf("\n");
    va_end(ap);
    std::fflush(stdout);
}

static uint32_t sigOf(U8 idx) {
    const LinkProfile& p = LINK_PROFILE_TABLE[idx];
    return p.mod == ModKind::LORA ? SIG_LORA(p.lora.sf) : SIG_GFSK(p.gfsk.bitrate_bps);
}
static void msleep(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
template <class F>
static bool waitFor(F cond, int timeout_ms) {
    for (int i = 0; i < timeout_ms; i++) {
        if (cond()) return true;
        msleep(1);
    }
    return cond();
}

// ===========================================================================
// Option 2 component: the handlers of single-queue/UspRadio.cpp, verbatim
// except where the sketch elides a body (marked).
// ===========================================================================
#ifdef OPT2
class Opt2Radio final : public UspRadioComponentBase, public RadioSink {
  public:
    enum class Link : U8 { NO_CREDIT, CREDIT_OUT, FRAME_HELD };
    explicit Opt2Radio(const char* n) : UspRadioComponentBase(n) {}
    void configure(RalSessionSt& s) { m_session = &s; }
    std::atomic<int> engineKicks{0};

    bool startRadio(UspTransmitState initialTransmitState) {
        m_transmitState = initialTransmitState;
        if (m_session->init(*this) != 0) {
            this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
            return false;
        }
        rearmRxWhenIdle();
        if (initialTransmitState == UspTransmitState::ENABLED) {
            emitCredit();
        }
        return true;
    }
    // RadioSink
    void rxFrame(const uint8_t* data, std::size_t size, int16_t, int8_t) override {
        if (m_policy.frameReceived() == ProfilePolicy::Action::kConfirmRx) {
            this->log_ACTIVITY_LO_ProfileConfirmed(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
            this->tlmWrite_RxProfile(static_cast<LinkProfileId::T>(m_policy.rxProfile()));
        }
        // deliverRx (elided in sketch): raw mode, allocate + dataOut
        Fw::Buffer b = this->allocate_out(0, size);
        std::memcpy(b.getData(), data, size);
        ComCfg::FrameContext c;
        this->dataOut_out(0, b, c);
    }
    void txDone(int rc) override {
        if (rc != 0) this->log_WARNING_HI_SendFailed(static_cast<I32>(rc));
        finishFrame();
    }
    void postEngineKick() override {
        engineKicks++;
        this->postMsg("engineKick", PRIO_INTERNAL, [this] { m_session->onEngineKick(); });
    }

  protected:
    void emitCredit() {
        FW_ASSERT(m_link != Link::CREDIT_OUT);
        Fw::Success ok = Fw::Success::SUCCESS;
        this->comStatusOut_out(0, ok);
        m_link = Link::CREDIT_OUT;
    }
    bool txInFlight() const { return m_link == Link::FRAME_HELD; }
    void rearmRxWhenIdle() {
        if (txInFlight()) return;
        const U8 idx = m_policy.armedRxProfile();
        m_rxArmFailed = (m_session->armRx(LINK_PROFILE_TABLE[idx]) != 0);
        if (m_rxArmFailed) this->log_WARNING_HI_ConfigurationFailed(UspRadioDirection::RX);
    }
    void finishFrame() {
        FW_ASSERT(m_link == Link::FRAME_HELD);
        this->dataReturnOut_out(0, m_held, m_heldContext);
        emitCredit();
        rearmRxWhenIdle();
    }
    void run_handler(FwIndexType, U32) override {
        const uint64_t now = k_uptime_get();
        m_session->checkTxDeadline(now);
        if ((m_cwUntilMs != 0) && (now >= m_cwUntilMs)) {
            m_cwUntilMs = 0;
            rearmRxWhenIdle();
        }
        if (m_policy.tick(now) == ProfilePolicy::Action::kRevert) {
            this->log_WARNING_LO_ProfileReverted(static_cast<LinkProfileId::T>(m_policy.pendingRxProfile()),
                                                 static_cast<LinkProfileId::T>(m_policy.rxProfile()));
            rearmRxWhenIdle();
        } else if (m_rxArmFailed) {
            rearmRxWhenIdle();
        }
    }
    void dataIn_handler(FwIndexType, Fw::Buffer& data, const ComCfg::FrameContext& context) override {
        this->deferredTxPacket_internalInterfaceInvoke(data, context);
    }
    void dataReturnIn_handler(FwIndexType, Fw::Buffer&, const ComCfg::FrameContext&) override {}
    void enableTransmit_handler(FwIndexType) override {}
    void disableTransmit_handler(FwIndexType) override {}
    void deferredRxDone_internalInterfaceHandler() override {}
    void deferredTxPacket_internalInterfaceHandler(const Fw::Buffer& data, const ComCfg::FrameContext& context) override {
        FW_ASSERT(m_link == Link::CREDIT_OUT);
        m_link = Link::FRAME_HELD;
        m_held = data;
        m_heldContext = context;
        if (m_transmitState != UspTransmitState::ENABLED) {
            if (m_transmitState == UspTransmitState::DISABLING) m_transmitState = UspTransmitState::DISABLED;
            finishFrame();
            return;
        }
        m_txData = data.getData();  // RadioHead staging elided: raw mode
        m_txSize = static_cast<std::size_t>(data.getSize());
        const LinkProfile& p = LINK_PROFILE_TABLE[m_policy.txProfile()];
        const int rc = m_session->startTx(p, m_txData, m_txSize, k_uptime_get());
        if (rc != 0) {
            this->log_WARNING_HI_SendFailed(static_cast<I32>(rc));
            finishFrame();
        }
    }
    void deferredTransmitCmd_internalInterfaceHandler(const UspTransmitState& enabled) override {
        if (enabled == UspTransmitState::ENABLED) {
            if (m_link == Link::NO_CREDIT) emitCredit();
            m_transmitState = UspTransmitState::ENABLED;
            return;
        }
        m_transmitState = txInFlight() ? UspTransmitState::DISABLING : UspTransmitState::DISABLED;
        rearmRxWhenIdle();
    }
    void deferredSetTxProfile_internalInterfaceHandler(const LinkProfileId& profile) override {
        if (m_policy.setTxProfile(static_cast<U8>(profile.e)) == ProfilePolicy::Action::kInvalidIndex) {
            this->log_WARNING_HI_InvalidProfile(profile);
            return;
        }
        this->log_ACTIVITY_HI_ProfileChanged(UspRadioDirection::TX, profile);
    }
    void deferredSetRxProfile_internalInterfaceHandler(const LinkProfileId& profile, U16 revert_s) override {
        const ProfilePolicy::Action action = m_policy.setRxProfile(static_cast<U8>(profile.e), revert_s, k_uptime_get());
        if (action == ProfilePolicy::Action::kInvalidIndex) {
            this->log_WARNING_HI_InvalidProfile(profile);
            return;
        }
        rearmRxWhenIdle();
        this->log_ACTIVITY_HI_ProfileChanged(UspRadioDirection::RX, profile);
        this->tlmWrite_RxProfile(profile);
    }
    void deferredContinuousWave_internalInterfaceHandler(U16) override {}
    void TRANSMIT_cmdHandler(FwOpcodeType o, U32 s, UspTransmitState e) override {
        this->deferredTransmitCmd_internalInterfaceInvoke(e);
        this->cmdResponse_out(o, s, Fw::CmdResponse::OK);
    }
    void CONTINUOUS_WAVE_cmdHandler(FwOpcodeType o, U32 s, U16) override { this->cmdResponse_out(o, s, Fw::CmdResponse::OK); }
    void SET_TX_PROFILE_cmdHandler(FwOpcodeType o, U32 s, LinkProfileId p) override {
        this->deferredSetTxProfile_internalInterfaceInvoke(p);
        this->cmdResponse_out(o, s, Fw::CmdResponse::OK);
    }
    void SET_RX_PROFILE_cmdHandler(FwOpcodeType o, U32 s, LinkProfileId p, U16 r) override {
        this->deferredSetRxProfile_internalInterfaceInvoke(p, r);
        this->cmdResponse_out(o, s, Fw::CmdResponse::OK);
    }

  public:
    RalSessionSt* m_session = nullptr;
    ProfilePolicy m_policy;
    UspTransmitState m_transmitState = UspTransmitState::DISABLED;
    Link m_link = Link::NO_CREDIT;
    bool m_rxArmFailed = false;
    uint64_t m_cwUntilMs = 0;
    Fw::Buffer m_held;
    ComCfg::FrameContext m_heldContext;
    const U8* m_txData = nullptr;
    std::size_t m_txSize = 0;
};
using Radio = Opt2Radio;
using Session = RalSessionSt;
#else
using Radio = UspRadio;
using Session = RalSessionImpl;
#endif

// ===========================================================================
// Rig: chip + HAL + (USP thread) + session + component
// ===========================================================================
static void hold_report(Radio& r) {
    k_hold_stats_t h{};
    r.onComponentThread([&] { h = *k_hold_stats_self(); });
    result("comp_mutex_holds", static_cast<double>(h.holds));
    result("comp_mutex_max_hold_ral_calls", static_cast<double>(h.max_hold_work));
    result("comp_mutex_max_hold_us_host", static_cast<double>(h.max_hold_us));
    result("usp_mutex_holds", static_cast<double>(g_usp_hold.holds));
    result("usp_mutex_max_hold_ral_calls", static_cast<double>(g_usp_hold.max_hold_work));
}

struct Rig {
    Session session{437400000u, 0};
    Radio radio{"uspRadio"};
    bool started = false;
    explicit Rig(bool startUspThread = true) {
        k_hold_probe = chip_ral_calls_self;
        chip_init();
#ifndef OPT2
        if (startUspThread) hal_fake_start_usp_thread();
#else
        (void)startUspThread;
#endif
        radio.configure(session);
    }
    bool start(UspTransmitState s = UspTransmitState::DISABLED) {
        radio.start();  // F´ startTasks precedes startRadio in PCR topology
        bool ok = false;
        // PCR ReferenceDeploymentTopology.cpp calls startRadio on main after startTasks;
        // option 2 must run it on the component thread (it runs the engine).
#ifdef OPT2
        radio.onComponentThread([&] { ok = radio.startRadio(s); });
#else
        ok = radio.startRadio(s);
#endif
        started = ok;
        return ok;
    }
};

// ===========================================================================
// A: stale-frame false confirm
// ===========================================================================
static int scn_stale(bool inject, int trials) {
    Rig rig;
    if (!rig.start()) {
        note("startRadio failed");
        return 1;
    }
    const U8 OLD = 0, NEW = 3;  // P0 LoRa SF8 -> P3 GFSK 38k4
    std::atomic<bool> armed{false};
    int falseConfirms = 0, reverts = 0, oldFramesHarvested = 0, oldFramesLost = 0;
    for (int t = 0; t < trials; t++) {
        const uint64_t rx0 = g_chip.frames_received;
        const int conf0 = rig.radio.count("EVR ProfileConfirmed");
        const int rev0 = rig.radio.count("EVR ProfileReverted");
        const int out0 = rig.radio.count("dataOut");
        armed = inject;
        // The frame lands while deferredSetRxProfile sits in the queue: the
        // ground was still talking on the old profile when the command was
        // dispatched (retry, ping, second command of a burst).
        rig.radio.afterPost = [&](const char* name) {
            if (!armed || std::strcmp(name, "deferredSetRxProfile") != 0) return;
            armed = false;
            uint8_t f[40];
            std::memset(f, 0xA0 + t % 16, sizeof f);
            chip_air_frame(sigOf(OLD), f, sizeof f);
            // let the DIO1 -> engine path run before the handler does
            msleep(5);
        };
        rig.radio.cmd_SET_RX_PROFILE(static_cast<LinkProfileId::T>(NEW), 1);
        rig.radio.drain();
        msleep(20);
        rig.radio.drain();
        const bool confirmed = rig.radio.count("EVR ProfileConfirmed") > conf0;
        // tick past the 1 s revert deadline
        msleep(1100);
        rig.radio.port_run();
        rig.radio.drain();
        const bool reverted = rig.radio.count("EVR ProfileReverted") > rev0;
        const bool harvested = rig.radio.count("dataOut") > out0;
        if (inject && g_chip.frames_received > rx0) (harvested ? oldFramesHarvested : oldFramesLost)++;
        falseConfirms += (confirmed && inject) ? 1 : 0;
        reverts += reverted ? 1 : 0;
        // back to OLD for the next trial (revert does this; force if confirmed)
        if (!reverted) {
            rig.radio.cmd_SET_RX_PROFILE(static_cast<LinkProfileId::T>(OLD), 0);
            rig.radio.drain();
            msleep(5);
            rig.radio.drain();
        }
        rig.radio.drain();
    }
    result(inject ? "false_confirms" : "confirms_without_frame", falseConfirms);
    result("reverts", reverts);
    result("trials", trials);
    result("old_frames_delivered", oldFramesHarvested);
    result("old_frames_dropped", oldFramesLost);
    result("chip_rx_sig_is_old_profile_at_end", g_chip.rx_sig == sigOf(OLD));
    return 0;
}

// Positive control for A: a frame on the NEW profile after the re-arm must
// confirm it (and disarm the revert).
static int scn_confirm_new(int trials) {
    Rig rig;
    if (!rig.start()) return 1;
    int confirms = 0, reverts = 0;
    for (int t = 0; t < trials; t++) {
        const U8 NEW = static_cast<U8>(3 + t % 3);
        const int c0 = rig.radio.count("EVR ProfileConfirmed"), r0 = rig.radio.count("EVR ProfileReverted");
        rig.radio.cmd_SET_RX_PROFILE(static_cast<LinkProfileId::T>(NEW), 1);
        rig.radio.drain();
        msleep(10);
        rig.radio.drain();
        uint8_t f[30] = {0x11};
        chip_air_frame(sigOf(NEW), f, sizeof f);
        msleep(20);
        rig.radio.drain();
        msleep(1100);
        rig.radio.port_run();
        rig.radio.drain();
        confirms += rig.radio.count("EVR ProfileConfirmed") > c0;
        reverts += rig.radio.count("EVR ProfileReverted") > r0;
        rig.radio.cmd_SET_RX_PROFILE(static_cast<LinkProfileId::T>(0), 0);
        rig.radio.drain();
        msleep(10);
        rig.radio.drain();
    }
    result("trials", trials);
    result("confirms_on_new_profile_frame", confirms);
    result("reverts", reverts);
    return 0;
}

// ===========================================================================
// B: failed init
// ===========================================================================
static int scn_init_fail() {
    // Part 1: can smtc_rac_open_radio() return RAC_INVALID_RADIO_ID?  Take the
    // RAC_HIGH_PRIORITY hook first, then run the real startRadio().
    Rig rig;
#ifndef OPT2
    zephyr_usp_initialization_wait();
#endif
    smtc_rac_init();
    const uint8_t first = smtc_rac_open_radio(RAC_HIGH_PRIORITY);
    note("pre-opened RAC_HIGH_PRIORITY -> id %u; now startRadio()", first);
    const bool ok = rig.start();
    result("startRadio_ok_with_hook_pre_taken", ok);
    result("panics_part1", g_panic_count);
    // Part 2: a session whose init never ran (m_radio_id == RAC_INVALID_RADIO_ID),
    // driven through every RalSession entry UspRadio calls after a failed init.
    // SIM_PANIC_CONTINUE=1 lets each panic return, to see what runs next.
    Session dead{437400000u, 0};
    const char* which = std::getenv("SIM_DEAD_OP");
    const std::string op = which ? which : "stop";
    int rc = 0;
#ifndef OPT2
    if (op == "stop") rc = dead.stopRadio();
    else if (op == "apply") rc = dead.applyProfile(LINK_PROFILE_TABLE[0]);
    else if (op == "rx") rc = dead.startReceive();
    else if (op == "tx") { uint8_t b[4] = {1, 2, 3, 4}; rc = dead.transmitPacket(b, 4); }
#else
    if (op == "stop") rc = dead.stop();
    else if (op == "apply" || op == "rx") rc = dead.armRx(LINK_PROFILE_TABLE[0]);
    else if (op == "tx") { uint8_t b[4] = {1, 2, 3, 4}; rc = dead.startTx(LINK_PROFILE_TABLE[0], b, 4, 0); }
#endif
    result(("dead_session_rc_" + op).c_str(), rc);
    result(("dead_session_panics_" + op).c_str(), g_panic_count);
    note("dead-session op %s: last panic at %s", op.c_str(), g_panic_where);
    return 0;
}

// ===========================================================================
// E: concurrency stress (run under TSan for the race verdict)
// ===========================================================================
static int scn_stress(int seconds) {
    Rig rig;
    if (!rig.start()) return 1;
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> air{0};
    // Air: ground transmitting on a random profile every 1-4 ms.
    std::thread airT([&] {
        std::mt19937 rng(1);
        uint8_t f[64];
        while (!stop) {
            std::memset(f, static_cast<int>(rng() & 0xFF), sizeof f);
            chip_air_frame(sigOf(static_cast<U8>(rng() % LINK_PROFILE_COUNT)), f, sizeof f);
            air++;
            std::this_thread::sleep_for(std::chrono::microseconds(1000 + rng() % 3000));
        }
    });
    // ComQueue model: one frame per SUCCESS credit.
    std::atomic<int> credits{0};
    rig.radio.onComStatus = [&](Fw::Success s) {
        if (s.e == Fw::Success::SUCCESS) credits++;
    };
    g_chip.tx_airtime_us = 3000;
    rig.radio.cmd_TRANSMIT(UspTransmitState::ENABLED);
    const int comDelayMs = std::getenv("SIM_COMDELAY_MS") ? std::atoi(std::getenv("SIM_COMDELAY_MS")) : 10;
    std::thread down([&] {
        static uint8_t frames[4][128];
        int k = 0;
        while (!stop) {
            // ComDelay (PCR topology): the status is released on a rate-group
            // tick, never back-to-back.  10 ms ticks here (real: >=100 ms).
            std::this_thread::sleep_for(std::chrono::milliseconds(comDelayMs));
            if (credits > 0) {
                credits--;
                Fw::Buffer b(frames[k++ % 4], 100);
                rig.radio.port_dataIn(b);
            }
        }
    });
    std::mt19937 rng(2);
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    int cmds = 0;
    while (std::chrono::steady_clock::now() < end) {
        switch (rng() % 4) {
            case 0: rig.radio.cmd_SET_RX_PROFILE(static_cast<LinkProfileId::T>(rng() % LINK_PROFILE_COUNT), 1); break;
            case 1: rig.radio.cmd_SET_TX_PROFILE(static_cast<LinkProfileId::T>(rng() % LINK_PROFILE_COUNT)); break;
            default: rig.radio.port_run(); break;
        }
        cmds++;
        msleep(20 + static_cast<int>(rng() % 40));
    }
    stop = true;
    airT.join();
    down.join();
    rig.radio.drain();
    result("seconds", seconds);
    result("cmds", cmds);
    result("air_frames", static_cast<double>(air));
    result("frames_received_by_chip", static_cast<double>(g_chip.frames_received));
    result("rx_duty_cycle", chip_rx_seconds() / seconds);
    result("dataOut", rig.radio.count("dataOut"));
    result("tx_started", static_cast<double>(g_chip.tx_count));
    result("tx_done", static_cast<double>(g_chip.tx_done_count));
    result("send_failed_evr", rig.radio.count("EVR SendFailed"));
    result("config_failed_evr", rig.radio.count("EVR ConfigurationFailed"));
    result("change_deferred_evr", rig.radio.count("EVR ProfileChangeDeferred"));
    result("ral_calls_overlapping_2_threads", static_cast<double>(g_chip.overlapping_calls));
    result("set_tx_while_chip_in_rx", static_cast<double>(g_chip.tx_from_rx));
    result("panics", g_panic_count);
    result("queue_high_water", rig.radio.queueHighWater);
    std::fflush(stdout);
    std::_Exit(0);  // skip static destructors racing the still-running USP thread
}

// ===========================================================================
// F: engine passes while RX is armed (v1.2.1 busy loop)
// ===========================================================================
static double cpu_s() {
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    return ru.ru_utime.tv_sec + ru.ru_stime.tv_sec + (ru.ru_utime.tv_usec + ru.ru_stime.tv_usec) / 1e6;
}
static int scn_busyloop(int seconds) {
    Rig rig;
    if (!rig.start()) return 1;
    msleep(200);
    const uint64_t p0 = g_engine_passes, s0 = g_engine_spins;
    const double c0 = cpu_s();
    const auto w0 = std::chrono::steady_clock::now();
    msleep(seconds * 1000);
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
    const double cpu = cpu_s() - c0;
    result("chip_in_rx", g_chip.mode == CHIP_RX);
    result("engine_passes_per_s_idle_rx", (g_engine_passes - p0) / wall);
    result("engine_spins_per_s_idle_rx", (g_engine_spins - s0) / wall);
    result("process_cpu_fraction_idle_rx", cpu / wall);
#ifdef OPT2
    result("engine_kicks", rig.radio.engineKicks.load());
#endif
    // Phase 2: one frame arrives (DIO1 -> engine pass with the RX LOCK current),
    // then the link goes quiet again.  Same measurement.
    {
        uint8_t f[20] = {0x55};
        chip_air_frame(sigOf(0), f, sizeof f);
        msleep(100);
        const uint64_t p1 = g_engine_passes, s1 = g_engine_spins;
        const double c1 = cpu_s();
        const auto w1 = std::chrono::steady_clock::now();
        msleep(seconds * 1000);
        const double wall1 = std::chrono::duration<double>(std::chrono::steady_clock::now() - w1).count();
        result("engine_passes_per_s_after_1_frame", (g_engine_passes - p1) / wall1);
        result("engine_spins_per_s_after_1_frame", (g_engine_spins - s1) / wall1);
        result("process_cpu_fraction_after_1_frame", (cpu_s() - c1) / wall1);
    }
    // Does RX still work?  10 frames on the armed profile.
    const int out0 = rig.radio.count("dataOut");
    for (int i = 0; i < 10; i++) {
        uint8_t f[20] = {static_cast<uint8_t>(i)};
        chip_air_frame(sigOf(0), f, sizeof f);
        msleep(30);
    }
    rig.radio.drain();
    result("rx_frames_delivered_of_10", rig.radio.count("dataOut") - out0);
    std::fflush(stdout);
    std::_Exit(0);
}

// ===========================================================================
// Re-arm cost / TX saturation (structure: waits and sleeps per frame)
// ===========================================================================
static int scn_txsat(int frames) {
    Rig rig;
    if (!rig.start()) return 1;
    g_chip.tx_airtime_us = 25000;  // ~P5 252 B frame
    std::atomic<int> credits{0};
    rig.radio.onComStatus = [&](Fw::Success s) {
        if (s.e == Fw::Success::SUCCESS) credits++;
    };
    rig.radio.onComponentThread([] { k_thread_stats_reset_self(); });
    const uint64_t arms0 = g_chip.rx_arms, ral0 = g_chip.ral_calls;
    rig.radio.cmd_TRANSMIT(UspTransmitState::ENABLED);
    static uint8_t buf[252];
    const auto t0 = std::chrono::steady_clock::now();
    int sent = 0;
    // ComQueue answers a credit within ~1 ms (sync port chain)
    while (sent < frames) {
        if (credits > 0) {
            credits--;
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            Fw::Buffer b(buf, sizeof buf);
            rig.radio.port_dataIn(b);
            sent++;
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
    waitFor([&] { return g_chip.tx_done_count >= static_cast<uint64_t>(frames); }, 20000);
    rig.radio.drain();
    msleep(50);
    rig.radio.drain();
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    k_thread_stats_t st{};
    rig.radio.onComponentThread([&] { st = *k_thread_stats_self(); });
    result("frames", frames);
    result("tx_done", static_cast<double>(g_chip.tx_done_count));
    result("ms_per_frame", wall * 1000.0 / frames);
    result("overhead_ms_per_frame_beyond_airtime", wall * 1000.0 / frames - 25.0);
    result("rx_duty_cycle", chip_rx_seconds() / wall);
    result("rx_arms_per_frame", static_cast<double>(g_chip.rx_arms - arms0) / frames);
    result("ral_calls_per_frame", static_cast<double>(g_chip.ral_calls - ral0) / frames);
    result("comp_thread_blocking_sem_waits_per_frame", static_cast<double>(st.sem_waits) / frames);
    result("comp_thread_sem_wait_ms_per_frame", st.sem_wait_us / 1000.0 / frames);
    result("comp_thread_k_sleeps_per_frame", static_cast<double>(st.sleeps) / frames);
    result("comp_thread_k_sleep_ms_per_frame", st.sleep_us / 1000.0 / frames);
    result("set_tx_while_chip_in_rx", static_cast<double>(g_chip.tx_from_rx));
    hold_report(rig.radio);
    std::fflush(stdout);
    std::_Exit(0);
}

// stop -> re-arm cycle cost via the same-profile SET_RX_PROFILE lever
static int scn_rearm(int cycles) {
    Rig rig;
    if (!rig.start()) return 1;
    uint64_t t_start = 0, total_us = 0, max_us = 0;
    rig.radio.beforeDispatch = [&](const char* n) {
        if (std::strcmp(n, "deferredSetRxProfile") == 0) t_start = k_uptime_us();
    };
    rig.radio.onComponentThread([] { k_thread_stats_reset_self(); });
    const uint64_t ral0 = g_chip.ral_calls;
    for (int i = 0; i < cycles; i++) {
        rig.radio.cmd_SET_RX_PROFILE(static_cast<LinkProfileId::T>(0), 0);
        rig.radio.drain();
        rig.radio.onComponentThread([&] {
            const uint64_t d = k_uptime_us() - t_start;
            total_us += d;
            if (d > max_us) max_us = d;
        });
    }
    k_thread_stats_t st{};
    rig.radio.onComponentThread([&] { st = *k_thread_stats_self(); });
    result("cycles", cycles);
    result("host_us_per_stop_rearm_mean", static_cast<double>(total_us) / cycles);
    result("host_us_per_stop_rearm_max", static_cast<double>(max_us));
    result("cross_thread_sem_waits_per_cycle", static_cast<double>(st.sem_waits) / cycles);
    result("k_sleeps_per_cycle", static_cast<double>(st.sleeps) / cycles);
    result("ral_calls_per_cycle", static_cast<double>(g_chip.ral_calls - ral0) / cycles);
    result("chip_in_rx_at_end", g_chip.mode == CHIP_RX);
    hold_report(rig.radio);
    std::fflush(stdout);
    std::_Exit(0);
}

// ComDelay bypassed (divider removed): credits come back immediately, so the
// downlink saturates the radio.  1 Hz async run ticks (rateGroup1Hz) must still
// be served; a queue that never serves priority 0 fills and FW_ASSERTs.
static int scn_starve(int seconds) {
    Rig rig;
    if (!rig.start()) return 1;
    g_chip.tx_airtime_us = 25000;
    std::atomic<int> credits{0}, runs{0};
    rig.radio.onComStatus = [&](Fw::Success s) {
        if (s.e == Fw::Success::SUCCESS) credits++;
    };
    rig.radio.beforeDispatch = [&](const char* n) {
        if (std::strcmp(n, "run") == 0) runs++;
    };
    rig.radio.cmd_TRANSMIT(UspTransmitState::ENABLED);
    std::atomic<bool> stop{false};
    std::thread down([&] {
        static uint8_t buf[252];
        while (!stop) {
            if (credits > 0) {
                credits--;
                Fw::Buffer b(buf, sizeof buf);
                rig.radio.port_dataIn(b);
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    });
    int posted = 0;
    for (int i = 0; i < seconds; i++) {
        msleep(1000);
        rig.radio.port_run();
        posted++;
        note("t=%ds run posted=%d served=%d queue_high_water=%d tx_done=%llu", i + 1, posted, runs.load(),
             rig.radio.queueHighWater, static_cast<unsigned long long>(g_chip.tx_done_count));
    }
    stop = true;
    down.join();
    result("run_posted", posted);
    result("run_served", runs.load());
    result("tx_done", static_cast<double>(g_chip.tx_done_count));
    result("queue_high_water", rig.radio.queueHighWater);
    std::fflush(stdout);
    std::_Exit(0);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s stale|stale_ctrl|init_fail|stress|busyloop|txsat|rearm [n]\n", argv[0]);
        return 2;
    }
    g_scn = argv[1];
    const int n = argc > 2 ? std::atoi(argv[2]) : 0;
    const std::string s = argv[1];
    int rc = 2;
    if (s == "stale") rc = scn_stale(true, n ? n : 5);
    else if (s == "stale_ctrl") rc = scn_stale(false, n ? n : 3);
    else if (s == "init_fail") rc = scn_init_fail();
    else if (s == "confirm_new") rc = scn_confirm_new(n ? n : 3);
    else if (s == "stress") rc = scn_stress(n ? n : 5);
    else if (s == "busyloop") rc = scn_busyloop(n ? n : 2);
    else if (s == "txsat") rc = scn_txsat(n ? n : 40);
    else if (s == "rearm") rc = scn_rearm(n ? n : 50);
    else if (s == "starve") rc = scn_starve(n ? n : 12);
    std::fflush(stdout);
    std::_Exit(rc);
}
