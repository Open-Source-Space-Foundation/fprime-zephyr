// OPTION 2 (single queue): RalSessionSt, completed from PROTOTYPE-589-concurrency/single-queue/RalSessionSt.hpp.
// Deviations from the sketch are marked "ARENA-c3:" in RalSessionSt.cpp.
#pragma once
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"
#include <zephyr/kernel.h>
#include <zephyr/usp/smtc_zephyr_usp_api.h>
#include <zephyr/lorawan_lbm/lorawan_hal_init.h>
#include <atomic>
#include <cstddef>
#include <cinttypes>
extern "C" {
#include <smtc_rac_api.h>
#include <ralf.h>
#include <ralf_sx126x.h>
#include <ral.h>
#include <ral_defs.h>
#include <radio_planner.h>
}

namespace Zephyr {

struct RadioSink {
    virtual void rxFrame(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr) = 0;
    virtual void txDone(int rc) = 0;
    virtual void postEngineKick() = 0;
};

struct RadioId { uint8_t id; };

class RalSessionSt {
  public:
    RalSessionSt(uint32_t freq_hz, int8_t tx_power_dbm);
    int  init(RadioSink& sink);
    void onEngineKick();
    int  armRx(const LinkProfile& profile);
    int  startTx(const LinkProfile& profile, const uint8_t* data, std::size_t size, uint64_t nowMs);
    int  startCw();
    int  stop();
    void checkTxDeadline(uint64_t nowMs);
    bool ready() const { return m_ral != nullptr; }

    std::atomic<bool> m_kickQueued{false};
    RadioSink*        m_sink = nullptr;
    // instrumentation (sim only)
    long m_enginePasses = 0, m_kicks = 0, m_kicksCoalesced = 0;
    int64_t m_maxPassUs = 0;

  private:
    enum class State : uint8_t { IDLE, RX, TX, CW };
    static constexpr int      ENGINE_PASSES_MAX   = 4;
    static constexpr uint64_t TX_DONE_DEADLINE_MS = 10000;
    static constexpr int      KICK_PUMP_PRIO      = 1;

    void pump();
    int  lock(State next, void (*pre)(void));
    bool applyInWindow();
    void quiesce();
    void restorePktParams();
    void programTx();
    void harvestRx();
    int  applyLoRa(const LoRaParams& p);
    int  applyGfsk(const GfskParams& p);
    static void onPreRx(void);
    static void onPreTx(void);
    static void onPreCw(void);
    static void onPost(rp_status_t status);
    friend void kickPump(void*, void*, void*);

    uint32_t           m_freq_hz;
    int8_t             m_tx_power_dbm;
    RadioId            m_radio{RAC_INVALID_RADIO_ID};
    ral_t*             m_ral = nullptr;
    State              m_state = State::IDLE;
    const LinkProfile* m_profile = nullptr;
    struct { const uint8_t* data; std::size_t size; } m_tx{};
    int                m_lastError = 0;
    int                m_txResult = 0;
    bool               m_txTimedOut = false;
    uint64_t           m_txDeadlineMs = 0;
    ral_pkt_type_t        m_pktType;
    ral_lora_pkt_params_t m_loraPktParams;
    ral_gfsk_pkt_params_t m_gfskPktParams;
    struct k_thread    m_pump;
    static RalSessionSt* s_instance;
};

}  // namespace Zephyr
