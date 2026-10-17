// PROTOTYPE #589 (throwaway, not built).  See RalSessionSt.cpp.
#pragma once
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"
#include <zephyr/kernel.h>
#include <zephyr/usp/smtc_sw_platform_helper.h>
#include <atomic>
extern "C" {
#include <smtc_rac_api.h>
#include <ralf.h>
#include <ral.h>
}

namespace Zephyr {

//! What the session reports back.  Called only from inside pump(), on the
//! component thread, except postEngineKick() (kick pump thread).
struct RadioSink {
    virtual void rxFrame(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr) = 0;
    virtual void txDone(int rc) = 0;
    virtual void postEngineKick() = 0;
};

//! A radio id that exists only after smtc_rac_open_radio() succeeded.
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

    std::atomic<bool> m_kickQueued{false};  // the only cross-thread field
    RadioSink*        m_sink = nullptr;

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
    int  applyLoRa(const LoRaParams& p);  // unchanged
    int  applyGfsk(const GfskParams& p);  // unchanged
    static void onPreRx(void);
    static void onPreTx(void);
    static void onPreCw(void);
    static void onPost(rp_status_t status);

    uint32_t           m_freq_hz;
    int8_t             m_tx_power_dbm;
    RadioId            m_radio{RAC_INVALID_RADIO_ID};
    ral_t*             m_ral = nullptr;
    State              m_state = State::IDLE;
    const LinkProfile* m_profile = nullptr;
    struct { const uint8_t* data; std::size_t size; } m_tx{};
    int                m_lastError = 0;
    bool               m_txTimedOut = false;
    uint64_t           m_txDeadlineMs = 0;
    ral_pkt_type_t        m_pktType;
    ral_lora_pkt_params_t m_loraPktParams;
    ral_gfsk_pkt_params_t m_gfskPktParams;
    struct k_thread    m_pump;
    static RalSessionSt* s_instance;
};

}  // namespace Zephyr
