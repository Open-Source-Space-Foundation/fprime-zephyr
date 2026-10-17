// Option 2 session, completed from the prototype sketch
// PROTOTYPE-589-concurrency/single-queue/RalSessionSt.{hpp,cpp} so that it
// compiles and runs against the real smtc_rac / radio planner.  Differences
// from the sketch are marked "SKETCH:" in the .cpp.
#pragma once
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"
#include <zephyr/kernel.h>
#include <atomic>
#include <cstddef>
#include <cstdint>
extern "C" {
#include <smtc_rac_api.h>
#include <ralf.h>
#include <ral.h>
}

namespace Zephyr {

struct RadioSink {
    virtual ~RadioSink() = default;
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
    bool txInFlight() const { return m_state == State::TX; }

    std::atomic<bool> m_kickQueued{false};
    RadioSink*        m_sink = nullptr;
    uint64_t          m_passes = 0;        // engine passes run from this thread
    uint64_t          m_boundedExits = 0;  // pump() hit ENGINE_PASSES_MAX

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
    ral_pkt_type_t        m_pktType = RAL_PKT_TYPE_LORA;
    ral_lora_pkt_params_t m_loraPktParams{};
    ral_gfsk_pkt_params_t m_gfskPktParams{};
    struct k_thread    m_pump{};
    static RalSessionSt* s_instance;
};

}  // namespace Zephyr
