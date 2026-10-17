// Fake of the FPP-generated UspRadioComponentBase for the host simulation.
//
// What is real: the handler call graph, the priority queue semantics (higher priority value dequeued
// first, FIFO inside a priority; priorities taken from UspRadio.fpp: internal ports 10, run/commands 0),
// the async/sync split of each port (dataIn/enableTransmit/disableTransmit/dataReturnIn are sync input
// ports, so they run on the CALLER's thread; run and the commands are async).
// What is modelled: per-call CPU costs (zs_burn) for events/telemetry/ports.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>
#include <atomic>
#include <cinttypes>
#include <zephyr/kernel.h>

#define FW_ASSERT(c, ...) do { if (!(c)) { fprintf(stderr, "FW_ASSERT failed %s:%d: %s\n", __FILE__, __LINE__, #c); abort(); } } while (0)

using U8 = uint8_t; using U16 = uint16_t; using U32 = uint32_t; using U64 = uint64_t;
using I8 = int8_t; using I16 = int16_t; using I32 = int32_t; using I64 = int64_t;
using FwSizeType = uint64_t; using FwIndexType = int32_t; using FwOpcodeType = uint32_t;

namespace Fw {
struct Success { enum T { SUCCESS, FAILURE }; T e; Success(T t = SUCCESS) : e(t) {} operator T() const { return e; } };
struct CmdResponse { enum T { OK, EXECUTION_ERROR }; T e; CmdResponse(T t = OK) : e(t) {} };
struct ParamValid { enum T { UNINIT, VALID, INVALID, DEFAULT }; T e; ParamValid(T t = UNINIT) : e(t) {} operator T() const { return e; } };
class Buffer {
  public:
    Buffer() : m_data(nullptr), m_size(0) {}
    Buffer(uint8_t* d, FwSizeType s) : m_data(d), m_size(s) {}
    uint8_t* getData() const { return m_data; }
    FwSizeType getSize() const { return m_size; }
    bool isValid() const { return m_data != nullptr; }
  private:
    uint8_t* m_data; FwSizeType m_size;
};
}  // namespace Fw
namespace ComCfg { struct FrameContext { uint32_t x = 0; }; }

namespace Zephyr {
struct LinkProfileId { enum T { P0, P1, P2, P3, P4, P5 }; T e; LinkProfileId(T t = P0) : e(t) {} };
struct UspTransmitState { enum T : U8 { ENABLED, DISABLED, DISABLING }; T e; UspTransmitState(T t = DISABLED) : e(t) {}
    bool operator==(T o) const { return e == o; } bool operator!=(T o) const { return e != o; } };
struct UspRadioDirection { enum T : U8 { TX = 0, RX = 1 }; T e; UspRadioDirection(T t = TX) : e(t) {} };

// Observation hooks installed by a scenario.  All called on the thread that executes the port call.
struct SimPorts {
    std::function<void(int /*Success*/)> comStatusOut;
    std::function<void(const Fw::Buffer&)> dataOut;       // RX payload delivered upstream
    std::function<void(const Fw::Buffer&)> dataReturnOut; // TX buffer returned
    std::function<void(const std::string& name, long a, long b)> event;  // event log
    long cost_event_us = 60, cost_tlm_us = 6, cost_port_us = 15, cost_alloc_us = 12;
};

class UspRadioComponentBase {
  public:
    explicit UspRadioComponentBase(const char* name) : m_name(name) { k_sem_init(&m_sem, 0, K_SEM_MAX_LIMIT); }
    virtual ~UspRadioComponentBase() {}

    SimPorts ports;
    int queueDepth = 20;   // Default.QUEUE_SIZE * 2
    long queueOverflows = 0;
    long maxQueueLen = 0;
    bool radioHeadCompat = false;  // paramGet_RADIOHEAD_COMPAT (false: payload offset 0)

    // start the component thread (Zephyr priority = F' priority, see Os/Task.cpp)
    void startThread(int zephyrPrio) {
        zs_spawn(m_name.c_str(), zephyrPrio, [](void* p, void*, void*) { ((UspRadioComponentBase*)p)->loop(); }, this, nullptr, nullptr);
    }
    // ---- input ports (what the topology / scenarios call) ----
    void run_in(U32 ctx) { post(0, [this, ctx] { run_handler(0, ctx); }); }
    void dataIn_in(Fw::Buffer& b, const ComCfg::FrameContext& c) { dataIn_handler(0, b, c); }           // sync
    void dataReturnIn_in(Fw::Buffer& b, const ComCfg::FrameContext& c) { dataReturnIn_handler(0, b, c); }  // sync
    void enableTransmit_in() { enableTransmit_handler(0); }
    void disableTransmit_in() { disableTransmit_handler(0); }
    void cmdTransmit(UspTransmitState s) { post(0, [this, s] { TRANSMIT_cmdHandler(1, 1, s); }); }
    void cmdContinuousWave(U16 sec) { post(0, [this, sec] { CONTINUOUS_WAVE_cmdHandler(2, 1, sec); }); }
    void cmdSetTxProfile(LinkProfileId p) { post(0, [this, p] { SET_TX_PROFILE_cmdHandler(3, 1, p); }); }
    void cmdSetRxProfile(LinkProfileId p, U16 rev) { post(0, [this, p, rev] { SET_RX_PROFILE_cmdHandler(4, 1, p, rev); }); }

    // handlers implemented by the component
    virtual void run_handler(FwIndexType, U32) = 0;
    virtual void dataIn_handler(FwIndexType, Fw::Buffer&, const ComCfg::FrameContext&) = 0;
    virtual void dataReturnIn_handler(FwIndexType, Fw::Buffer&, const ComCfg::FrameContext&) = 0;
    virtual void enableTransmit_handler(FwIndexType) = 0;
    virtual void disableTransmit_handler(FwIndexType) = 0;
    virtual void deferredRxDone_internalInterfaceHandler() {}
    virtual void deferredTxPacket_internalInterfaceHandler(const Fw::Buffer&, const ComCfg::FrameContext&) = 0;
    virtual void deferredTransmitCmd_internalInterfaceHandler(const UspTransmitState&) = 0;
    virtual void deferredSetTxProfile_internalInterfaceHandler(const LinkProfileId&) = 0;
    virtual void deferredSetRxProfile_internalInterfaceHandler(const LinkProfileId&, U16) = 0;
    virtual void deferredContinuousWave_internalInterfaceHandler(U16) = 0;
    virtual void engineKick_internalInterfaceHandler() {}
    virtual void TRANSMIT_cmdHandler(FwOpcodeType, U32, UspTransmitState) = 0;
    virtual void CONTINUOUS_WAVE_cmdHandler(FwOpcodeType, U32, U16) = 0;
    virtual void SET_TX_PROFILE_cmdHandler(FwOpcodeType, U32, LinkProfileId) = 0;
    virtual void SET_RX_PROFILE_cmdHandler(FwOpcodeType, U32, LinkProfileId, U16) = 0;

  protected:
    // ---- internal ports: priority 10 ----
    void deferredRxDone_internalInterfaceInvoke() { post(10, [this] { deferredRxDone_internalInterfaceHandler(); }); }
    void deferredTxPacket_internalInterfaceInvoke(const Fw::Buffer& b, const ComCfg::FrameContext& c) {
        post(10, [this, b, c] { deferredTxPacket_internalInterfaceHandler(b, c); });
    }
    void deferredTransmitCmd_internalInterfaceInvoke(const UspTransmitState& s) { post(10, [this, s] { deferredTransmitCmd_internalInterfaceHandler(s); }); }
    void deferredSetTxProfile_internalInterfaceInvoke(const LinkProfileId& p) { post(10, [this, p] { deferredSetTxProfile_internalInterfaceHandler(p); }); }
    void deferredSetRxProfile_internalInterfaceInvoke(const LinkProfileId& p, U16 r) { post(10, [this, p, r] { deferredSetRxProfile_internalInterfaceHandler(p, r); }); }
    void deferredContinuousWave_internalInterfaceInvoke(U16 s) { post(10, [this, s] { deferredContinuousWave_internalInterfaceHandler(s); }); }
    void engineKick_internalInterfaceInvoke() { post(10, [this] { engineKick_internalInterfaceHandler(); }); }

    // ---- output ports ----
    void comStatusOut_out(FwIndexType, Fw::Success s) { zs_burn(ports.cost_port_us); if (ports.comStatusOut) ports.comStatusOut(s.e); }
    void dataOut_out(FwIndexType, Fw::Buffer& b, const ComCfg::FrameContext&) { zs_burn(ports.cost_port_us); if (ports.dataOut) ports.dataOut(b); }
    void dataReturnOut_out(FwIndexType, Fw::Buffer& b, const ComCfg::FrameContext&) { zs_burn(ports.cost_port_us); if (ports.dataReturnOut) ports.dataReturnOut(b); }
    Fw::Buffer allocate_out(FwIndexType, FwSizeType sz) { zs_burn(ports.cost_alloc_us); return Fw::Buffer((uint8_t*)calloc(1, sz ? sz : 1), sz); }
    void deallocate_out(FwIndexType, Fw::Buffer& b) { zs_burn(ports.cost_alloc_us); free(b.getData()); }
    void radioFirstStart_out(FwIndexType) {}
    bool isConnected_radioFirstStart_OutputPort(FwIndexType) { return true; }
    void cmdResponse_out(FwOpcodeType, U32, Fw::CmdResponse) { zs_burn(ports.cost_port_us); }
    bool paramGet_RADIOHEAD_COMPAT(Fw::ParamValid& v) { v = Fw::ParamValid::VALID; return radioHeadCompat; }

    // ---- events ----
    void ev(const char* n, long a = 0, long b = 0) { zs_burn(ports.cost_event_us); if (ports.event) ports.event(n, a, b); }
    void log_ACTIVITY_HI_ProfileChanged(UspRadioDirection d, LinkProfileId p) { ev("ProfileChanged", d.e, p.e); }
    void log_ACTIVITY_LO_ProfileConfirmed(LinkProfileId p) { ev("ProfileConfirmed", p.e); }
    void log_WARNING_LO_ProfileReverted(LinkProfileId f, LinkProfileId t) { ev("ProfileReverted", f.e, t.e); }
    void log_WARNING_HI_ConfigurationFailed(UspRadioDirection d) { ev("ConfigurationFailed", d.e); }
    void log_WARNING_LO_ProfileChangeDeferred(UspRadioDirection d) { ev("ProfileChangeDeferred", d.e); }
    void log_WARNING_HI_SendFailed(I32 s) { ev("SendFailed", s); }
    void log_WARNING_HI_AllocationFailed(FwSizeType s) { ev("AllocationFailed", (long)s); }
    void log_WARNING_HI_InvalidProfile(LinkProfileId p) { ev("InvalidProfile", p.e); }
    void log_WARNING_HI_RxOverrun(U32 n) { ev("RxOverrun", n); }
    void log_WARNING_HI_ConfigurationFailed_ThrottleClear() {}
    void log_WARNING_HI_SendFailed_ThrottleClear() {}

    // ---- telemetry ----
    template <typename T> void tlm(const char*, T) { zs_burn(ports.cost_tlm_us); }
    void tlmWrite_BytesSent(FwSizeType v) { tlm("BytesSent", v); }
    void tlmWrite_BytesReceived(FwSizeType v) { tlm("BytesReceived", v); }
    void tlmWrite_LastRssi(I16 v) { tlm("LastRssi", v); }
    void tlmWrite_LastSnr(I8 v) { tlm("LastSnr", v); }
    void tlmWrite_TxProfile(LinkProfileId v) { tlm("TxProfile", v.e); }
    void tlmWrite_RxProfile(LinkProfileId v) { tlm("RxProfile", v.e); }
    void tlmWrite_RxReverts(U32 v) { tlm("RxReverts", v); }
    void tlmWrite_RxDropped(U32 v) { tlm("RxDropped", v); }

  public:
    // queue introspection
    size_t queueLen() const { return m_q.size(); }
    // enqueue (callable from any sim thread or ISR)
    void post(int prio, std::function<void()> fn) {
        m_q.emplace(Key{prio, ++m_seq}, std::move(fn));
        if ((long)m_q.size() > maxQueueLen) maxQueueLen = m_q.size();
        if ((int)m_q.size() > queueDepth) queueOverflows++;
        k_sem_give(&m_sem);
    }

  private:
    struct Key { int prio; uint64_t seq; bool operator<(const Key& o) const { return prio != o.prio ? prio > o.prio : seq < o.seq; } };
    void loop() {
        for (;;) {
            k_sem_take(&m_sem, K_FOREVER);
            auto it = m_q.begin();
            auto fn = std::move(it->second);
            m_q.erase(it);
            fn();
        }
    }
    std::string m_name;
    struct k_sem m_sem;
    std::map<Key, std::function<void()>> m_q;
    uint64_t m_seq = 0;
};

}  // namespace Zephyr
