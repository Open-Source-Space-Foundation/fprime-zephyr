// Host fake of the FPP-generated UspRadioComponentBase (fprime 4.3.0 shape).
//
// The one piece of F´ behaviour that matters for #589 is modelled faithfully:
// an active component has ONE thread draining ONE stable max-priority queue
// (Os/Generic/Types/MaxHeap: higher priority first, FIFO within a priority),
// internal ports at priority 10 (UspRadio.fpp), async commands and the async
// run port at priority 0, queue depth Default.QUEUE_SIZE = 10, and a full
// queue on an internal-port send is an FW_ASSERT (generated code sends
// NONBLOCKING then asserts OP_OK).  Output ports, events and telemetry are
// recorded into a trace the scenarios inspect.
#pragma once

#include <cassert>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Fw/FPrimeBasicTypes.hpp>

namespace Fw {
class Buffer {
  public:
    Buffer() : m_data(nullptr), m_size(0) {}
    Buffer(U8* d, FwSizeType s) : m_data(d), m_size(s) {}
    U8* getData() const { return m_data; }
    FwSizeType getSize() const { return m_size; }
    void setSize(FwSizeType s) { m_size = s; }
    bool isValid() const { return m_data != nullptr; }
  private:
    U8* m_data;
    FwSizeType m_size;
};
struct Success {
    enum T { SUCCESS = 0, FAILURE = 1 };
    T e;
    Success(T v = SUCCESS) : e(v) {}
    operator T() const { return e; }
};
struct CmdResponse {
    enum T { OK = 0, INVALID_OPCODE, VALIDATION_ERROR, FORMAT_ERROR, EXECUTION_ERROR, BUSY };
    T e;
    CmdResponse(T v = OK) : e(v) {}
};
struct ParamValid {
    enum T { UNINIT = 0, VALID = 1, INVALID = 2, DEFAULT = 3 };
    T e;
    ParamValid(T v = UNINIT) : e(v) {}
    bool operator==(T o) const { return e == o; }
    bool operator!=(T o) const { return e != o; }
};
}  // namespace Fw

namespace ComCfg {
struct FrameContext {
    U32 tag = 0;
};
}  // namespace ComCfg

#define FPP_ENUM(Name, ...)                                   \
    struct Name {                                             \
        enum T { __VA_ARGS__ };                               \
        T e;                                                  \
        Name() : e(static_cast<T>(0)) {}                      \
        Name(T v) : e(v) {}                                   \
        operator T() const { return e; }                      \
        bool operator==(T o) const { return e == o; }         \
        bool operator!=(T o) const { return e != o; }         \
    };

namespace Zephyr {

FPP_ENUM(UspTransmitState, ENABLED = 0, DISABLED = 1, DISABLING = 2)
FPP_ENUM(UspRadioDirection, TX = 0, RX = 1)
FPP_ENUM(LinkProfileId, P0_LORA_SF8 = 0, P1_LORA_SF10 = 1, P2_LORA_SF5 = 2, P3_GFSK_38K = 3,
         P4_GFSK_75K = 4, P5_GMSK_83K = 5)

//! One recorded output (event, telemetry, port call).
struct TraceRec {
    uint64_t t_us;
    std::string what;
    long a;
    long b;
};

class UspRadioComponentBase {
  public:
    static constexpr int QUEUE_DEPTH = 10;  // Default.QUEUE_SIZE in PCR instances.fpp
    static constexpr int PRIO_INTERNAL = 10;
    static constexpr int PRIO_ASYNC = 0;

    explicit UspRadioComponentBase(const char*) {}
    virtual ~UspRadioComponentBase() { stop(); }

    // ---------------- harness control ------------------------------------
    void start() {
        m_run = true;
        m_thread = std::thread([this] { this->loop(); });
    }
    void stop() {
        {
            std::lock_guard<std::mutex> g(m_qm);
            m_run = false;
        }
        m_qc.notify_all();
        if (m_thread.joinable()) m_thread.join();
    }
    //! Block until the queue is empty and the dispatcher is idle.
    void drain() {
        std::unique_lock<std::mutex> g(m_qm);
        m_idle.wait(g, [this] { return m_q.empty() && !m_busy; });
    }
    //! Hook run on the component thread before each dispatched message.
    std::function<void(const char* name)> beforeDispatch;
    //! Hook run on the posting thread right after a message is enqueued.
    std::function<void(const char* name)> afterPost;

    // recorded outputs
    std::mutex traceMx;
    std::vector<TraceRec> trace;
    void rec(const char* w, long a = 0, long b = 0);
    int count(const char* w);
    std::function<void(Fw::Buffer&)> onDataOut;
    std::function<void(Fw::Success)> onComStatus;
    int queueHighWater = 0;
    uint64_t dispatched = 0;

    // ---------------- test-side "ports" (what the topology would call) ----
    void cmd_TRANSMIT(UspTransmitState s) {
        post("TRANSMIT_cmd", PRIO_ASYNC, [this, s] { this->TRANSMIT_cmdHandler(1, 0, s); });
    }
    void cmd_SET_RX_PROFILE(LinkProfileId p, U16 r) {
        post("SET_RX_PROFILE_cmd", PRIO_ASYNC, [this, p, r] { this->SET_RX_PROFILE_cmdHandler(2, 0, p, r); });
    }
    void cmd_SET_TX_PROFILE(LinkProfileId p) {
        post("SET_TX_PROFILE_cmd", PRIO_ASYNC, [this, p] { this->SET_TX_PROFILE_cmdHandler(3, 0, p); });
    }
    void port_run(U32 ctx = 0) {
        post("run", PRIO_ASYNC, [this, ctx] { this->run_handler(0, ctx); });
    }
    //! Generic enqueue for components the harness builds itself (option 2).
    void postMsg(const char* name, int prio, std::function<void()> fn) { post(name, prio, std::move(fn)); }
    //! Run fn on the component thread and wait for it.
    void onComponentThread(std::function<void()> fn) {
        std::mutex mx;
        std::condition_variable cv;
        bool done = false;
        post("harness", PRIO_INTERNAL, [&] {
            fn();
            std::lock_guard<std::mutex> g(mx);
            done = true;
            cv.notify_all();
        });
        std::unique_lock<std::mutex> g(mx);
        cv.wait(g, [&] { return done; });
    }
    void port_dataIn(Fw::Buffer b) {  // sync port: runs on the caller's thread
        ComCfg::FrameContext c;
        this->dataIn_handler(0, b, c);
    }

  protected:
    // ---------------- handlers the component implements ------------------
    virtual void run_handler(FwIndexType, U32) = 0;
    virtual void dataIn_handler(FwIndexType, Fw::Buffer&, const ComCfg::FrameContext&) = 0;
    virtual void dataReturnIn_handler(FwIndexType, Fw::Buffer&, const ComCfg::FrameContext&) = 0;
    virtual void enableTransmit_handler(FwIndexType) = 0;
    virtual void disableTransmit_handler(FwIndexType) = 0;
    virtual void deferredRxDone_internalInterfaceHandler() = 0;
    virtual void deferredTxPacket_internalInterfaceHandler(const Fw::Buffer&, const ComCfg::FrameContext&) = 0;
    virtual void deferredTransmitCmd_internalInterfaceHandler(const UspTransmitState&) = 0;
    virtual void deferredSetTxProfile_internalInterfaceHandler(const LinkProfileId&) = 0;
    virtual void deferredSetRxProfile_internalInterfaceHandler(const LinkProfileId&, U16) = 0;
    virtual void deferredContinuousWave_internalInterfaceHandler(U16) = 0;
    virtual void TRANSMIT_cmdHandler(FwOpcodeType, U32, UspTransmitState) = 0;
    virtual void CONTINUOUS_WAVE_cmdHandler(FwOpcodeType, U32, U16) = 0;
    virtual void SET_TX_PROFILE_cmdHandler(FwOpcodeType, U32, LinkProfileId) = 0;
    virtual void SET_RX_PROFILE_cmdHandler(FwOpcodeType, U32, LinkProfileId, U16) = 0;

    // ---------------- internal interface invokes --------------------------
    void deferredRxDone_internalInterfaceInvoke() {
        post("deferredRxDone", PRIO_INTERNAL, [this] { this->deferredRxDone_internalInterfaceHandler(); });
    }
    void deferredTxPacket_internalInterfaceInvoke(const Fw::Buffer& b, const ComCfg::FrameContext& c) {
        post("deferredTxPacket", PRIO_INTERNAL, [this, b, c] { this->deferredTxPacket_internalInterfaceHandler(b, c); });
    }
    void deferredTransmitCmd_internalInterfaceInvoke(const UspTransmitState& s) {
        post("deferredTransmitCmd", PRIO_INTERNAL, [this, s] { this->deferredTransmitCmd_internalInterfaceHandler(s); });
    }
    void deferredSetTxProfile_internalInterfaceInvoke(const LinkProfileId& p) {
        post("deferredSetTxProfile", PRIO_INTERNAL, [this, p] { this->deferredSetTxProfile_internalInterfaceHandler(p); });
    }
    void deferredSetRxProfile_internalInterfaceInvoke(const LinkProfileId& p, U16 r) {
        post("deferredSetRxProfile", PRIO_INTERNAL,
             [this, p, r] { this->deferredSetRxProfile_internalInterfaceHandler(p, r); });
    }
    void deferredContinuousWave_internalInterfaceInvoke(U16 s) {
        post("deferredContinuousWave", PRIO_INTERNAL, [this, s] { this->deferredContinuousWave_internalInterfaceHandler(s); });
    }

    // ---------------- output ports ----------------------------------------
    Fw::Buffer allocate_out(FwIndexType, FwSizeType size) {
        return Fw::Buffer(static_cast<U8*>(std::malloc(size ? size : 1)), size);
    }
    void deallocate_out(FwIndexType, Fw::Buffer& b) { std::free(b.getData()); }
    void dataOut_out(FwIndexType, Fw::Buffer& b, const ComCfg::FrameContext&) {
        rec("dataOut", static_cast<long>(b.getSize()), b.getSize() ? b.getData()[0] : -1);
        if (onDataOut) onDataOut(b);
        std::free(b.getData());
    }
    void dataReturnOut_out(FwIndexType, Fw::Buffer&, const ComCfg::FrameContext&) { rec("dataReturnOut"); }
    void comStatusOut_out(FwIndexType, Fw::Success& s) {
        rec("comStatus", s.e);
        if (onComStatus) onComStatus(s);
    }
    void cmdResponse_out(FwOpcodeType op, U32, Fw::CmdResponse r) { rec("cmdResponse", op, r.e); }
    bool isConnected_radioFirstStart_OutputPort(FwIndexType) { return true; }
    void radioFirstStart_out(FwIndexType) { rec("radioFirstStart"); }
    bool paramGet_RADIOHEAD_COMPAT(Fw::ParamValid& v) {
        v = Fw::ParamValid::VALID;
        return false;  // raw mode: payload == on-air packet
    }

    // ---------------- events ----------------------------------------------
    void log_ACTIVITY_HI_ProfileChanged(UspRadioDirection d, LinkProfileId p) { rec("EVR ProfileChanged", d.e, p.e); }
    void log_ACTIVITY_LO_ProfileConfirmed(LinkProfileId p) { rec("EVR ProfileConfirmed", p.e); }
    void log_WARNING_LO_ProfileReverted(LinkProfileId f, LinkProfileId t) { rec("EVR ProfileReverted", f.e, t.e); }
    void log_WARNING_HI_ConfigurationFailed(UspRadioDirection d) { rec("EVR ConfigurationFailed", d.e); }
    void log_WARNING_HI_ConfigurationFailed_ThrottleClear() {}
    void log_WARNING_LO_ProfileChangeDeferred(UspRadioDirection d) { rec("EVR ProfileChangeDeferred", d.e); }
    void log_WARNING_HI_SendFailed(I32 rc) {
        rec("EVR SendFailed", rc);
        std::fprintf(stderr, "EVR SendFailed rc=%d\n", static_cast<int>(rc));
    }
    void log_WARNING_HI_SendFailed_ThrottleClear() {}
    void log_WARNING_HI_AllocationFailed(FwSizeType s) { rec("EVR AllocationFailed", static_cast<long>(s)); }
    void log_WARNING_HI_InvalidProfile(LinkProfileId p) { rec("EVR InvalidProfile", p.e); }
    void log_WARNING_HI_RxOverrun(U32 n) { rec("EVR RxOverrun", n); }

    // ---------------- telemetry -------------------------------------------
    void tlmWrite_BytesSent(FwSizeType) {}
    void tlmWrite_BytesReceived(FwSizeType) {}
    void tlmWrite_LastRssi(I16) {}
    void tlmWrite_LastSnr(I8) {}
    void tlmWrite_TxProfile(LinkProfileId p) { rec("TLM TxProfile", p.e); }
    void tlmWrite_RxProfile(LinkProfileId p) { rec("TLM RxProfile", p.e); }
    void tlmWrite_RxReverts(U32) {}
    void tlmWrite_RxDropped(U32 n) { rec("TLM RxDropped", n); }

  private:
    struct Msg {
        int prio;
        uint64_t order;
        const char* name;
        std::function<void()> fn;
    };
    void post(const char* name, int prio, std::function<void()> fn);
    void loop();

    std::mutex m_qm;
    std::condition_variable m_qc;
    std::condition_variable m_idle;
    std::deque<Msg> m_q;
    uint64_t m_order = 0;
    bool m_run = false;
    bool m_busy = false;
    const char* m_current = nullptr;
    uint64_t m_currentSince = 0;
    std::thread m_thread;
};

}  // namespace Zephyr
