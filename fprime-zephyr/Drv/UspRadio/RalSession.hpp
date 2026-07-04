// ======================================================================
// \title  RalSession.hpp
// \brief  Pure-virtual seam over every USP RAL/RAC touch (Phase 3, ADR 0001).
//
// This header intentionally contains NO USP or Zephyr includes so it
// compiles host-side (unit tests, GRC) without the usp_zephyr west module.
//
// Threading contract (from spikes/REPORT-bench-phase0.md RSSI-delta closure):
//   ALL radio operations must execute inside a RAC lock/transaction window
//   on the USP thread.  RalSessionImpl enforces this; callers (UspRadio
//   component thread) invoke these methods which internally acquire the
//   appropriate RAC lock before touching any RAL/SPI registers.
//   The component thread itself NEVER calls smtc_rac_* directly.
// ======================================================================

#ifndef ZEPHYR_USP_RADIO_RAL_SESSION_HPP
#define ZEPHYR_USP_RADIO_RAL_SESSION_HPP

#include <cstddef>
#include <cstdint>
#include <functional>

namespace Zephyr {

// Forward declare (defined in LinkProfiles.hpp)
struct LinkProfile;

// ---------------------------------------------------------------------------
// Callback types
// ---------------------------------------------------------------------------

//! Called by RalSessionImpl when a packet is received.
//! Runs on the USP thread — implementation must NOT block or do heavy work.
//! rssi: RSSI in dBm (typically negative I16 range).
//! snr:  SNR in dB   (I8 range, LoRa only; GFSK passes 0).
using RxDoneCallback = std::function<void(const uint8_t* data, std::size_t size, int16_t rssi, int8_t snr)>;

//! Called by RalSessionImpl when a TX completes (optional; UspRadio does not
//! require this for the ping-pong comStatus protocol but keeps the seam open).
using TxDoneCallback = std::function<void()>;

// ---------------------------------------------------------------------------
// RalSession — pure-virtual interface
// ---------------------------------------------------------------------------

class RalSession {
  public:
    virtual ~RalSession() = default;

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    //! Initialize RAC/RAL after lorawan_smtc_modem_hal_init() has been
    //! called.  Calls zephyr_smtc_rac_init() (which smtc_rac_init() does NOT
    //! do automatically in the Zephyr port — per REPORT-ral-architecture §Init).
    //! Must be called once before any other method.
    //! @returns 0 on success, negative errno on failure.
    virtual int init() = 0;

    // ------------------------------------------------------------------
    // Profile configuration
    // ------------------------------------------------------------------

    //! Apply a link profile to the radio (GFSK or LoRa config via RAL).
    //! Runs inside a RAC lock window on the USP thread.
    //! The implementation translates LinkProfile fields to the appropriate
    //! ral_set_pkt_type / ral_set_lora_mod_params (or ral_set_gfsk_*) calls.
    //! @returns 0 on success, negative errno on failure.
    virtual int applyProfile(const LinkProfile& profile) = 0;

    // ------------------------------------------------------------------
    // RX / TX
    // ------------------------------------------------------------------

    //! Start continuous RX.  Uses RAL_RX_TIMEOUT_CONTINUOUS_MODE.
    //! The rxDone callback (registered via setCallbacks) fires on the USP
    //! thread when a frame is received.
    //! @returns 0 on success, negative errno on failure.
    virtual int startReceive() = 0;

    //! Transmit a packet synchronously (blocks until TX_DONE IRQ or error).
    //! Internally: RAC lock → payload → ral_set_tx → poll IRQ → unlock.
    //! Caller is on the component thread; this method acquires the RAC lock
    //! internally so the component thread never touches RAC directly.
    //! @returns 0 on success, negative errno on failure.
    virtual int transmitPacket(const uint8_t* data, std::size_t size) = 0;

    // ------------------------------------------------------------------
    // CW
    // ------------------------------------------------------------------

    //! Start a continuous-wave carrier.  Caller supplies the CW profile
    //! (used for freq/power; pkt-type is set to LoRa per RAL CW requirement).
    //! @returns 0 on success, negative errno on failure.
    virtual int startCw(const LinkProfile& cwProfile) = 0;

    //! Stop any ongoing radio operation (CW, TX, continuous RX) and return
    //! the chip to standby.  Must be safe to call at any time.
    //! @returns 0 on success, negative errno on failure.
    virtual int stopRadio() = 0;

    // ------------------------------------------------------------------
    // Callback registration
    // ------------------------------------------------------------------

    //! Register RX/TX done callbacks before calling startReceive().
    virtual void setCallbacks(RxDoneCallback rxDone, TxDoneCallback txDone) = 0;
};

}  // namespace Zephyr

#endif  // ZEPHYR_USP_RADIO_RAL_SESSION_HPP
