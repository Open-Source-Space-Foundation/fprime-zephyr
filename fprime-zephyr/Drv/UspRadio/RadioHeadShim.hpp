// ======================================================================
// \title  RadioHeadShim.hpp
// \brief  4-byte RadioHead-style header compatibility shim (on-air format)
//
// This header is intentionally free of USP/RAL, Zephyr, and F´ component
// includes so it compiles host-side (unit tests, GRC) as well as on-target.
//
// Why this exists
// ---------------
// The legacy UHF path (Zephyr::LoRa) and the ground tooling built around it
// frame every LoRa packet with a 4-byte RadioHead-style header:
//   [destination, source, identifier, flags]
// - adafruit_rfm9x (the CI ground-side "CircuitPython passthrough" board)
//   unconditionally prepends these 4 bytes on send() and strips the first
//   4 bytes on receive(with_header=False).  Its default node address 0xFF
//   (broadcast) accepts any destination byte.
// - Zephyr::LoRa mirrors that: LoRa.cpp prepends LoRaConfig::HEADER
//   ({0, 0, 0, 0}) on TX and drops the first 4 bytes on RX.
//
// UspRadio radiates raw F´ frames, so against a RadioHead peer every
// downlink frame loses its first 4 real bytes at the peer and every uplink
// frame arrives with 4 foreign bytes prepended — nothing deframes.  (Root
// cause of the PR #439 integration-radio CI failure, diagnosed 2026-07-13.)
//
// When the UspRadio RADIOHEAD_COMPAT parameter is set (the default), this
// shim restores on-air parity for LoRa profiles: TX prepends four zero
// bytes (byte-identical to LoRaConfig::HEADER) and RX strips the first
// four bytes of every received frame.  GFSK profiles never carried the
// header on any legacy path and are always raw.
// ======================================================================

#ifndef ZEPHYR_USP_RADIO_RADIOHEAD_SHIM_HPP
#define ZEPHYR_USP_RADIO_RADIOHEAD_SHIM_HPP

// Same host-type escape hatch as LinkProfiles.hpp: host builds (unit tests,
// GRC) define LINK_PROFILES_USE_HOST_TYPES instead of pulling in F´.
#ifndef LINK_PROFILES_USE_HOST_TYPES
#include <Fw/FPrimeBasicTypes.hpp>
#else
#include <cstdint>
using U8  = uint8_t;
using U32 = uint32_t;
#endif

#include <cstring>

namespace Zephyr {
namespace RadioHeadShim {

//! RadioHead header length in bytes: [destination, source, identifier, flags]
static constexpr U32 HEADER_SIZE = 4;

//! Largest payload that fits in an on-air packet of onAirCap bytes.
//! In compat mode the header consumes HEADER_SIZE of the on-air budget.
constexpr U32 maxPayload(U32 onAirCap, bool compatEnabled) {
    return compatEnabled ? (onAirCap - HEADER_SIZE) : onAirCap;
}

//! Build a compat-mode on-air packet: HEADER_SIZE zero bytes then the
//! payload.  out must have room for payloadSize + HEADER_SIZE bytes
//! (callers cap payloadSize via maxPayload()).  Returns the on-air length.
inline U32 buildTxPacket(const U8* payload, U32 payloadSize, U8* out) {
    (void)::memset(out, 0, HEADER_SIZE);
    (void)::memcpy(out + HEADER_SIZE, payload, payloadSize);
    return payloadSize + HEADER_SIZE;
}

//! Locate the deliverable payload inside a received on-air packet.
//! Raw mode: the whole packet, delivered as-is (pre-shim behavior).
//! Compat mode: skip the peer's HEADER_SIZE header bytes.
//! Returns false when there is nothing to deliver — a compat-mode runt
//! (header-only or shorter) carries no payload.
inline bool locateRxPayload(U32 rxLen, bool compatEnabled, U32& offset, U32& length) {
    if (!compatEnabled) {
        offset = 0;
        length = rxLen;
        return true;
    }
    if (rxLen <= HEADER_SIZE) {
        offset = 0;
        length = 0;
        return false;
    }
    offset = HEADER_SIZE;
    length = rxLen - HEADER_SIZE;
    return true;
}

}  // namespace RadioHeadShim
}  // namespace Zephyr

#endif  // ZEPHYR_USP_RADIO_RADIOHEAD_SHIM_HPP
