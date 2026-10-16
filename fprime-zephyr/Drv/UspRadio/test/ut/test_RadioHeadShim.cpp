// ======================================================================
// test_RadioHeadShim.cpp
// Host-side unit tests for the RadioHead-header compatibility shim.
//
// Covers:
//   - Payload budget math (compat vs raw)
//   - TX packet build: zero header bytes, payload placement, on-air length
//   - RX payload location: raw passthrough, compat strip, runt rejection
//   - TX→RX round trip preserves payload bytes exactly
//
// Build: included via pcr's CMakeLists.txt host-UT target.
// LINK_PROFILES_USE_HOST_TYPES is injected via target_compile_definitions in
// the test CMakeLists, activating cstdint typedefs in the header.
// ======================================================================

#include "fprime-zephyr/Drv/UspRadio/RadioHeadShim.hpp"

#include <gtest/gtest.h>

using namespace Zephyr;

// ---------------------------------------------------------------------------
// Budget math
// ---------------------------------------------------------------------------

TEST(RadioHeadShim, MaxPayloadCompatShrinksByHeader) {
    EXPECT_EQ(RadioHeadShim::maxPayload(252, true), 248u);
    EXPECT_EQ(RadioHeadShim::maxPayload(252, false), 252u);
}

// ---------------------------------------------------------------------------
// TX build
// ---------------------------------------------------------------------------

TEST(RadioHeadShim, BuildTxPacketPrependsZeroHeader) {
    const U8 payload[3] = {0xAA, 0xBB, 0xCC};
    U8 out[16];
    (void)::memset(out, 0xEE, sizeof out);

    const U32 onAirLen = RadioHeadShim::buildTxPacket(payload, 3, out);

    EXPECT_EQ(onAirLen, 7u);
    // Header bytes are all zero — byte-identical to LoRaConfig::HEADER.
    EXPECT_EQ(out[0], 0x00);
    EXPECT_EQ(out[1], 0x00);
    EXPECT_EQ(out[2], 0x00);
    EXPECT_EQ(out[3], 0x00);
    // Payload lands immediately after the header, unmodified.
    EXPECT_EQ(out[4], 0xAA);
    EXPECT_EQ(out[5], 0xBB);
    EXPECT_EQ(out[6], 0xCC);
    // Nothing written past the on-air length.
    EXPECT_EQ(out[7], 0xEE);
}

TEST(RadioHeadShim, BuildTxPacketEmptyPayload) {
    U8 out[8];
    (void)::memset(out, 0xEE, sizeof out);
    const U32 onAirLen = RadioHeadShim::buildTxPacket(out + 4, 0, out);
    EXPECT_EQ(onAirLen, 4u);  // header-only packet
    EXPECT_EQ(out[0], 0x00);
    EXPECT_EQ(out[3], 0x00);
}

TEST(RadioHeadShim, BuildTxPacketMaxCompatPayload) {
    U8 payload[248];
    for (U32 i = 0; i < 248; i++) {
        payload[i] = static_cast<U8>(i);
    }
    U8 out[252];
    const U32 onAirLen = RadioHeadShim::buildTxPacket(payload, 248, out);
    EXPECT_EQ(onAirLen, 252u);
    EXPECT_EQ(out[4], 0x00);
    EXPECT_EQ(out[251], static_cast<U8>(247));
}

// ---------------------------------------------------------------------------
// RX locate
// ---------------------------------------------------------------------------

TEST(RadioHeadShim, LocateRawDeliversWholePacket) {
    U32 offset = 99, length = 99;
    EXPECT_TRUE(RadioHeadShim::locateRxPayload(252, false, offset, length));
    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(length, 252u);
}

TEST(RadioHeadShim, LocateRawDeliversEvenTinyPackets) {
    // Raw mode is a passthrough — matches pre-shim UspRadio behavior.
    U32 offset = 99, length = 99;
    EXPECT_TRUE(RadioHeadShim::locateRxPayload(1, false, offset, length));
    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(length, 1u);
}

TEST(RadioHeadShim, LocateCompatStripsHeader) {
    U32 offset = 0, length = 0;
    EXPECT_TRUE(RadioHeadShim::locateRxPayload(252, true, offset, length));
    EXPECT_EQ(offset, 4u);
    EXPECT_EQ(length, 248u);
}

TEST(RadioHeadShim, LocateCompatSingleBytePayload) {
    U32 offset = 0, length = 0;
    EXPECT_TRUE(RadioHeadShim::locateRxPayload(5, true, offset, length));
    EXPECT_EQ(offset, 4u);
    EXPECT_EQ(length, 1u);
}

TEST(RadioHeadShim, LocateCompatRejectsRunts) {
    // Header-only or shorter: a valid LoRa frame, but no payload to deliver.
    for (U32 rxLen = 0; rxLen <= 4; rxLen++) {
        U32 offset = 99, length = 99;
        EXPECT_FALSE(RadioHeadShim::locateRxPayload(rxLen, true, offset, length))
            << "rxLen=" << rxLen;
        EXPECT_EQ(length, 0u);
    }
}

// ---------------------------------------------------------------------------
// Round trip
// ---------------------------------------------------------------------------

TEST(RadioHeadShim, TxRxRoundTripPreservesPayload) {
    const U8 payload[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02};
    U8 onAir[16];
    const U32 onAirLen = RadioHeadShim::buildTxPacket(payload, 6, onAir);

    U32 offset = 0, length = 0;
    ASSERT_TRUE(RadioHeadShim::locateRxPayload(onAirLen, true, offset, length));
    ASSERT_EQ(length, 6u);
    EXPECT_EQ(0, ::memcmp(onAir + offset, payload, length));
}
