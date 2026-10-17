// Claim A: stale-frame false confirm.
// Drives the REAL ProfilePolicy (ProfilePolicy.cpp, unmodified) through the
// event orderings that the UspRadio code produces.  Only the radio is modelled.
//
// Orderings:
//   S1  option 1 as shipped: setRxProfile(pending) -> stopRadio -> USP thread
//       had already pushed a stale RX_DONE into the ring -> deferredRxDone ->
//       frameReceived().
//   S2  option 2 as sketched (single-queue/UspRadio.cpp): setRxProfile(pending)
//       -> rearmRxWhenIdle -> stop() drains the RX lock inline -> rxFrame() ->
//       frameReceived() synchronously.  Same order as S1, no ring.
//   S3  control: stop BEFORE setRxProfile.  The stale frame is harvested while
//       nothing is pending, so it cannot confirm.
//   S4  control: genuine frame on the NEW profile after arm -> must confirm.
#include "fprime-zephyr/Drv/UspRadio/ProfilePolicy.hpp"
#include <cstdio>

using namespace Zephyr;

static const char* act(ProfilePolicy::Action a) {
    switch (a) {
        case ProfilePolicy::Action::kNoOp: return "kNoOp";
        case ProfilePolicy::Action::kApplyTx: return "kApplyTx";
        case ProfilePolicy::Action::kApplyRx: return "kApplyRx";
        case ProfilePolicy::Action::kConfirmRx: return "kConfirmRx";
        case ProfilePolicy::Action::kRevert: return "kRevert";
        case ProfilePolicy::Action::kInvalidIndex: return "kInvalidIndex";
    }
    return "?";
}

struct Result { bool falseConfirm; bool revertFiredLater; };

static Result run(const char* name, bool stopFirst, bool staleFrameInRing) {
    ProfilePolicy p;                       // boot: rx = P0, no pending
    const uint64_t t0 = 1000;              // ms
    const U8 newProfile = 1;               // P1
    const U16 revert_s = 30;
    std::printf("== %s ==\n", name);

    if (stopFirst && staleFrameInRing) {
        // Fixed order: stop drains the old-profile frame while nothing is pending.
        auto a = p.frameReceived();
        std::printf("  stale frame (old P0 arm, harvested before SET)  -> %s\n", act(a));
    }
    auto s = p.setRxProfile(newProfile, revert_s, t0);
    std::printf("  setRxProfile(P1, revert=%us)                    -> %s\n", revert_s, act(s));
    if (!stopFirst && staleFrameInRing) {
        // Shipped order: the stale old-profile frame is delivered after pending is set.
        auto a = p.frameReceived();
        std::printf("  stale frame (old P0 arm) delivered after SET   -> %s\n", act(a));
    }
    if (!staleFrameInRing) {
        // Control: genuine frame on the new arm.
        auto a = p.frameReceived();
        std::printf("  genuine frame on P1                            -> %s\n", act(a));
    }
    const bool confirmed = p.rxProfile() == newProfile && !p.hasPendingRx();
    auto r = p.tick(t0 + revert_s * 1000ULL + 1);
    std::printf("  rx committed=P%u pending=%d  tick(+31s) -> %s  revertCount=%u\n",
                p.rxProfile(), p.hasPendingRx(), act(r), p.revertCount());
    const bool falseConfirm = staleFrameInRing && confirmed;
    std::printf("  RESULT: committed=P%u  auto-revert %s\n", p.rxProfile(),
                (p.revertCount() == 0) ? "DISARMED" : "fired");
    return {falseConfirm, p.revertCount() == 1};
}

int main() {
    Result s1 = run("S1 option 1 shipped order (stale ring frame after SET)", false, true);
    Result s2 = run("S2 option 2 sketch order (stale frame harvested inside stop after SET)", false, true);
    Result s3 = run("S3 control: stop before SET (stale frame before pending)", true, true);
    Result s4 = run("S4 control: genuine frame on new profile", false, false);
    std::printf("\nSUMMARY\n");
    std::printf("  S1 false confirm: %s, revert disarmed: %s\n", s1.falseConfirm ? "YES" : "no", s1.revertFiredLater ? "no" : "yes");
    std::printf("  S2 false confirm: %s, revert disarmed: %s\n", s2.falseConfirm ? "YES" : "no", s2.revertFiredLater ? "no" : "yes");
    std::printf("  S3 false confirm: %s, revert fires:   %s\n", s3.falseConfirm ? "YES" : "no", s3.revertFiredLater ? "yes" : "no");
    std::printf("  S4 confirm correct: %s\n", (!s4.revertFiredLater) ? "yes" : "no");
    return 0;
}
