# Recommendation for #589 (from arena c4)

## Recommended option

**Option 2 (single queue, `CONFIG_USP_MAIN_THREAD=n`), conditional on two things:**

1. Fix the stale-frame ordering in option 2 before merge. The sketch as written does not avoid the false confirm
   (`raw/claim_A.txt` S2). Either drain the RX lock (stop/harvest) before `ProfilePolicy::setRxProfile`, or tag
   each received frame with the arm epoch and drop frames from an earlier arm. The first is a two-line reorder in
   `deferredSetRxProfile`; the second is stronger. This applies to every option: option 1 and option 3 have the
   same false confirm and no fix in their diffs.
2. Measure kick latency on the bench at P5 under the real F´ thread mix. Pass if the worst-case engine-kick
   latency stays well under one frame airtime (25 ms at P5, 252 B). Option 2 has no USP thread, so the
   F´ queue is the only thing in the RX path.

Reasons, from measured or code-confirmed points only:

- Option 1 (#14 + #26, USP thread) has the open E race (lock without protect) and the open B path after failed init,
  and it needs the usp_zephyr RAC-mutex fork commit. Option 2 removes all three.
- The v1.2.1 busy-loop (F) is exposed by option 1 and option 3 and does not apply to option 2 (no USP thread
  running the loop). Mechanism confirmed in code, not on target.
- Option 3 keeps the USP thread and so keeps E-style concerns only if the mutex wraps every caller. Its stale-frame
  issue is the same as option 1 and is not fixed by the mutex.

## What would change my mind

- Kick latency worst case in the bench run above one-half frame airtime (about 12 ms at P5). Then option 2 risks
  RX loss under load and option 3 with the mutex fix is the fallback.
- A cheap, reliable stale-frame fix that works in option 1 or 3 but cannot be expressed in option 2's queue.
  Not found; I looked for it and did not find one.
- A v1.2.1 usp_zephyr loop that does not busy-loop (would drop F's weight, keeping option 1 viable with the fixes).
- The option 2 line count is not established. The table's 690 vs 920 numbers do not reproduce (see RESULTS.md).
  The decision does not depend on it, but any line-count argument in the ticket should be redone on the full files.

## Bench drills remaining

1. Kick latency histogram on RP2350 at P5, under the real F´ mix, with synthetic message load (no TX).
2. Stale-frame injection: arm P0, force a received frame near a SET_RX_PROFILE(P1), and check the confirm
   result. This is the bench version of `claim_A`.
3. Re-arm cost: time stop + re-arm per frame in option 1 (expect ~50 ms per the table) versus option 2 synchronous SPI.
4. v1.2.1 busy-loop: measure USP thread CPU share under continuous RX on the bench.
5. Option 3 priority inversion: measure rac_api_mutex hold time from the component thread.

No RF transmission in any drill (dev-bench rule).
