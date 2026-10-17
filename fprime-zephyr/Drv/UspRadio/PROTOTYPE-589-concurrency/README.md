# PROTOTYPE #589: UspRadio concurrency model

Throwaway branch. Do not merge. This prototype answers wayfinder ticket
[#589](https://github.com/Open-Source-Space-Foundation/proves-core-reference/issues/589):
should UspRadio keep #14 with #26's fixes folded in, move to a single queue, or adopt
upstream's mutex mode?

Base: `fix/uspradio-integration-c123` @ 25d5cd2 (#26 head).

**Superseded numbers:** the side-by-side table below is the pre-arena estimate. The measured verdicts and the
updated recommendation are in [SYNTHESIS.md](SYNTHESIS.md); the simulation is `arena-c3/` (`./run_all.sh`).

| File | What it is |
| --- | --- |
| `concurrency-demo.html` | Open it by double-click. Runs 7 scenarios against each option, step by step. |
| `single-queue/` | Option 2 sketch: `UspRadio.cpp` (changed handlers only), `RalSessionSt.{hpp,cpp}`, the FPP port diff. Not built. |
| `upstream-mutex/RalSessionImpl.cpp.diff` | Option 3 as a real diff on #26's `RalSessionImpl.cpp` (+27/−20). |

## Option 2 in one paragraph

Set `CONFIG_USP_MAIN_THREAD=n` (a supported upstream mode, so no fork). The
component thread runs `smtc_rac_run_engine()` from an `engineKick` internal
message. DIO1 and the RAC timer only set planner flags and give the HAL event
semaphore (radio_planner.c:597-609). A 30-line pump thread turns that semaphore
into one coalesced kick. Because the engine runs inline, `stop()` is synchronous
and needs no semaphore or `tasks[]` polling. TX becomes start-then-complete:
TX_DONE arrives on a later kick, so the queue never blocks for up to 10 s.
`armRx(profile)` and `startTx(profile, …)` apply the profile inside the same
lock window as the arm. One 3-state field (`NO_CREDIT`, `CREDIT_OUT`,
`FRAME_HELD`) replaces `m_comStatusOwed`, `m_pendingTxFrames` and
`m_rearmAfterTx`.

## Side by side

| | 1 · #14 + #26 | 2 · Single queue | 3 · Upstream mutex |
| --- | --- | --- | --- |
| Code lines, UspRadio + RalSession | 920 (measured) | ~690 (est.) | ~927 |
| k_sem · atomics · k_sleep · tasks[] | 16 · 26 · 4 · 8 | 1 · 3 · 0 · 0 | 16 · 26 · 4 · 8 |
| usp_zephyr RAC-mutex fork commit | needed | dropped | dropped |
| Stale-frame false confirm | open | impossible | open |
| tasks[0xFF] after failed init | open | refused | open |
| lock_radio_access / quiesce race | open | impossible | closed |
| v1.2.1 USP-thread busy-loop | exposed | not exposed | exposed |
| Queue blocked in TX / CW | yes | no | yes |
| New risk | none | RX service waits behind the queue (one frame airtime, ~25 ms at P5); TX path rewritten | priority inversion |

Bench questions option 2 raises: kick latency under P3 saturation, and whether
dropping the "skip re-arm while frames are queued" optimisation costs
throughput now that stop and re-arm are synchronous SPI.
