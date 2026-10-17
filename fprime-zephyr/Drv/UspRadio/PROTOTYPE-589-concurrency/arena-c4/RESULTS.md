# Arena c4 results for #589 (UspRadio concurrency)

Scope: host only. No board, no RF. Measured = run here on the Mac. Inferred = read from source, not run.
Reproduce: `sh run.sh` in this directory. Raw outputs in `raw/`.

| Claim | Verdict | Evidence | Measured how |
| --- | --- | --- | --- |
| A. Stale-frame false confirm, opt 1/3 yes, opt 2 impossible | **Opt 1: confirmed. Opt 2 "impossible": refuted as sketched.** | `raw/claim_A.txt` S1, S2 | Host, real `ProfilePolicy.cpp` unmodified; radio/queue orderings modelled from the code |
| B. `tasks[0xFF]` OOB after failed init (opt 1/3) | **Confirmed in code, failure mode differs** | `raw/source_evidence.txt` (B) | Code reading. Not run |
| E. Engine pass races `smtc_rac_lock_radio_access` (opt 1) | **Confirmed in code** (no protect call in the function body, usp v1.0.0 smtc_rac.c:495-524) | `raw/source_evidence.txt` (E) | Code reading. Race not reproduced |
| F. v1.2.1 `rp_get_irq_flag` busy-loops prio-2 USP thread | **Mechanism confirmed in code; v1.0.0 pinned fork does NOT have it** | `raw/source_evidence.txt` (F) | Code reading of usp v1.2.1 tag and the pinned fork loop. Not run |
| G. Opt 2 RX service latency vs one frame airtime | **Budget computed, latency not tested** | `raw/airtime_G.txt` | Arithmetic. P5 252 B = 25.25 ms (includes an assumed 8-bit length field). Kick latency on RP2350 not measured |
| Re-arm cost ~50 ms | **Not tested** | none | Needs bench |
| Opt 3 priority inversion on rac_api_mutex | **Not tested** | none | Needs bench |
| Line counts 920 / ~690 / ~927 | **Not reproduced** | `raw/linecount.txt` | Non-blank, non-comment count: opt 1 = 775 (UspRadio.cpp 395 + RalSessionImpl.cpp 380). Opt 2 sketch = 364, but it is changed handlers only, so not comparable. Raw line count of UspRadio.cpp alone is 690, which matches the table's "~690" figure |

## A, detail

`raw/claim_A.txt`, from `sim/claim_A_stale_frame.cpp` (real `ProfilePolicy.cpp`):

- S1 (option 1 as shipped): SET_RX_PROFILE(P1, 30 s) sets pending, then a stale frame from the old P0 arm reaches
  `frameReceived()`. Result: committed P1, revert disarmed, tick(+31 s) = kNoOp. False confirm.
- S2 (option 2 sketch): same result. The sketch's `UspRadio::deferredSetRxProfile` calls
  `ProfilePolicy::setRxProfile` (UspRadio.cpp:524 in base) before `rearmRxWhenIdle()` runs `stop()`.
  `stop()` drains the RX lock inline and calls `rxFrame()`, which calls `frameReceived()` synchronously.
  The comment in `single-queue/UspRadio.cpp:81-84` ("a confirm from a frame of the previous profile cannot
  happen") does not hold, because the arm change is not what orders the frame. The policy state change is.
- S3 (control, stop before SET): stale frame is a no-op, revert fires correctly.
- S4 (control, genuine frame): confirms correctly.

So the claim's premise (option 2 is immune) is wrong. Option 2 needs the same fix as option 1: harvest or drain
before `setRxProfile`, or tag each frame with the arm epoch / profile it was received on.

## B, detail

`RalSessionImpl::init()` returns -ENODEV after `smtc_rac_open_radio()` returns RAC_INVALID_RADIO_ID (0xFF),
and `m_initialized` stays false. `UspRadio` returns false from startup on that path, but later commands still
reach the session: `deferredSetRxProfile` and `transmit` call `stopRadio()`, and `rearmRx()` calls
`startReceive()`. `startReceive()` reads `rp->tasks[m_radio_id]` at RalSessionImpl.cpp:398 before any RAC call.
`stopRadio()` calls `smtc_rac_unlock_radio_access(m_radio_id)` first, and the USP guard
`SMTC_RAC_CHECK_RADIO_ACCESS_ID` panics for ids >= RP_HOOK_ID_MAX. So the likely failure is a USP panic on the
first RAC call, not a silent read. I did not confirm RP_HOOK_ID_MAX's value, so the panic path is inferred.
The OOB read in `startReceive()` is real if the guard is bypassed. Option 2's `if (!ready()) return -ENODEV`
refuses before any tasks[] access.

## E, detail

`smtc_rac_lock_radio_access` (usp v1.0.0 pinned, smtc_rac.c:495-524) builds a task and calls
`rp_task_enqueue` with no `smtc_modem_hal_protect_api_call`. The count of protect calls inside those lines is 0.
Other RAC entry points do call protect (for example lines 421-422, 459-460). The race needs the engine on the USP
thread concurrently with a caller-thread lock, which is exactly option 1. Not reproduced.

## F, detail

- usp v1.2.1 `rp_get_irq_flag` (radio_planner.c:600-613) also returns true while
  `tasks[radio_task_id].type == RP_TASK_TYPE_LOCK_RADIO_ACCESS`. The v1.0.0 pinned version (radio_planner.c) does not.
- The pinned usp_zephyr loop (`zephyr_usp_thread.c:131-133`) does `continue` without sleeping when
  `smtc_rac_is_irq_flag_pending()` is true. With continuous RX holding the lock, that means no msleep, so the
  prio-2 thread busy-loops. This applies to v1.2.1 only if the usp_zephyr loop is the same at v1.2.1. I read the
  pinned fork's loop, not a v1.2.1 usp_zephyr checkout. Also, `radio_task_id` as the lock task index is inferred.
- Not run on target. Stated as "confirmed in code, mechanism assumed to hold on v1.2.1 usp_zephyr".

## G, detail

Option 2 RX service waits behind the component queue. One frame airtime at P5 is 25.25 ms (252 B payload).
That is the budget; the real kick latency on RP2350 with the F´ deployment thread mix is not measured.
The board was not used.

## Skipped

Re-arm cost (50 ms), opt 3 priority inversion, RP2350 kick-latency distribution, and the on-target
reproduction of A, B, E. All need the dev bench with the lock.
