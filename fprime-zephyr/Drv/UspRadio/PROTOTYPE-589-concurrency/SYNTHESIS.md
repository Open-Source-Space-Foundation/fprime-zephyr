# #589 arena synthesis: concurrency model, measured

Four candidates simulated the three options on the host, all compiling the real usp planner/RAC code.
Nobody used the dev board: two candidates had SSH to the bench Pi denied by the permission classifier,
and two chose to stay on the host. **Every timing number here is a host-model number.**

| Candidate | Branch @ sha | Approach | Judge score /30 |
| --- | --- | --- | --- |
| **c3 (base)** | `arena/589-c3` @ 9c84398 | virtual-time strict-priority Zephyr kernel + SX1262/SPI model; real planner, RAC, RAL, sx126x driver, `sx126x_hal.c`, `zephyr_usp_thread.c`, UspRadio.cpp, RalSessionImpl.cpp | 30 |
| c2 | `arena/589-c2` @ 45bb86a | pthreads + SPI byte-time model; real stack down to the HAL | 25 |
| c1 | `arena/589-c1` @ d4db2e9 | pthreads, fake at `ral_drv_t` (20 us SPI) | 24 |
| c4 | `arena/589-c4` @ e3676cd | real ProfilePolicy only, orderings hand-modelled | 16 |

Base: **c3**, merged here as `arena-c3/` (rerun `arena-c3/run_all.sh`, ~50 s, deterministic). The cross-judge
(a different model family) picked the same base. Verification: a full rerun reproduced `results/SUMMARY.txt`
byte for byte.

## Verdicts after the arena

| Claim (prototype table) | Verdict | Numbers (host) |
| --- | --- | --- |
| A. Old-profile frame confirms a profile switch (1, 3) | **confirmed** | 3–15% of SET_RX_PROFILE at 10–83 fps stale stream (c3); 25/200 (c2); 10/10 (c1). Option 2: 0/360 |
| A. Option 2 immune | **confirmed, with a cost** | abort retypes the RX LOCK to UNLOCK (radio_planner.c:398-400), so the pending IRQ reports RADIO_UNLOCKED (:797-800) and the frame is never read: the in-flight old-profile frame is **dropped** (c1). c4's contrary claim was a hand model and is wrong |
| A. Fixable inside 1/3 | **yes** | epoch tag on ring slots: 0 false confirms, old frame still delivered (c3 `fixes/epoch_tag.diff`, 8 lines; c1 24-line variant) |
| B. `tasks[0xFF]` OOB after failed init | **refuted; real effect is a reset** | first RAC call with id 0xFF panics at `SMTC_RAC_CHECK_RADIO_ACCESS_ID` (smtc_rac.c:475/498), resetting the MCU; and `open_radio` cannot even return INVALID (rp_hook_init panics first, :257-269). Does not separate the options |
| E. Engine pass races caller RAL access (1) | **confirmed, severity depends on SPI mode** | polled SPI (the v5e `.config`): radio stalls permanently in 26–58% of 20 s storms, deterministic 0–650 us window after RX_DONE (c3). Options 3 and 2: 0 in 16k/17k ops |
| F. usp v1.2.1 USP thread busy-loop (1, 3) | **confirmed** | 98.7% CPU whenever a LOCK is current; rateGroup10Hz/1Hz/modeManager run 0 cycles in 9.3 s (c3); 1.2×10⁷ passes/s (c1). Option 2: 0 |
| G. Option 2 RX service waits behind the queue | **confirmed; bounded by thread priorities, not the queue** | kick latency 0.03 ms median. uspRadio is Zephyr prio 11, *below* rate groups 1–3 and modeManager 4, so P5 252 B frames at 25 ms gaps lose 2/3.5/8% under 5/20/50 ms rate-group bursts (option 1: 0). Flip side: option 1's prio-2 harvest delays rateGroup1Hz 16 ms and modeManager up to 26 ms |
| Re-arm ~50 ms (1) vs ~1 ms (2) | **both wrong** | 19–24 ms in both; the floor is the planner sleeping the chip + 10 ms TCXO restart (DTS), plus 125 kHz SPI (c3). The skip-re-arm never fires in PCR: ComDelay paces downlink ≥100 ms (10 Hz tick, `DIVIDER` param) (c1, checked) |
| Queue blocked in TX / CW (1, 3) | **confirmed** | a command waits 0.58 s behind a 252 B SF8 TX, 4.9 s behind 5 s CW, 9.9 s behind a hung TX; option 2: 0.1 ms |
| Option 3 priority inversion | **bounded** | USP wait on `rac_api_mutex` < 5 us normally, ≤ 29 ms under stray-IRQ stress (c3) |
| Option 3 diff compiles | **no** | `SMTC_SW_PLATFORM` uses `__auto_type`; the build's `arm-zephyr-eabi-g++` 14.3.0 rejects it in C++ (c1, c2); needs a C++ guard |
| Code size | **option 2 smaller than estimated** | 920 / 924 / 638 code lines (c3, option 2 completed) |

## Option-independent findings (fix whichever option ships)

1. **SPI is 125 kHz and polled** (`spi-max-frequency = <125000>`, `CONFIG_SPI_PL022` without INTERRUPT/DMA). A
   252 B harvest is 16–18 ms of CPU, 72% of a P5 airtime; at 5 ms frame gaps every option delivers half the frames.
   Raising the clock (and checking IRQ/DMA mode) is worth more than the concurrency choice.
2. **The async `run` port is priority 0 behind priority-10 internal ports**: under continuous TX the 1 Hz tick
   starved for 6 s, and with ComDelay bypassed the queue fills and FW_ASSERTs (c1, c2). Hidden today by ComDelay.
3. **`RadioInstances_Usp.fppi` says priority 11 is "above the rate groups".** It is below them (Zephyr: larger
   number = lower priority).
4. **RAC abort corner** in options 1/3: the hook sits `ABORTED` until `stopRadio`'s 1 s deadline about once per
   4 s stress run (c2); not seen in option 2.

## Grafts and rejections

- From **c1**: the LOCK→UNLOCK mechanism and the frame-drop cost of option 2; ComDelay pacing makes the
  skip-re-arm moot (so c3's "restore the skip, +9% on a burst" is **rejected** for PCR: bursts cannot happen while
  ComDelay paces); the nested `pump()` in `txDone → finishFrame → armRx` as the sketch's top structural risk; the
  `run`-port starvation; gcc 14.3.0.
- From **c2**: the SPI byte budget (P5 slack ~7 ms, not 25) and the abort corner.
- From **c4**: nothing measured survived; its one durable point is that option 2's A-immunity rests on
  radio_planner.c:400, so a planner upgrade must be re-checked against it.
- **Rejected:** c1's "run UspRadio at prio ≤2" as a default. c3 measured that prio 3 removes G's loss but brings back
  option 1's rate-group disturbance, and c1 itself notes the whole RX deframe chain would then run at prio 2.
  Decide the number from bench rate-group bursts.

## Updated recommendation

**Option 2 (single queue, `CONFIG_USP_MAIN_THREAD=n`), unchanged in direction, with stronger evidence and four
corrections:**

1. It is the only option that closes A, E, F and the blocked queue together, and the only one that can take
   usp v1.2.1 without patching the planner. Stay on v1.0.0 until it lands.
2. Fold in the sketch fixes the candidates needed to run it: set `m_state` before `smtc_rac_lock_radio_access`
   (the pre-callback can fail and unlock inside it), `rp_disable_failsafe(rp, true)`, the missing
   `lorawan_hal_init.h` include.
3. Keep always-re-arm (ComDelay ≥100 ms makes the skip worthless in PCR). Accept that an old-profile frame in
   flight at a profile change is dropped, not delivered.
4. Choose uspRadio's priority from measured rate-group bursts, and fix the misleading fppi comment.

**Fallback** if the bench shows rate-group bursts above ~20 ms *and* back-to-back P4/P5 uplink is needed: option 3
+ epoch tag + a C++-safe mutex guard, pinned to usp v1.0.0. Closes A and E; leaves F and the blocked queue.

## Bench drills that remain (dev v5e, RX and synthetic load only, no RF)

| # | Drill | Decides |
| --- | --- | --- |
| 1 | Time a 252 B ReadBuffer and a GetStatus; confirm 125 kHz and polled mode | every magnitude above |
| 2 | `RgMaxTime` of rateGroup50Hz/10Hz/1Hz and modeManager in the real deployment (or a prio-11 probe thread) | G and uspRadio's priority |
| 3 | Option-2 build: DIO1→harvest histogram, and 10k start/stop/arm + `txDone→armRx` nested-pump cycles | G on target; nested-pump safety |
| 4 | Option 1 on v1.2.1 with `CONFIG_THREAD_RUNTIME_STATS` | F on target |
| 5 | Sleep→RX-listening time (cycle counter) | the 10 ms TCXO floor |
| 6 | HAL re-entrancy counter on option 1 with a second radio sending (CI bench has the peer) | E on target |
