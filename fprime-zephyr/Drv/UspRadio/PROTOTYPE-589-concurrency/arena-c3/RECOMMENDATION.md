# Recommendation (c3): option 2, single queue, after three bench checks

Evidence is host simulation of the real code (see RESULTS.md); no on-target run was possible from this session.

## Why option 2

| | 1 (#14+#26) | 3 (upstream mutex) | 2 (single queue) |
| --- | --- | --- | --- |
| A stale-frame false confirm | open (3 to 15% of commands at 10 to 83 fps) | open | closed (0/360) |
| E engine vs caller race | radio stalls, 1 per 100 to 3000 ops | closed (0 in 16.3k ops) | closed (0 in 17.7k ops) |
| F usp v1.2.1 busy loop | rg10/rg1/modeManager starve | same | not exposed |
| Queue blocked by TX / CW / hung TX | 0.6 s / 4.9 s / 9.9 s | same | none |
| Disturbance of rg1 / modeManager by RX harvest | up to 16 / 26 ms | same | none (they preempt it) |
| RX loss when a higher-priority burst exceeds the idle gap | none | none | 2 to 8% at 25 ms gap, bursts 5 to 50 ms |
| Code | 920 lines | 924 | 638 |
| Changes needed | n/a | 11 call sites + Kconfig | rewrite of RalSession and UspRadio handlers |

Option 2 is the only one that closes A, E, F and the blocking at once. Its one new cost (G) is bounded and has
a knob. Option 3 is the best fallback: it closes E, and with `fixes/epoch_tag.diff` it closes A, but F and the
blocked queue remain, and v1.2.1 stays unusable.

## Do before building it for target

1. Keep `ARENA-c3:` fixes: set `m_state` before `smtc_rac_lock_radio_access`, `rp_disable_failsafe(rp, true)`.
2. Restore a skip-re-arm: the sketch always re-arms, costing +9% on a TX burst (+2% at 8 MHz SPI). A single
   "next TX frame already queued" flag is enough; it does not need the 26 atomics.
3. Fix the `RadioInstances_Usp.fppi` comment (priority 11 is below the rate groups) and decide the number with
   the bench rate-group bursts: if rateGroup1Hz or modeManager can run longer than the shortest inter-frame gap
   you must carry (25 ms at P5 252 B), either accept loss, or move uspRadio to 3 and accept option-1-like
   disturbance of those two groups (RESULTS G).
4. Pin usp v1.0.0 until option 2 lands. Do not move to v1.2.1 with options 1 or 3.

## What would change my mind

- Bench shows SPI is IRQ-driven or much faster than 125 kHz, and no frame rate above 1 per second is planned:
  E drops to a benign lost quiesce command and A to under 1%, so option 3 plus the epoch tag (a day of work)
  is enough, and option 2's rewrite risk is not worth it.
- Bench rateGroup1Hz / modeManager bursts exceed about 20 ms and the mission needs back-to-back GFSK frames:
  option 2's 4 to 8% loss then costs more than option 3's disturbance.
- A requirement to adopt v1.2.1 soon pushes the other way (option 2 only).

## Bench drills that remain (dev board, RX and synthetic load only, no RF)

1. **SPI mode and speed:** time a 252-byte ReadBuffer and a GetStatus through `sx126x_hal_read`; confirm the
   125 kHz clock and that the thread burns CPU (cycle counter). Every magnitude scales with this.
2. **Rate-group bursts:** read `RgMaxTime` of rateGroup50Hz/10Hz/1Hz and modeManager (or a priority-11 probe
   thread logging wake latency for 10 minutes in the real deployment). This is G's missing input.
3. **HAL re-entrancy detector** (flag set on entry to `sx126x_hal_write/read`, count and log overlap): run
   SET_RX/SET_TX/TX loops with a second radio sending a frame about every 100 to 500 ms. Option 1 should show
   overlaps; this converts E from model to target evidence and gives the true stall rate.
4. **v1.2.1 CPU:** build option 1 on v1.2.1, read thread runtime stats; expect about 98% in idle RX.
5. **TCXO/BUSY timing:** measure sleep-to-RX-listen on the scope or with the cycle counter; the 10 ms floor in
   the re-arm numbers comes from the DTS, not from a measurement.
6. Then port `RalSessionSt` + `UspRadioSt` to the F-prime deployment and rerun `scen/stale`, `phase` and `blocked`
   as bench scripts.
