# Recommendation for #589

**Adopt option 2 (single queue, `CONFIG_USP_MAIN_THREAD=n`), and do it before any move to usp v1.2.1.** Keep #26 flying until the option-2 RalSession has passed the bench drills below. Do not spend effort on option 3.

## Why (what the simulation measured; details and log paths in RESULTS.md)

1. **The stale-frame false confirm is real in options 1 and 3 and absent in option 2.** With the real planner, RAC and `RalSessionImpl.cpp`, 4-8 of 100 `SET_RX_PROFILE` changes under RX load were confirmed by a frame received on the *old* profile (up to 15 % in the stress run). Option 2: 0 of 100 / 0 of 200. The fix inside options 1/3 is small (tag ring slots with the armed profile, or drain the ring before arming), so by itself this is not decisive, but it is the kind of cross-thread ordering bug the ring-plus-messages design keeps producing.
2. **usp v1.2.1 makes the option-1/3 USP thread spin at 100 % CPU for the whole continuous-RX window, starting at the first received frame** (85.7 M engine passes in 2 s on the host). Option 2's bounded pump does not spin (0 extra passes on v1.2.1). PCR will have to take v1.2.1 eventually (Zephyr 4.4 support, the DT changes); option 2 is the only shape that survives that without patching the planner or demoting the USP thread below every F´ thread.
3. **A failed `smtc_rac_open_radio` is a reboot loop in options 1/3, not an out-of-bounds read.** The first `stopRadio()` after a failed init calls `smtc_rac_unlock_radio_access(0xFF)`, which hits `SMTC_RAC_CHECK_RADIO_ACCESS_ID` and `SMTC_MODEM_HAL_PANIC` (MCU reset). `tasks[0xFF]` is never reached. Option 2's `ready()` guard refuses every operation with `-ENODEV` and nothing panics.
4. **The lock/engine race (claim E) is measurable but rare, and the fork mutex already removes most of it.** Cross-thread overlaps of `rp_task_enqueue` with a running `rp_callback`: bare upstream stub 9-15 per 4 s stress (with 3-4 TX/config failures), fork mutex 0-2, option 3 and option 2 0. Option 3 closes the last gap but costs a C++-incompatible macro (`__auto_type` does not compile with the Zephyr SDK's `arm-zephyr-eabi-g++`; the shipped diff needs its own wrapper) and keeps every other defect of option 1.
5. **Option 2's claimed costs did not materialise.** With the v5e SPI clock (125 kHz) modelled from the real driver's byte counts: a stop + profile apply + RX arm is 6.6-7.2 ms of SPI in both designs, not "~50 ms of cross-thread waits"; option 1's skip-the-re-arm trick saves ~18 ms per saturated frame (81 -> 63.5 ms/frame) and option 2 without the trick is 66 ms/frame, i.e. 4 % slower at saturation and 19 % faster when frames are paced. RX service latency is 17.9 ms of SPI read in every option; under light mixed load option 2's end-to-end RX delivery was 18.3 ms max against 103 ms max for option 1 (the component thread was inside a blocking TX episode). Under saturated downlink both options are deaf to RX; that is half-duplex, not a design property.
6. **Line counts are as claimed** (920 / ~924 / smaller for option 2: the completed session is 360 code lines against 775 for `UspRadio.cpp`+`RalSessionImpl.cpp`, with 3 semaphores and 0 `tasks[]` reads).

## What would change my mind

- An on-target measurement showing the engineKick message waits > 7 ms behind other UspRadio handlers at P4/P5 frame rates (the slack between 25.2 ms airtime and the 17.9 ms harvest). The host sim cannot see Zephyr priorities; the component runs at priority 11 behind three rate groups and ModeManager.
- An RX-overwrite rate on the bench at P5 higher in option 2 than in #26 with the same frame spacing.
- A reason PCR will never take v1.2.1 (then finding 2 is moot and a patched option 1 is defensible).

## Bench drills that remain (dev board, no RF)

1. **Kick latency.** Build option 2 with a GPIO toggle (or cycle counter) at DIO1 edge and at the start of the engineKick handler; drive the component with synthetic `SET_TX_PROFILE`/`TRANSMIT` traffic at 1-10 Hz; record max. Budget: 7 ms (P5), 10 ms (P4), 37 ms (P3).
2. **Re-arm per frame at P3/P4/P5** with the real SPI: confirm the 6.6 ms host figure and the per-frame cost against #26's skip.
3. **v1.2.1 busy loop** on target with `CONFIG_THREAD_RUNTIME_STATS`: confirm the host result (USP thread CPU after the first RX frame) before and after the move.
4. **Queue sizing.** The async `run` port is priority 0 behind the priority-10 internal ports; under continuous TX the 1 Hz tick waited the whole 6 s run in both designs. 20 queued ticks is an `FW_ASSERT`. Either raise `run`'s priority or bound TX bursts; this is independent of the option chosen.
5. The RAC abort corner seen once per 4 s stress in options 1/3 (hook stuck `ABORTED` for the 1 s `stopRadio` deadline): check whether option 2's synchronous `pump()` after abort removes it on hardware as it did on the host.
