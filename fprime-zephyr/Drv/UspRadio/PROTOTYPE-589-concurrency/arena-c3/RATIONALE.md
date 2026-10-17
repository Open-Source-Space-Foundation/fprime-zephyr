# RATIONALE: approaches considered and rejected

The task: test the prototype table's claims about the three UspRadio concurrency options with real code
where possible. What I built: the real usp radio planner, smtc_rac, ral/ralf/sx126x driver, usp_zephyr
`sx126x_hal.c`, `smtc_modem_hal.c`, `zephyr_usp_thread.c`, plus the real `UspRadio.cpp` and
`RalSessionImpl.cpp`, all compiled for macOS. The fakes sit at four edges: a virtual-time Zephyr kernel
(`zsim`), an SX1262 + SPI bus + GPIO model, a fake of the FPP-generated component base, and the rate-group
threads. Option 2 is the sketch completed into compilable code, and option 3 is the prototype's own diff
applied to #26's `RalSessionImpl.cpp`.

## Rejected

1. **Re-model the three options in Python or an abstract state machine (the HTML demo's approach).**
   That tests my model of the code, not the code. Every interesting finding here (the planner's
   `rp_task_arbiter` tail running after an inline pre-callback, the HAL's unlocked `radio_status`, the
   8-hook sleep loop after a stray DIO1) lives in code the demo abstracts away. Rejected.

2. **Host build on real pthreads and the OS scheduler.**
   macOS has no strict-priority preemptive scheduling for normal processes. Claims F and G are about
   priority 2 vs 3 vs 11, and about CPU starvation. A race that depends on "a prio-2 thread preempts a
   prio-11 thread inside `quiesceRadio()`" would also be unrepeatable and untimed. The deterministic
   virtual-time kernel (`zsim`) gives strict priorities, priority-inheriting mutexes, 20 ms equal-priority
   slicing, tick-rounded sleeps, and exact repeatability. Its cost is that preemption happens only at
   kernel calls, SPI transfers and `zs_burn()`. It cannot interleave inside a C statement, so races that
   need instruction-level interleaving (the planner tearing a task struct, which is what HWIL saw in July)
   cannot be found this way. I say so under claim E.

3. **Zephyr `native_sim` or QEMU with the real kernel.**
   This is the better fidelity story (real `k_mutex`, real scheduler, real `spi_pl022` replaced at the
   driver API). I did not take it because a west workspace with the PCR out-of-tree modules, F-prime
   autocoding and a fake SPI/GPIO driver was a much longer path than the time-box allowed, and the
   guardrail was "do not build in the shared pcr-588 tree". It is the natural next step if the decision
   needs more than this evidence. `zsim`'s kernel semantics are small enough to re-check against Zephyr
   by reading (see RESULTS "Model limits").

4. **Compile the real F-prime autocoded `UspRadioComponentBase`.**
   It needs `fpp`, the F-prime Fw/Os layers, and a queue implementation. What matters for these claims is
   narrow: handler dispatch order from one priority queue (higher priority value first, FIFO within a
   priority, as in `Os/Generic/PriorityQueue`), which ports are sync vs async, and the per-call CPU cost.
   The fake base reproduces those and nothing else. `UspRadio.cpp` itself is compiled unmodified except
   for `-D__ZEPHYR__` (without it `nowMs()` returns 0 on a host build and the auto-revert can never fire).

5. **Model the radio as "SPI takes ~1 ms".**
   Rejected after reading the board DTS and `.config`: `spi-max-frequency = <125000>` and
   `CONFIG_SPI_PL022=y` with neither `_INTERRUPT` nor `_DMA`. A 252-byte `ReadBuffer` is 16 ms of
   *polled* CPU at the caller's priority. That one fact changes G, the re-arm cost and the severity of E,
   so the chip model keeps the real `sx126x_hal.c` (wake glitch, BUSY polling, `radio_status`) above it
   and puts the fake at the SPI/GPIO API. An IRQ-driven SPI variant is kept as a switch
   (`CHIP_SPI_POLL=0`) so the sensitivity is visible.

6. **Use the dev board for timing.**
   The grounding allows it, and I wanted it for rate-group burst lengths and SPI timing. The bench Pi
   connection was refused by this session's permission classifier, and I did not look for a way around
   that. So everything timing-related here is host-model evidence, flagged as such. RECOMMENDATION.md
   lists the bench drills to turn it into target evidence.

7. **Fix option 2's sketch silently.**
   I completed it as written and marked every deviation `ARENA-c3:` in the source. Three needed changes
   (state set before the enqueue, `rp_disable_failsafe(rp, true)`, a thread stack argument); the
   throughput regression (always re-arm) is left as sketched and measured, not patched, so the table's
   claim is tested as written.
