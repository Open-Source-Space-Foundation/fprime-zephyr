# Arena #589 candidate c2: host simulation of the three UspRadio concurrency options

Everything above the SX126x SPI edge is real code:

| Layer | Source | Real or fake |
| --- | --- | --- |
| radio planner, smtc_rac, RAL, RALF, sx126x driver | `usp` v1.0.0 (PCR pin) and Lora-net v1.2.1 | real, compiled unmodified |
| `RalSessionImpl.cpp`, `ProfilePolicy.cpp` | this worktree (#26 @ 25d5cd2) | real, compiled unmodified (option 3: + the prototype's diff, `sim/opt3/`) |
| `RalSessionSt` (option 2) | `sim/opt2/`, completed from the prototype sketch | real-shaped, runs against the real planner |
| UspRadio handlers | `sim/opt1/UspRadioSim1.cpp` (options 1 and 3), `sim/opt2/UspRadioSim2.cpp` | lifted from `UspRadio.cpp` / the sketch; F´ autocode replaced by a priority queue + counters |
| `usp_main_thread` | `sim/usp_thread.cpp` | replica of `zephyr_usp_thread.c:96-141` |
| `smtc_modem_hal`, SX1262, DIO1, SPI timing | `sim/hal_fake.cpp`, `sim/sx126x_hal_fake.cpp` | fake (opcode-level chip model; SPI bytes x 8 / 125 kHz, the v5e `spi-max-frequency`) |
| Zephyr kernel | `sim/shim/zephyr/kernel.h`, `sim/zkernel.cpp` | pthread shim; no priorities |

## Run

```sh
make        # builds 10 binaries into ~/GitHut/arena-589/c2-scratch/build (override BUILD=)
make run    # reruns every measurement; raw logs in results/, digest in results/SUMMARY.txt
```

Needs `USP_V100` (default: the pcr-588 zephyr workspace) and `USP_V121` (default: a clone at
`~/GitHut/arena-589/c2-scratch/usp-v1.2.1`; `git clone --branch v1.2.1 https://github.com/Lora-net/usp`).

Binaries: `opt{1,2,3}_v100`, `opt{1,2}_v121`, `_bounds` (UBSan array bounds), `_tsan` (built, not used; see RATIONALE.md).
Scenarios: `smoke`, `stale`, `initfail`, `race`, `busyloop`, `txcost`, `rxlat` (see `sim/scenarios.cpp`).
Knobs: `SIM_SPI_HZ`, `SIM_TX_AIRTIME_MS`, `SIM_PROTECT_MODE` (0 = upstream bare stub, 1 = fork mutex), `SIM_VERBOSE`.

## Files

- `RESULTS.md` per-claim verdicts with numbers and log paths.
- `RECOMMENDATION.md` which option, what would change it, remaining bench drills.
- `RATIONALE.md` approaches considered and rejected.
- `tools/gen_opt3.sh` applies the upstream-mutex diff to #26's `RalSessionImpl.cpp` (output `sim/opt3/`).
- `tools/linecount.sh` the table's line / primitive counts.
