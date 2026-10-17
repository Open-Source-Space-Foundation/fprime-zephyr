# arena-c3: host simulation of the three UspRadio concurrency options (#589)

`./run_all.sh` rebuilds and reruns everything (needs clang, make, python3, network for the v1.2.1 clones).
Read RESULTS.md, then RECOMMENDATION.md, then RATIONALE.md.

- `zsim/` virtual-time Zephyr kernel; `chip/` SX1262 + SPI + GPIO model; `shim/` Zephyr/F-prime headers;
  `hal/` host stubs and call-site interposers; `opt2/` option 2 completed from the sketch; `fixes/` epoch-tag patch.
- `scen/*.cpp` one program per claim; `tools/sweep.py` runs a scenario over an env grid; `tools/summarize.py`,
  `tools/plots.py` build `results/SUMMARY.txt` and `results/figs/`.
- Variants: `make VARIANT=opt0|opt1|opt2|opt3[_v121|_asan|_epoch] bins` (see Makefile header).
