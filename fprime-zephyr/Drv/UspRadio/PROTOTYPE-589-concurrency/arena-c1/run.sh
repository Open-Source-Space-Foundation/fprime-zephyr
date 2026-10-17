#!/usr/bin/env bash
# One command: build the #589 host harness and rerun every measurement.
#   ./run.sh            # ~8 min on an M-series Mac
# Writes results/raw/<variant>_<scenario>[_<k>].log and results/summary.tsv.
# Needs: clang (Xcode CLT), python3, the usp v1.0.0 fork at
# ~/GitHut/pcr-588/lib/zephyr-workspace/modules/lib/usp and usp v1.2.1 cloned
# to ~/GitHut/arena-589/c1-scratch/usp-v1.2.1 (the script clones it if absent).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SCRATCH="${SCRATCH:-$HOME/GitHut/arena-589/c1-scratch}"
B="$SCRATCH/build"
RAW="$HERE/results/raw"
TMO="$HERE/sim/tmo"
mkdir -p "$RAW"
rm -f "$RAW"/*.log

if [ ! -d "$SCRATCH/usp-v1.2.1" ]; then
  git clone -q --depth 1 --branch v1.2.1 https://github.com/Lora-net/usp.git "$SCRATCH/usp-v1.2.1"
fi
make -C "$HERE/sim" -j8 B="$B" \
  "$B/sim_opt1" "$B/sim_opt1nomx" "$B/sim_opt3" "$B/sim_opt1v121" "$B/sim_opt2" "$B/sim_opt2v121" \
  "$B/sim_opt1fix" >"$SCRATCH/build.log" 2>&1 || { tail -30 "$SCRATCH/build.log"; exit 1; }

run() {  # run <log-name> <timeout-s> <variant> <scenario> [n]   (env passes through)
  local log="$RAW/$1.log"; shift
  local t="$1"; shift
  local v="$1"; shift
  "$TMO" "$t" "$B/sim_$v" "$@" >"$log" 2>&1 || echo "EXIT	$v	$1	rc=$?" >>"$log"
}

# A: stale-frame false confirm (10 injected trials, 3 controls)
# (opt1fix = option 1 + fix-stale-epoch.diff)
for v in opt1 opt3 opt1v121 opt2 opt2v121 opt1fix; do
  run "${v}_stale" 120 "$v" stale 10
  run "${v}_stale_ctrl" 60 "$v" stale_ctrl 3
  run "${v}_confirm_new" 60 "$v" confirm_new 3
done
# B: failed init / dead session, each RalSession entry point
for v in opt1 opt3 opt2; do
  for op in stop apply rx tx; do
    SIM_DEAD_OP=$op run "${v}_init_fail_${op}" 30 "$v" init_fail
  done
done
# E: concurrency stress, 3 x 10 s per variant (10 ms ComDelay ticks)
for v in opt1 opt1nomx opt3 opt2 opt1fix; do
  for k in 1 2 3; do run "${v}_stress_${k}" 60 "$v" stress 10; done
done
# RX duty cycle with the real ComDelay pacing (100 ms ticks)
for v in opt1 opt2; do SIM_COMDELAY_MS=100 run "${v}_stress_comdelay100" 60 "$v" stress 10; done
# F: engine passes while RX is armed, v1.0.0 vs v1.2.1
for v in opt1 opt1v121 opt3 opt2 opt2v121; do run "${v}_busyloop" 60 "$v" busyloop 2; done
# Re-arm cost (structure) and TX saturation
for v in opt1 opt3 opt2; do
  run "${v}_rearm" 60 "$v" rearm 100
  run "${v}_txsat" 60 "$v" txsat 40
  run "${v}_starve" 30 "$v" starve 12
done
# option 3 compile check with the target toolchain (GCC rejects __auto_type in C++)
GXX="$(ls "$HOME"/zephyr-sdk-*/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-g++ 2>/dev/null | head -1 || true)"
if [ -n "$GXX" ]; then
  { "$GXX" --version | head -1; "$GXX" -std=gnu++17 -c "$HERE/sim/src/check_auto_type.cpp" -o /dev/null 2>&1 \
      && echo "GCC_CXX_AUTO_TYPE=ok" || echo "GCC_CXX_AUTO_TYPE=error"; } >"$RAW/opt3_gcc_auto_type.log" 2>&1
fi
clang++ -std=gnu++17 -c "$HERE/sim/src/check_auto_type.cpp" -o /dev/null && echo "CLANG_CXX_AUTO_TYPE=ok" >>"$RAW/opt3_gcc_auto_type.log"

python3 "$HERE/tools/count.py" "$(cd "$HERE/../../../../.." && pwd)" "$B" >"$RAW/line_counts.tsv"
python3 "$HERE/tools/airtime.py" >"$RAW/airtime.tsv"

{ printf 'variant\tscenario\tkey\tvalue\tlog\n'
  for f in "$RAW"/*.log; do
    awk -v L="$(basename "$f")" -F'\t' '$1=="RESULT"{print $2"\t"$3"\t"$4"\t"$5"\t"L}' "$f"
  done; } >"$HERE/results/summary.tsv"
grep -H -E "^(EXIT|TIMEOUT|FW_ASSERT|SMTC_MODEM_HAL_PANIC)" "$RAW"/*.log | sed "s|^$RAW/||" >"$HERE/results/aborts.txt" || true
echo "wrote $HERE/results/summary.tsv ($(wc -l <"$HERE/results/summary.tsv") rows)"
