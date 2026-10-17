#!/bin/sh
# One-command rerun of the arena-c4 measurements (host only; no board, no RF).
# Usage: sh run.sh [usp_v100_dir] [usp_v121_dir] [usp_zephyr_dir]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../../../.." && pwd)          # worktree root (c4)
SCR=${ARENA_SCRATCH:-$ROOT/../c4-scratch}
mkdir -p "$HERE/raw" "$SCR"
USP100=${1:-$HOME/GitHut/pcr-588/lib/zephyr-workspace/modules/lib/usp}
USP121=${2:-$SCR/usp-v1.2.1}
USPZ=${3:-$HOME/GitHut/pcr-588/lib/zephyr-workspace/modules/lib/usp_zephyr}
RAW="$HERE/raw"

# Claim A: real ProfilePolicy, host build
clang++ -std=c++20 -Wall -DLINK_PROFILES_USE_HOST_TYPES -I"$ROOT" \
  "$HERE/sim/claim_A_stale_frame.cpp" "$ROOT/fprime-zephyr/Drv/UspRadio/ProfilePolicy.cpp" -o "$SCR/claim_A"
"$SCR/claim_A" > "$RAW/claim_A.txt"

# Claim G arithmetic
python3 "$HERE/sim/airtime_G.py" > "$RAW/airtime_G.txt"

# Line counts
sh "$HERE/sim/linecount.sh" "$ROOT/fprime-zephyr/Drv/UspRadio" > "$RAW/linecount.txt"

# Claims B, E, F: source evidence (grep + sed on pinned and v1.2.1 sources)
{
  echo "## B: init failure leaves m_radio_id = RAC_INVALID_RADIO_ID; startReceive() reads tasks[] first"
  grep -n "RAC_INVALID_RADIO_ID" "$ROOT/fprime-zephyr/Drv/UspRadio/RalSessionImpl.cpp" "$USP100/smtc_rac_lib/smtc_rac/smtc_rac.h"
  grep -n "tasks\[m_radio_id\]" "$ROOT/fprime-zephyr/Drv/UspRadio/RalSessionImpl.cpp" | head -3
  echo "## E: smtc_rac_lock_radio_access has no protect call (v1.0.0 pinned)"
  sed -n 495,524p "$USP100/smtc_rac_lib/smtc_rac/smtc_rac.c" | grep -c "protect_api_call" || true
  echo "## F: rp_get_irq_flag at v1.0.0 vs v1.2.1"
  sed -n '/^bool rp_get_irq_flag/,/^}/p' "$USP100/smtc_rac_lib/radio_planner/src/radio_planner.c"
  sed -n '/^bool rp_get_irq_flag/,/^}/p' "$USP121/smtc_rac_lib/radio_planner/src/radio_planner.c"
  echo "## F: usp_zephyr loop (pinned fork): busy loop when irq flag pending"
  grep -n "is_irq_flag_pending\|msleep\|continue;" "$USPZ/subsys/usp/zephyr_usp_thread.c"
} > "$RAW/source_evidence.txt"
echo "done: $RAW"
