#!/bin/sh
# Produce sim/opt3/RalSessionImpl.cpp = #26's RalSessionImpl.cpp + the
# prototype's upstream-mutex diff (PROTOTYPE-589-concurrency/upstream-mutex/
# RalSessionImpl.cpp.diff).  The diff was generated with 1-line context, so
# `patch` is run with fuzz; any rejected hunk fails the build loudly.
set -e
USPRADIO="$1"; HERE="$2"
OUT="$HERE/sim/opt3"
mkdir -p "$OUT"
cp "$USPRADIO/RalSessionImpl.cpp" "$OUT/RalSessionImpl.cpp"
cp "$USPRADIO/RalSessionImpl.hpp" "$OUT/RalSessionImpl.hpp"
# Strip the prose header (everything before the first 'diff --git').
sed -n '/^diff --git/,$p' "$HERE/../upstream-mutex/RalSessionImpl.cpp.diff" > "$OUT/opt3.patch"
cd "$OUT"
patch -p3 -F3 --no-backup-if-mismatch RalSessionImpl.cpp < opt3.patch
# The diff says the header "also needs: rp_task_states_t hookState() const;".
grep -q hookState RalSessionImpl.hpp || \
  sed -i '' 's|    void quiesceRadio();|    void quiesceRadio();\n    rp_task_states_t hookState() const;|' RalSessionImpl.hpp
# Make the sim build pick up the opt3 header instead of #26's.
sed -i '' 's|#include "fprime-zephyr/Drv/UspRadio/RalSessionImpl.hpp"|#include "RalSessionImpl.hpp"|' RalSessionImpl.cpp
echo "opt3 sources generated in $OUT"
