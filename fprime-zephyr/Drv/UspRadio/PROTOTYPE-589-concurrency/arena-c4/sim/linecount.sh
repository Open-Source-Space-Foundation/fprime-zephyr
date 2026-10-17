#!/bin/sh
# Code lines = non-blank lines that are not pure // or /* */ comment lines (rough cloc-style count).
count() { grep -v -E '^[[:space:]]*($|//|/\*|\*)' "$@" | wc -l | tr -d ' '; }
R=$1
echo "option 1 (base, shipped):"
echo "  UspRadio.cpp $(count $R/UspRadio.cpp)  RalSessionImpl.cpp $(count $R/RalSessionImpl.cpp)  total $(( $(count $R/UspRadio.cpp) + $(count $R/RalSessionImpl.cpp) ))"
P=$R/PROTOTYPE-589-concurrency
echo "option 2 (sketch, changed handlers + RalSessionSt):"
echo "  single-queue/UspRadio.cpp $(count $P/single-queue/UspRadio.cpp)  RalSessionSt.cpp+hpp $(count $P/single-queue/RalSessionSt.cpp $P/single-queue/RalSessionSt.hpp)  total $(( $(count $P/single-queue/UspRadio.cpp) + $(count $P/single-queue/RalSessionSt.cpp $P/single-queue/RalSessionSt.hpp) ))"
echo "  (sketch files contain only changed handlers; unchanged code not counted)"
echo "option 3 (upstream-mutex diff, added+removed lines):"
echo "  $(grep -c '^+[^+]' $P/upstream-mutex/RalSessionImpl.cpp.diff) added, $(grep -c '^-[^-]' $P/upstream-mutex/RalSessionImpl.cpp.diff) removed"
