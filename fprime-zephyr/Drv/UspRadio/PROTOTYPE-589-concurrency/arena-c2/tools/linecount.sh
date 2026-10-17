#!/bin/sh
# Code-line counts for the prototype table's "Code lines, UspRadio + RalSession"
# row.  Code line = not blank, not a //-only or /* */-only line.
# Usage: linecount.sh <UspRadio dir> <arena-c2 dir>
USPRADIO="$1"; HERE="$2"
count() {
    # strip block comments, then drop blank and //-comment lines
    cat "$@" | sed -e 's://.*$::' | awk '
        /\/\*/ && !/\*\// { inblk=1; sub(/\/\*.*$/, ""); }
        inblk && /\*\// { inblk=0; sub(/^.*\*\//, ""); }
        inblk { next }
        /^[[:space:]]*$/ { next }
        { n++ }
        END { print n+0 }'
}
printf "%-44s %6s %6s\n" "set" "code" "raw"
o1="$USPRADIO/UspRadio.cpp $USPRADIO/UspRadio.hpp $USPRADIO/RalSessionImpl.cpp $USPRADIO/RalSessionImpl.hpp"
printf "%-44s %6s %6s\n" "opt1 UspRadio.{cpp,hpp}+RalSessionImpl.{cpp,hpp}" "$(count $o1)" "$(cat $o1 | wc -l | tr -d ' ')"
o1c="$USPRADIO/UspRadio.cpp $USPRADIO/RalSessionImpl.cpp"
printf "%-44s %6s %6s\n" "opt1 UspRadio.cpp+RalSessionImpl.cpp" "$(count $o1c)" "$(cat $o1c | wc -l | tr -d ' ')"
o3="$USPRADIO/UspRadio.cpp $USPRADIO/UspRadio.hpp $HERE/sim/opt3/RalSessionImpl.cpp $HERE/sim/opt3/RalSessionImpl.hpp"
printf "%-44s %6s %6s\n" "opt3 (opt1 + upstream-mutex diff applied)" "$(count $o3)" "$(cat $o3 | wc -l | tr -d ' ')"
o3c="$USPRADIO/UspRadio.cpp $HERE/sim/opt3/RalSessionImpl.cpp"
printf "%-44s %6s %6s\n" "opt3 .cpp only" "$(count $o3c)" "$(cat $o3c | wc -l | tr -d ' ')"
sk="$HERE/../single-queue/UspRadio.cpp $HERE/../single-queue/RalSessionSt.cpp $HERE/../single-queue/RalSessionSt.hpp"
printf "%-44s %6s %6s\n" "opt2 sketch as shipped (incomplete)" "$(count $sk)" "$(cat $sk | wc -l | tr -d ' ')"
o2="$HERE/sim/opt2/RalSessionSt.cpp $HERE/sim/opt2/RalSessionSt.hpp"
printf "%-44s %6s %6s\n" "opt2 RalSessionSt completed (this sim)" "$(count $o2)" "$(cat $o2 | wc -l | tr -d ' ')"
o2u="$HERE/sim/opt2/UspRadioSim2.cpp"
printf "%-44s %6s %6s\n" "opt2 component model (this sim, incl. F' stand-ins)" "$(count $o2u)" "$(cat $o2u | wc -l | tr -d ' ')"
o1u="$HERE/sim/opt1/UspRadioSim1.cpp"
printf "%-44s %6s %6s\n" "opt1 component model (this sim, incl. F' stand-ins)" "$(count $o1u)" "$(cat $o1u | wc -l | tr -d ' ')"
echo
echo "primitive counts (grep -c over opt1 UspRadio.cpp+RalSessionImpl.cpp / opt2 RalSessionSt.cpp+UspRadioSim2.cpp):"
for pat in "k_sem" "std::atomic\|atomic<" "k_sleep" "tasks\[" "k_mutex\|SMTC_SW_PLATFORM"; do
    printf "  %-28s opt1=%-4s opt3=%-4s opt2=%s\n" "$pat" \
        "$(cat $o1c | grep -c "$pat")" "$(cat $o3c | grep -c "$pat")" "$(cat $HERE/sim/opt2/RalSessionSt.cpp $o2u | grep -c "$pat")"
done
