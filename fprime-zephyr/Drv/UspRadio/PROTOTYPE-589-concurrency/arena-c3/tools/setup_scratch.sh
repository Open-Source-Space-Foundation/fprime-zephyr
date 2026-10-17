#!/bin/bash
# Prepare the (read-only-sourced) usp / usp_zephyr trees the simulation compiles.  Idempotent.
#   usp-v121, usp_zephyr-v121 : upstream tag v1.2.1 (network clone)
#   uspz-fork                 : the pinned fork (pcr-588 usp_zephyr @ 0bf3e20, contains bbdd729 RAC mutex)
#   uspz-nofork               : same with bbdd729 reverted (option 2/3: "fork mutex dropped"; opt0 control)
#   uspz121-fork              : v1.2.1 + bbdd729 cherry-picked (option 1 on v1.2.1)
set -e
SCR=${SCR:-$HOME/GitHut/arena-589/c3-scratch}
PCR=${PCR:-$HOME/GitHut/pcr-588/lib/zephyr-workspace/modules/lib}
mkdir -p "$SCR"; cd "$SCR"
G="git -c user.email=sim@example -c user.name=sim"
[ -d usp-v121 ]          || git clone -q --depth 1 --branch v1.2.1 https://github.com/Lora-net/usp.git usp-v121
[ -d usp_zephyr-v121 ]   || git clone -q --depth 1 --branch v1.2.1 https://github.com/Lora-net/usp_zephyr.git usp_zephyr-v121
[ -d uspz-fork ]         || { git clone -q "$PCR/usp_zephyr" uspz-fork && git -C uspz-fork checkout -q 0bf3e20; }
[ -d uspz-nofork ]       || { git clone -q "$PCR/usp_zephyr" uspz-nofork && git -C uspz-nofork checkout -q 0bf3e20 && $G -C uspz-nofork revert --no-edit bbdd729 >/dev/null; }
[ -d uspz121-fork ]      || { cp -R usp_zephyr-v121 uspz121-fork && git -C uspz121-fork fetch -q "$SCR/uspz-fork" bbdd729 && $G -C uspz121-fork cherry-pick bbdd729 >/dev/null; }
echo "scratch ready in $SCR"
git -C "$PCR/usp" describe --tags 2>/dev/null | sed 's/^/pinned usp: /'
git -C usp-v121 describe --tags | sed 's/^/usp: /'
