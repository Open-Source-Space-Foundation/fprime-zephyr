#!/bin/bash
# One command: rebuild the simulation for every variant and rerun every measurement.  Raw outputs go to results/.
#   ./run_all.sh            full run (about 10-15 minutes on an M-series Mac)
#   QUICK=1 ./run_all.sh    fewer seeds/offsets (about 2 minutes)
# Nothing here touches the dev board or any shared tree: sources are compiled from read-only trees into build/.
set -u
cd "$(dirname "$0")"
Q=${QUICK:-0}
J=$(sysctl -n hw.ncpu 2>/dev/null || echo 4)
say() { printf '\n=== %s ===\n' "$*"; }

say "scratch trees (usp v1.0.0 pin, usp_zephyr fork/no-fork, v1.2.1 variants)"
tools/setup_scratch.sh || exit 1

say "build"
rm -rf build results; mkdir -p results
for v in opt0 opt1 opt2 opt3 opt1_v121 opt2_v121 opt3_v121; do make -j$J VARIANT=$v bins 2>&1 | grep -E "error|\*\*\*" ; done
for v in opt1_epoch opt3_epoch; do make -j$J VARIANT=$v $PWD/build/$v/stale.bin $PWD/build/$v/race.bin 2>&1 | grep -E "error|\*\*\*"; done
for v in opt1_asan opt2_asan; do make -j$J VARIANT=$v $PWD/build/$v/initfail.bin 2>&1 | grep -E "error|\*\*\*"; done
make -j$J VARIANT=opt1_v121 USP_PRIO=14 $PWD/build/opt1_v121_prio14/busyloop.bin $PWD/build/opt1_v121_prio14/latency.bin 2>&1 | grep -E "error|\*\*\*"
echo "built: $(ls build/*/*.bin | wc -l) binaries"

NOFF=$([ "$Q" = 1 ] && echo 20 || echo 60)
NSEED=$([ "$Q" = 1 ] && echo 30 || echo 100)
SEEDS=$(seq -s, 1 $NSEED)
offs() { python3 -c "print(','.join(str(int(i*$1/$NOFF)) for i in range($NOFF)))"; }

say "A. stale-frame false confirm (offset sweep of SET_RX_PROFILE against an old-profile frame stream)"
for P in 12000 25000 50000 100000 250000 1000000; do
  for v in opt1 opt3 opt2; do tools/sweep.py results/stale/p$((P/1000))ms_$v.txt stale $v OFFSET_US=$(offs $P) --const PERIOD_US=$P >/dev/null; done
done
for v in opt1 opt3 opt2; do tools/sweep.py results/stale/p60ms_len252_$v.txt stale $v OFFSET_US=$(offs 60000) --const PERIOD_US=60000 --const LEN=252 >/dev/null; done
for hz in 1000000 8000000; do tools/sweep.py results/stale/p25ms_spi${hz}_opt1.txt stale opt1 OFFSET_US=$(offs 25000) --const PERIOD_US=25000 --const CHIP_SPI_HZ=$hz >/dev/null; done
tools/sweep.py results/stale/p25ms_irqspi_opt1.txt stale opt1 OFFSET_US=$(offs 25000) --const PERIOD_US=25000 --const CHIP_SPI_POLL=0 >/dev/null
tools/sweep.py results/stale/p25ms_newlive_opt1.txt stale opt1 OFFSET_US=$(offs 25000) --const PERIOD_US=25000 --const NEW_LIVE=1 >/dev/null
# fix for A on options 1/3: RX-epoch tag (fixes/epoch_tag.diff)
for P in 12000 25000 50000 100000; do for v in opt1_epoch opt3_epoch; do tools/sweep.py results/stale/p$((P/1000))ms_$v.txt stale $v OFFSET_US=$(offs $P) --const PERIOD_US=$P >/dev/null; done; done
# one annotated timeline for the write-up
(cd build/opt1 && SIM_DUMP_EVENTS=1 PERIOD_US=100000 OFFSET_US=55000 ./stale.bin > ../../results/stale/timeline_opt1_p100ms_off55000.txt 2>&1)

say "B. tasks[0xFF] / failed init"
mkdir -p results/initfail
for v in opt1 opt2; do
  (cd build/${v}_asan && FAULT_OPEN=1 MODE=fault_open ./initfail.bin > ../../results/initfail/${v}_fault_open.txt 2>&1)
  (cd build/${v}_asan && MODE=double_open ./initfail.bin > ../../results/initfail/${v}_double_open.txt 2>&1)
done

say "E. engine vs caller race: randomized storms (heavy RX), hazard vs RX rate, control without any mutex (opt0)"
tools/sweep.py results/race/gap200_opt3_epoch.txt race opt3_epoch SEED=$SEEDS --const DUR_US=20000000 --const GAP_MAX_MS=200 >/dev/null
for g in 200 1000; do for v in opt0 opt1 opt3 opt2; do tools/sweep.py results/race/gap${g}_$v.txt race $v SEED=$SEEDS --const DUR_US=20000000 --const GAP_MAX_MS=$g >/dev/null; done; done
for rx in 30000 300000 2000000 100000000; do for v in opt1 opt3; do tools/sweep.py results/race/hz_rx${rx}_$v.txt race $v SEED=$SEEDS --const DUR_US=20000000 --const GAP_MAX_MS=1000 --const RX_GAP_US=$rx >/dev/null; done; done
for v in opt1 opt3 opt2; do tools/sweep.py results/race/irqspi_gap200_$v.txt race $v SEED=$SEEDS --const DUR_US=20000000 --const GAP_MAX_MS=200 --const CHIP_SPI_POLL=0 >/dev/null; done

say "F. usp v1.2.1 busy loop"
for v in opt1 opt1_v121 opt3 opt3_v121 opt2 opt2_v121; do
  tools/sweep.py results/busyloop/$v.txt busyloop $v PROBE=0,1 --const DUR_US=10000000 --const RX_PERIOD_US=500000 >/dev/null
  tools/sweep.py results/busyloop/${v}_norx.txt busyloop $v PROBE=0 --const DUR_US=10000000 --const RX_PERIOD_US=100000000 >/dev/null
done

# mitigation for F on options 1/3: run the USP thread at the lowest priority (CONFIG_USP_MAIN_THREAD_PRIORITY=14)
tools/sweep.py results/busyloop/opt1_v121_prio14.txt busyloop opt1_v121_prio14 PROBE=0,1 --const DUR_US=10000000 --const RX_PERIOD_US=500000 >/dev/null
tools/sweep.py results/busyloop/opt1_v121_prio14_norx.txt busyloop opt1_v121_prio14 PROBE=0 --const DUR_US=10000000 --const RX_PERIOD_US=100000000 >/dev/null
tools/sweep.py results/latency/v121_prio14_gap25.txt latency opt1_v121_prio14 RG1_US=300,20000 --const GAP_US=25000 --const COUNT=200 >/dev/null

say "G. RX service latency / loss under the real thread mix (priorities from instances.fpp, CPU bursts assumed)"
for v in opt1 opt3 opt2; do
  tools/sweep.py results/latency/rg1_gap60_$v.txt latency $v RG1_US=300,5000,20000,50000 --const GAP_US=60000 --const COUNT=200 >/dev/null
  tools/sweep.py results/latency/rg1_gap25_$v.txt latency $v RG1_US=300,5000,20000,50000 --const GAP_US=25000 --const COUNT=200 >/dev/null
  tools/sweep.py results/latency/rg10_gap25_$v.txt latency $v RG10_US=1500,10000,30000 --const GAP_US=25000 --const COUNT=200 >/dev/null
  tools/sweep.py results/latency/mm_gap25_$v.txt latency $v MM_US=500,20000 --const GAP_US=25000 --const COUNT=200 >/dev/null
  tools/sweep.py results/latency/gaps_$v.txt latency $v GAP_US=60000,25000,5000 --const COUNT=100 >/dev/null
  tools/sweep.py results/latency/irqspi_rg1_gap25_$v.txt latency $v RG1_US=300,20000,50000 --const GAP_US=25000 --const COUNT=200 --const CHIP_SPI_POLL=0 >/dev/null
done

# what-if: where should uspRadio's thread priority sit (option 2)?  Priority 11 = deployed; 3 = same as rateGroup1Hz; 2 = same as the old USP thread
for pr in 11 5 3 2; do tools/sweep.py results/latency/radioprio${pr}_gap25_opt2.txt latency opt2 RG1_US=4000,20000 --const GAP_US=25000 --const COUNT=200 --const MM_US=2000 --const RADIO_PRIO=$pr >/dev/null; done

say "deterministic race hunt: one RX frame vs one component-thread operation, phase swept in 50 us steps"
PH=$(seq -s, -2000 50 2000)
for poll in 1 0; do for c in settx setrx tx; do for v in opt1 opt3 opt2; do tools/sweep.py results/phase/fine_poll${poll}_${c}_$v.txt phase $v PHASE_US=$PH --const CMD=$c --const CHIP_SPI_POLL=$poll >/dev/null; done; done; done

say "re-arm cost"
for v in opt1 opt2; do for flow in setrx tx1 txN; do tools/sweep.py results/rearm/${flow}_$v.txt rearm $v CHIP_SPI_HZ=125000,1000000,8000000 CHIP_TCXO_MS=10,2 --const FLOW=$flow --const NFRAMES=8 >/dev/null; done; done

say "queue blocked in TX / CW / hung TX"
mkdir -p results/blocked
for v in opt1 opt3 opt2; do
  (cd build/$v
   for spec in "tx 0" "tx 5" "cw 0"; do set -- $spec; echo "$v flow=$1 txprof=$2 :: $(FLOW=$1 TXPROF=$2 SEC=5 ./blocked.bin 2>&1 | tail -1)"; done
   echo "$v flow=hang :: $(FLOW=hang CHIP_TX_HANG=1 TXPROF=5 ./blocked.bin 2>&1 | tail -1)") > results/blocked/$v.txt
done

say "option 3 priority inversion on rac_api_mutex"
for v in opt1 opt3; do
  tools/sweep.py results/mutexwait/$v.txt mutexwait $v HOG_BURST_US=0,3000,20000,40000 NO_PI=0,1 --const DUR_US=20000000 >/dev/null
  # stress: a stray DIO1 flag every 20..40 ms forces engine passes to collide with the component thread's lock windows
  tools/sweep.py results/mutexwait/${v}_spurious.txt mutexwait $v HOG_BURST_US=0,20000 NO_PI=0,1 --const DUR_US=20000000 --const SPUR_US=20000 >/dev/null
done

say "line / primitive counts"
python3 tools/count_loc.py > results/loc.txt

say "summary"
python3 tools/summarize.py > results/SUMMARY.txt
cat results/SUMMARY.txt
python3 tools/plots.py
