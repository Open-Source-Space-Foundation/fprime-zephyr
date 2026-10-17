#!/bin/sh
# Rerun every measurement.  Usage: run_all.sh <build dir> <results dir>
# Each run's stdout+stderr goes to results/<name>.txt; a one-line digest of
# the key numbers goes to results/SUMMARY.txt.
set -u
B="$1"; R="$2"
mkdir -p "$R"
SUM="$R/SUMMARY.txt"
: > "$SUM"

run() {  # name, env, binary, args...
    name="$1"; envs="$2"; bin="$3"; shift 3
    if [ ! -x "$B/$bin" ]; then echo "$name: MISSING $bin" | tee -a "$SUM"; return; fi
    echo "== $name: env[$envs] $bin $*" | tee -a "$SUM"
    ( cd "$B" && env $envs "./$bin" "$@" ) > "$R/$name.txt" 2>&1
    grep -E "^(stale_iters|startRadio_ok|race_rounds|busyloop|txcost|rxlat|rx_delivered|tx_done_ok|handlers=|rx_service|fifo_read|tx_episode|rearm_n|spi_trans|engine_calls|air_del)" "$R/$name.txt" | sed 's/^/   /' >> "$SUM"
    n=$(grep -c "WARNING: ThreadSanitizer: data race" "$R/$name.txt"); [ "$n" != "0" ] && echo "   tsan_data_race_reports=$n" >> "$SUM"
    n=$(grep -c "runtime error" "$R/$name.txt"); [ "$n" != "0" ] && echo "   ubsan_bounds_reports=$n" >> "$SUM"
    n=$(grep -c "PANIC" "$R/$name.txt"); [ "$n" != "0" ] && echo "   hal_panics=$n" >> "$SUM"
    n=$(grep -c "FALSE CONFIRM" "$R/$name.txt"); [ "$n" != "0" ] && echo "   false_confirm_lines=$n" >> "$SUM"
}

# smoke: every option boots, receives, transmits, changes profile
for o in opt1_v100 opt3_v100 opt2_v100 opt1_v121 opt2_v121; do run "smoke_$o" "" $o smoke; done

# A. stale-frame false confirm (host timing, and with the v5e SPI clock modelled)
for o in opt1_v100 opt3_v100 opt2_v100; do
    run "A_stale_$o" "SIM_VERBOSE=1" $o stale 200 1500
    run "A_stale_spi_$o" "SIM_VERBOSE=1 SIM_SPI_HZ=125000" $o stale 60 20000
done

# B. failed init: every handler, bounds sanitizer on
for o in opt1_v100_bounds opt2_v100_bounds; do run "B_initfail_$o" "" $o initfail; done

# E. engine vs lock_radio_access race: cross-thread overlap counter (real
# rp_callback / rp_task_enqueue wrapped), functional anomaly counters, and the
# upstream bare stub as reference.  (ThreadSanitizer builds exist -- make
# opt1_v100_tsan etc. -- but the host shim's semaphores stall under TSan on
# this machine, so they are not part of the run; see RATIONALE.md.)
for o in opt1_v100 opt3_v100 opt2_v100; do run "E_race_$o" "" $o race 4; done
run "E_race_opt1_v100_bare" "SIM_PROTECT_MODE=0" opt1_v100 race 4
run "E_race_opt1_v121" "" opt1_v121 race 4

# F. v1.2.1 busy loop: engine passes while a LOCK is held, by trigger
for v in v100 v121; do for t in 0 1 2; do
    run "F_busyloop_opt1_${v}_t$t" "" opt1_$v busyloop 2 $t
    run "F_busyloop_opt2_${v}_t$t" "" opt2_$v busyloop 2 $t
done; done

# Re-arm / TX cost with the real SPI clock (125 kHz) and 25 ms airtime
for o in opt1_v100 opt2_v100; do
    run "R_txcost_paced_$o" "SIM_SPI_HZ=125000 SIM_TX_AIRTIME_MS=25" $o txcost 20 0
    run "R_txcost_sat_$o" "SIM_SPI_HZ=125000 SIM_TX_AIRTIME_MS=25" $o txcost 20 1
done

# G. RX service latency under component load, 252 B frames every 30 ms
for o in opt1_v100 opt2_v100; do
    run "G_rxlat_idle_$o" "SIM_SPI_HZ=125000 SIM_TX_AIRTIME_MS=25" $o rxlat 30 6 0
    run "G_rxlat_light_$o" "SIM_SPI_HZ=125000 SIM_TX_AIRTIME_MS=25" $o rxlat 30 6 2
    run "G_rxlat_saturated_$o" "SIM_SPI_HZ=125000 SIM_TX_AIRTIME_MS=25" $o rxlat 30 6 1
done

# Line / primitive counts for the table row
sh "$(dirname "$0")/linecount.sh" "$(dirname "$0")/../../.." "$(dirname "$0")/.." > "$R/linecount.txt" 2>&1
echo "== linecount: see linecount.txt" >> "$SUM"

echo "done; see $SUM"
