#!/usr/bin/env python3
"""Digest results/ into the numbers quoted in RESULTS.md.  Pure parsing: every figure comes from a raw RESULT line."""
import glob, os, re, sys
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results')

def kv(line):
    return {k: v for k, v in re.findall(r'(\w+)=("[^"]*"|\[[^\]]*\]|[^\s|]+)', line)}

def rows(path):
    out = []
    if not os.path.exists(path): return out
    for l in open(path):
        if l.startswith('#') or '|' not in l: continue
        tag, res = l.rstrip('\n').split(' | ', 1)
        d = kv(res); d['_tag'] = tag
        out.append(d)
    return out

def fnum(x, default=float('nan')):
    try: return float(x)
    except Exception: return default

print('=' * 100); print('A. stale-frame false confirm: runs with a false ProfileConfirmed(P3) from a P5 frame / runs where the 3 s auto-revert never fired (stranded) / n')
print(f"{'stream period':>16} | {'opt1':>14} | {'opt3':>14} | {'opt2':>14}")
for P in (12, 25, 50, 100, 250, 1000):
    cells = []
    for v in ('opt1', 'opt3', 'opt2'):
        rs = rows(f'{R}/stale/p{P}ms_{v}.txt')
        fc = sum(1 for r in rs if r.get('false_confirm') == '1'); st = sum(1 for r in rs if r.get('reverted') == '0')
        cells.append(f'{fc}/{st}/{len(rs)}')
    print(f"{str(P)+' ms':>16} | {cells[0]:>14} | {cells[1]:>14} | {cells[2]:>14}")
print('   with the epoch-tag fix (fixes/epoch_tag.diff):')
for P in (12, 25, 50, 100):
    cells = []
    for v in ('opt1_epoch', 'opt3_epoch'):
        rs = rows(f'{R}/stale/p{P}ms_{v}.txt')
        fc = sum(1 for r in rs if r.get('false_confirm') == '1'); st = sum(1 for r in rs if r.get('reverted') == '0')
        cells.append(f'{fc}/{st}/{len(rs)}')
    print(f"{str(P)+' ms':>16} | {cells[0]:>14} | {cells[1]:>14} |")
for name, label in (('p60ms_len252', '60 ms, 252 B frames'), ('p25ms_spi1000000', '25 ms, SPI 1 MHz'), ('p25ms_spi8000000', '25 ms, SPI 8 MHz'), ('p25ms_irqspi', '25 ms, IRQ-driven SPI'), ('p25ms_newlive', '25 ms, new profile live')):
    cells = []
    for v in ('opt1', 'opt3', 'opt2'):
        rs = rows(f'{R}/stale/{name}_{v}.txt')
        if not rs: cells.append('-'); continue
        fc = sum(1 for r in rs if r.get('false_confirm') == '1'); st = sum(1 for r in rs if r.get('reverted') == '0')
        cells.append(f'{fc}/{st}/{len(rs)}')
    print(f"{label:>26} | {cells[0]:>14} | {cells[1]:>14} | {cells[2]:>14}")

print('=' * 100); print('B. failed init (ASAN builds)')
for f in sorted(glob.glob(f'{R}/initfail/*.txt')):
    print('--', os.path.basename(f))
    for l in open(f):
        if l.startswith(('RESULT', 'PANIC', 'REBOOT', 'INFO', '==')) or 'ERROR: AddressSanitizer' in l: print('   ', l.rstrip())

print('=' * 100); print('E. randomized storms: stalled seeds (panic or RX dead after drain) / seeds ; ops executed until stall-or-end per stall')
def storm(path):
    rs = rows(path)
    if not rs: return None
    n = len(rs); bad = sum(1 for r in rs if r.get('panics') != '0' or r.get('live_rx') != '6/6')
    ops = sum(int(r.get('ops_before_stall', 0)) for r in rs)
    lost = sum(int(r.get('lost_asleep', 0)) for r in rs)
    return n, bad, ops, lost
for g in (200, 1000):
    print(f'-- action gap up to {g} ms, RX frames every 3..33 ms (heavy)')
    for v in ('opt0', 'opt1', 'opt3', 'opt2'):
        s = storm(f'{R}/race/gap{g}_{v}.txt')
        if s: print(f"   {v}: stalled {s[1]}/{s[0]} seeds; ops={s[2]}; " + (f'1 stall per {s[2]/s[1]:.0f} ops' if s[1] else 'no stall observed') + f'; lost-cmd-to-sleeping-chip={s[3]}')
s = storm(f'{R}/race/gap200_opt3_epoch.txt')
if s: print(f"   opt3+epoch fix: stalled {s[1]}/{s[0]} seeds; ops={s[2]}")
print('-- hazard vs RX rate (opt1 vs opt3), action gap up to 1000 ms')
for rx in (30000, 300000, 2000000, 100000000):
    cells = []
    for v in ('opt1', 'opt3'):
        s = storm(f'{R}/race/hz_rx{rx}_{v}.txt')
        if s: cells.append(f"{v}: {s[1]}/{s[0]} seeds stalled, " + (f'1 per {s[2]/s[1]:.0f} ops' if s[1] else f'0 in {s[2]} ops'))
    print(f'   RX gap 3..{rx//1000+3} ms: ' + ' | '.join(cells))
print('-- IRQ-driven SPI model (CHIP_SPI_POLL=0), gap 200 ms')
for v in ('opt1', 'opt3', 'opt2'):
    s = storm(f'{R}/race/irqspi_gap200_{v}.txt')
    if s: print(f"   {v}: stalled {s[1]}/{s[0]}; " + (f'1 per {s[2]/s[1]:.0f} ops' if s[1] else f'0 in {s[2]} ops') + f'; lost-cmd={s[3]}')

print('=' * 100); print('F. busy loop (usp v1.0.0 vs v1.2.1), thread mix per instances.fpp, RX traffic 2 Hz unless noted')
print(f"{'variant':>12} {'case':>10} | usp_cpu% idle% | rg1 cyc/late_ms | rg10 cyc/late_ms | mm cyc/late_ms | probe_lat_ms max/mean (run/posted)")
for v in ('opt1', 'opt1_v121', 'opt3', 'opt3_v121', 'opt2', 'opt2_v121'):
    for case, path in (('RX 2Hz', f'{R}/busyloop/{v}.txt'), ('no RX', f'{R}/busyloop/{v}_norx.txt')):
        for r in rows(path):
            if case == 'RX 2Hz' and r['_tag'] != 'PROBE=0' and r['_tag'] != 'PROBE=1': continue
            tag = case + (' +probe' if r['_tag'] == 'PROBE=1' else '')
            print(f"{v:>12} {tag:>16} | {r['usp_thread_cpu_pct']:>6} {r['idle_pct']:>6} | {r['rg1_cycles']:>3}/{r['rg1_late_ms_max']:>6} | {r['rg10_cycles']:>3}/{r['rg10_late_ms_max']:>6} | {r['mm_cycles']:>3}/{r['mm_late_ms_max']:>6} | {r['probe_latency_ms_max']}/{r['probe_latency_ms_mean']} ({r['probes_run']}/{r['probes_posted']})")

print('=' * 100); print('G. RX service (P5, 252 B frames; gap = idle time between frames).  sent / chip RX complete / delivered ; kick = RX_DONE -> first SPI of the harvest')
def lat(path, key):
    for r in rows(path):
        print(f"   {key}={r['_tag']:>16} sent={r['sent']} rx_complete={r['rx_complete']} delivered={r['delivered']} kick_ms p50/p95/max={r['kick_ms_p50']}/{r['kick_ms_p95']}/{r['kick_ms_max']} deliver_ms p50/p95/max={r['deliver_ms_p50']}/{r['deliver_ms_p95']}/{r['deliver_ms_max']} | rg1_late_max={r.get('rg1_late_ms_max')} mm_late_max={r.get('mm_late_ms_max')} usp_cpu%={r.get('usp_thread_cpu_pct')}")
for name in ('rg1_gap60', 'rg1_gap25', 'rg10_gap25', 'mm_gap25', 'gaps', 'irqspi_rg1_gap25'):
    for v in ('opt1', 'opt3', 'opt2'):
        p = f'{R}/latency/{name}_{v}.txt'
        if os.path.exists(p): print(f'-- {name} {v}'); lat(p, 'var')

print('=' * 100); print('E (deterministic): one RX frame, one operation started PHASE us after its RX_DONE, 81 phases in -2000..+2000 us; radio dead = no RX after 3 s drain')
for poll, label in ((1, 'polled SPI (as built)'), (0, 'IRQ-driven SPI')):
    for c in ('settx', 'setrx', 'tx'):
        cells = []
        for v in ('opt1', 'opt3', 'opt2'):
            rs = rows(f'{R}/phase/fine_poll{poll}_{c}_{v}.txt')
            dead = [int(r['phase_us']) for r in rs if r.get('rx_alive') == '0']
            cells.append(f"{v}: {len(dead)}/{len(rs)}" + (f" [{min(dead)}..{max(dead)} us]" if dead else ''))
        print(f'   {label:>22} cmd={c:>5}: ' + ' | '.join(cells))

print('=' * 100); print('F mitigation / what-if: USP thread at priority 14 on v1.2.1 (opt1_v121_prio14)')
for name in ('opt1_v121_prio14', 'opt1_v121_prio14_norx'):
    for r in rows(f'{R}/busyloop/{name}.txt'):
        print(f"   {name} {r['_tag']}: usp_cpu%={r['usp_thread_cpu_pct']} idle%={r['idle_pct']} rg1 cyc/late={r['rg1_cycles']}/{r['rg1_late_ms_max']} rg10={r['rg10_cycles']}/{r['rg10_late_ms_max']} mm={r['mm_cycles']}/{r['mm_late_ms_max']} probe_lat_max={r['probe_latency_ms_max']} ({r['probes_run']}/{r['probes_posted']})")
for r in rows(f'{R}/latency/v121_prio14_gap25.txt'):
    print(f"   RX stream P5/252B gap25 {r['_tag']}: sent={r['sent']} delivered={r['delivered']} kick_max={r['kick_ms_max']} rg1_late_max={r['rg1_late_ms_max']} mm_late_max={r['mm_late_ms_max']} usp_cpu%={r['usp_thread_cpu_pct']}")

print('=' * 100); print('option 2 what-if: uspRadio thread priority (11 = as deployed; lower number = higher priority)')
for pr in (11, 5, 3, 2):
    for r in rows(f'{R}/latency/radioprio{pr}_gap25_opt2.txt'):
        print(f"   prio={pr:>2} {r['_tag']:>14}: sent={r['sent']} delivered={r['delivered']} kick_ms max={r['kick_ms_max']} | rg1_late_max={r['rg1_late_ms_max']} rg10_late_max={r['rg10_late_ms_max']} mm_late_max={r['mm_late_ms_max']}")

print('=' * 100); print('re-arm cost (ms from t_post=dataIn/command)')
print(f"{'flow':>6} {'var':>5} {'spi_hz':>8} {'tcxo':>5} | first_tx_start txdone relisten rx_deaf burst_total | rearm_after_txdone")
for flow in ('setrx', 'tx1', 'txN'):
    for v in ('opt1', 'opt2'):
        for r in rows(f'{R}/rearm/{flow}_{v}.txt'):
            t = dict(x.split('=') for x in r['_tag'].split())
            rearm = fnum(r['t_rx_relisten_ms']) - fnum(r['t_last_txdone_ms']) if fnum(r['t_last_txdone_ms']) > 0 else float('nan')
            print(f"{flow:>6} {v:>5} {t['CHIP_SPI_HZ']:>8} {t['CHIP_TCXO_MS']:>5} | {r['t_first_tx_start_ms']:>8} {r['t_last_txdone_ms']:>8} {r['t_rx_relisten_ms']:>8} {r['rx_deaf_total_ms']:>7} {r['burst_total_ms']:>8} | {rearm:8.1f}")

print('=' * 100); print('queue blocked in TX / CW / hung TX (probe command delay, ms)')
for f in sorted(glob.glob(f'{R}/blocked/*.txt')):
    for l in open(f):
        d = kv(l)
        print('  ', l.split(' :: ')[0], '->', 'probe_delay_ms=' + d.get('probe_delay_ms', '?'), 'airtime_ms=' + d.get('airtime_ms', '?'), 'rx_alive_after=' + d.get('rx_alive_after', '?'), 'send_failed=' + d.get('send_failed', '?'))

print('=' * 100); print('option 1 (fork mutex) vs option 3 (rac_api_mutex): longest wait of the USP thread / uspRadio on the mutex, ms')
for suffix, label in (('', 'normal traffic'), ('_spurious', 'stress: stray DIO1 every 20..40 ms')):
    print('--', label)
    for v in ('opt1', 'opt3'):
        for r in rows(f'{R}/mutexwait/{v}{suffix}.txt'):
            print(f"   {v} {r['_tag']:>28}: usp_waits={r.get('usp_waits','-')} usp_wait_max={r['usp_wait_max_ms']} comp_wait_max={r['comp_wait_max_ms']} usp_hold_max={r['usp_hold_max_ms']} comp_hold_max={r['comp_hold_max_ms']} tx_returned={r['tx_returned']} config_failed={r['config_failed']} panics={r['panics']}")

print('=' * 100); print('line / primitive counts'); print(open(f'{R}/loc.txt').read() if os.path.exists(f'{R}/loc.txt') else '(missing)')
