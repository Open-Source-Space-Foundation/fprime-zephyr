#!/usr/bin/env python3
"""Dependency-free SVG charts from results/ (grouped bars, direct value labels, legend, Okabe-Ito colours)."""
import os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(HERE, '..', 'results'); OUT = os.path.join(R, 'figs')
os.makedirs(OUT, exist_ok=True)
COL = {'opt1': '#0072B2', 'opt3': '#E69F00', 'opt2': '#009E73', 'opt0': '#999999'}
NAME = {'opt1': '1  #14+#26', 'opt3': '3  upstream mutex', 'opt2': '2  single queue', 'opt0': '0  no mutex (control)'}

def kv(l): return dict(re.findall(r'(\w+)=([^\s|]+)', l))
def rows(p):
    out = []
    if os.path.exists(p):
        for l in open(p):
            if not l.startswith('#') and '|' in l: out.append(kv(l.split(' | ', 1)[1]))
    return out

def chart(fname, title, sub, groups, series, vals, ylab, ymax=None, fmt='{:.0f}'):
    """groups: x labels; series: list of keys; vals[series][i] numeric."""
    W, H, L, B, T = 760, 380, 70, 70, 70
    ymax = ymax or max(max(v) for v in vals.values()) * 1.15 or 1
    gw = (W - L - 20) / len(groups); bw = gw * 0.8 / len(series)
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" font-family="Helvetica,Arial,sans-serif" font-size="12">',
         f'<rect width="{W}" height="{H}" fill="#fff"/>',
         f'<text x="{L}" y="24" font-size="16" font-weight="bold" fill="#222">{title}</text>',
         f'<text x="{L}" y="42" fill="#555">{sub}</text>']
    for k, key in enumerate(series):
        x = L + k * 190
        s.append(f'<rect x="{x}" y="50" width="12" height="12" rx="2" fill="{COL[key]}"/><text x="{x+17}" y="61" fill="#222">{NAME[key]}</text>')
    y0 = H - B
    for t in range(5):
        v = ymax * t / 4; y = y0 - (y0 - T - 20) * t / 4
        s.append(f'<line x1="{L}" x2="{W-10}" y1="{y:.1f}" y2="{y:.1f}" stroke="#e5e5e5"/><text x="{L-6}" y="{y+4:.1f}" text-anchor="end" fill="#666">{fmt.format(v)}</text>')
    s.append(f'<text x="14" y="{(T+y0)/2:.0f}" fill="#555" transform="rotate(-90 14 {(T+y0)/2:.0f})" text-anchor="middle">{ylab}</text>')
    for i, g in enumerate(groups):
        gx = L + i * gw + gw * 0.1
        for k, key in enumerate(series):
            v = vals[key][i]
            h = (y0 - T - 20) * min(v, ymax) / ymax
            x = gx + k * bw
            s.append(f'<rect x="{x+1:.1f}" y="{y0-h:.1f}" width="{bw-2:.1f}" height="{max(h,0):.1f}" rx="3" fill="{COL[key]}"/>')
            s.append(f'<text x="{x+bw/2:.1f}" y="{y0-h-4:.1f}" text-anchor="middle" fill="#222" font-size="11">{fmt.format(v)}</text>')
        s.append(f'<text x="{gx+gw*0.4:.1f}" y="{y0+18}" text-anchor="middle" fill="#222">{g}</text>')
    s.append(f'<line x1="{L}" x2="{W-10}" y1="{y0}" y2="{y0}" stroke="#888"/></svg>')
    open(os.path.join(OUT, fname), 'w').write('\n'.join(s))

# A: false-confirm probability per command vs stale-frame period
periods = [12, 25, 50, 100, 250, 1000]
vals = {}
for v in ('opt1', 'opt3', 'opt2'):
    vals[v] = []
    for P in periods:
        rs = rows(f'{R}/stale/p{P}ms_{v}.txt'); vals[v].append(100.0 * sum(r.get('false_confirm') == '1' for r in rs) / max(len(rs), 1))
chart('A_false_confirm.svg', 'Claim A: old-profile frame confirms the NEW profile', 'share of SET_RX_PROFILE commands (60 phases each) that disarm the auto-revert wrongly; 125 kHz polled SPI, 40 B P5 frames',
      [f'{p} ms' for p in periods], ('opt1', 'opt3', 'opt2'), vals, 'false confirm, % of commands', 100)

# E: stalled seeds
labels = ['gap<=200 ms', 'gap<=1000 ms']; vals = {}
for v in ('opt0', 'opt1', 'opt3', 'opt2'):
    vals[v] = []
    for g in (200, 1000):
        rs = rows(f'{R}/race/gap{g}_{v}.txt'); vals[v].append(100.0 * sum(r.get('panics') != '0' or r.get('live_rx') != '6/6' for r in rs) / max(len(rs), 1))
chart('E_stalls.svg', 'Claim E: radio stalls (HAL BUSY wait forever) in 20 s command/TX/RX storms', '% of 101 seeds that never recover RX; RX frames every 3..33 ms; polled SPI',
      labels, ('opt0', 'opt1', 'opt3', 'opt2'), vals, 'stalled seeds, %', 100)

# F: v1.2.1 starvation
cases = [('opt1', 'opt1_v121'), ('opt2', 'opt2_v121')]
labels = ['USP thread CPU %\n(no RX traffic)']
groups = ['v1.0.0 CPU %', 'v1.2.1 CPU %']; vals = {}
for v in ('opt1', 'opt3', 'opt2'):
    vals[v] = []
    for suf in ('', '_v121'):
        rs = rows(f'{R}/busyloop/{v}{suf}_norx.txt'); vals[v].append(float(rs[0]['usp_thread_cpu_pct']) if rs else 0)
chart('F_busy_loop.svg', 'Claim F: USP thread CPU share in idle continuous RX', 'usp_zephyr thread loop + radio_planner at v1.0.0 vs v1.2.1 (option 2 has no USP thread)',
      groups, ('opt1', 'opt3', 'opt2'), vals, 'CPU of lbm_main_thread, %', 100)
groups = ['v1.0.0', 'v1.2.1']; vals = {}
for v in ('opt1', 'opt3', 'opt2'):
    vals[v] = []
    for suf in ('', '_v121'):
        rs = rows(f'{R}/busyloop/{v}{suf}_norx.txt'); vals[v].append(float(rs[0]['rg10_cycles']) if rs else 0)
chart('F_rategroup_cycles.svg', 'Claim F: 10 Hz rate-group cycles completed in 9.3 s (nominal 93)', 'same runs; rateGroup10Hz and 1 Hz/modeManager get nothing once the USP thread spins',
      groups, ('opt1', 'opt3', 'opt2'), vals, 'cycles run', 100)

# G: delivered vs rg1 burst (gap 25 ms)
bursts = ['0.3', '5', '20', '50']; vals = {}
for v in ('opt1', 'opt2'):
    rs = rows(f'{R}/latency/rg1_gap25_{v}.txt'); vals[v] = [100.0 * int(r['delivered']) / int(r['sent']) for r in rs]
chart('G_loss_vs_rg1_burst.svg', 'Claim G: frames delivered vs rateGroup1Hz burst length', 'P5, 252 B frames, 25 ms idle gap between frames; rg1 runs at priority 3, uspRadio at 11',
      [b + ' ms' for b in bursts], ('opt1', 'opt2'), vals, 'delivered, % of sent', 100)
vals = {}
for v in ('opt1', 'opt2'):
    rs = rows(f'{R}/latency/rg1_gap25_{v}.txt'); vals[v] = [float(r['rg1_late_ms_max']) for r in rs]
chart('G_disturbance.svg', 'The other side of claim G: how late rateGroup1Hz runs while the radio receives', 'same runs; option 1 harvests on a priority-2 thread that burns polled-SPI CPU ahead of rg1',
      [b + ' ms' for b in bursts], ('opt1', 'opt2'), vals, 'rg1 max lateness, ms', None, '{:.1f}')

# queue blocked
vals = {k: [] for k in ('opt1', 'opt2')}
for v in vals:
    d = {}
    for l in open(f'{R}/blocked/{v}.txt'):
        k = l.split(' :: ')[0]; d[k] = float(kv(l)['probe_delay_ms'])
    for key in (f'{v} flow=tx txprof=0', f'{v} flow=cw txprof=0', f'{v} flow=hang'): vals[v].append(d[key] / 1000.0)
chart('queue_blocked.svg', 'Queue blocked: delay of an unrelated command posted 200 ms into the operation', '252 B LoRa SF8 TX (0.70 s airtime) | 5 s CONTINUOUS_WAVE | TX_DONE never arrives (anomaly B)',
      ['TX 252 B P0', 'CW 5 s', 'hung TX'], ('opt1', 'opt2'), vals, 'command delayed, s', None, '{:.2f}')
print('wrote', len(os.listdir(OUT)), 'figures to', OUT)
