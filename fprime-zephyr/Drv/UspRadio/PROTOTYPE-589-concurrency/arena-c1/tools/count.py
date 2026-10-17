#!/usr/bin/env python3
"""Line and primitive counts for the #589 side-by-side table.

Code line = not blank, not only a comment (// or /* ... */ blocks).
Primitives: k_sem_* calls, atomic operations (.load/.store/.fetch_*/.exchange
on std::atomic members, plus std::atomic declarations), k_sleep calls, and
rp->tasks[] reads.  Usage: count.py <worktree root>
"""
import re, sys, pathlib

def code_lines(text):
    n, in_block = 0, False
    for line in text.splitlines():
        s = line.strip()
        if in_block:
            if '*/' in s:
                in_block = False
                s = s.split('*/', 1)[1].strip()
            else:
                continue
        if s.startswith('/*'):
            if '*/' not in s:
                in_block = True
            continue
        if not s or s.startswith('//'):
            continue
        n += 1
    return n

PRIMS = {
    'k_sem': r'\bk_sem_(take|give|init)\s*\(',
    'atomic': r'(std::atomic<|\.(load|store|fetch_add|fetch_sub|exchange|compare_exchange_\w+)\s*\()',
    'k_sleep': r'\bk_sleep\s*\(',
    'tasks[]': r'tasks\[',
}

def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)

def row(name, files):
    tot = {'lines': 0, **{k: 0 for k in PRIMS}}
    for f in files:
        t = f.read_text()
        tot['lines'] += code_lines(t)
        c = strip_comments(t)
        for k, rx in PRIMS.items():
            tot[k] += len(re.findall(rx, c))
    print('\t'.join([name] + [str(tot[k]) for k in ['lines'] + list(PRIMS)] + [' '.join(f.name for f in files)]))

root = pathlib.Path(sys.argv[1]) / 'fprime-zephyr/Drv/UspRadio'
p = root / 'PROTOTYPE-589-concurrency'
print('set\tcode_lines\tk_sem\tatomic\tk_sleep\ttasks[]\tfiles')
row('opt1 .cpp', [root / 'UspRadio.cpp', root / 'RalSessionImpl.cpp'])
row('opt1 .cpp+.hpp', [root / 'UspRadio.cpp', root / 'RalSessionImpl.cpp', root / 'UspRadio.hpp', root / 'RalSessionImpl.hpp', root / 'RalSession.hpp'])
row('opt2 sketch (partial)', [p / 'single-queue/UspRadio.cpp', p / 'single-queue/RalSessionSt.cpp'])
row('opt2 sketch+hpp', [p / 'single-queue/UspRadio.cpp', p / 'single-queue/RalSessionSt.cpp', p / 'single-queue/RalSessionSt.hpp'])
b = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else None
if b and (b / 'RalSessionImpl_opt3.cpp').exists():
    row('opt3 .cpp (diff applied)', [root / 'UspRadio.cpp', b / 'RalSessionImpl_opt3.cpp'])
