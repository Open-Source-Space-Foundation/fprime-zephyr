#!/usr/bin/env python3
"""Code-line and concurrency-primitive counts for the three options.
Code line = non-blank line that is not comment-only (// and /* */ stripped).  Reproduces the table's '920 (measured)'
for option 1 if the 4 UspRadio/RalSessionImpl files are counted; option 2 counts the completed sketch (opt2/ files);
option 3 counts the patched RalSessionImpl.cpp built by the Makefile."""
import re, sys, os, subprocess

def code_lines(path):
    s = open(path, encoding='utf-8', errors='replace').read()
    s = re.sub(r'/\*.*?\*/', lambda m: '\n' * m.group(0).count('\n'), s, flags=re.S)
    n = 0
    for line in s.split('\n'):
        t = re.sub(r'//.*', '', line).strip()
        if t:
            n += 1
    return n

def text_nocomments(path):
    s = open(path, encoding='utf-8', errors='replace').read()
    s = re.sub(r'/\*.*?\*/', '', s, flags=re.S)
    return '\n'.join(re.sub(r'//.*', '', l) for l in s.split('\n'))

PRIMS = {
    'k_sem take+give calls': r'\bk_sem_(take|give)\(',
    'atomics (std::atomic uses)': r'std::atomic|\.load\(|\.store\(|\.fetch_(add|sub)\(|\.exchange\(',
    'k_sleep': r'\bk_(m|u)?sleep\(',
    'tasks[] reads': r'->tasks\[',
    'k_mutex / SMTC_SW_PLATFORM': r'\bk_mutex_(lock|unlock)\(|SMTC_SW_PLATFORM',
    'k_thread_create': r'k_thread_create',
}

def report(name, files):
    tot = 0
    txt = ''
    for f in files:
        tot += code_lines(f)
        txt += text_nocomments(f) + '\n'
    prim = {k: len(re.findall(v, txt)) for k, v in PRIMS.items()}
    print(f'{name}: code_lines={tot} files={[os.path.basename(f) for f in files]}')
    for k, v in prim.items():
        print(f'    {k:36s} {v}')

here = os.path.dirname(os.path.abspath(__file__))
ur = os.path.abspath(os.path.join(here, '..', '..', '..'))      # .../Drv/UspRadio
b3 = os.path.join(here, '..', 'build', 'opt3', 'patched', 'fprime-zephyr', 'Drv', 'UspRadio')
o2 = os.path.join(here, '..', 'opt2')
common = [os.path.join(ur, 'UspRadio.cpp'), os.path.join(ur, 'UspRadio.hpp')]
report('option 1 (#14+#26, as committed)', common + [os.path.join(ur, 'RalSessionImpl.cpp'), os.path.join(ur, 'RalSessionImpl.hpp')])
if os.path.exists(os.path.join(b3, 'RalSessionImpl.cpp')):
    report('option 3 (diff applied)', common + [os.path.join(b3, 'RalSessionImpl.cpp'), os.path.join(b3, 'RalSessionImpl.hpp')])
report('option 2 (sketch completed; excludes the ~95-line apply_params.inc, see below)',
       [os.path.join(o2, f) for f in ('UspRadioSt.cpp', 'UspRadioSt.hpp', 'RalSessionSt.cpp', 'RalSessionSt.hpp')])
print('option 2 apply_params.inc (applyLoRa/applyGfsk, unchanged from RalSessionImpl):', code_lines(os.path.join(o2, 'apply_params.inc')))
# the option-1 file also contains applyLoRa/applyGfsk; count its share for a like-for-like number
