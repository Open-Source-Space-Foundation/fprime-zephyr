#!/usr/bin/env python3
"""Run one scenario binary over a grid of environment settings, in parallel, and collect its RESULT lines.

usage: sweep.py OUTFILE BINARY VARIANT 'VAR=a,b,c' 'VAR2=...' [--const K=V ...]
Every combination of the listed variables is run once; each process prints one 'RESULT k=v ...' line.
The raw RESULT lines are written to OUTFILE (one per run) so every number in RESULTS.md can be re-derived.
"""
import itertools, os, subprocess, sys, concurrent.futures as cf

def main():
    out, binary, variant = sys.argv[1:4]
    args = sys.argv[4:]
    consts = {}
    grid = []
    i = 0
    while i < len(args):
        if args[i] == '--const':
            k, v = args[i + 1].split('=', 1); consts[k] = v; i += 2
        else:
            k, v = args[i].split('=', 1); grid.append((k, v.split(','))); i += 1
    here = os.path.dirname(os.path.abspath(__file__))
    exe = os.path.join(here, '..', 'build', variant, binary + '.bin')
    if not os.path.exists(exe):
        sys.exit(f'missing {exe}: build it first (make VARIANT={variant} bins)')
    combos = list(itertools.product(*[[(k, x) for x in vs] for k, vs in grid])) if grid else [()]
    def run(c):
        env = dict(os.environ); env.update(consts); env.update(dict(c))
        p = subprocess.run([exe], env=env, capture_output=True, text=True, timeout=600, cwd=os.path.join(here, '..', 'build', variant))
        lines = [l for l in p.stdout.splitlines() if l.startswith('RESULT')]
        tag = ' '.join(f'{k}={v}' for k, v in c)
        return (tag, lines[-1] if lines else f'RESULT FAILED rc={p.returncode} {p.stderr[-300:]!r}')
    with cf.ThreadPoolExecutor(max_workers=8) as ex:
        res = list(ex.map(run, combos))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, 'w') as f:
        f.write(f'# {binary} variant={variant} consts={consts}\n')
        for tag, line in res:
            f.write(f'{tag} | {line}\n')
    print(f'{len(res)} runs -> {out}')

main()
