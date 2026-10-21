"""Dry-run every existing runner mode; no --live and no inference."""
import os
import subprocess
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
OUT = HERE / 'runs/sweep/validation/baseline_modes'
OUT.mkdir(parents=True, exist_ok=True)
variants = [(r, m, t, w) for r, m, tasks, wrists in [('yam', 'station', ('apple','orange','cube','box'), ('maker',)), ('yam', 'vision', ('apple','orange','cube','box'), ('maker',)), ('viscous','vision',('apple','cube','box'),('maker','legacy')), ('makers','vision',('apple','box'),('maker',)), ('metal','vision',('apple','box'),('maker',))] for t in tasks for w in wrists]
for rig, mode, task, wrist in variants:
    name = f'{rig}_{mode}_{task}_{wrist}'
    with (OUT / (name + '.console.log')).open('w') as log:
        p = subprocess.run([sys.executable, str(HERE/'yam_baseline.py'), '--rig', rig, '--mode', mode, '--task', task, '--viscous-wrist', wrist, '--out', str(OUT/name)], stdout=log, stderr=subprocess.STDOUT, cwd=HERE)
    if p.returncode: raise RuntimeError(name + ' failed; see console log')
    print('PASS ' + name, flush=True)
print(f'PASS all {len(variants)} existing variants; zero inference', flush=True)
