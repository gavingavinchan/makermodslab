#!/usr/bin/env python3
"""Fixed-v001, alternating six-run A/B. No consolidation during this experiment."""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import statistics
import subprocess as sp
import sys
from outcome import ROOT, RUNS

PLAN = [(f'astra_ab_{arm}{i}', arm) for i in range(1, 4) for arm in ('L', 'N')]
COLUMNS = ['Episode', 'Arm', 'Success', 'Seated', 'Commands', 'Sim s', 'Wall s', 'Drops', 'Ended by', 'Claim mismatch']


def command(name, arm):
    cmd = [str(ROOT / 'astra_learn/run_episode.sh'), '--episode', name,
           '--profile', 'astra_rov_backup2', '--prompt',
           str(ROOT / ('ASTRA_OPERATOR_PROMPT_BACKUP.md' if arm == 'L' else 'ASTRA_OPERATOR_PROMPT_BACKUP_NOLESSONS.md')),
           '--wall-cap-min', '240']
    if arm == 'L':
        cmd += ['--lessons', str(ROOT / 'astra_learn/lessons/v001.md')]
    return cmd


def cell(v):
    if v is None: return '—'
    if isinstance(v, float): return f'{v:.2f}'
    return str(v).replace('|', '/').replace('\n', ' ')


def row(o):
    return '| ' + ' | '.join(cell(o.get(k)) for k in ['episode', 'arm', 'success', 'nut_seated', 'commands',
                      'sim_s', 'wall_s', 'nut_drops', 'ended_by', 'claim_vs_truth']) + ' |\n'


def arm_summary(rows):
    text = '\n| Arm | Successes | Seated | Median commands | Median sim s | Median wall s | Total drops | Claim mismatches |\n'
    text += '| --- | --- | --- | --- | --- | --- | --- | --- |\n'
    for arm in ('L', 'N'):
        group = [o for o in rows if o['arm'] == arm]
        def median(k):
            values = [o[k] for o in group if o.get(k) is not None]
            return statistics.median(values) if values else None
        values = [arm, f"{sum(o.get('success') is True for o in group)}/{len(group)}",
                  sum(o.get('nut_seated') is True for o in group), median('commands'), median('sim_s'), median('wall_s'),
                  sum(o.get('nut_drops') or 0 for o in group), sum(bool(o.get('claim_vs_truth')) for o in group)]
        text += '| ' + ' | '.join(cell(v) for v in values) + ' |\n'
    return text + '\nMedians include every recorded episode with that metric, including failures/caps; missing crash metrics are excluded.\n'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dry-run', action='store_true', help='print plan without creating files or launching anything')
    a = p.parse_args()
    if a.dry_run:
        import shlex
        for name, arm in PLAN: print(shlex.join(command(name, arm)))
        return
    dest = RUNS / 'ab'
    dest.mkdir()  # refuse to overwrite an experiment
    summary = dest / 'summary.md'
    logfile = dest / 'ab.log'
    def progress(text):
        with logfile.open('a') as f:
            f.write(dt.datetime.now(dt.timezone.utc).isoformat() + ' ' + text + '\n')
    lessons_hash = hashlib.sha256((ROOT / 'astra_learn/lessons/v001.md').read_bytes()).hexdigest()
    summary.write_text('# Astra lessons A/B\n\nFixed profile astra_rov_backup2, seed 20261024, G3; 240 min operator cap.\n'
                       f'L uses fixed v001 SHA256 `{lessons_hash}`; N has no LESSONS.md.\n\n'
                       '| ' + ' | '.join(COLUMNS) + ' |\n' + '| ' + ' | '.join(['---'] * len(COLUMNS)) + ' |\n')
    rows = []
    for name, arm in PLAN:
        progress(f'START {name} arm={arm}')
        cmd = command(name, arm)
        # Lock the treatment even if somebody edits the notes during the day-long chain.
        if hashlib.sha256((ROOT / 'astra_learn/lessons/v001.md').read_bytes()).hexdigest() != lessons_hash:
            raise RuntimeError('v001 changed during A/B; aborting to preserve treatment')
        existed = (RUNS / name).exists()
        with (dest / f'{name}_runner.log').open('x') as log:
            try:
                rc = sp.run(cmd, cwd=ROOT, stdin=sp.DEVNULL, stdout=log, stderr=sp.STDOUT).returncode
            except OSError as exc:
                log.write(str(exc)); rc = 127
        path = RUNS / name / 'outcome.json'
        if path.exists() and not existed:
            o = json.loads(path.read_text())
        else:
            o = dict(episode=name, success=False, nut_seated=None, ended_by='crash',
                     runner_error=f'launch failed ({rc}); see {name}_runner.log')
        o.update(arm=arm, runner_exit_code=rc)
        rows.append(o)
        (dest / 'outcomes.json').write_text(json.dumps(rows, indent=2) + '\n')
        with summary.open('a') as f: f.write(row(o))
        progress(f"END {name} rc={rc} ended_by={o['ended_by']} success={o['success']}")
    with summary.open('a') as f: f.write(arm_summary(rows))
    progress('COMPLETE six episodes; summary.md updated')


if __name__ == '__main__': main()
