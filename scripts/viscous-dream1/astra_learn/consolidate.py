#!/usr/bin/env python3
import argparse
import datetime as dt
import difflib
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess as sp
from outcome import ROOT, RUNS, score, outcome_line

STORE = ROOT / 'astra_learn/lessons'
# Conservative lint: numeric tuples/arrays, explicit Cartesian assignments, named
# joints with numbers, absolute coordinate/angle recipes. Relative scalars are allowed.
NUMBER = r'[-+]?\d*\.?\d+(?:e[-+]?\d+)?'
COORD = re.compile(rf'[\[(]\s*{NUMBER}\s*,\s*{NUMBER}\s*,\s*{NUMBER}'
                   rf'|\b[xyz]\s*[:=]\s*{NUMBER}'
                   rf'|\b(?:j[1-7]|joint\s*[1-7]|shoulder|elbow)\s*[:=]\s*{NUMBER}'
                   r'|\b(?:absolute coordinates?|joint angles?|quaternion)\s*[:=]', re.I)


def validate(text):
    errors = []
    if not text.strip(): errors.append('empty file')
    if len(text.splitlines()) > 100: errors.append('more than 100 lines')
    if COORD.search(text): errors.append('coordinate / joint-angle pattern')
    if not all(re.search(label, text, re.I) for label in ['general', 'ROV', 'tabletop']):
        errors.append('missing scene/general labels')
    return errors


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--from', dest='parent', required=True)
    p.add_argument('--episode', required=True)
    a = p.parse_args()
    if not re.fullmatch(r'v\d{3}', a.parent) or not re.fullmatch(r'[\w-]+', a.episode):
        p.error('expected vNNN and safe episode name')
    lock = (STORE / '.consolidate.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    parent = STORE / (a.parent + '.md')
    version = f'v{int(a.parent[1:]) + 1:03}'
    target = STORE / (version + '.md')
    if target.exists(): raise RuntimeError('version already exists')
    if not (RUNS / a.episode / 'episode.jsonl').exists(): raise RuntimeError('episode log missing')
    log = Path(str(RUNS / a.episode) + '_astra.log')
    outcome = score(RUNS / a.episode)
    ws = ROOT.parent / 'astra_ws' / f'consolidate_{version}_{a.episode}'
    ws.mkdir()  # fresh, isolated sibling; no store, robot, sim log or other episodes
    (ws / 'LESSONS.md').write_bytes(parent.read_bytes())
    (ws / 'previous_attempt.log').write_bytes(log.read_bytes())
    (ws / 'outcome.txt').write_text('POST-HOC SIMULATOR GROUND TRUTH\n' + json.dumps(outcome, indent=2) + '\n')
    # Do not expose the host's session path in the supplied outcome.
    data = json.loads(json.dumps(outcome)); data.pop('session', None)
    (ws / 'outcome.txt').write_text('POST-HOC SIMULATOR GROUND TRUTH\n' + json.dumps(data, indent=2) + '\n')
    prompt = (ROOT / 'astra_learn/ASTRA_CONSOLIDATE_PROMPT.md').read_text()
    env = os.environ.copy()
    env['PATH'] = str(Path.home() / '.nvm/versions/node/v22.17.1/bin') + ':' + env['PATH']
    attempts = []
    for attempt in range(1, 3):
        attempt_log = STORE / f'{version}_attempt{attempt}.log'
        with attempt_log.open('x') as output:
            cmd = ['nohup', 'setsid', 'codex', 'exec', '-m', 'gpt-6-astra', '-c', 'model_reasoning_effort="high"',
                   '--dangerously-bypass-approvals-and-sandbox', '--skip-git-repo-check', '-C', str(ws),
                   '-o', str(STORE / f'{version}_attempt{attempt}_final.md'), prompt]
            status = sp.run(cmd, cwd=ws, env=env, stdin=sp.DEVNULL, stdout=output, stderr=sp.STDOUT).returncode
        result = (ws / 'LESSONS.md').read_text()
        errors = validate(result)
        if status: errors.append(f'Codex exit {status}')
        attempts.append(dict(attempt=attempt, exit_code=status, lines=len(result.splitlines()), errors=errors))
        if not errors: break
        prompt += '\nYour previous output failed these mechanical checks: ' + ', '.join(errors) + '. Correct LESSONS.md now.\n'
    (STORE / f'{version}_checks.json').write_text(json.dumps(attempts, indent=2) + '\n')
    if errors: raise RuntimeError('lesson rules failed after retry: ' + ', '.join(errors))
    # Never overwrite an existing version, including after interrupted attempts.
    with target.open('x') as f: f.write(result)
    diff = ''.join(difflib.unified_diff(parent.read_text().splitlines(True), result.splitlines(True),
                                        fromfile=parent.name, tofile=target.name))
    (STORE / f'{version}.diff').write_text(diff)
    with (STORE / 'CHANGELOG.md').open('a') as f:
        f.write(f'\n- **{version}**; parent {a.parent}; episode `{a.episode}`; {len(result.splitlines())} lines.\n'
                f'  Outcome: {outcome_line(outcome)}\n')
    print(f'{version}: {len(result.splitlines())} lines; {outcome_line(outcome)}', flush=True)


if __name__ == '__main__': main()
