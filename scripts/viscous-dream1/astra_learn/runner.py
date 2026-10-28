#!/usr/bin/env python3
"""Host-side orchestration only; never exposed to the operator."""
import argparse
import datetime as dt
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import subprocess as sp
import sys
import time
from outcome import ROOT, RUNS, score, session_for


def memory_check():
    db = Path.home() / '.codex/memories_1.sqlite'
    with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as con:
        rows = con.execute('SELECT count(*) FROM stage1_outputs').fetchone()[0]
    check = dict(checked=True, database=str(db), table='stage1_outputs', rows=rows,
                 checked_at=dt.datetime.now(dt.timezone.utc).isoformat())
    if rows != 0:
        raise RuntimeError(f'cross-session memory contamination: {rows} stage1_outputs rows')
    return check


def terminate(proc):
    if proc and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except sp.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


def simulator_finished(out):
    """The socket remains alive during export; inspect only the host-side log tail."""
    path = out / 'episode.jsonl'
    if not path.exists():
        return False
    with path.open('rb') as f:
        f.seek(max(0, path.stat().st_size - 262144))
        lines = f.read().splitlines()
    for line in reversed(lines):
        try:
            r = json.loads(line)
        except (ValueError, UnicodeDecodeError):
            continue
        if r.get('kind') == 'end' or (r.get('kind') == 'command_end' and r.get('command', '').startswith('done ')):
            return True
    return False


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode', required=True)
    p.add_argument('--profile', required=True)
    p.add_argument('--prompt', required=True, type=Path)
    p.add_argument('--lessons', type=Path)
    p.add_argument('--wall-cap-min', type=float, default=240)
    p.add_argument('--operator-cmd', help='TEST ONLY: shell command instead of Astra')
    a = p.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', a.episode) or a.wall_cap_min <= 0:
        p.error('safe episode name and positive wall cap required')
    prompt = a.prompt.resolve().read_text()
    lessons = a.lessons.resolve().read_bytes() if a.lessons else None
    # Serialize launch checks, including the interval before server.py appears in ps.
    lock = (ROOT / 'astra_learn/.runner.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if sp.run(['pgrep', '-f', '[a]stra_op/server.py'], stdout=sp.DEVNULL).returncode == 0:
        raise RuntimeError('refusing launch: a sim server is already running')
    check = memory_check()
    out = RUNS / a.episode
    ws = ROOT.parent / 'astra_ws' / a.episode
    if out.exists() or ws.exists() or list(RUNS.glob(a.episode + '_*')):
        raise RuntimeError('fresh episode required: output/workspace/log already exists')
    out.mkdir()  # atomic reservation; server requires an empty directory
    meta_path = Path(str(out) + '_runner.json')
    meta = dict(episode=a.episode, profile=a.profile, memory_check=check,
                lessons=str(a.lessons.resolve()) if a.lessons else None,
                prompt=str(a.prompt.resolve()), test_operator=bool(a.operator_cmd),
                wall_cap_min=a.wall_cap_min)
    def save():
        meta_path.write_text(json.dumps(meta, indent=2) + '\n')
    save()
    server = operator = None
    error = None
    env = os.environ.copy()
    env.update(MUJOCO_GL='egl', PYTHONDONTWRITEBYTECODE='1', TMPDIR=str(ROOT / 'astra_op/.cache/tmp'))
    env['PATH'] = str(Path.home() / '.nvm/versions/node/v22.17.1/bin') + ':' + env['PATH']
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'runner signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        with Path(str(out) + '_server.log').open('x') as log:
            server = sp.Popen(['nohup', 'setsid', '/home/gavin/dreamtrial/.venv/bin/python',
                               'astra_op/server.py', '--scene', 'rov', '--profile', a.profile,
                               '--condition', 'G3', '--seed', '20261024', '--episode', a.episode,
                               '--output', str(out)], cwd=ROOT, env=env, stdin=sp.DEVNULL,
                              stdout=log, stderr=sp.STDOUT)
        deadline = time.monotonic() + 360
        while '"ready"' not in Path(str(out) + '_server.log').read_text():
            if server.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('server exited or did not become ready in 360 s')
            time.sleep(1)
        if lessons is not None:
            (ws / 'LESSONS.md').write_bytes(lessons)
        else:
            assert not (ws / 'LESSONS.md').exists(), 'unexpected LESSONS.md'
        expected = {'README.md', 'robot', 'obs'} | ({'LESSONS.md'} if lessons is not None else set())
        assert {x.name for x in ws.iterdir()} == expected, 'unexpected workspace contents'
        meta['workspace_initial_files'] = sorted(expected)
        save()
        if a.operator_cmd:
            cmd = ['bash', '-c', a.operator_cmd]
        else:
            cmd = ['codex', 'exec', '-m', 'gpt-6-astra', '-c', 'model_reasoning_effort="high"',
                   '--dangerously-bypass-approvals-and-sandbox', '--skip-git-repo-check',
                   '-C', str(ws), '-o', str(out) + '_astra_final.md', prompt]
        with Path(str(out) + '_astra.log').open('x') as log:
            started = time.monotonic()
            meta['operator_started_at'] = dt.datetime.now(dt.timezone.utc).isoformat()
            operator = sp.Popen(['nohup', 'setsid', *cmd], cwd=ws, env=env,
                                stdin=sp.DEVNULL, stdout=log, stderr=sp.STDOUT)
            while operator.poll() is None:
                if time.monotonic() - started >= a.wall_cap_min * 60:
                    meta['ended_by'] = 'wall_cap'
                    terminate(operator)
                    break
                if server.poll() is not None:
                    terminate(operator)
                    break
                time.sleep(.2)
            meta['operator_wall_s'] = time.monotonic() - started
            meta['operator_exit_code'] = operator.returncode
            save()
        # Close if the operator forgot; queued requests may take up to the screw deadline.
        if server.poll() is None and (out / 'robot.sock').exists() and not simulator_finished(out):
            text = 'wall cap reached' if meta.get('ended_by') == 'wall_cap' else 'Operator process ended without calling done'
            meta['host_cleanup_claim'] = text
            meta.setdefault('ended_by', 'crash')
            save()
            try:
                with (out / 'runner_cleanup.txt').open('w') as log:
                    sp.run(['./robot', 'done', text], cwd=ws, stdin=sp.DEVNULL,
                           stdout=log, stderr=sp.STDOUT, timeout=315)
            except sp.TimeoutExpired:
                server.terminate()
        # Do not score incomplete video export; process exit is the completion boundary.
        while server.poll() is None:
            time.sleep(2)
        if meta.get('operator_exit_code') != 0 and not meta.get('ended_by'):
            # Preserve simulator caps; classify other abnormal operator exits as crashes.
            provisional = score(out)
            if provisional['ended_by'] not in ('wall_cap', 'command_cap', 'done'):
                meta['ended_by'] = 'crash'
        if server.returncode:
            raise RuntimeError(f'server exited {server.returncode}; inspect server log')
    except BaseException as exc:
        error = exc
        meta['error'] = f'{type(exc).__name__}: {exc}'
        meta.setdefault('ended_by', 'crash')
        terminate(operator)
        if server and server.poll() is None:
            server.terminate()
            server.wait()  # allow shutdown/export to finish
    finally:
        save()
        result = score(out)
        if (out / 'episode.jsonl').exists():
            with (out / 'timeline.txt').open('w') as log:
                sp.run([sys.executable, str(ROOT / 'astra_learn/timeline.py'), str(out / 'episode.jsonl')], stdout=log, stderr=sp.STDOUT)
        session = session_for(Path(str(out) + '_astra.log'))
        with (out / 'audit.txt').open('w') as audit:
            if session:
                sp.run([sys.executable, str(ROOT / 'astra_learn/audit_astra.py'), str(session)], stdout=audit, stderr=sp.STDOUT)
                audit.write(f'\nsession {session}\n')
            else:
                audit.write('No Codex session (test operator or launch failure).\n')
                audit.write(f'Test command: {a.operator_cmd}\n')
            audit.write('Initial workspace: ' + json.dumps(meta.get('workspace_initial_files')) + '\n')
        print(json.dumps(result, indent=2))
    if error:
        raise error
    return 0 if result['ended_by'] != 'crash' else 1


if __name__ == '__main__':
    sys.exit(main())
