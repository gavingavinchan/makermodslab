#!/usr/bin/env python3
"""Seeded VISCOUS trials. No inference unless --live; --stages executes the budgeted experiment.
Examples: sweep.py --config B0 --task apple --episodes 6 --live
          sweep.py --stages --live
          sweep.py --check
Resuming replays saved joint chunks from the seeded scene (no requests), then continues.
Uncertain requests consume budget permanently; only explicit pool_warming frees a reservation.
"""
import argparse
import fcntl
import json
import os
import statistics
import time
import warnings
from collections import deque
from pathlib import Path
import yam_baseline as y
import mujoco
import numpy as np
from sweep_scene import HERE, SweepEnv, PARK, fingertip_cad, table_region
from cross_scenes import success
from hand_path_tools import IKTrace, plain, clip_viscous_grips

ROOT = HERE / 'runs/sweep'
DEFAULT = dict(tcp='pad', arms='both', rows=48, region='off', wrist_r='normal')
PROMPTS = {'apple': 'put apple into the bowl', 'box': 'pick up the black and white box and put it on the black square'}
CAPS = {'apple': 18, 'box': 24}


def atomic(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    with temp.open('w') as f:
        json.dump(obj, f, indent=1, allow_nan=False)
        f.flush(); os.fsync(f.fileno())
    os.replace(temp, path)
    fd = os.open(path.parent, os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


class BudgetEnd(Exception):
    pass


class Ledger:
    def __init__(self, root=ROOT, cap=None):
        self.path = root / 'ledger.json'
        self.lock_path = root / 'ledger.lock'
        self.cap = cap if cap is not None else (json.loads(self.path.read_text())['cap'] if self.path.exists() else 900)

    def update(self, fn):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            data = json.loads(self.path.read_text()) if self.path.exists() else {'cap': self.cap, 'requests': []}
            assert data['cap'] == self.cap
            result = fn(data)
            data['confirmed_billed'] = sum(r['status'] == 'billed' for r in data['requests'])
            data['uncertain'] = sum(r['status'] == 'reserved' for r in data['requests'])
            data['budget_used'] = sum(r['status'] != 'pool_warming' for r in data['requests'])
            atomic(self.path, data)
            return result

    def read(self):
        return self.update(lambda d: d)

    def remaining(self):
        return self.cap - self.read()['budget_used']

    def episode_calls(self, episode):
        return sum(r['episode'] == episode and r['status'] != 'pool_warming' for r in self.read()['requests'])

    def reserve(self, episode, cap):
        def reserve(d):
            used = [r for r in d['requests'] if r['status'] != 'pool_warming']
            if len(used) >= self.cap or sum(r['episode'] == episode for r in used) >= cap:
                raise BudgetEnd()
            r = dict(id=len(d['requests']) + 1, episode=episode, call=1 + sum(r['episode'] == episode for r in used), status='reserved', utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
            d['requests'].append(r)
            return dict(r)
        return self.update(reserve)

    def finish(self, request_id, status):
        self.update(lambda d: d['requests'][request_id - 1].update(status=status))


def config_name(c):
    if c.get('rig') == 'yam': return 'YAM'
    parts = []
    for key, label in [('tcp', 'tip'), ('arms', c['arms']), ('wrist_r', c.get('wrist_r', 'normal')), ('rows', 'r24'), ('region', 'table')]:
        if c.get(key, DEFAULT[key]) != DEFAULT[key]:
            parts.append(label)
    return '_'.join(parts) or 'B0'


def parse_config(name):
    c = dict(DEFAULT)
    for part in name.split('_'):
        if part == 'B0': continue
        key, val = {'tip': ('tcp', 'fingertip'), 'single': ('arms', 'single'), 'parked': ('arms', 'parked'), 'copy': ('wrist_r', 'copy'), 'fixedcam': ('wrist_r', 'fixedcam'), 'blind': ('wrist_r', 'blind'), 'r24': ('rows', 24), 'table': ('region', 'table')}[part]
        c[key] = val
    return c


def ensure_tip():
    spec, faces = fingertip_cad()
    path = HERE / 'viscous_arm_fingertip.json'
    if path.exists():
        assert json.loads(path.read_text()) == spec
    else:
        atomic(path, spec)
    return spec, faces


def initialize(task, c, seed):
    if task == 'propeller' and c.get('rig') == 'yam':
        from propeller_scene import yam_env
        env = yam_env()
    else:
        env = SweepEnv(task, c)
    jitter = np.zeros(2) if task == 'flange' else np.random.default_rng(seed).uniform(-.02, .02, 2)
    env.data.joint(('bolt' if task == 'flange' else 'sponge' if task == 'propeller' else task) + '_free').qpos[:2] += jitter
    mujoco.mj_forward(env.model, env.data)
    windows = {k: deque(maxlen=y.FRAMES) for k in y.VIDEO_ROLES}
    steps = 0
    while any(len(w) < y.FRAMES for w in windows.values()):
        state = env.state(); env.step(state['left'], state['right'])
        if steps % y.FRAME_EVERY_STEPS == 0:
            images = y.capture(env, windows)
        steps += 1
    return env, windows, steps, images, jitter


def policy(c, env):
    result = y.vision_config(y.BETA_MODEL)
    if c.get('rig') == 'yam': return result
    result.pop('robot')
    result['ik']['arm'] = str(HERE / ('viscous_arm_fingertip.json' if c['tcp'] == 'fingertip' else 'viscous_arm.json'))
    if c['region'] == 'table':
        result['ik']['region'] = table_region(env)
    return result


def base_object(env, task, side):
    # The absent right arm retains its original YAM base frame for diagnostic coordinates only.
    if side == 'right' and env.single:
        pos = np.array([-.305, .24, .772])
        rot = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.]])
    else:
        base = env.data.body(side + ('_base_link' if hasattr(env, 'qadr') else '_base')); pos = base.xpos; rot = base.xmat.reshape(3, 3)
    return rot.T @ (env.data.body(task).xpos - pos)


def prepared_rows(env, reply, count):
    notices = []
    result = clip_viscous_grips(reply, notices)
    left = np.array(result['q_l_chunk'], float)[:count]
    right = np.repeat(PARK[None], len(left), axis=0) if env.ignore_right else np.array(result['q_r_chunk'], float)[:count]
    for side, rows in [('left', left), ('right', right)]:
        assert rows.shape == (count, 7) and np.isfinite(rows).all()
        if side == 'right' and env.ignore_right: continue
        limits = np.array([env.model.joint(f'{side}_joint{i}').range for i in range(1, 7)] + [[0, 1]])
        clipped = np.clip(rows, limits[:, 0], limits[:, 1])
        if np.any(rows != clipped): notices.append(side + ' joint limits clipped')
        rows[:] = clipped
    return left, right, notices


def measure_row(env, task, requested):
    p = env.data.body(task).xpos
    distances = {s: float(np.linalg.norm(requested[s] - base_object(env, task, s))) for s in env.sides}
    contact = False
    for con in env.data.contact:
        names = [env.model.body(env.model.geom_bodyid[g]).name for g in (con.geom1, con.geom2)]
        geoms = [env.model.geom(g).name or '' for g in (con.geom1, con.geom2)]
        if any('gripper_left' in n or 'gripper_right' in n for n in names) and any('table' in n for n in names + geoms):
            contact = True
    return distances, float(np.linalg.norm(p[:2] - [0, -.04])), float(p[2]), contact


def sheet(rec, path, labels):
    from PIL import Image, ImageDraw
    indices = rec.milestones
    canvas = Image.new('RGB', (960, 204 * ((len(indices) + 1) // 2)), 'white')
    draw = ImageDraw.Draw(canvas)
    for i, index in enumerate(indices):
        x, yy = i % 2 * 480, i // 2 * 204
        canvas.paste(Image.fromarray(rec.frames[index]).resize((480, 180)), (x, yy + 24))
        draw.text((x + 8, yy + 5), 'Start' if i == 0 else f'Call {labels[i - 1]}', fill='black')
    canvas.save(path)


def input_sheet(windows, path):
    """Latest JPEGs in the call-1 observation, with labels outside the images."""
    import base64
    import io
    from PIL import Image, ImageDraw
    images = {key: Image.open(io.BytesIO(base64.b64decode(frames[-1]))).convert('RGB') for key, frames in windows.items()}
    width, height = images['top'].size
    canvas = Image.new('RGB', (width * 3, height + 28), 'white')
    draw = ImageDraw.Draw(canvas)
    for i, (key, label) in enumerate([('top', 'top'), ('left', 'wrist_l'), ('right', 'wrist_r')]):
        canvas.paste(images[key], (width * i, 28))
        draw.text((width * i + 8, 8), 'Call 1: ' + label, fill='black')
    canvas.save(path, quality=95)


def episode(task, c, index, live, ledger, dry_root=None, prompt=None, variant=None, stub=False, first_episode_index=0):
    if task == 'flange' and live:
        raise ValueError('Flange scene is local-only; live inference is not authorized.')
    name = config_name(c)
    directory = (dry_root or ROOT) / name / task
    if variant: directory = directory / variant
    out = directory / f'ep_{index:02d}'
    instruction = prompt or PROMPTS.get(task, 'insert the bolt through the flange holes and screw the nut onto the other end' if task == 'flange' else 'clean the propeller with the sponge')
    flange = task == 'flange'
    call_cap = 4 if flange else 30 if task == 'propeller' else CAPS[task]
    propeller = task == 'propeller'
    obj = 'bolt' if flange else 'sponge' if propeller else task
    out.mkdir(parents=True, exist_ok=True)
    log_path = out / 'log.json'
    prior = json.loads(log_path.read_text()) if log_path.exists() else None
    if prior and prior.get('finished') and (out / 'run.mp4').exists():
        return prior
    seed = (20261023 if propeller else 20261020 + (10000 if task == 'box' else 0)) + index
    episode_id = f'{name}/{task}/' + (f'{variant}/' if variant else '') + f'ep_{index:02d}'
    env, windows, steps, images, jitter = initialize(task, c, seed)
    new_mode = env.parked or env.copy_right or env.fixed_right
    if (index == 0 and new_mode) or ((propeller or flange) and index == first_episode_index):
        input_sheet(windows, out / 'contact_sheet.jpg')
    if flange:
        from flange_scene import Measurements
        from propeller_scene import ViewsRecorder
        observer = Measurements(env)
        rec = ViewsRecorder(env)
    elif propeller:
        from propeller_scene import Measurements, ViewsRecorder
        observer = Measurements(env)
        rec = ViewsRecorder(env)
    else:
        rec = y.Recorder(env)
    rec.grab(env, images['top'])
    start_z = float(env.data.body(obj).xpos[2])
    metrics = dict(seed=seed, jitter_xy_m=jitter.tolist(), task=task, configuration=c, config_name=name,
                   episode=index, calls_used=0, success=False, first_success_call=None, confirmed_success_call=None,
                   closest_requested_hand_to_object_m={s: None for s in env.sides}, object_to_target_min_m=1.,
                   grasped=False, max_lift_m=0., finger_table_contact_rows=0, executed_rows=0,
                   calls=[], finished=False, start_object_xyz=env.data.body(obj).xpos.tolist())
    if propeller:
        metrics.update(prompt=instruction, prompt_id=variant, rig=c.get('rig','viscous'),
                       tool_propeller_contact_rows=0, gripper_propeller_contact_rows=0,
                       scrub_reversals=0, path_in_contact_m=0., path_within_1cm_m=0., row_measurements=[],
                       closest_requested_hand_to_sponge_m={s:None for s in env.sides})
    if flange:
        metrics.update(prompt=instruction, closest_requested_hand_to_nut_m={s:None for s in env.sides}, closest_requested_hand_to_hole_m={s:None for s in env.sides}, row_measurements=[], nut_engaged=False, max_bolt_inserted_depth_m=0., max_nut_turns=0., tool_fixture_contact_rows=0, gripper_fixture_contact_rows=0)
    metrics['policy'] = policy(c, env)
    metrics['calls_used'] = ledger.episode_calls(episode_id) if live else 0
    atomic(log_path, metrics)
    client = None; commanded = None; consecutive = 0
    saved = []
    chunks_path = out / 'chunks.jsonl'
    if chunks_path.exists():
        # Incomplete final JSON line can only be from a crash before its fsync; never request it again.
        for line in chunks_path.read_text().splitlines():
            try: saved.append(json.loads(line))
            except json.JSONDecodeError: break
        with chunks_path.open('w') as f:
            for chunk in saved: f.write(json.dumps(chunk) + '\n')
            f.flush(); os.fsync(f.fileno())
    else:
        chunks_path.touch()

    def execute(chunk):
        nonlocal steps, images, commanded, consecutive
        rows = chunk['executed_rows']; call = chunk['call']
        left = np.array(chunk['execution']['left']); right = np.array(chunk['execution']['right'])
        for i, (l, r) in enumerate(zip(left, right)):
            requested = {s: np.array(chunk['per_key'][f'action.{s}_eef_xyz'][i]) for s in env.sides}
            # Compare requested positions to the object's position before this row, as in earlier reports.
            if flange:
                requested_dist = observer.requested(requested)
                env.step(l, r)
                sample = observer.row(call, i, requested_dist)
                metrics['row_measurements'].append(sample)
                metrics['grasped'] |= sample['tool_grasped']
                metrics['max_lift_m'] = max(metrics['max_lift_m'], sample['grasp']['bolt']['lift_m'], sample['grasp']['nut']['lift_m'])
                for side, distances in requested_dist.items():
                    for key, field in [('bolt_m','closest_requested_hand_to_object_m'),('nut_m','closest_requested_hand_to_nut_m'),('hole_m','closest_requested_hand_to_hole_m')]:
                        old=metrics[field][side]
                        metrics[field][side]=distances[key] if old is None else min(old,distances[key])
                metrics['max_bolt_inserted_depth_m'] = max(metrics['max_bolt_inserted_depth_m'],sample['bolt_inserted_depth_m'])
                metrics['max_nut_turns'] = max(metrics['max_nut_turns'],sample['nut_turns'])
                metrics['nut_engaged'] |= sample['nut_engaged']
                for field, source in [('tool_fixture_contact_rows','tool_fixture_contact'),('gripper_fixture_contact_rows','gripper_fixture_contact'),('finger_table_contact_rows','finger_table_contact')]:
                    metrics[field] += sample[source]
                metrics['executed_rows'] += 1
            elif propeller:
                requested_dist = observer.requested(requested)
                env.step(l, r)
                sample = observer.row(call, i, requested_dist)
                metrics['row_measurements'].append(sample)
                for side, d in requested_dist.items():
                    for key, field in [('propeller_m','closest_requested_hand_to_object_m'),('sponge_m','closest_requested_hand_to_sponge_m')]:
                        old = metrics[field][side]
                        metrics[field][side] = d[key] if old is None else min(old,d[key])
                metrics['grasped'] |= sample['tool_grasped']
                metrics['max_lift_m'] = max(metrics['max_lift_m'],sample['sponge_lift_m'])
                for field,source in [('tool_propeller_contact_rows','tool_propeller_contact'),('gripper_propeller_contact_rows','gripper_propeller_contact'),('scrub_reversals','scrub_reversals'),('path_in_contact_m','path_in_contact_m'),('path_within_1cm_m','path_within_1cm_m')]:
                    metrics[field] += sample[source]
                metrics['executed_rows'] += 1
            else:
                distances = {s: float(np.linalg.norm(requested[s] - base_object(env, task, s))) for s in env.sides}
                env.step(l, r)
                _, distance, z, contact = measure_row(env, task, requested)
                for s, value in distances.items():
                    old = metrics['closest_requested_hand_to_object_m'][s]
                    metrics['closest_requested_hand_to_object_m'][s] = value if old is None else min(old, value)
                metrics['object_to_target_min_m'] = min(metrics['object_to_target_min_m'], distance)
                metrics['max_lift_m'] = max(metrics['max_lift_m'], z - start_z)
                metrics['grasped'] |= z - start_z > .03
                metrics['finger_table_contact_rows'] += int(contact)
                metrics['executed_rows'] += 1
            commanded = {'q_l_chunk': l, 'q_r_chunk': r}
            if steps % y.FRAME_EVERY_STEPS == 0:
                images = y.capture(env, windows)
                if client:
                    y.feed(client if c.get('rig') == 'yam' else y.CustomGripFeed(client), env, windows, instruction, y.BETA_MODEL, steps)
                rec.grab(env, images['top'])
            steps += 1
        rest = False if (propeller or flange) else success(env, task)
        consecutive = consecutive + 1 if rest else 0
        if rest and metrics['first_success_call'] is None: metrics['first_success_call'] = call
        if consecutive >= 2:
            metrics['success'] = True; metrics['confirmed_success_call'] = call
        metrics['calls'].append(dict(call=call, rows=rows, resting_at_target=rest,
                                     object_xyz=env.data.body(obj).xpos.tolist(), notices=chunk['notices']))
        rec.milestones.append(len(rec.frames) - 1)
        metrics['calls_used'] = ledger.episode_calls(episode_id) if live else 0
        if env.parked:
            metrics['parked_max_joint_deviation_rad'] = env.parked_max_joint_deviation
            metrics['parked_max_solver_deviation_rad'] = env.parked_max_solver_deviation
        atomic(log_path, metrics)

    try:
        for chunk in saved: execute(chunk)
        if (propeller or flange) and (stub or not live):
            # Execute full-length local hold replies through the same durable chunk/measurement path.
            for call in range(len(saved)+1,call_cap+1):
                state=env.state()
                per_key={f'action.{side}_eef_xyz':np.repeat(base_object(env,'bolt' if flange else 'propeller',side)[None],c['rows'],axis=0).tolist() for side in env.sides}
                chunk=dict(call=call,executed_rows=c['rows'],per_key=per_key,notices=['local stub; no inference'],execution={side:np.repeat(state[side][None],c['rows'],axis=0).tolist() for side in env.sides})
                with chunks_path.open('a') as f:
                    f.write(json.dumps(chunk)+'\n');f.flush();os.fsync(f.fileno())
                execute(chunk)
            metrics.update(finished=True,dry_run=True,termination=f'stub_fixed_{call_cap}_calls',stub_calls=call_cap)
            atomic(log_path,metrics)
            return metrics
        if not live:
            if new_mode:
                # Full maximum-length episode, holding the working arm's initial pose.
                left = env.state()['left'].copy()
                for row in range(CAPS[task] * c['rows']):
                    env.step(left, PARK)
                    if row % y.FRAME_EVERY_STEPS == 0:
                        images = y.capture(env, windows)
                        rec.grab(env, images['top'])
                metrics['dry_executed_rows'] = CAPS[task] * c['rows']
                metrics['dry_sim_seconds'] = float(env.data.time)
                if env.parked:
                    metrics['parked_max_joint_deviation_rad'] = env.parked_max_joint_deviation
                    metrics['parked_max_solver_deviation_rad'] = env.parked_max_solver_deviation
            atomic(log_path, {**metrics, 'dry_run': True, 'policy': policy(c, env)})
            return metrics
        import dream
        warnings.filterwarnings('ignore', category=dream.DreamSlowNetworkWarning)
        client = dream.DreamClient(api_key=y.settings().get('DREAM_API_KEY', ''))
        client.policy(policy(c, env))
        ik = client._policy._ik
        trace = IKTrace(ik, task)
        # Sim time, not network latency, drives motion. Match the existing client path.
        y.feed(client if c.get('rig') == 'yam' else y.CustomGripFeed(client), env, windows, instruction, y.BETA_MODEL, steps, every=0)
        while not metrics['success'] and ledger.episode_calls(episode_id) < call_cap:
            request = ledger.reserve(episode_id, call_cap)
            try:
                reply = client.get_action(commanded=commanded)
            except dream.DreamServerError as error:
                if error.code == 'DA-2000' and error.reason == 'pool_warming':
                    ledger.finish(request['id'], 'pool_warming')
                    print(f'{episode_id}: pool_warming, not billed', flush=True)
                    time.sleep(min(error.retry_after_s or 10, 60))
                    continue
                raise
            ledger.finish(request['id'], 'billed')
            left, right, notices = prepared_rows(env, reply, c['rows'])
            chunk = dict(call=request['call'], ledger_id=request['id'], anchor=trace.anchor,
                         per_key=plain(reply['per_key']), solved_joint_chunk=plain({k: reply[k] for k in ('q_l_chunk', 'q_r_chunk')}),
                         client_ik=trace.context, executed_rows=len(left), execution=plain(dict(left=left, right=right)), notices=notices)
            with chunks_path.open('a') as f:
                f.write(json.dumps(chunk) + '\n'); f.flush(); os.fsync(f.fileno())
            execute(chunk)
            print(f"{episode_id}: call {request['call']}/{call_cap}, success={metrics['success']}, lift={metrics['max_lift_m']:.3f}, budget={ledger.read()['budget_used']}/{ledger.cap}", flush=True)
        metrics['calls_used'] = ledger.episode_calls(episode_id)
        metrics['finished'] = True
        metrics['termination'] = 'success_two_calls' if metrics['success'] else 'episode_cap'
        atomic(log_path, metrics)
        return metrics
    finally:
        if client:
            client.close()
        if rec.frames:
            import imageio.v2 as imageio
            imageio.mimsave(out / 'run.mp4', rec.frames, fps=5)
            if index == 0 and not (new_mode or propeller or flange): sheet(rec, out / 'contact_sheet.jpg', [c['call'] for c in metrics['calls']])
        env.close(); rec.renderer.close()


def summarize(c):
    name = config_name(c); directory = ROOT / name
    result = {'configuration': c, 'tasks': {}}
    for task in CAPS:
        episodes = [json.loads(p.read_text()) for p in sorted((directory / task).glob('ep_*/log.json'))]
        episodes = [e for e in episodes if e.get('finished')]
        wins = [e for e in episodes if e['success']]
        result['tasks'][task] = dict(episodes=len(episodes), successes=len(wins), grasps=sum(e['grasped'] for e in episodes),
            median_calls_to_success=statistics.median(e['confirmed_success_call'] for e in wins) if wins else None,
            calls_to_success_total=sum(e['confirmed_success_call'] for e in wins),
            billed_calls=sum(e['calls_used'] for e in episodes), finger_table_contact_rows=sum(e['finger_table_contact_rows'] for e in episodes),
            executed_rows=sum(e['executed_rows'] for e in episodes),
            good_video=next((str(directory / task / f"ep_{e['episode']:02d}/run.mp4") for e in episodes if e['success']), None),
            bad_video=next((str(directory / task / f"ep_{e['episode']:02d}/run.mp4") for e in episodes if not e['success']), None))
    atomic(directory / 'summary.json', result)
    lines = [f'# {name}', '', '| Task | Episodes | Successes | Grasps | Median calls to success | Finger/table rows |', '|---|---:|---:|---:|---:|---:|']
    for task, v in result['tasks'].items():
        lines.append(f"| {task} | {v['episodes']} | {v['successes']} | {v['grasps']} | {v['median_calls_to_success']} | {v['finger_table_contact_rows']} |")
    for task, v in result['tasks'].items():
        lines.extend(['', f"{task} good video: {v['good_video'] or 'none'}; bad video: {v['bad_video'] or 'none'}", ''])
    (directory / 'summary.md').write_text('\n'.join(lines))
    return result


def run_config(c, tasks, episodes, live, ledger):
    for task in tasks:
        for i in range(episodes):
            episode(task, c, i, live, ledger, dry_root=None if live else ROOT / 'dry')
            if live: summarize(c)
    return summarize(c) if live else None


def score(c, tasks=('apple', 'box')):
    s = summarize(c)['tasks']
    return (sum(s[t]['successes'] for t in tasks), -sum(s[t]['calls_to_success_total'] for t in tasks))


def stages(live, ledger):
    if not live: raise ValueError('--stages requires --live; use --check for no-call validation')
    plan_path = ROOT / 'stages.json'
    plan = json.loads(plan_path.read_text()) if plan_path.exists() else {'decisions': {}}
    d = plan['decisions']
    def save(): atomic(plan_path, plan)
    for c in [dict(DEFAULT), dict(DEFAULT, tcp='fingertip')]: run_config(c, CAPS, 6, True, ledger)
    winner = max([dict(DEFAULT), dict(DEFAULT, tcp='fingertip')], key=score)
    d['stage1_winner'] = config_name(winner); d['stage1_scores'] = {config_name(c): score(c) for c in [DEFAULT, dict(DEFAULT, tcp='fingertip')]}; save()
    single = dict(winner, arms='single')
    run_config(single, CAPS, 6, True, ledger)
    carried = single if score(single) >= score(winner) else winner
    d['stage2_carried'] = config_name(carried); d['stage2_scores'] = {config_name(c): score(c) for c in [winner, single]}; save()
    variants = [dict(carried, rows=24), dict(carried, region='table')]
    if 'stage3_episodes' not in d:
        d['stage3_episodes'] = min(6, ledger.remaining() // (2 * CAPS['box'])); save()
    n = d['stage3_episodes']
    for c in variants: run_config(c, ['box'], n, True, ledger)
    # Compare equal seed prefixes when budget shortened stage 3.
    def prefix_score(c):
        records = [json.loads((ROOT / config_name(c) / 'box' / f'ep_{i:02d}/log.json').read_text()) for i in range(n)]
        return sum(e['success'] for e in records), -sum(e['confirmed_success_call'] for e in records if e['success'])
    best = max([carried] + variants, key=prefix_score)
    combo = dict(carried)
    for c in variants:
        if prefix_score(c) > prefix_score(carried):
            if c['rows'] == 24: combo['rows'] = 24
            if c['region'] == 'table': combo['region'] = 'table'
    if combo == carried: combo = best
    d['stage3_scores'] = {config_name(c): prefix_score(c) for c in [carried] + variants}; d['stage4_config'] = config_name(combo); save()
    if 'stage4_start_budget_used' not in d:
        d['stage4_start_budget_used'] = ledger.read()['budget_used']; save()
    if 'stage4_episodes' not in d:
        # Existing paired episodes are reused; budget covers only missing episodes.
        n4 = 0
        for n4_candidate in range(1, 7):
            needed = sum(CAPS[t] for t in CAPS for i in range(n4_candidate) if not (ROOT / config_name(combo) / t / f'ep_{i:02d}/log.json').exists())
            if needed <= ledger.remaining(): n4 = n4_candidate
        d['stage4_episodes'] = n4; save()
    if d['stage4_episodes']: run_config(combo, CAPS, d['stage4_episodes'], True, ledger)
    d['complete'] = True
    state = ledger.read()
    d['ledger'] = {k: v for k, v in state.items() if k != 'requests'}
    d['stage4_additional_calls'] = state['budget_used'] - d['stage4_start_budget_used']
    if combo == carried:
        d['stage4_note'] = 'Selected carried configuration already has six episodes per task; no additional trials.'
    save()
    print(json.dumps({k: v for k, v in d.items() if k != 'ledger'}, indent=2), flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default='B0', help='B0 or underscore combination of tip,single,parked,copy,fixedcam,blind,r24,table')
    p.add_argument('--task', choices=['apple', 'box', 'both', 'propeller', 'flange'], default='both')
    p.add_argument('--episodes', type=int, default=6)
    p.add_argument('--prompt-id', choices=['P1','P2','P3','P4'], default='P1', help='Propeller prompt')
    p.add_argument('--live', action='store_true')
    p.add_argument('--stages', action='store_true')
    p.add_argument('--check', action='store_true')
    args = p.parse_args()
    ROOT.mkdir(parents=True, exist_ok=True)
    ensure_tip()
    with (ROOT / 'runner.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ledger = Ledger()
        if args.check:
            from sweep_check import check
            check()
        elif args.stages: stages(args.live, ledger)
        elif args.task == 'flange':
            from flange_scene import PROMPT
            for index in range(args.episodes):
                episode('flange', parse_config(args.config), index, args.live, ledger,
                        dry_root=HERE/'runs/flange/dry', prompt=PROMPT, stub=True)
        elif args.task == 'propeller':
            from propeller_sweep import PROMPTS, authorize
            if args.live: ledger = authorize()
            for index in range(args.episodes):
                episode('propeller', parse_config(args.config), index, args.live, ledger,
                        dry_root=None if args.live else ROOT/'dry',
                        prompt=PROMPTS[args.prompt_id], variant=args.prompt_id)
        else:
            assert args.episodes > 0
            run_config(parse_config(args.config), CAPS if args.task == 'both' else [args.task], args.episodes, args.live, ledger)


if __name__ == '__main__':
    main()
