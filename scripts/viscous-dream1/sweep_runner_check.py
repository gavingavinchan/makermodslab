"""Exercise the real runner using an in-memory fake client (no network)."""
import json
from types import SimpleNamespace
import numpy as np
import mujoco
import sweep
import dream


def check():
    out = sweep.ROOT / 'validation/episode_fixture'
    ledger = sweep.Ledger(out / 'budget', cap=3)
    if ledger.path.exists(): ledger.path.unlink()
    directory = out / 'B0/box/ep_00'
    for name in ('log.json', 'chunks.jsonl', 'run.mp4', 'contact_sheet.jpg'):
        path = directory / name
        if path.exists(): path.unlink()
    initialize, factory = sweep.initialize, dream.DreamClient
    state = {}
    def target_scene(task, c, seed):
        env, windows, steps, images, jitter = initialize(task, c, seed)
        env.data.joint('box_free').qpos[:3] = [0., -.04, .775]
        env.data.joint('box_free').qvel[:] = 0
        mujoco.mj_forward(env.model, env.data)
        for _ in range(20):
            s = env.state(); env.step(s['left'], s['right'])
        assert sweep.success(env, 'box')
        state['env'] = env
        return env, windows, steps, images, jitter
    class Fake:
        def __init__(self, **kwargs):
            self._policy = SimpleNamespace(_ik=SimpleNamespace(observe=lambda x: x, solve=lambda *a: None))
        def policy(self, config): pass
        def set_observation(self, *a, **kw): pass
        def close(self): pass
        def get_action(self, **kw):
            env = state['env']; q = env.state()
            per_key = {f'action.{s}_eef_xyz': np.repeat(sweep.base_object(env, 'box', s)[None], 48, axis=0) for s in q}
            return {'per_key': per_key, **{k: np.repeat(q[s][None], 48, axis=0) for k, s in [('q_l_chunk', 'left'), ('q_r_chunk', 'right')]}}
    try:
        sweep.initialize, dream.DreamClient = target_scene, Fake
        first = sweep.episode('box', sweep.DEFAULT, 0, True, ledger, dry_root=out)
        assert first['success'] and first['calls_used'] == 2 and first['first_success_call'] == 1 and first['confirmed_success_call'] == 2
        assert len(first['calls']) == 2 and first['executed_rows'] == 96
        # Mimic process death after chunks were fsynced but before final completion marker.
        record = json.loads((directory / 'log.json').read_text()); record['finished'] = False
        sweep.atomic(directory / 'log.json', record)
        second = sweep.episode('box', sweep.DEFAULT, 0, True, ledger, dry_root=out)
        assert second == first
        assert ledger.read()['confirmed_billed'] == 2
        # A finished episode is skipped without constructing a scene or fake client.
        def forbidden(*args): raise AssertionError('Finished episode was not skipped')
        sweep.initialize = forbidden
        assert sweep.episode('box', sweep.DEFAULT, 0, True, ledger, dry_root=out) == first
    finally:
        sweep.initialize, dream.DreamClient = initialize, factory
    print('PASS: two-call early stop, interrupted-episode replay, finished skip; fake client only', flush=True)

if __name__ == '__main__': check()
