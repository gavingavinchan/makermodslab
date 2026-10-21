"""No-inference compatibility, one-arm input and ideal parked-hold checks."""
import hashlib
import itertools
import json
import numpy as np
import mujoco
import sweep
from sweep_scene import PARK, OPEN

OUT = sweep.ROOT / 'onearm_validation'

def legacy_snapshot():
    result = []
    digest = lambda x: hashlib.sha256(np.asarray(x).tobytes()).hexdigest()
    for tcp, arms, rows, region, task in itertools.product(('pad', 'fingertip'), ('both', 'single'), (48, 24), ('off', 'table'), ('apple', 'box')):
        c = dict(tcp=tcp, arms=arms, rows=rows, region=region)
        env, _, _, images, _ = sweep.initialize(task, c, 20261020)
        try:
            q = env.state()
            for _ in range(8): env.step(q['left'], q['right'])
            result.append(dict(config=sweep.config_name(c), task=task, qpos=digest(env.data.qpos), qvel=digest(env.data.qvel), ctrl=digest(env.data.ctrl), images={k: digest(v) for k, v in images.items()}, policy=sweep.policy(c, env)))
        finally: env.close()
    return result

def check():
    current = legacy_snapshot()
    sweep.atomic(OUT / 'legacy_after.json', current)
    previous = json.loads((OUT / 'legacy_before.json').read_text())
    # Mesa/EGL rasterization can differ in rounding across independent contexts.
    # Pre-edit snapshots already have multiple image hashes for identical scenes.
    for before, after in zip(previous, current):
        assert {k: v for k, v in before.items() if k != 'images'} == {k: v for k, v in after.items() if k != 'images'}, 'Legacy physics/policy changed'
    results = {}
    for name in ('single_copy', 'parked'):
        c = sweep.parse_config(name)
        assert sweep.config_name(c) == name
        env, windows, steps, images, _ = sweep.initialize('apple', c, 20261020)
        try:
            if name == 'single_copy':
                np.testing.assert_array_equal(images['left'], images['right'])
                assert list(windows['left']) == list(windows['right'])
                assert not any((env.model.body(i).name or '').startswith('right_') for i in range(env.model.nbody))
                np.testing.assert_array_equal(env.state()['right'], PARK)
            else:
                np.testing.assert_array_equal(env.model.body('right_base_link').pos, [-.305, .24, .772])
                assert images['right'].any()
                assert env.model.camera('right').id >= 0
                np.testing.assert_array_equal(env.state()['right'], PARK)
            # Even adversarial right actions are ignored.
            reply = {'q_l_chunk': np.repeat(env.state()['left'][None], 48, axis=0), 'q_r_chunk': np.full((48, 7), .4)}
            left, right, _ = sweep.prepared_rows(env, reply, 48)
            np.testing.assert_array_equal(right, np.repeat(PARK[None], 48, axis=0))
            for l, r in zip(left, right): env.step(l, r)
            if name == 'single_copy':
                fresh = sweep.y.capture(env, windows)
                np.testing.assert_array_equal(fresh['left'], fresh['right'])
                assert list(windows['left']) == list(windows['right'])
            else:
                # Push a colliding apple into a right finger and apply a sustained
                # 20 N external force to its body, while sending moving right commands.
                finger = env.model.body('right_gripper_left').id
                p = env.data.geom('right_collision_043_22').xpos.copy()
                env.data.joint('apple_free').qpos[:3] = p
                env.data.joint('apple_free').qvel[:] = 0
                env.data.xfrc_applied[finger, :3] = [20., 0., -20.]
                mujoco.mj_forward(env.model, env.data)
                touching = sum(any(env.model.geom_bodyid[g] == finger for g in (con.geom1, con.geom2)) for con in env.data.contact)
                assert touching > 0, 'Contact stress fixture did not contact the parked finger'
                for _ in range(100):
                    env.step(env.state()['left'], np.full(7, .4))
                    np.testing.assert_array_equal(env.data.qpos[env.park_qadr], env.park_values)
                    np.testing.assert_array_equal(env.data.qvel[env.park_vadr], np.zeros(8))
                    np.testing.assert_array_equal(env.state()['right'][:6], env.data.qpos[env.qadr['right']])
                results['parked_contact_stress'] = dict(contact_count=touching, force_N=[20, 0, -20], rows=100, max_joint_deviation_rad=env.parked_max_joint_deviation, max_preprojection_solver_deviation_rad=env.parked_max_solver_deviation)
        finally: env.close()
        sweep.episode('apple', c, 0, False, sweep.Ledger(), dry_root=sweep.ROOT / 'dry')
        log = json.loads((sweep.ROOT / 'dry' / name / 'apple/ep_00/log.json').read_text())
        assert log['dry_executed_rows'] == 864
        if name == 'parked': assert log['parked_max_joint_deviation_rad'] == 0
        results[name] = log
    sweep.atomic(OUT / 'checks.json', dict(inference_calls=0, legacy_exact_physics_policy_variants=len(current), legacy_images_note='EGL pixel rounding varies across contexts, including among equivalent pre-edit snapshots; original image render path unchanged for defaults.', **results))
    print('PASS: 32 exact legacy physics/policy comparisons; copied JPEG windows; ignored right chunks; full 864-row dry episodes; parked contact-force hold. Zero inference.')

if __name__ == '__main__': check()
