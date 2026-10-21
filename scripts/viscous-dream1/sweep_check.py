"""No-inference validation of sweep geometry, controls, budget, and restart replay."""
import itertools
import json
from pathlib import Path
import yam_baseline as y
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from sweep_scene import HERE, SweepEnv, PARK


def check():
    from sweep import ROOT, DEFAULT, Ledger, BudgetEnd, atomic, initialize, policy, prepared_rows, config_name, ensure_tip, episode
    import dream
    out = ROOT / 'validation'; out.mkdir(parents=True, exist_ok=True)
    spec, tips = ensure_tip()
    results = []
    for tcp, arms, rows, region, task in itertools.product(('pad', 'fingertip'), ('both', 'single'), (48, 24), ('off', 'table'), ('apple', 'box')):
        c = dict(tcp=tcp, arms=arms, rows=rows, region=region)
        env, windows, steps, images, jitter = initialize(task, c, 20261020)
        try:
            if arms == 'single':
                assert not any((env.model.body(i).name or '').startswith('right_') for i in range(env.model.nbody))
                assert not any((env.model.camera(i).name or '') == 'right' for i in range(env.model.ncam))
                assert not images['right'].any()
                np.testing.assert_array_equal(env.state()['right'], PARK)
                if task == 'apple':
                    Image.fromarray(np.concatenate([images['top'], images['left'], images['right']], axis=1)).save(out / f'{config_name(c)}_cameras.jpg')
            fake = {k: np.repeat(env.state()[s][None], 48, axis=0) for k, s in [('q_l_chunk', 'left'), ('q_r_chunk', 'right')]}
            l, r, _ = prepared_rows(env, fake, rows)
            before = env.data.time
            for ll, rr in zip(l, r): env.step(ll, rr)
            assert abs(env.data.time - before - rows * .05) < 1e-8
            results.append(dict(config=config_name(c), task=task, rows_executed=len(l), single_absent=arms == 'single', black_right=not images['right'].any()))
        finally: env.close()
    # Four independent policy variants exercise FK and the documented client clamp. Never get_action.
    clients = []
    for tcp, region in itertools.product(('pad', 'fingertip'), ('off', 'table')):
        c = dict(DEFAULT, tcp=tcp, region=region)
        env, windows, steps, images, _ = initialize('box', c, 123)
        client = dream.DreamClient(api_key=y.settings().get('DREAM_API_KEY', ''))
        try:
            client.policy(policy(c, env)); ik = client._policy._ik
            ik.traj_ready.wait(60)
            y.feed(y.CustomGripFeed(client), env, windows, 'dry check', y.BETA_MODEL, steps, every=0)
            prepared = ik.observe(client._window.build_observation())
            rng = np.random.default_rng(5)
            for j in range(5):
                for s in env.sides:
                    limits = np.array([env.model.joint(f'{s}_joint{i}').range for i in range(1, 7)])
                    q = rng.uniform(limits[:, 0] * .7, limits[:, 1] * .7)
                    env.data.qpos[env.qadr[s]] = q
                    mujoco.mj_forward(env.model, env.data)
                    base = env.data.body(s + '_base_link'); site = env.data.site(s + '_tcp')
                    fk = ik._solver.fk(ik.wires[s].spec(q[None]), s)[0]
                    np.testing.assert_allclose(fk[:3, 3], base.xmat.reshape(3, 3).T @ (site.xpos - base.xpos), atol=2e-6)
                    np.testing.assert_allclose(fk[:3, :3], base.xmat.reshape(3, 3).T @ site.xmat.reshape(3, 3), atol=2e-6)
            targets = np.repeat(np.eye(4)[None, None], 2, axis=0)
            targets[:, :, :3, 3] = [2., -2., -2.]
            clipped = ik._solver.clip_to_region(targets, ik._solver.collision_margin)
            if region == 'table':
                i = ik._solver.sides.index('left'); box = policy(c, env)['ik']['region']['left']
                assert np.all(clipped[i, :, :3, 3] >= np.array(box['lo']))
                assert np.all(clipped[i, :, :3, 3] <= np.array(box['hi']))
                assert not np.array_equal(clipped[i], targets[i])
            else: np.testing.assert_array_equal(clipped, targets)
            clients.append(dict(tcp=tcp, region=region, fk_samples=10, clamp_active=region == 'table', input_conversion_passed=True, policy=policy(c, env)))
        finally: client.close(); env.close()
    # Durable reservations survive a new object, free only explicit warming, and stop at cap.
    ledger = Ledger(out / 'budget_fixture', cap=3)
    if ledger.path.exists(): ledger.path.unlink()
    a = ledger.reserve('episode', 3)
    ledger = Ledger(out / 'budget_fixture', cap=3)
    assert ledger.episode_calls('episode') == 1
    b = ledger.reserve('episode', 3); ledger.finish(b['id'], 'pool_warming')
    c = ledger.reserve('episode', 3); ledger.finish(c['id'], 'billed')
    ledger.reserve('episode', 3)
    try: ledger.reserve('another', 3)
    except BudgetEnd: pass
    else: raise AssertionError('Global cap failed')
    assert ledger.read()['budget_used'] == 3
    # Exact deterministic reset/replay for saved joint chunks.
    c = dict(DEFAULT)
    e1, _, _, _, j1 = initialize('box', c, 777)
    commands = []
    for _ in range(10):
        s = e1.state(); commands.append((s['left'].copy(), s['right'].copy())); e1.step(*commands[-1])
    e2, _, _, _, j2 = initialize('box', c, 777)
    for cmd in commands: e2.step(*cmd)
    np.testing.assert_array_equal(e1.data.qpos, e2.data.qpos)
    np.testing.assert_array_equal(j1, j2)
    e1.close(); e2.close()
    # Gripper close-up with old and new CAD points as spheres, rendered offline.
    env = SweepEnv('apple', dict(DEFAULT))
    try:
        env.data.qpos[:] = 0
        mujoco.mj_forward(env.model, env.data)
        flange = env.data.body('left_wrist_roll_link')
        old = json.loads((HERE / 'viscous_arm.json').read_text())['tcp']
        env.model.vis.global_.offwidth = 900
        env.model.vis.global_.offheight = 600
        renderer = mujoco.Renderer(env.model, height=600, width=900)
        cam = mujoco.MjvCamera()
        world_new = flange.xpos + flange.xmat.reshape(3, 3) @ np.array(spec['tcp'])
        world_old = flange.xpos + flange.xmat.reshape(3, 3) @ np.array(old)
        cam.lookat[:] = (world_new + world_old) / 2; cam.distance = .26; cam.azimuth = 130; cam.elevation = -20
        renderer.update_scene(env.data, camera=cam)
        for point, color in [(world_old, [1., .2, .05, 1.]), (world_new, [.05, 1., .3, 1.])]:
            geom = renderer.scene.geoms[renderer.scene.ngeom]; renderer.scene.ngeom += 1
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([.0035] * 3), point, np.eye(3).ravel(), np.array(color, np.float32))
        img = Image.fromarray(renderer.render()); draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, 900, 48), fill='white')
        draw.text((12, 8), 'RED: old finger-pad midpoint | GREEN: closed fingertip midpoint (+3.43 cm forward)', fill='black')
        draw.text((12, 26), 'Closed gripper CAD; axes unchanged. Orange/green markers are diagnostic only.', fill='black')
        img.save(out / 'tool_points.png'); renderer.close()
        annotate_tool_picture(out / 'tool_points.png')
    finally: env.close()
    atomic(out / 'checks.json', dict(inference_calls=0, scene_variants=results, client_checks=clients, tip_faces_flange_m=[p.tolist() for p in tips], new_tcp_flange_m=spec['tcp'], budget_crash_cap_passed=True, deterministic_replay_passed=True))
    print(f'PASS: {len(results)} scene/task variants, 4 client FK/clamp variants, durable budget and deterministic replay; zero inference calls', flush=True)


def annotate_tool_picture(path):
    """Add a CAD side projection so the pad point remains visible through the solid jaw."""
    from scipy.spatial import ConvexHull
    from sweep import DEFAULT
    env = SweepEnv('apple', DEFAULT)
    try:
        env.data.qpos[:] = 0; mujoco.mj_forward(env.model, env.data)
        geom = env.model.geom('left_visual_043'); mid = geom.dataid[0]
        v = env.model.mesh_vert[env.model.mesh_vertadr[mid]:env.model.mesh_vertadr[mid]+env.model.mesh_vertnum[mid]]
        flange = env.data.body('left_wrist_roll_link')
        v = ((v @ env.data.geom('left_visual_043').xmat.reshape(3, 3).T + env.data.geom('left_visual_043').xpos) - flange.xpos) @ flange.xmat.reshape(3, 3)
        poly = v[:, [0, 2]][ConvexHull(v[:, [0, 2]]).vertices]
        image = Image.open(path).convert('RGB'); draw = ImageDraw.Draw(image)
        draw.rectangle((460, 360, 898, 598), fill='white', outline='black', width=2)
        draw.text((474, 372), 'CAD side profile (points visible through the jaw)', fill='black')
        def pixel(point): return (480 + (point[0] - .075) * 3900, 562 - (point[1] + .04) * 2700)
        draw.polygon([pixel(p) for p in poly], fill='#344048', outline='black')
        for key, color, label, dy in [('viscous_arm.json', '#ff3300', 'old pad midpoint', -35), ('viscous_arm_fingertip.json', '#00bb44', 'new fingertip midpoint', 14)]:
            spec = json.loads((HERE / key).read_text()); x, yy = pixel(np.array(spec['tcp'])[[0, 2]])
            draw.ellipse((x-6, yy-6, x+6, yy+6), fill=color, outline='white', width=1)
            draw.text((x-86, yy+dy), label, fill=color)
        draw.text((474, 578), 'Same flange frame and axes; 3.433 cm shift along approach.', fill='black')
        image.save(path)
    finally: env.close()
