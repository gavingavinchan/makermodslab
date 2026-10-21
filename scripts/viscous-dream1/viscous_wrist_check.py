"""Offline Maker-view fixture, legacy reproduction, and Viscous pinch checks."""
import yam_baseline
import argparse
import base64
import io
import json
from collections import deque
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from cross_scenes import ViscousAppleEnv, OPEN
from supported_arms import SupportedArmEnv, wrist_mount

OUT = Path('runs/viscous_wrist_match')


def solve(env, xyz, rotation, seed, tolerance=.03):
    limits = np.array([env.model.joint(f'left_joint{i}').range for i in range(1, 7)])
    probe = mujoco.MjData(env.model)
    probe.qpos[:] = env.data.qpos
    def residual(q):
        probe.qpos[env.qadr['left'][:6]] = q
        mujoco.mj_kinematics(env.model, probe)
        tcp = probe.site('left_tcp')
        return np.r_[10 * (tcp.xpos - xyz), Rotation.from_matrix(rotation.T @ tcp.xmat.reshape(3, 3)).as_rotvec()]
    rng = np.random.default_rng(491)
    fits = [least_squares(residual, np.clip(q, limits[:, 0], limits[:, 1]),
                          bounds=(limits[:, 0], limits[:, 1]), max_nfev=250)
            for q in [seed, np.array([.7, -1.2, 1.5, -1.5, 0., 0.]), *rng.uniform(limits[:, 0] + .01, limits[:, 1] - .01, (8, 6))]]
    fit = min(fits, key=lambda f: np.linalg.norm(f.fun))
    assert np.linalg.norm(fit.fun) < tolerance, fit.fun
    return fit.x


def comparison():
    OUT.mkdir(parents=True, exist_ok=True)
    env, maker, old = ViscousAppleEnv(), SupportedArmEnv(), ViscousAppleEnv(wrist='legacy')
    report = {'inference_calls': 0, 'render_size': [640, 360], 'model_size': [320, 176],
              'tcp_camera_position_m': wrist_mount()[0].tolist(), 'tcp_camera_rotation': wrist_mount()[1].tolist()}
    try:
        # Verify legacy XML and image output against a snapshot of the original builder.
        import importlib.util
        spec = importlib.util.spec_from_file_location('old_cross_scenes', OUT / 'source_before/cross_scenes.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        ref_model = mujoco.MjModel.from_xml_string(module.viscous_xml())
        np.testing.assert_array_equal(old.model.cam_pos, ref_model.cam_pos)
        np.testing.assert_array_equal(old.model.cam_quat, ref_model.cam_quat)
        np.testing.assert_array_equal(old.model.cam_fovy, ref_model.cam_fovy)
        reference = module.ViscousAppleEnv()
        try:
            for k, v in old.images().items():
                np.testing.assert_allclose(v.astype(float), reference.images()[k].astype(float), atol=1, rtol=0)
        finally:
            reference.close()
        report['legacy_image_max_pixel_tolerance'] = 1
        windows = {k: deque(maxlen=9) for k in ('top', 'left', 'right')}
        images = yam_baseline.capture(env, windows)
        for k, rgb in images.items():
            assert rgb.shape == (360, 640, 3)
            buffer = io.BytesIO()
            Image.fromarray(rgb).resize((320, 176), Image.Resampling.BILINEAR).save(buffer, format='JPEG', quality=85)
            assert base64.b64encode(buffer.getvalue()).decode('ascii') == windows[k][-1]
        report['capture_exact_match'] = True
        for side in ('left', 'right'):
            c = env.model.camera(side)
            report[side + '_mount'] = {'link': side + '_tool0', 'position_m': c.pos.tolist(),
                'quaternion_wxyz': c.quat.tolist(), 'vfov_deg': float(c.fovy[0])}
        np.testing.assert_array_equal(env.model.camera('left').pos, env.model.camera('right').pos)
        canvas = Image.new('RGB', (1920, 1536), 'white')
        draw = ImageDraw.Draw(canvas)
        fixtures = []
        for i, (real, tilt, grip) in enumerate([('makerwrist_andre_0.jpg', 25., .4),
                                               ('makerwrist_andre_1.jpg', 5., 0.),
                                               ('makerwrist_maker_1.jpg', 15., .12)]):
            angle = np.radians(tilt)
            z, x = np.array([0., -np.cos(angle), -np.sin(angle)]), np.array([1., 0., 0.])
            r = np.column_stack([x, np.cross(z, x), z])
            fixture = {'tilt_deg': tilt, 'grip': grip, 'reference': real}
            for label, e in [('Viscous', env), ('Maker', maker)]:
                e.reset()
                q = solve(e, np.array([.22, -.15, .83]), r, e.state()['left'][:6], tolerance=.06)
                e.data.qpos[e.qadr['left'][:6]] = q
                if label == 'Viscous':
                    for sign in (-1, 1):
                        e.data.joint(f'left_finger_{sign}').qpos[0] = grip * OPEN
                else:
                    e.data.qpos[e.qadr['left'][6:]] = grip * e.open
                mujoco.mj_forward(e.model, e.data)
                obj = e.data.site('left_tcp').xpos + .15 * r[:, 2]
                obj[2] = .779333
                e.data.joint('apple_free').qpos[:3] = obj
                e.data.joint('bottle_free').qpos[0] = .55
                mujoco.mj_forward(e.model, e.data)
                rgb = e.images()['left']
                Image.fromarray(rgb).save(OUT / f'{label.lower()}_{i}_native.png')
                col = 0 if label == 'Viscous' else 1
                canvas.paste(Image.fromarray(rgb[:, 80:560]).resize((640, 480)), (640 * col, i * 512 + 32))
                draw.text((640 * col + 10, i * 512 + 9), f'{label}: hand {tilt:g} deg down, grip {grip:g}', fill='black')
                tcp = e.data.site('left_tcp')
                fixture[label] = {'joints': q.tolist(), 'tcp': tcp.xpos.tolist(),
                                  'tcp_rotation_world': tcp.xmat.reshape(3, 3).tolist(),
                                  'actual_hand_tilt_deg': float(np.degrees(np.arcsin(-tcp.xmat.reshape(3, 3)[2, 2])))}
            canvas.paste(Image.open(Path('real') / real).convert('RGB').resize((640, 480)), (1280, i * 512 + 32))
            draw.text((1290, i * 512 + 9), 'Real ' + real, fill='black')
            fixtures.append(fixture)
        canvas.save(OUT / 'compare.png')
        report['fixtures'] = fixtures
        report['comparison_crop'] = 'Central 4:3 horizontal crop for dataset comparison only; policy gets native 16:9'
        (OUT / 'validation.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != 'fixtures'}, indent=2))
    finally:
        env.close(); maker.close(); old.close()


def pinch(task):
    out = OUT / (task + '_pinch')
    out.mkdir(parents=True, exist_ok=True)
    env = ViscousAppleEnv(task)
    pictures, phases = [], []
    right = np.r_[np.zeros(6), 1.]
    r = np.diag([1., -1., -1.])
    for _ in range(33):
        env.step(*env.state().values())
    p = env.data.body(task).xpos.copy()
    def move(label, xyz, grip, rows=60):
        start = env.state()['left']
        target = np.r_[solve(env, xyz, r, start[:6]), grip]
        for i in range(rows):
            t = (i + 1) / rows
            env.step(start + t * t * (3 - 2 * t) * (target - start), right)
        for _ in range(20):
            env.step(target, right)
        phases.append({'phase': label, **yam_baseline.objects(env, task)})
        pictures.append(env.images()['top'])
    try:
        move('approach', p + [0, 0, .08], 1.)
        pinch_offset = .020 if task == 'box' else .003
        move('descend', p + [0, 0, pinch_offset], 1.)
        move('pinch', p + [0, 0, pinch_offset], 0., 30)
        move('lift', p + [0, 0, .083], 0.)
        rise = float(env.data.body(task).xpos[2] - p[2])
        report = {'inference_calls': 0, 'task': task, 'rise_m': rise, 'lifted': rise > .04, 'phases': phases}
        (out / 'validation.json').write_text(json.dumps(report, indent=2))
        Image.fromarray(np.concatenate(pictures, axis=1)).save(out / 'pinch_sequence.png')
        print(json.dumps(report, indent=2))
        assert report['lifted'], report
    finally:
        env.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--pinch', choices=('apple', 'box'))
    args = p.parse_args()
    pinch(args.pinch) if args.pinch else comparison()
