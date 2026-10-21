"""Offline FK, station/camera checks and physical pinch for built-in Dream arms."""
import yam_baseline  # Set workspace-only cache, log and temporary paths first.
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from dream_ik.solver import ChunkIK
from supported_arms import SupportedArmEnv, camera_mount, wrist_mount
from cross_scenes import BoxYamEnv
from dream_examples.yam_mujoco import yam_env


def check(arm, task, out=None):
    out = Path(out) if out else Path(f"runs/{arm}_{task}_dry")
    out.mkdir(parents=True, exist_ok=True)
    env = SupportedArmEnv(arm, task)
    ref = BoxYamEnv() if task == "box" else yam_env.YamEnv()
    report = {"arm": arm, "task": task, "inference_calls": 0,
              "initial_state": {s: q.tolist() for s, q in env.state().items()},
              "joint_zero_offsets_rad": [0] * 6,
              "vendor_joint_signs": [1, 1, 1, 1, -1, 1] if arm == "makers" else [1] * 6,
              "metal_yam_sign_map": [1, -1, 1, 1, 1, 1] if arm == "metal" else None,
              "finger_open_joint_m": env.open.tolist(),
              "yam_wrist_camera_tcp_pos_m": camera_mount()[0].tolist(),
              "wrist_camera_tcp_pos_m": wrist_mount(arm)[0].tolist(),
              "wrist_camera_tcp_rotation": wrist_mount(arm)[1].tolist()}
    # The Metal joint_sign maps YAM joint policies, not Metal EEF IK commands.
    # EEF commands remain vendor joint radians on Metal; never negate twice.
    for name in ((task, "place_square") if task == "box" else ("apple", "orange", "banana", "water_bottle", "bowl", "plate")):
        np.testing.assert_allclose(env.data.body(name).xpos, ref.data.body(name).xpos, atol=1e-12)
    for attr in ("cam_pos", "cam_quat", "cam_fovy", "cam_intrinsic", "cam_sensorsize", "cam_resolution"):
        np.testing.assert_allclose(getattr(env.model, attr)[env.model.camera("top").id],
                                   getattr(ref.model, attr)[ref.model.camera("top").id], atol=1e-12)
    report["objects_and_top_camera_match"] = True
    rng = np.random.default_rng(1729)
    limits = np.array([env.model.joint(f"left_joint{i}").range for i in range(1, 7)])
    qs = np.vstack([env.spec.rest_pose, np.zeros(6), rng.uniform(limits[:, 0] + .04, limits[:, 1] - .04, (100, 6))])
    solver = ChunkIK(arm, "tcp", 20)
    fk = solver.fk(qs, "left")
    probe = mujoco.MjData(env.model)
    probe.qpos[:] = env.data.qpos
    errors = []
    for side in ("left", "right"):
        base_r = env.data.body(f"{side}_base").xmat.reshape(3, 3)
        base_p = env.data.body(f"{side}_base").xpos
        for i, q in enumerate(qs):
            probe.qpos[env.qadr[side][:6]] = q
            mujoco.mj_kinematics(env.model, probe)
            site = probe.site(f"{side}_tcp")
            xyz = base_r.T @ (site.xpos - base_p)
            rot = base_r.T @ site.xmat.reshape(3, 3)
            errors.append([np.linalg.norm(xyz - fk[i, :3, 3]),
                           Rotation.from_matrix(rot.T @ fk[i, :3, :3]).magnitude() * 180 / np.pi])
    report["fk_match"] = {"vectors_per_side": len(qs), "max_position_m": float(np.max(errors, axis=0)[0]),
                          "max_rotation_deg": float(np.max(errors, axis=0)[1])}
    assert report["fk_match"]["max_position_m"] < .001
    assert report["fk_match"]["max_rotation_deg"] < .5
    for _ in range(40):
        hold = {s: np.r_[env.spec.rest_pose, 1] for s in ("left", "right")}
        env.step(hold["left"], hold["right"])
        ref.step(*ref.state().values())
    report["settled_state"] = {s: q.tolist() for s, q in env.state().items()}
    current, baseline = env.images(), ref.images()
    canvas = Image.new("RGB", (1280, 3 * 390), "white")
    draw = ImageDraw.Draw(canvas)
    for i, role in enumerate(("top", "left", "right")):
        for x, label, images in ((0, "YAM", baseline), (640, arm, current)):
            draw.text((x + 12, i * 390 + 8), f"{label}: {role}", fill="black")
            canvas.paste(Image.fromarray(images[role]), (x, i * 390 + 30))
            Image.fromarray(images[role]).save(out / f"{label}_{role}.png")
    canvas.save(out / "camera_comparison.png")
    (out / "station.xml").write_text(__import__("supported_arms").build_xml(arm, task))
    # Offline kinematic solve only; simulated position servos execute every row.
    probe.qpos[:] = env.data.qpos
    site_id = env.model.site("left_tcp").id
    # Maker's wrist pitch cannot hold a vertical approach high over this nearby
    # object. Use a 35-degree forward tilt, tangent jaws, for the pinch check.
    radial = env.data.body(task).xpos[:2] - env.data.body("left_base").xpos[:2]
    radial /= np.linalg.norm(radial)
    z = np.r_[np.sin(np.radians(35)) * radial, -np.cos(np.radians(35))]
    x = np.r_[-radial[1], radial[0], 0.]
    goal_r = np.column_stack([x, np.cross(z, x), z])

    def solve(xyz, seed):
        nonlocal goal_r
        rotations = [goal_r] if goal_r is not None else [Rotation.from_euler("z", a).as_matrix() @ np.diag([1., -1., -1.])
                                                       for a in np.linspace(-np.pi, np.pi, 13)[:-1]]
        fits = []
        for desired_r in rotations:
            def residual(q):
                probe.qpos[env.qadr["left"][:6]] = q
                mujoco.mj_kinematics(env.model, probe)
                rot = probe.site_xmat[site_id].reshape(3, 3)
                return np.r_[10 * (probe.site_xpos[site_id] - xyz), Rotation.from_matrix(desired_r.T @ rot).as_rotvec()]
            for q in [seed, *rng.uniform(limits[:, 0], limits[:, 1], (4, 6))]:
                fit = least_squares(residual, np.clip(q, limits[:, 0], limits[:, 1]),
                                    bounds=(limits[:, 0], limits[:, 1]), max_nfev=180)
                fits.append((fit, desired_r))
        fit, desired_r = min(fits, key=lambda f: np.linalg.norm(f[0].fun))
        if np.linalg.norm(fit.fun) > .03:
            raise RuntimeError(f"Pinch IK residual too large: {fit.fun}; q={fit.x}")
        goal_r = desired_r
        return fit.x

    phases, pictures = [], []
    right = np.r_[env.spec.rest_pose, 1.]

    def move(label, xyz, grip, rows=60):
        start = env.state()["left"]
        goal = np.r_[solve(xyz, start[:6]), grip]
        for i in range(rows):
            t = (i + 1) / rows
            env.step(start + t * t * (3 - 2 * t) * (goal - start), right)
        for _ in range(20):
            env.step(goal, right)
        phases.append({"phase": label, "goal": goal.tolist(), **yam_baseline.objects(env, task),
                       "measured_joints": env.state()["left"].tolist()})
        image = env.images()["top"]
        pictures.append(image)
        Image.fromarray(image).save(out / f"{label}.png")

    try:
        p = env.data.body(task).xpos.copy()
        move("approach", p - .06 * goal_r[:, 2], 1.)
        # The 3 cm box is pinched near its upper sidewall, keeping the tilted
        # CAD fingertips clear of the table rather than forcing the palm down.
        pinch_z = .022 if task == "box" else .003
        move("descend", p + [0, 0, pinch_z], 1.)
        move("pinch", p + [0, 0, pinch_z], 0., rows=30)
        move("lift", p + [0, 0, pinch_z + .10], 0.)
        rise = float(env.data.body(task).xpos[2] - p[2])
        report.update(object_rise_m=rise, lifted=rise > .04)
    except RuntimeError as error:
        report["pinch_error"] = str(error)
    finally:
        report["phases"] = phases
        (out / "validation.json").write_text(json.dumps(report, indent=2))
        if pictures:
            Image.fromarray(np.concatenate(pictures, axis=1)).save(out / "pinch_sequence.png")
        print(json.dumps({k: v for k, v in report.items() if k != "phases"}, indent=2), flush=True)
        env.close()
        ref.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--arm", choices=("makers", "metal"), default="makers")
    p.add_argument("--task", choices=("apple", "box"), default="apple")
    p.add_argument("--out", type=Path, help="Preserve earlier dry artifacts by selecting a new directory")
    args = p.parse_args()
    check(args.arm, args.task, args.out)
