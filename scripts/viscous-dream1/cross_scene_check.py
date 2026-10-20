"""Offline scene invariants and a scripted Viscous apple pinch; no Dream requests."""

# Import the runner first to establish workspace-only log/cache/temp paths.
import yam_baseline  # noqa: F401
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from cross_scenes import ViscousAppleEnv, OPEN
from dream_examples.yam_mujoco import yam_env


def main():
    out = Path("runs/viscous_apple_pinch_dry")
    out.mkdir(parents=True, exist_ok=True)
    env, ref = ViscousAppleEnv(), yam_env.YamEnv()
    report = {"inference_calls": 0}
    for name in ("apple", "orange", "banana", "water_bottle", "bowl", "plate"):
        np.testing.assert_allclose(env.data.body(name).xpos, ref.data.body(name).xpos, atol=1e-12)
    for attr in ("cam_pos", "cam_quat", "cam_intrinsic", "cam_sensorsize", "cam_resolution"):
        np.testing.assert_allclose(getattr(env.model, attr)[env.model.camera("top").id],
                                   getattr(ref.model, attr)[ref.model.camera("top").id], atol=1e-12)
    report["objects_and_top_camera_match"] = True
    report["initial_state"] = {k: v.tolist() for k, v in env.state().items()}
    vertices = np.array([[float(x) for x in line.split()[1:4]] for line in (
        yam_env.ASSETS / "fruits/apple/visual.obj").read_text().splitlines() if line.startswith("v ")])
    report["apple_mesh_dimensions_m"] = np.ptp(vertices, axis=0).tolist()
    report["finger_travel_m"] = 2 * OPEN
    probe = mujoco.MjData(env.model)
    probe.qpos[:] = env.data.qpos
    site = env.model.site("left_tcp").id
    limits = np.array([env.model.joint(f"left_joint{i}").range for i in range(1, 7)])
    goal_r = np.diag([1., -1., -1.])
    rng = np.random.default_rng(1)

    def solve(xyz, seed):
        def residual(q):
            probe.qpos[env.qadr["left"]] = q
            mujoco.mj_forward(env.model, probe)
            rot = probe.site_xmat[site].reshape(3, 3)
            return np.r_[10 * (probe.site_xpos[site] - xyz), Rotation.from_matrix(goal_r.T @ rot).as_rotvec()]
        attempts = [seed, np.array([.7, -1.2, 1.5, -1.5, 0, 0]),
                    *rng.uniform(limits[:, 0], limits[:, 1], (6, 6))]
        fits = [least_squares(residual, np.clip(q, limits[:, 0], limits[:, 1]),
                              bounds=(limits[:, 0], limits[:, 1]), max_nfev=150) for q in attempts]
        fit = min(fits, key=lambda f: np.linalg.norm(f.fun))
        if np.linalg.norm(fit.fun) > .03:
            raise RuntimeError(f"Pinch IK residual too large: {fit.fun}")
        return fit.x

    log, pictures = [], []
    right = np.r_[np.zeros(6), 1.]

    def move(label, xyz, grip, rows=60):
        start = env.state()["left"]
        goal = np.r_[solve(np.asarray(xyz), start[:6]), grip]
        for i in range(rows):
            t = (i + 1) / rows
            blend = t * t * (3 - 2 * t)
            env.step(start + blend * (goal - start), right)
        for _ in range(20):
            env.step(goal, right)
        contacts = []
        for c in env.data.contact:
            names = [env.model.body(env.model.geom_bodyid[g]).name for g in (c.geom1, c.geom2)]
            if "apple" in names:
                contacts.append(names)
        log.append({"phase": label, "apple_xyz": env.data.body("apple").xpos.tolist(),
                    "left_tcp_xyz": env.data.site("left_tcp").xpos.tolist(),
                    "gripper": float(env.state()["left"][6]), "apple_contacts": contacts})
        rgb = env.images()["top"]
        Image.fromarray(rgb).save(out / f"{label}.png")
        pictures.append(rgb)

    try:
        p = env.data.body("apple").xpos.copy()
        move("approach", p + [0, 0, .08], 1.)
        move("descend", p + [0, 0, .003], 1.)
        move("pinch", p + [0, 0, .003], 0., rows=30)
        move("lift", p + [0, 0, .083], 0.)
        report["lifted_apple"] = bool(env.data.body("apple").xpos[2] > p[2] + .04)
    except RuntimeError as error:
        report["pinch_skipped_or_failed"] = str(error)
    finally:
        report["phases"] = log
        (out / "validation.json").write_text(json.dumps(report, indent=2))
        if pictures:
            Image.fromarray(np.concatenate(pictures, axis=1)).save(out / "pinch_sequence.png")
        print(json.dumps(report, indent=2))
        env.close()
        ref.close()


if __name__ == "__main__":
    main()
