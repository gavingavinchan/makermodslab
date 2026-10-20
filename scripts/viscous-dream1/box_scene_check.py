"""Offline box geometry/target checks, camera preview, and a scripted YAM pinch."""
import yam_baseline  # Establish workspace cache/temp paths before other imports.
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from cross_scenes import BoxYamEnv, CubeYamEnv, success


def main():
    out = Path("runs/yam_box_pinch_dry")
    out.mkdir(parents=True, exist_ok=True)
    env, cube = BoxYamEnv(), CubeYamEnv()
    report = {"inference_calls": 0}
    geom = env.model.geom("box_geom")
    np.testing.assert_allclose(geom.size, [.025, .025, .015])
    np.testing.assert_allclose(env.model.body("box").mass, [.04])
    np.testing.assert_allclose(geom.friction, [1.6, .01, .001])
    for axis in range(2):
        for sign in (-1, 1):
            name = f"square_{axis}_{sign}"
            np.testing.assert_array_equal(env.model.geom(name).pos, cube.model.geom(name).pos)
            np.testing.assert_array_equal(env.model.geom(name).size, cube.model.geom(name).size)
    for _ in range(40):
        s = env.state()
        env.step(s["left"], s["right"])
    Image.fromarray(env.images()["top"]).save("real/topdown_box_sim_20261020.png")
    Image.fromarray(env.images()["left"]).save(out / "wrist_start.png")
    report.update(dimensions_m=(2 * geom.size).tolist(), mass_kg=float(env.model.body("box").mass[0]),
                  friction=geom.friction.tolist(), opening_m=.095,
                  settled_box_xyz=env.data.body("box").xpos.tolist())
    # Exercise acceptance of an overhanging box, and rejection outside the outline.
    probe = env.data.joint("box_free").qpos.copy()
    env.data.joint("box_free").qpos[:3] = [0, -.04, .775]
    mujoco.mj_forward(env.model, env.data)
    assert success(env, "box")
    env.data.joint("box_free").qpos[0] = .023
    mujoco.mj_forward(env.model, env.data)
    assert not success(env, "box")
    env.data.joint("box_free").qpos[:] = probe
    mujoco.mj_forward(env.model, env.data)
    report["geometry_and_success_checks"] = "passed"
    qadr = np.array([env.model.joint(f"left_joint{i}").qposadr[0] for i in range(1, 7)])
    limits = np.array([env.model.joint(f"left_joint{i}").range for i in range(1, 7)])
    data = mujoco.MjData(env.model)
    data.qpos[:] = env.data.qpos
    site = env.model.site("left_tcp").id
    goal_r = np.diag([1., -1., -1.])
    rng = np.random.default_rng(7)

    def solve(xyz, seed):
        def residual(q):
            data.qpos[qadr] = q
            # Trial IK postures may interpenetrate many collision meshes; only
            # kinematics is needed here, so don't allocate/contact-solve them.
            mujoco.mj_kinematics(env.model, data)
            rot = data.site_xmat[site].reshape(3, 3)
            return np.r_[10 * (data.site_xpos[site] - xyz), Rotation.from_matrix(goal_r.T @ rot).as_rotvec()]
        attempts = [seed, *rng.uniform(limits[:, 0], limits[:, 1], (6, 6))]
        fits = [least_squares(residual, np.clip(q, limits[:, 0], limits[:, 1]),
                              bounds=(limits[:, 0], limits[:, 1]), max_nfev=150) for q in attempts]
        fit = min(fits, key=lambda f: np.linalg.norm(f.fun))
        if np.linalg.norm(fit.fun) > .03:
            raise RuntimeError(f"Pinch IK residual too large: {fit.fun}")
        return fit.x

    phases, pictures = [], []
    right = np.r_[np.zeros(6), 1.]

    def move(label, xyz, grip, rows=50):
        start = env.state()["left"]
        goal = np.r_[solve(np.asarray(xyz), start[:6]), grip]
        for i in range(rows):
            t = (i + 1) / rows
            env.step(start + t * t * (3 - 2 * t) * (goal - start), right)
        for _ in range(20):
            env.step(goal, right)
        phases.append({"phase": label, **yam_baseline.objects(env, "box")})
        rgb = env.images()["top"]
        pictures.append(rgb)
        Image.fromarray(rgb).save(out / f"{label}.png")

    try:
        p = env.data.body("box").xpos.copy()
        move("approach", p + [0, 0, .09], 1.)
        move("descend", p + [0, 0, .003], 1.)
        move("pinch", p + [0, 0, .003], 0., rows=30)
        move("lift", p + [0, 0, .103], 0.)
        report["lifted_box"] = bool(env.data.body("box").xpos[2] > p[2] + .04)
        report["box_rise_m"] = float(env.data.body("box").xpos[2] - p[2])
    except RuntimeError as error:
        report["pinch_skipped_or_failed"] = str(error)
    finally:
        report["phases"] = phases
        (out / "validation.json").write_text(json.dumps(report, indent=2))
        if pictures:
            Image.fromarray(np.concatenate(pictures, axis=1)).save(out / "pinch_sequence.png")
        print(json.dumps(report, indent=2))
        env.close()
        cube.close()


if __name__ == "__main__":
    main()
