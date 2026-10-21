"""Offline CAD-based KD wrist comparison and capture-path checks; no client."""
import yam_baseline  # Workspace-only temp/cache/log paths, before Dream imports.
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

from supported_arms import SupportedArmEnv, wrist_mount


def main():
    out = Path("runs/maker_wrist_match")
    out.mkdir(parents=True, exist_ok=True)
    env = SupportedArmEnv()
    report = {"inference_calls": 0, "cad_bracket": "part_054_solid_054.stl",
              "cad_camera_lens": "part_056_solid_056.stl",
              "comparison_crop": "central horizontal 4:3 crop for real-frame comparison only",
              "render_resolution": [640, 360], "model_resolution": [320, 176],
              "vfov_deg": 77.2,
              "hfov_deg": float(np.degrees(2 * np.arctan(16 / 9 * np.tan(np.radians(77.2 / 2))))),
              "tcp_camera_position_m": wrist_mount()[0].tolist(),
              "tcp_camera_rotation": wrist_mount()[1].tolist()}
    for side in ("left", "right"):
        cam = env.model.camera(side)
        report[f"{side}_mount"] = {"link": f"{side}_link_007", "position_m": cam.pos.tolist(),
                                   "quaternion_wxyz": cam.quat.tolist(), "fovy": float(cam.fovy[0])}
    # Compare several openings/poses. Only this offline fixture moves the apple;
    # live task objects and the mandated rest pose remain unchanged.
    canvas = Image.new("RGB", (1280, 3 * 512), "white")
    draw = ImageDraw.Draw(canvas)
    limits = np.array([env.model.joint(f"left_joint{i}").range for i in range(1, 7)])
    rng = np.random.default_rng(491)
    fixtures = []
    for i, (reference, tilt, grip) in enumerate([
        ("makerwrist_andre_0.jpg", 25., .40),
        ("makerwrist_andre_1.jpg", 5., 0.),
        ("makerwrist_maker_1.jpg", 15., .12),
    ]):
        env.reset()
        angle = np.radians(tilt)
        z = np.array([0., -np.cos(angle), -np.sin(angle)])
        x = np.array([1., 0., 0.])
        goal_r = np.column_stack([x, np.cross(z, x), z])
        goal_p = np.array([.22, -.15, .83])

        def residual(q):
            env.data.qpos[env.qadr["left"][:6]] = q
            mujoco.mj_kinematics(env.model, env.data)
            tcp = env.data.site("left_tcp")
            return np.r_[10 * (tcp.xpos - goal_p),
                         Rotation.from_matrix(goal_r.T @ tcp.xmat.reshape(3, 3)).as_rotvec()]

        fits = [least_squares(residual, seed, bounds=(limits[:, 0], limits[:, 1]), max_nfev=250)
                for seed in [np.array(env.spec.rest_pose),
                             *rng.uniform(limits[:, 0] + .01, limits[:, 1] - .01, (6, 6))]]
        fit = min(fits, key=lambda f: np.linalg.norm(f.fun))
        assert np.linalg.norm(fit.fun) < .01, fit.fun
        env.data.qpos[env.qadr["left"]] = np.r_[fit.x, grip * env.open]
        mujoco.mj_forward(env.model, env.data)
        tcp = env.data.site("left_tcp")
        object_p = tcp.xpos + .15 * goal_r[:, 2]
        object_p[2] = .779333  # Apple centre on the existing table.
        adr = env.model.jnt_qposadr[env.model.body("apple").jntadr[0]]
        env.data.qpos[adr:adr + 3] = object_p
        # Clear the bottle from the comparison camera's physical location.
        # This is a visual fixture only; live scenes retain every spawn.
        bottle_adr = env.model.jnt_qposadr[env.model.body("water_bottle").jntadr[0]]
        env.data.qpos[bottle_adr] = .55
        mujoco.mj_forward(env.model, env.data)
        images = env.images()
        Image.fromarray(images["left"]).save(out / f"match_{i}_native.png")
        Image.fromarray(images["top"]).save(out / f"match_{i}_top.png")
        real = Image.open(Path("real") / reference).convert("RGB")
        # A dataset 4:3 crop retains vertical extent, losing the stream's sides.
        sim = Image.fromarray(images["left"][:, 80:560]).resize((640, 480), Image.Resampling.BILINEAR)
        y = i * 512
        canvas.paste(real, (0, y + 32))
        canvas.paste(sim, (640, y + 32))
        draw.text((10, y + 9), f"Real {reference}", fill="black")
        draw.text((650, y + 9), f"Sim CAD lens: hand {tilt:g} deg down, grip {grip:g}, apple ~15 cm ahead", fill="black")
        fixtures.append({"reference": reference, "hand_tilt_down_deg": tilt, "grip": grip,
                         "joints": fit.x.tolist(), "tcp_world": tcp.xpos.tolist(),
                         "object_world": object_p.tolist(),
                         "tcp_object_distance_m": float(np.linalg.norm(object_p - tcp.xpos))})
    canvas.save(out / "compare.png")
    report["fixtures"] = fixtures
    # Both mounts must be identical in their link frame. Image up must agree
    # with +TCP y, and optical forward must agree with the finger approach.
    np.testing.assert_allclose(env.model.camera("left").pos, env.model.camera("right").pos)
    np.testing.assert_allclose(env.model.camera("left").quat, env.model.camera("right").quat)
    p, r = wrist_mount()
    assert r[1, 1] > .9 and -r[2, 2] > .9 and -r[1, 2] < 0
    env.reset()
    windows = {name: deque(maxlen=9) for name in ("top", "left", "right")}
    images = yam_baseline.capture(env, windows)
    for name, rgb in images.items():
        assert rgb.shape == (360, 640, 3)
        Image.fromarray(rgb).save(out / f"initial_{name}.png")
        decoded = Image.open(io.BytesIO(base64.b64decode(windows[name][-1])))
        assert decoded.size == (320, 176)
        decoded.save(out / f"model_{name}.png")
        # Match Dream's JPEG bytes, including its filter and quality.
        buffer = io.BytesIO()
        Image.fromarray(rgb).resize((320, 176), Image.Resampling.BILINEAR).save(buffer, format="JPEG", quality=85)
        assert base64.b64encode(buffer.getvalue()).decode("ascii") == windows[name][-1]
    report["capture_exact_match"] = True
    (out / "validation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    env.close()


if __name__ == "__main__":
    main()
