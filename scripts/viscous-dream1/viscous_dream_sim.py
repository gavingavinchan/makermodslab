"""Viscous Arm in MuJoCo, driven zero-shot by Dream-1 as the LEFT arm of a two-arm body.

--rig bench keeps the original single arm and black right wrist image.
--rig yam uses Dream's YAM station with two Viscous arms; the right arm is parked, its answers dropped.

Modes:
  --preview   render overhead, wrist and third-person PNGs plus real | sim comparisons. No dream client at all.
  (default)   dry run: the whole dream client path runs (observation build, forward kinematics, IK on the reply),
              but its network call is replaced by a local stub that answers a small synthetic hand motion.
              The model metadata lookup is also stubbed locally. No network requests leave the machine.
  --live      real get_action calls to Dream, at most --max-calls of them. Billed per successful request.

Sim time stops while a request is in flight, so latency never changes what the arm does here.
Episode recording to Dream (the client's default) is off: `_record: False`.
"""

import argparse
import base64
import io
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
SCENE = Path.home() / "Documents/viscous-arm-model/demo/pick_place_scene.xml"
ARM_FILE = HERE / "viscous_arm.json"

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
REST = np.array([0.0, -1.895, 2.094, -1.608, 0.0, 0.0])
# viscous_01 neutral: its commissioned zero, all joints 0 (the folded CAD pose), gripper closed.
NEUTRAL = np.zeros(6)
START_POSES = {"neutral": (NEUTRAL, 0.0), "work": (REST, 1.0)}
FINGER_OPEN_M = 0.0524125  # gripper_left_joint's upper limit; the right finger mirrors it
PHYS_DT = 0.001
CONTROL_DT = 0.05  # one action row, 20 Hz
RENDER_EVERY = 2  # control rows per camera tick (10 Hz; the client keeps 5 fps of them)
EXEC_ROWS = 30  # rows run per chunk before the next request (the client's default execution_horizon)
MAX_JOINT_STEP = 2.0 * CONTROL_DT  # rad per row, the URDF's 2 rad/s: anything faster is clipped and logged
FRAME_W, FRAME_H = 640, 480  # match the two real camera stills, without cropping or stretching
SQUARE_CENTER = np.array([0.19777, -0.10599, 0.0])
SQUARE_YAW = np.deg2rad(-50.0)
SQUARE_OUTER_M = 0.045
SQUARE_LINE_M = 0.004
CUBE_HALF_M = 0.01
MODELS = {"beta": "dreamlabs/dream-1-beta", "flash": "dreamlabs/dream-1-beta-flash"}

EMBODIMENT = {
    "state": {"left_arm": [0, 6], "left_gripper": [6, 7], "right_arm": [7, 13], "right_gripper": [13, 14]},
    "action": {"left_arm": [0, 6], "left_gripper": [6, 7], "right_arm": [7, 13], "right_gripper": [13, 14]},
    "video": {role: f"observation.images.{role}" for role in ("overhead", "wrist_l", "wrist_r")},
    "head_camera": "overhead",
}


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """MuJoCo `xyaxes` for a camera at `eye` looking at `target` (a camera looks along its -z, y up)."""
    eye, target, up = (np.asarray(v, float) for v in (eye, target, up))
    forward = target - eye
    forward /= np.linalg.norm(forward)
    x = np.cross(forward, up)
    x /= np.linalg.norm(x)
    y = np.cross(x, forward)
    return list(x) + list(y)


def build_model(overhead_eye, overhead_target, wrist_pos, wrist_target, overhead_fovy=60, wrist_fovy=90,
                overhead_up=(0.39047, 0.91512, -0.10043)):
    spec = mujoco.MjSpec.from_file(str(SCENE))
    spec.meshdir = str((SCENE.parent / "../model/meshes").resolve())
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 720
    # Softer light than the demo's film look: the bright headlight washes out a pale tabletop seen from above.
    spec.visual.headlight.diffuse = [0.25, 0.25, 0.25]
    spec.visual.headlight.ambient = [0.40, 0.40, 0.40]
    spec.visual.quality.shadowsize = 4096
    world = spec.worldbody
    # Deterministic fine laminate grain, generated in memory (no external asset writes).
    rng = np.random.default_rng(7)
    grain = np.clip(rng.normal(164, 24, (1024, 1024)), 65, 225).astype(np.uint8)
    texture = spec.add_texture(name="bench_grain", type=mujoco.mjtTexture.mjTEXTURE_2D,
                               width=1024, height=1024, nchannel=3)
    texture.data = np.repeat(grain[..., None], 3, axis=2).tobytes()
    material = spec.add_material(name="bench", texuniform=True, texrepeat=[2.5, 2.5],
                                 specular=0.05, shininess=0.1)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "bench_grain"
    spec.geom("floor").material = "bench"
    spec.geom("floor").rgba = [1, 1, 1, 1]
    for light in spec.lights:
        light.diffuse = [0.30, 0.30, 0.30]
        light.specular = [0.05, 0.05, 0.05]
        light.castshadow = False
    # The task uses the desktop itself: remove the demo's raised plinth and cup.
    spec.delete(spec.geom("pickup_platform"))
    spec.delete(spec.body("cup"))
    spec.body("cube").pos = [0.24273, -0.05154, CUBE_HALF_M + 0.0001]
    spec.geom("cube_geom").rgba = [0.95, 0.95, 0.95, 1.0]
    for part in (29, 30, 31, 36, 37, 38, 41):
        spec.geom(f"visual_{part:03d}").rgba = [0.68, 0.69, 0.70, 1]
    for part in (32, 39, 42, 43):
        spec.geom(f"visual_{part:03d}").rgba = [0.22, 0.22, 0.23, 1]
    spec.site("tool_center").rgba = [0, 0, 0, 0]
    square = world.add_body(name="place_square", pos=SQUARE_CENTER,
                            quat=[np.cos(SQUARE_YAW / 2), 0, 0, np.sin(SQUARE_YAW / 2)])
    half, line = SQUARE_OUTER_M / 2, SQUARE_LINE_M
    for axis in range(2):
        for sign in (-1, 1):
            pos = [0, 0, 0.0001]
            size = [half, half, 0.0001]
            pos[axis] = sign * (half - line / 2)
            size[axis] = line / 2
            square.add_geom(name=f"square_{axis}_{sign}", type=mujoco.mjtGeom.mjGEOM_BOX,
                            pos=pos, size=size, rgba=[0.025, 0.025, 0.025, 1],
                            contype=0, conaffinity=0)
    world.add_camera(name="overhead", pos=list(overhead_eye),
                     xyaxes=look_at(overhead_eye, overhead_target, up=overhead_up), fovy=overhead_fovy)
    tp_eye, tp_target = (0.75, 0.45, 0.50), (0.12, -0.03, 0.12)
    world.add_camera(name="third_person", pos=list(tp_eye), xyaxes=look_at(tp_eye, tp_target), fovy=45)
    # tool0 coordinates: +z toward fingertips, +x jaw opening. The optical axis is predominantly +z;
    # image up is -tool0.y, placing the jaws along the top edge as in wrist_cam_index2.jpg.
    spec.body("tool0").add_camera(
        name="wrist", pos=list(wrist_pos),
        xyaxes=look_at(wrist_pos, wrist_target, up=(0.0, -1.0, 0.0)), fovy=wrist_fovy
    )
    return spec.compile()


class Sim:
    def __init__(self, args):
        self.rig = args.rig
        if self.rig == "yam":
            from yam_rig import build_model as build_yam_model

            self.m = build_yam_model(SCENE, args.wrist_pos, args.wrist_target, args.wrist_fovy, look_at)
        else:
            self.m = build_model(args.overhead_eye, args.overhead_target, args.wrist_pos, args.wrist_target,
                                 args.overhead_fovy, args.wrist_fovy, args.overhead_up)
        self.m.opt.timestep = PHYS_DT
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.joint(n).qposadr[0] for n in JOINTS])
        self.vadr = np.array([self.m.joint(n).dofadr[0] for n in JOINTS])
        self.fl = self.m.joint("gripper_left_joint")
        self.fr = self.m.joint("gripper_right_joint")
        self.kp = self.m.actuator_gainprm[:, 0].copy()
        self.lo = self.m.jnt_range[[self.m.joint(n).id for n in JOINTS], 0]
        self.hi = self.m.jnt_range[[self.m.joint(n).id for n in JOINTS], 1]
        self.cube = self.m.body("cube").id
        self.square = self.m.body("place_square").id
        self.renderer = mujoco.Renderer(self.m, height=FRAME_H, width=FRAME_W)
        self.big = mujoco.Renderer(self.m, height=720, width=1280)
        start, start_grip = START_POSES[args.start]
        self.start, self.start_grip = start, start_grip
        self.right_qadr = np.array([], dtype=int)
        if self.rig == "yam":
            names = ["right_" + n for n in (*JOINTS, "gripper_left_joint", "gripper_right_joint")]
            self.right_qadr = np.array([self.m.joint(n).qposadr[0] for n in names])
            self.right_vadr = np.array([self.m.joint(n).dofadr[0] for n in names])
            self.right_park = np.r_[start, [start_grip * FINGER_OPEN_M] * 2]
            self._park_right()
        self.d.qpos[self.qadr] = start
        self.d.qpos[self.fl.qposadr[0]] = self.d.qpos[self.fr.qposadr[0]] = start_grip * FINGER_OPEN_M
        mujoco.mj_forward(self.m, self.d)
        self.target = np.r_[start, start_grip * FINGER_OPEN_M]
        # Let the cube settle on the tabletop before anything is rendered.
        for _ in range(int(0.5 / PHYS_DT)):
            self._servo()
            mujoco.mj_step(self.m, self.d)
            self._park_right()
        mujoco.mj_forward(self.m, self.d)
        self.time = 0.0

    def joints(self):
        return self.d.qpos[self.qadr].copy()

    def grip(self):
        """0 closed, 1 open, from the left finger's travel."""
        return float(np.clip(self.d.qpos[self.fl.qposadr[0]] / FINGER_OPEN_M, 0.0, 1.0))

    def _park_right(self):
        if self.right_qadr.size:
            self.d.qpos[self.right_qadr] = self.right_park
            self.d.qvel[self.right_vadr] = 0

    def right_state(self):
        """Joint coordinates are in this arm's own base frame, as expected by the unchanged IK file."""
        if self.right_qadr.size:
            q = self.d.qpos[self.right_qadr]
            return q[:6].copy(), float(np.clip(q[6] / FINGER_OPEN_M, 0, 1))
        return self.start.copy(), self.start_grip

    def _servo(self):
        bias = self.d.qfrc_bias
        self.d.ctrl[:6] = self.target[:6] + bias[self.vadr] / self.kp[:6]
        self.d.ctrl[6] = self.target[6] + (bias[self.fl.dofadr[0]] + bias[self.fr.dofadr[0]]) / self.kp[6]

    def run_row(self, joints, grip):
        self.target = np.r_[joints, grip * FINGER_OPEN_M]
        for _ in range(round(CONTROL_DT / PHYS_DT)):
            self._servo()
            mujoco.mj_step(self.m, self.d)
            self._park_right()
        mujoco.mj_forward(self.m, self.d)
        self.time += CONTROL_DT

    def render(self, camera, big=False):
        camera = {"wrist_l": "wrist", "wrist_r": "right_wrist"}.get(camera, camera)
        if camera == "right_wrist" and self.rig == "bench":
            return np.zeros((FRAME_H, FRAME_W, 3), np.uint8)
        r = self.big if big else self.renderer
        r.update_scene(self.d, camera=camera)
        return r.render().copy()

    def policy_image(self, camera):
        rgb = self.render(camera)
        if self.rig == "yam":
            # yam_env.images() and dream_control.capture(): station crop then bilinear resize.
            rgb = np.asarray(Image.fromarray(rgb[60:420]).resize((320, 176), Image.Resampling.BILINEAR))
        return jpeg_b64(rgb)

    def cube_in_square(self):
        """All cube corners inside the outline's inner edge, supported by the desk and at rest."""
        rotation = self.d.xmat[self.cube].reshape(3, 3)
        corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        corners = self.d.xpos[self.cube] + CUBE_HALF_M * corners @ rotation.T
        local = (corners - self.d.xpos[self.square]) @ self.d.xmat[self.square].reshape(3, 3)
        inside = np.all(np.abs(local[:, :2]) < SQUARE_OUTER_M / 2 - SQUARE_LINE_M)
        floor, cube = self.m.geom("floor").id, self.m.geom("cube_geom").id
        supported = any({c.geom1, c.geom2} == {floor, cube} for c in self.d.contact)
        held = any(cube in (c.geom1, c.geom2) and floor not in (c.geom1, c.geom2)
                   for c in self.d.contact)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_BODY, self.cube, velocity, 0)
        return bool(inside and supported and not held
                    and abs(corners[:, 2].min() - self.d.xpos[self.square][2]) < 0.001
                    and np.linalg.norm(velocity[3:]) < 0.01 and np.linalg.norm(velocity[:3]) < 0.1)


def jpeg_b64(rgb):
    buffer = io.BytesIO()
    Image.fromarray(rgb).save(buffer, format="JPEG", quality=85)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


BLACK = jpeg_b64(np.zeros((FRAME_H, FRAME_W, 3), np.uint8))


class DryRunTransport:
    """Stands in for the client's HTTP channel: records each request and answers a synthetic chunk.

    Bench moves the left hand 4 cm down and 2 cm forward. YAM uses FK of a small shoulder/elbow bend to
    stay inside neutral's joint bounds. Both close the grip and hold the right hand still, in the reply
    shape the client's IK reads (`per_key` `action.<side>_eef_xyz` / `_eef_rot6` / `_grip`, 48 rows).
    """

    def __init__(self, real, sim=None):
        self.real = real
        self.sim = sim
        self.requests = []
        self.last_exchange = getattr(real, "last_exchange", None)

    def request(self, request, headers, **kwargs):
        if kwargs.get("path"):  # the session release on close(): nothing to release
            return {"status": {"code": "DA-0000"}}
        obs = request["observation"]
        self.requests.append(request)
        rows = 48
        s = np.linspace(0.0, 1.0, rows)[:, None]
        per_key = {}
        for side in ("left", "right"):
            xyz = np.asarray(obs[f"state.{side}_eef_xyz"], float)
            rot6 = np.asarray(obs[f"state.{side}_eef_rot6"], float)
            grip = float(np.asarray(obs[f"state.{side}_grip"], float).reshape(-1)[0])
            if side == "left":
                path = xyz + s * np.array([0.02, 0.0, -0.04])
                rotations = np.repeat(rot6[None], rows, axis=0)
                if self.sim is not None:
                    # Generate reachable synthetic poses using FK of a small joint bend away
                    # from neutral's shoulder/elbow limits. The real client's IK still solves them.
                    sim = self.sim
                    data = mujoco.MjData(sim.m)
                    data.qpos[:] = sim.d.qpos
                    start = sim.joints()
                    end = np.clip(start + [0, -.04, .08, 0, 0, 0], sim.lo, sim.hi)
                    base, tool = sim.m.body("base_link").id, sim.m.body("tool0").id
                    path, rotations = [], []
                    for fraction in s[:, 0]:
                        data.qpos[sim.qadr] = start + fraction * (end - start)
                        mujoco.mj_kinematics(sim.m, data)
                        base_rotation = data.xmat[base].reshape(3, 3)
                        path.append(base_rotation.T @ (data.xpos[tool] - data.xpos[base]))
                        rotation = base_rotation.T @ data.xmat[tool].reshape(3, 3)
                        rotations.append(rotation[:, :2].T.ravel())
                    path, rotations = np.array(path), np.array(rotations)
                grips = grip + s * (0.2 - grip)
            else:
                path = np.repeat(xyz[None], rows, axis=0)
                rotations = np.repeat(rot6[None], rows, axis=0)
                grips = np.full((rows, 1), grip)
            per_key[f"action.{side}_eef_xyz"] = path.tolist()
            per_key[f"action.{side}_eef_rot6"] = rotations.tolist()
            per_key[f"action.{side}_grip"] = np.asarray(grips).reshape(rows, 1).tolist()
        return {"status": {"code": "DA-0000"}, "session_id": "dry-run", "actions": {"per_key": per_key}}

    def close(self):
        self.real.close()


def summarize_request(request):
    """The request without its frames: keys, shapes and the state values."""
    out = {}
    for key, value in request["observation"].items():
        if isinstance(value, str) and len(value) > 200:
            out[key] = f"<jpeg {len(value)} b64 chars>"
        elif isinstance(value, list) and value and isinstance(value[0], str) and len(value[0]) > 200:
            out[key] = f"<{len(value)} jpegs>"
        else:
            out[key] = np.round(np.asarray(value, float), 4).tolist() if not isinstance(value, str) else value
    return {k: v for k, v in request.items() if k != "observation"} | {"observation": out}


def preview(sim, out):
    for camera in ("overhead", "wrist", "wrist_l", "wrist_r", "third_person"):
        Image.fromarray(sim.render(camera, big=camera == "third_person")).save(out / f"preview_{camera}.png")
    for camera, reference in (("overhead", "topdown_cam_index0.jpg"), ("wrist", "wrist_cam_index2.jpg")):
        if sim.rig != "bench":
            continue
        with Image.open(HERE / "real" / reference) as real, Image.open(out / f"preview_{camera}.png") as rendered:
            comparison = Image.new("RGB", (2 * FRAME_W, FRAME_H))
            comparison.paste(real, (0, 0))
            comparison.paste(rendered, (FRAME_W, 0))
            comparison.save(out / f"comparison_{camera}.png")
    parameters = {}
    for name in ("overhead", "wrist", "third_person", *(("right_wrist",) if sim.rig == "yam" else ())):
        camera = sim.m.camera(name)
        parameters[name] = {
            "frame": sim.m.body(int(camera.bodyid[0])).name or "world",
            "pos_m": camera.pos.tolist(), "quat_wxyz": camera.quat.tolist(),
            "fovy_deg": float(camera.fovy[0]),
            "intrinsic": sim.m.cam_intrinsic[camera.id].tolist(),
            "sensorsize": sim.m.cam_sensorsize[camera.id].tolist(),
            "size_px": [1280, 720] if name == "third_person" else [FRAME_W, FRAME_H],
        }
    (out / "camera_parameters.json").write_text(json.dumps(parameters, indent=2) + "\n")
    if sim.rig == "yam":
        from yam_rig import comparison

        comparison(sim, out)
    print(f"wrote previews, comparison images and camera_parameters.json in {out}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--preview", action="store_true")
    p.add_argument("--rig", choices=("bench", "yam"), default="bench",
                   help="yam uses the calibrated top camera; --overhead-* only affects bench")
    p.add_argument("--live", action="store_true")
    p.add_argument("--model", choices=sorted(MODELS), default="flash")
    p.add_argument("--task", default="pick up the white cube and place into black square")
    p.add_argument("--start", choices=sorted(START_POSES), default="neutral")
    p.add_argument("--max-calls", type=int, default=5)
    p.add_argument("--seconds", type=float, default=60.0, help="sim-time cap")
    p.add_argument("--out", type=Path, default=HERE / "runs" / time.strftime("%Y%m%d-%H%M%S"))
    p.add_argument("--overhead-eye", type=float, nargs=3, default=(0.40030, -0.16871, 0.28048))
    p.add_argument("--overhead-target", type=float, nargs=3, default=(0.29481, -0.15448, 0.0))
    p.add_argument("--overhead-up", type=float, nargs=3, default=(0.39047, 0.91512, -0.10043),
                   help="world-space up vector; sets camera roll to match the real top-down still")
    p.add_argument("--overhead-fovy", type=float, default=60)
    p.add_argument("--wrist-pos", type=float, nargs=3, default=(0.0, 0.055, -0.09))
    p.add_argument("--wrist-target", type=float, nargs=3, default=(0.0, 0.025, 0.08))
    p.add_argument("--wrist-fovy", type=float, default=90)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    sim = Sim(args)
    if args.preview:
        try:
            preview(sim, args.out)
        finally:
            sim.big.close()
            sim.renderer.close()
        return

    if not args.live:
        # Fail closed if a future client version introduces another network path.
        def deny_network(event, _args):
            if event in {"socket.connect", "socket.getaddrinfo", "socket.sendto"}:
                raise RuntimeError(f"network is disabled in DRY RUN ({event})")

        sys.addaudithook(deny_network)
    # The client's default log directory is ~/.dream; keep this run's writes with its artifacts.
    os.environ["DREAM_LOG_DIR"] = str(args.out.resolve() / "dream_logs")
    os.environ["JAX_COMPILATION_CACHE_DIR"] = str(HERE / "runs" / ".jax_cache")
    import dream

    client = dream.DreamClient()
    config = {
        "checkpoint": MODELS[args.model],
        "embodiment": EMBODIMENT,
        "ik": {"arm": str(ARM_FILE), "head": "eef"},
        "timeout_ms": 60000,
        "strict": False,  # the client's own IK adds hand-pose keys the strict check does not know; we bound every row ourselves
        "_record": False,
    }
    if args.rig == "yam":
        config.update(obs_window=9, obs_fps=5)
    if args.live:
        client.policy(config)
    else:
        # Only the platform metadata is substituted; the client's FK/IK and observation path are unchanged.
        from unittest.mock import patch
        from dream.models import Model

        model = Model(id=MODELS[args.model], owner="public", base_model=MODELS[args.model],
                      embodiment="yam", servable=True, created_at=None,
                      cameras=["overhead", "wrist_l", "wrist_r"])
        with patch("dream.models._record", return_value=model):
            client.policy(config)
    policy = client._policy
    stub = None
    if not args.live:
        stub = DryRunTransport(policy._transport, sim=sim if args.rig == "yam" else None)
        policy._transport = stub
        args.max_calls = min(args.max_calls, 3)

    log = []
    frames = []
    commanded = None
    calls = 0
    print(f"{'LIVE' if args.live else 'DRY RUN'}: model={MODELS[args.model]} task={args.task!r} "
          f"max_calls={args.max_calls} out={args.out}")

    def observe():
        right_joints, right_grip = sim.right_state()
        client.set_observation(
            {f"video.{role}": sim.policy_image(role) for role in ("overhead", "wrist_l", "wrist_r")},
            {"state.left_arm": sim.joints(), "state.left_grip": [sim.grip()],
             "state.right_arm": right_joints, "state.right_grip": [right_grip], "prompt": args.task},
            timestamp=sim.time,
        )

    observe()
    if args.rig == "yam":
        # Fill nine real ticks at 5 Hz before the first request, as dream_control does.
        for _ in range(8):
            for _ in range(4):
                sim.run_row(sim.start, sim.start_grip)
            observe()
    warming_since = None
    try:
        while calls < args.max_calls and sim.time < args.seconds:
            started = time.monotonic()
            try:
                chunk = client.get_action(commanded=commanded)
            except dream.DreamServerError as error:
                # A model still starting (DA-2000 pool_warming) is refused, not billed: wait and ask again.
                warming_since = warming_since or time.monotonic()
                if not error.retryable or time.monotonic() - warming_since > 600:
                    raise
                wait = error.retry_after_s or 10.0
                print(f"model not ready ({error.reason}); retrying in {wait:.0f}s", flush=True)
                time.sleep(wait)
                continue
            warming_since = None
            calls += 1
            rtt = time.monotonic() - started
            left = np.asarray(chunk["q_l_chunk"], float)
            entry = {"call": calls, "sim_time": round(sim.time, 2), "rtt_s": round(rtt, 3), "rows": len(left),
                     "keys": sorted(chunk), "clipped_rows": 0, "limit_rows": 0}
            if stub is not None:
                entry["request"] = summarize_request(stub.requests[-1])
            prev = sim.joints()
            for row in left[:EXEC_ROWS]:
                joints, grip = row[:6], float(np.clip(row[6], 0.0, 1.0))
                bounded = np.clip(joints, sim.lo, sim.hi)
                if not np.allclose(bounded, joints) and stub is not None:
                    entry.setdefault("limit_details", []).append({
                        JOINTS[i]: float(joints[i]) for i in range(6)
                        if not np.isclose(bounded[i], joints[i])
                    })
                entry["limit_rows"] += int(not np.allclose(bounded, joints))
                stepped = prev + np.clip(bounded - prev, -MAX_JOINT_STEP, MAX_JOINT_STEP)
                entry["clipped_rows"] += int(not np.allclose(stepped, bounded))
                sim.run_row(stepped, grip)
                prev = stepped
                right_joints, right_grip = sim.right_state()
                commanded = {"q_l_chunk": np.r_[stepped, grip], "q_r_chunk": np.r_[right_joints, right_grip]}
                if round(sim.time / CONTROL_DT) % RENDER_EVERY == 0:
                    observe()
                    frames.append(sim.render("third_person", big=True))
            entry["tool_z_m"] = round(float(sim.d.xpos[sim.m.body("tool0").id][2]), 3)
            entry["cube_xyz_m"] = np.round(sim.d.xpos[sim.cube], 3).tolist()
            log.append(entry)
            print(json.dumps({k: v for k, v in entry.items() if k != "request"}))
            if sim.cube_in_square():
                print("cube resting inside the square")
                break
    finally:
        (args.out / "log.json").write_text(json.dumps(log, indent=1))
        try:
            client.close()
        except Exception as error:
            print(f"client.close(): {error!r}", file=sys.stderr)
        if frames:
            for i, f in enumerate(frames[:: max(1, len(frames) // 6)]):
                Image.fromarray(f).save(args.out / f"frame_{i:02d}.png")
            try:
                import imageio.v2 as imageio

                imageio.mimsave(args.out / "run.mp4", frames, fps=round(1 / (CONTROL_DT * RENDER_EVERY)))
            except Exception as error:  # video is a convenience; the PNGs and log are the record
                print(f"no video: {error}", file=sys.stderr)
        print(f"{calls} calls ({'billed' if args.live else 'none billed'}); log in {args.out}")
        sim.big.close()
        sim.renderer.close()


if __name__ == "__main__":
    main()
