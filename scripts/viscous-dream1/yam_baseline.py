"""Baseline for the Viscous Dream-1 trial: does Dream-1 drive Dream's OWN simulated YAM, and does it still when it
is called the way the Viscous arm has to call it?

Two modes, both in Dream's shipped `dream_examples/yam_mujoco` scene and through its own helpers:

  --mode station  exactly what that example does for `dream-1-beta`: the YAM station keys, so the serve computes
                  the hand poses from the joints and the model READS the arm's state (joints, hand pose, grip).
  --mode vision   the path every non-YAM arm takes (the Viscous trial's): client-side IK (`ik` arm `yam`, head
                  `eef`) with `robot` `other_embodiment`, on which the serve withholds the whole state, so the model
                  sees only the cameras and the prompt and its hand poses are composed on the pose sent.

--task cube swaps the apple/bowl for the local cube/square task. --rig viscous swaps
both arms for Viscous in the unchanged apple scene (vision only).

If station works and vision fails the same way the Viscous did, the Viscous failure is the vision-only mode, not
our arm, cameras or frames. Sim time stops while a request is in flight (as in Dream's example).

Without --live no inference is sent (the observation and policy config are checked; IK fetches model metadata).
With --live each
successful request is billed; --max-calls caps them. Writes log.json, run.mp4 and a few PNGs to --out.
"""

import argparse
import json
import os
import sys
import time
import warnings
from collections import deque
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("JAX_COMPILATION_CACHE_DIR", str(Path(__file__).resolve().parent / ".cache/dream_ik/jax"))
os.environ.setdefault("XDG_CACHE_HOME", str(Path(__file__).resolve().parent / ".cache"))
os.environ.setdefault("DREAM_LOG_DIR", str(Path(__file__).resolve().parent / "runs/dream_logs"))
_tmp = Path(__file__).resolve().parent / ".cache/tmp"
_tmp.mkdir(parents=True, exist_ok=True)
os.environ["TMPDIR"] = str(_tmp)
sys.pycache_prefix = str(Path(__file__).resolve().parent / ".cache/pycache")

import dream_examples.yam_mujoco as _example  # noqa: E402

sys.path.insert(0, str(next(iter(_example.__path__))))  # namespace package; modules use bare imports

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from dream_control import (  # noqa: E402
    BETA_MODEL,
    FRAME_EVERY_STEPS,
    FRAMES,
    VIDEO_ROLES,
    YAM_EMBODIMENT,
    capture,
    feed,
    observation,
    policy_config,
    settings,
    validate_chunk,
)
from yam_env import VIEWER_AZIMUTH, VIEWER_DISTANCE, VIEWER_ELEVATION, VIEWER_LOOKAT, YamEnv  # noqa: E402

BOWL_XY = np.array([0.0, -0.04])  # yam_env.build_xml's bowl body
TASKS = {"apple": "put apple into the bowl", "orange": "put orange into the bowl"}


def vision_config(model):
    """The Viscous trial's path, on a YAM: client IK, state withheld by the serve."""
    return {
        "checkpoint": model,
        "embodiment": YAM_EMBODIMENT,
        "ik": {"arm": "yam", "head": "eef"},
        "robot": "other_embodiment",
        "timeout_ms": 45000,
        "strict": False,
        "action_hz": 20,
        "_record": False,
    }


class Recorder:
    """Third-person frames from the example's own viewer pose (never sent to Dream)."""

    def __init__(self, env):
        self.renderer = mujoco.Renderer(env.model, height=480, width=640)
        self.cam = mujoco.MjvCamera()
        self.cam.lookat[:] = VIEWER_LOOKAT
        self.cam.distance, self.cam.azimuth, self.cam.elevation = VIEWER_DISTANCE, VIEWER_AZIMUTH, VIEWER_ELEVATION
        self.frames = []
        self.milestones = [0]

    def grab(self, env, top):
        self.renderer.update_scene(env.data, camera=self.cam)
        third = self.renderer.render()
        # Third-person on the left, the policy's top camera on the right, so a reader sees what the model saw.
        top = np.asarray(top)
        pad = np.zeros((480, 640, 3), np.uint8)
        pad[60:420] = top
        self.frames.append(np.concatenate([third, pad], axis=1))


class CustomGripFeed:
    """Adapt only the custom-arm client's normalized grip spelling; reuse Dream's feed."""

    def __init__(self, client):
        self.client = client

    def set_observation(self, images, state, **kwargs):
        state = dict(state)
        for side in ("left", "right"):
            state[f"state.{side}_grip"] = state.pop(f"state.{side}_gripper")
        self.client.set_observation(images, state, **kwargs)


def objects(env, fruit):
    p = env.data.body(fruit).xpos
    target = "square" if fruit == "cube" else "bowl"
    object_id = env.model.body(fruit).id
    touched = set()
    for contact in env.data.contact:
        bodies = [env.model.geom_bodyid[g] for g in (contact.geom1, contact.geom2)]
        if object_id in bodies:
            for body in bodies:
                name = env.model.body(body).name
                if name.startswith(("left_", "right_")):
                    touched.add(name)
    return {
        f"{fruit}_xyz": np.round(p, 3).tolist(),
        f"{fruit}_to_{target}_xy_m": round(float(np.linalg.norm(p[:2] - BOWL_XY)), 6),
        "object_height_above_table_m": round(float(p[2] - .76), 4),
        "object_robot_contacts": sorted(touched),
        "left_hand_z": round(float(env.data.site("left_tcp").xpos[2]), 3),
        "right_hand_z": round(float(env.data.site("right_tcp").xpos[2]), 3),
        **{f"{side}_tcp_xyz": np.round(env.data.site(f"{side}_tcp").xpos, 4).tolist()
           for side in ("left", "right")},
        **{f"{side}_gripper": round(float(env.state()[side][6]), 4) for side in ("left", "right")},
    }


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=("station", "vision"), required=True)
    p.add_argument("--fruit", choices=sorted(TASKS), default="apple")
    p.add_argument("--task", choices=("apple", "orange", "cube"), help="Defaults to --fruit; cube replaces apple/bowl")
    p.add_argument("--rig", choices=("yam", "viscous"), default="yam")
    p.add_argument("--live", action="store_true")
    p.add_argument("--max-calls", type=int, default=20)
    p.add_argument("--actions", type=int, default=48, help="rows run per chunk (Dream's example default 48)")
    p.add_argument("--out", type=Path, required=True)
    args = p.parse_args()
    if not 1 <= args.max_calls <= 20 or not 1 <= args.actions <= 48:
        p.error("Use 1–20 calls and 1–48 actions.")
    if args.rig == "viscous" and (args.mode != "vision" or args.task not in (None, "apple") or args.fruit != "apple"):
        p.error("The Viscous comparison uses --mode vision and the apple task.")
    args.fruit = args.task or args.fruit
    distance_key = f"{args.fruit}_to_{'square' if args.fruit == 'cube' else 'bowl'}_xy_m"
    args.out.mkdir(parents=True, exist_ok=True)
    if args.live and (args.out / "requests.jsonl").exists():
        p.error("This output already has live requests; preserve its call accounting.")
    task = "pick up the white cube and place into black square" if args.fruit == "cube" else TASKS[args.fruit]
    station = args.mode == "station"

    from cross_scenes import CubeYamEnv, ViscousAppleEnv, preview_comparison, success

    env = ViscousAppleEnv() if args.rig == "viscous" else CubeYamEnv() if args.fruit == "cube" else YamEnv()
    rec = Recorder(env)
    windows = {name: deque(maxlen=FRAMES) for name in VIDEO_ROLES}
    steps = 0
    while any(len(w) < FRAMES for w in windows.values()):
        s = env.state()
        env.step(s["left"], s["right"])
        images = capture(env, windows, keep=steps % FRAME_EVERY_STEPS == 0)
        steps += 1
    rec.grab(env, images["top"])
    if args.rig == "viscous":
        preview_comparison(env, args.out)
    obs = observation(env, windows, task, BETA_MODEL, station)
    print(f"{args.mode}: task={task!r} start {objects(env, args.fruit)}")
    print("observation keys:", sorted(obs))
    os.environ["DREAM_LOG_DIR"] = str(args.out.resolve() / "dream_logs")
    import dream

    warnings.filterwarnings("ignore", category=dream.DreamSlowNetworkWarning)
    key = settings().get("DREAM_API_KEY", "")
    client = dream.DreamClient(api_key=key)
    log, trajectory, calls, commanded, best = [], [], 0, None, objects(env, args.fruit)[distance_key]
    ever_success = False
    try:
        config = policy_config(BETA_MODEL, None, True) if station else vision_config(BETA_MODEL)
        if args.rig == "viscous":
            config["ik"]["arm"] = str(Path(__file__).resolve().parent / "viscous_arm.json")
            config.pop("robot")  # Dream defaults a custom arm to other_embodiment.
        (args.out / "configuration.json").write_text(json.dumps({
            "policy": config, "frames_per_camera": FRAMES, "camera_hz": 5,
            "row_hz": 20, "rows_per_chunk": args.actions, "own_speed_clipping": False,
            "initial_joints": "zeros", "initial_grippers": "open", "sim_held_during_request": True,
        }, indent=2))
        client.policy(config)
        feed_client = CustomGripFeed(client) if args.rig == "viscous" else client
        feed(feed_client, env, windows, task, BETA_MODEL, steps, every=0, station=station)
        # set_observation only queues frames. Exercise its deferred FK/grip conversion
        # without get_action or any inference request, in dry AND live preflight.
        queued = client._window.build_observation()
        ik = client._policy._ik
        prepared = queued if ik is None else ik.observe(queued)
        (args.out / "preflight.json").write_text(json.dumps({
            "queued_keys": sorted(queued), "prepared_keys": sorted(prepared),
            "resolved_robot": client._policy._robot,
            "queued_camera_lengths": {k: len(v) for k, v in queued.items() if k.startswith("video.")},
            "input_conversion_passed": True, "inference_calls": 0,
        }, indent=2))
        print("policy opened; config accepted (no action requested yet)", flush=True)
        if not args.live:
            print("dry run: observation and policy checked, zero inference calls", flush=True)
            return
        while calls < args.max_calls:
            started = time.monotonic()
            with (args.out / "requests.jsonl").open("a") as ledger:
                ledger.write(json.dumps({"event": "request", "call": calls + 1}) + "\n")
            try:
                reply = client.get_action(commanded=commanded) if not station else client.get_action()
            except dream.DreamServerError as error:
                with (args.out / "requests.jsonl").open("a") as ledger:
                    ledger.write(json.dumps({"event": "refusal", "code": error.code, "reason": error.reason}) + "\n")
                if error.code != "DA-2000" or error.reason != "pool_warming":
                    raise
                print(f"not ready ({error.reason}); retry in {error.retry_after_s or 10:.0f}s", flush=True)
                time.sleep(error.retry_after_s or 10)
                continue
            calls += 1
            with (args.out / "requests.jsonl").open("a") as ledger:
                ledger.write(json.dumps({"event": "success", "call": calls}) + "\n")
            rtt = time.monotonic() - started
            adjustments = []
            try:
                left, right = validate_chunk(env, reply, args.actions, verbose=False, adjustments=adjustments)
            except ValueError as error:
                (args.out / f"rejected_reply_{calls:02d}.json").write_text(json.dumps(
                    reply, indent=1, default=lambda value: np.asarray(value).tolist()))
                entry = {"call": calls, "rtt_s": round(rtt, 2), "rows": 0,
                         "error": str(error), "limit_notices": [str(error)],
                         "closest_so_far_m": best, "success": success(env, args.fruit),
                         "ever_success": ever_success, "commanded_gripper_ranges": None,
                         "last_gripper_commands": None if commanded is None else {
                             side: float(commanded[f"q_{side[0]}_chunk"][-1]) for side in ("left", "right")},
                         **objects(env, args.fruit)}
                log.append(entry)
                rec.milestones.append(len(rec.frames) - 1)
                print(json.dumps(entry), flush=True)
                raise
            for l_row, r_row in zip(left, right):
                env.step(l_row, r_row)
                row_success = success(env, args.fruit)
                ever_success |= row_success
                sample = {"call": calls, "step": steps, "success": row_success,
                          "commanded_left": l_row.tolist(), "commanded_right": r_row.tolist(),
                          **objects(env, args.fruit)}
                trajectory.append(sample)
                best = min(best, sample[distance_key])
                commanded = {"q_l_chunk": l_row, "q_r_chunk": r_row}
                kept = steps % FRAME_EVERY_STEPS == 0
                images = capture(env, windows, keep=kept)
                if kept:
                    feed(feed_client, env, windows, task, BETA_MODEL, steps, station=station)
                    rec.grab(env, images["top"])
                steps += 1
            rec.milestones.append(len(rec.frames) - 1)
            entry = {"call": calls, "rtt_s": round(rtt, 2), "rows": len(left), "limit_notices": adjustments,
                     "closest_so_far_m": best,
                     "success": success(env, args.fruit), "ever_success": ever_success,
                     "last_gripper_commands": {"left": float(left[-1, 6]), "right": float(right[-1, 6])},
                     "commanded_gripper_ranges": {side: [float(rows[:, 6].min()), float(rows[:, 6].max())]
                                                   for side, rows in (("left", left), ("right", right))},
                     "reply_keys": sorted(reply), **objects(env, args.fruit)}
            best = min(best, entry[distance_key])
            log.append(entry)
            (args.out / "log.json").write_text(json.dumps(log, indent=1))
            print(json.dumps(entry), flush=True)
            from PIL import Image
            Image.fromarray(rec.frames[rec.milestones[-1]]).save(args.out / f"frame_{calls:02d}.png")
    finally:
        (args.out / "log.json").write_text(json.dumps(log, indent=1))
        (args.out / "trajectory.json").write_text(json.dumps(trajectory, indent=1))
        (args.out / "summary.json").write_text(json.dumps({"mode": args.mode, "rig": args.rig, "task": task,
                                                         "successful_billed_calls": calls,
                                                         f"closest_{distance_key}": best,
                                                         "executed_rows": len(trajectory),
                                                         "completed_chunks": sum(entry["rows"] > 0 for entry in log),
                                                         "termination_reason": log[-1].get("error") if log else None,
                                                         "ever_success": ever_success,
                                                         "final_success": success(env, args.fruit)}, indent=1))
        try:
            client.close()
        except Exception as error:  # noqa: BLE001
            print(f"close: {error!r}", file=sys.stderr)
        if rec.frames:
            from PIL import Image

            for i, frame_index in enumerate(rec.milestones):
                Image.fromarray(rec.frames[frame_index]).save(args.out / f"frame_{i:02d}.png")
            from PIL import ImageDraw
            sheet = Image.new("RGB", (1280, 268 * ((len(rec.milestones) + 1) // 2)), "white")
            draw = ImageDraw.Draw(sheet)
            for i, frame_index in enumerate(rec.milestones):
                x, y = (i % 2) * 640, (i // 2) * 268
                sheet.paste(Image.fromarray(rec.frames[frame_index]).resize((640, 240)), (x, y + 24))
                label = "Start" if i == 0 else f"Call {i}"
                if i and log[i - 1].get("error"):
                    label += ": rejected, zero rows executed"
                draw.text((x + 8, y + 5), label, fill="black")
            sheet.save(args.out / "contact_sheet.jpg")
            try:
                import imageio.v2 as imageio

                imageio.mimsave(args.out / "run.mp4", rec.frames, fps=5)
            except Exception as error:  # noqa: BLE001
                print(f"no video: {error}", file=sys.stderr)
        print(f"{calls} calls ({'billed' if args.live else 'none'}); closest {distance_key} {best:.6f} m; "
              f"log in {args.out}")
        env.close()
        rec.renderer.close()


if __name__ == "__main__":
    main()
