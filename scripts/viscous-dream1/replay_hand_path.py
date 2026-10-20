"""Replay raw YAM hand heads through the unchanged Viscous ClientIK; zero network calls."""
import yam_baseline as baseline  # establishes workspace-only cache/temp/log paths
import argparse
import json
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from cross_scenes import ViscousAppleEnv, success
from dream.client_ik import ClientIK, POSE_KEYS
from dream.models import Model
from dream.smoothing import SeamSmoother
from hand_path_tools import IKTrace, apple_in_base, clip_viscous_grips, tracking


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, default=Path("runs/yam_vision_live2/chunks.jsonl"))
    p.add_argument("--out", type=Path, default=Path("runs/replay_yam_path_on_viscous"))
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if (args.out / "chunks.jsonl").exists():
        p.error("Output already contains a replay; use a fresh directory")
    source = [json.loads(line) for line in args.source.read_text().splitlines()]
    model = Model(baseline.BETA_MODEL, "public", baseline.BETA_MODEL, "yam", True, None,
                  ["overhead", "wrist_l", "wrist_r"])
    ik = ClientIK({"arm": str(Path("viscous_arm.json").resolve()), "head": "eef"},
                  20, baseline.YAM_EMBODIMENT, model)
    # No inference latency exists offline. Wait for the same default traj solve to compile.
    if not ik.traj_ready.wait(120):
        raise RuntimeError("Trajectory IK did not compile")
    smoother = SeamSmoother(20, pose_keys=POSE_KEYS)
    trace = IKTrace(ik)
    env, rec = ViscousAppleEnv(), None
    rec = baseline.Recorder(env)
    windows = {name: deque(maxlen=baseline.FRAMES) for name in baseline.VIDEO_ROLES}
    steps, commanded, trajectory, log = 0, None, [], []
    while any(len(w) < baseline.FRAMES for w in windows.values()):
        s = env.state()
        env.step(s["left"], s["right"])
        images = baseline.capture(env, windows, keep=steps % baseline.FRAME_EVERY_STEPS == 0)
        steps += 1
    rec.grab(env, images["top"])
    initial_z = float(env.data.body("apple").xpos[2])
    try:
        for chunk in source:
            obs = baseline.observation(env, windows, "put apple into the bowl", baseline.BETA_MODEL, False)
            for side in ("left", "right"):
                obs[f"state.{side}_grip"] = obs.pop(f"state.{side}_gripper")
            ik.observe(obs)
            # Keep the source RTT for Dream's takeover clock; sim remains held during a request.
            rtt = chunk["client_ik"]["rtt_s"]
            action = ik.solve({"per_key": chunk["per_key"]}, obs, smoother, rtt, commanded)
            reply = smoother.smooth(obs, action, 0, rtt, commanded)
            notices = []
            rows = min(chunk["executed_rows"], len(reply["q_l_chunk"]))
            left, right = baseline.validate_chunk(env, clip_viscous_grips(reply, notices), rows,
                                                  verbose=False, adjustments=notices)
            record = trace.chunk(chunk["call"], reply, env, len(left))
            with (args.out / "chunks.jsonl").open("a") as f:
                f.write(json.dumps(record) + "\n")
            errors = tracking(ik, chunk["per_key"], left, right)
            for row, (l, r) in enumerate(zip(left, right)):
                apple_before = {s: apple_in_base(env, s).tolist() for s in ("left", "right")}
                env.step(l, r)
                contacts = []
                for contact in env.data.contact:
                    geoms = [env.model.geom(g).name for g in (contact.geom1, contact.geom2)]
                    bodies = [env.model.body(env.model.geom_bodyid[g]).name for g in (contact.geom1, contact.geom2)]
                    if any(b.startswith(("left_", "right_")) for b in bodies):
                        contacts.append({"geoms": geoms, "bodies": bodies, "distance_m": float(contact.dist)})
                trajectory.append({"call": chunk["call"], "row": row, "step": steps,
                                   "tracking": errors[row], "apple_base_before_step": apple_before,
                                   "commanded_left": l.tolist(), "commanded_right": r.tolist(),
                                   "arm_joints": {s: env.state()[s][:6].tolist() for s in ("left", "right")},
                                   "robot_contacts": contacts,
                                   "success": success(env, "apple"), **baseline.objects(env, "apple")})
                commanded = {"q_l_chunk": l, "q_r_chunk": r}
                kept = steps % baseline.FRAME_EVERY_STEPS == 0
                images = baseline.capture(env, windows, keep=kept)
                if kept:
                    rec.grab(env, images["top"])
                steps += 1
            rec.milestones.append(len(rec.frames) - 1)
            log.append({"call": chunk["call"], "limit_notices": notices, **baseline.objects(env, "apple")})
            print(json.dumps(log[-1]), flush=True)
    finally:
        (args.out / "trajectory.json").write_text(json.dumps(trajectory, indent=1))
        (args.out / "log.json").write_text(json.dumps(log, indent=1))
        (args.out / "summary.json").write_text(json.dumps({
            "source": str(args.source), "inference_calls": 0, "executed_rows": len(trajectory),
            "ever_success": any(t["success"] for t in trajectory), "final_success": success(env, "apple"),
            "max_apple_lift_m": max((t["apple_xyz"][2] - initial_z for t in trajectory), default=0),
            "seam": "Fresh Viscous neutral/open start; own measured joints and prior Viscous commands; source RTT; default traj/filter/seam. Raw base-frame poses unchanged.",
        }, indent=2))
        sheet = Image.new("RGB", (1280, 268 * ((len(rec.milestones) + 1) // 2)), "white")
        draw = ImageDraw.Draw(sheet)
        for i, frame in enumerate(rec.milestones):
            im = Image.fromarray(rec.frames[frame])
            im.save(args.out / f"frame_{i:02d}.png")
            x, y = i % 2 * 640, i // 2 * 268
            sheet.paste(im.resize((640, 240)), (x, y + 24))
            draw.text((x + 8, y + 5), "Start" if not i else f"Chunk {i}", fill="black")
        sheet.save(args.out / "contact_sheet.jpg")
        import imageio.v2 as imageio
        imageio.mimsave(args.out / "run.mp4", rec.frames, fps=5)
        env.close()
        rec.renderer.close()


if __name__ == "__main__":
    main()
