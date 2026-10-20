#!/usr/bin/env python3
"""Hold the Viscous follower at planned static poses and record what it takes to hold them.

The data feeds ``viscous-gravity-fit.py``. The arm is energized with its normal
follow gains and walked, slowly, through poses that are collision-checked
against the URDF's collision meshes (self and floor) along every leg. Each pose
is approached once from above and once from below so motor friction cancels in
the fit. Each hold records joint angles, RS00 torque telemetry, temperatures
and the commanded target (stiffness x error is a second torque estimate).

    viscous-gravity-collect.py --robot viscous_01 --dry-run   # plan only, reads the arm read-only
    viscous-gravity-collect.py --robot viscous_01             # energizes; asks first

Stop: Ctrl-C, or SIGINT/SIGTERM to a detached run, returns the arm to its
start pose, then releases torque. A second signal abandons the return and
releases torque WHERE THE ARM IS, which drops it: support the arm first.
``--stop-test`` proves the signal path on a detached launch without touching
the arm.

Heat: when a winding reaches ``--cool-at`` the arm parks at COOL, a pose the
CAD model holds with under 0.1 N m on every joint, until every winding is back
under ``--cool-to``; ``--max-temp`` aborts and returns home. ``--start-at``
resumes a plan at a pose index (the plan is deterministic for a given seed and
start pose); every leg is re-routed from wherever the arm actually is.
"""

import argparse
import datetime
import os
import signal
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
os.environ.setdefault("MAKERMODSLAB_HOME", str(Path.home() / ".makermods/makermodslab-viscous"))
sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402
from viscous_bench import (  # noqa: E402
    ARM,
    COOL,
    HUB,
    RunAbortError,
    Runner,
    _interrupt,
    build_robot,
    clipped,
    load_record,
    read_pose_readonly,
    summarize,
    write_json,
)

from makermodslab.arm_gravity import (  # noqa: E402
    CollisionChecker,
    GravityModel,
    path_clear,
    plan_poses,
    stored_path,
)

# Where poses are drawn from. Pan stays where the arm starts: its axis is
# vertical, so it never carries gravity.
BOX = {
    "shoulder_lift": (-160.0, -20.0),
    "elbow_flex": (20.0, 235.0),
    "wrist_flex": (-90.0, 80.0),
    "wrist_yaw": (-50.0, 50.0),
    "wrist_roll": (-90.0, 90.0),
}
DEFAULT_MODEL_DIR = REPO / "configs/viscous/model"


def plan(args, robot, start):
    model = GravityModel(args.urdf, ARM)
    checker = CollisionChecker(args.urdf, args.model_dir, ARM, floor_z=args.floor_z)
    limits = robot.config.joint_limits
    start_arm = clipped({k: start[k] for k in ARM}, limits)
    hub = {"shoulder_pan": start_arm["shoulder_pan"], **HUB}
    gains = robot.config.gains

    def sagged(pose):
        # Holding torque = kp * (command - position): with soft gains the arm
        # settles below its command by roughly CAD torque / kp.
        # A joint cannot sag through its own end stop.
        tau = model.torque(pose)
        return clipped({k: pose[k] - np.degrees(tau[k] / gains[k][0]) for k in ARM}, limits)

    def is_clear(pose):
        return checker.is_clear(pose) and checker.is_clear(sagged(pose))

    cool = {"shoulder_pan": start_arm["shoulder_pan"], **COOL}
    for name, pose in (("Hub", hub), ("Cooling", cool)):
        if not is_clear(pose):
            raise SystemExit(f"{name} pose is not clear: {checker.problems(pose)}")
    if not path_clear(hub, cool, is_clear):
        raise SystemExit("No clear path between the hub and the cooling pose")
    if not checker.is_clear(start_arm):
        print(f"Note: the start pose itself reports {checker.problems(start_arm)} (resting on a stop?)")
    visits = plan_poses(
        model,
        start=start_arm,
        limits=limits,
        box=BOX,
        count=args.poses,
        is_clear=is_clear,
        hub=hub,
        max_torque_nm=args.max_torque,
        seed=args.seed,
    )
    if not visits:
        problems = checker.problems(start_arm)
        raise SystemExit(
            "No pose is reachable from the start pose"
            + (f": it reports {problems}. Fold the arm against its stops and retry." if problems else ".")
        )
    home_via = [] if path_clear(visits[-1].pose, start_arm, is_clear) else [hub]
    if home_via and not path_clear(visits[-1].pose, hub, is_clear):
        raise SystemExit("No clear path home from the last pose")
    return model, is_clear, start_arm, hub, cool, visits, home_via


def describe(model, start_arm, visits, home_via, args):
    travel, here = 0.0, start_arm
    for v in visits:
        for via, approach in zip(v.via, v.approaches, strict=True):
            for p in [*via, approach]:
                travel += max(abs(p[k] - here[k]) for k in ARM) / args.speed
                here = p
            travel += max(abs(v.pose[k] - approach[k]) for k in ARM) / args.approach_speed
            here = v.pose
    for p in [*home_via, start_arm]:
        travel += max(abs(p[k] - here[k]) for k in ARM) / args.speed
        here = p
    holds = sum(len(v.approaches) for v in visits) * (args.settle + args.sample)
    print(f"{len(visits)} poses, {sum(len(v.approaches) for v in visits)} holds")
    print(
        f"Estimated time: {(travel + holds) / 60:.1f} min ({travel / 60:.1f} moving, {holds / 60:.1f} holding)"
    )
    print(
        f"{'#':>3} "
        + " ".join(f"{k[:11]:>11}" for k in ARM)
        + "   CAD gravity Nm (lift elbow wflex)  via hub"
    )
    for i, v in enumerate(visits):
        tau = model.torque(v.pose)
        print(
            f"{i:3d} "
            + " ".join(f"{v.pose[k]:11.1f}" for k in ARM)
            + f"   {tau['shoulder_lift']:6.2f} {tau['elbow_flex']:6.2f} {tau['wrist_flex']:6.2f}"
            + f"   {'yes' if any(v.via) else ''}"
        )
    worst = max(max(abs(t) for t in model.torque(v.pose).values()) for v in visits)
    print(f"Highest CAD-predicted holding torque: {worst:.2f} Nm (cap {args.max_torque:.1f})")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robot", default="viscous_01", help="robot record name")
    p.add_argument("--urdf", type=Path, default=DEFAULT_MODEL_DIR / "viscous_arm.urdf")
    p.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR, help="collision mesh root")
    p.add_argument("--condition", default="air", choices=("air", "water"))
    p.add_argument("--poses", type=int, default=20)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--max-torque", type=float, default=5.0, help="CAD-predicted cap per joint, N m")
    p.add_argument("--floor-z", type=float, default=0.03, help="clearance above the base plate underside, m")
    p.add_argument("--speed", type=float, default=15.0, help="travel speed, deg/s")
    p.add_argument("--approach-speed", type=float, default=6.0, help="final approach speed, deg/s")
    p.add_argument("--settle", type=float, default=1.5)
    p.add_argument("--sample", type=float, default=1.0)
    p.add_argument(
        "--max-temp", type=float, default=82.0, help="abort and return home at this winding temp, C"
    )
    p.add_argument(
        "--max-lag", type=float, default=25.0, help="abort if a joint lags its command this far, deg"
    )
    p.add_argument("--cool-at", type=float, default=80.0, help="park at the cooling pose at this temp, C")
    p.add_argument("--cool-to", type=float, default=60.0, help="resume once every winding is below this, C")
    p.add_argument("--max-cool", type=float, default=900.0, help="abort if cooling takes longer, s")
    p.add_argument("--start-at", type=int, default=0, help="resume the plan at this pose index")
    p.add_argument("--stop-test", action="store_true", help="wait for a stop signal; never opens hardware")
    p.add_argument("--dry-run", action="store_true", help="plan and print; never enables torque")
    p.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    p.add_argument("--out", type=Path)
    args = p.parse_args()
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    if args.stop_test:
        print(f"Stop test: pid {os.getpid()} waiting for SIGINT or SIGTERM", flush=True)
        try:
            time.sleep(120)
            print("Stop test FAILED: no signal arrived within 120 s")
        except KeyboardInterrupt:
            print("Stop test passed: signal received", flush=True)
        return

    record = load_record(args.robot)
    robot = build_robot(record)
    start = read_pose_readonly(robot)
    print("Start pose (read-only):", {k: round(v, 1) for k, v in start.items()})
    model, is_clear, start_arm, hub, cool, visits, home_via = plan(args, robot, start)
    if not 0 <= args.start_at < len(visits):
        raise SystemExit(f"--start-at must be in 0..{len(visits) - 1}")
    describe(model, start_arm, visits, home_via, args)
    if args.dry_run:
        return

    if not args.yes and input("Energize the arm and run this plan? Type 'go': ").strip() != "go":
        return
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = args.out or REPO / f"configs/viscous/{args.robot}/evidence/gravity-{args.condition}-{stamp}.json"
    out.parent.mkdir(parents=True, exist_ok=True)

    robot = build_robot(record)
    robot.connect(calibrate=False)
    gripper = min(
        robot.config.joint_limits["gripper"][1],
        max(robot.config.joint_limits["gripper"][0], start["gripper"]),
    )
    run = Runner(robot, args, gripper)
    data = {
        "robot": args.robot,
        "follower_config": record["follower_config"],
        "condition": args.condition,
        "started": stamp,
        "urdf": stored_path(args.urdf),
        "urdf_sha256": model.urdf_sha256,
        "gains": {k: list(v) for k, v in robot._resolved_gains.items()},
        "start": start,
        "start_at": args.start_at,
        "samples": [],
        "cooling": [],
        "outcome": "running",
    }
    write_json(out, data)
    print(f"Recording to {out}")

    def route(a, b):
        if path_clear(a, b, is_clear):
            return [b]
        if path_clear(a, hub, is_clear) and path_clear(hub, b, is_clear):
            return [hub, b]
        return None

    def go(target):
        legs = route(run.command, target)
        if legs is None:
            raise RunAbortError(f"no clear path from {run.command} to {target}")
        for waypoint in legs:
            run.move(waypoint, args.speed)

    def cool_down():
        hottest = max(run.last_obs[f"{k}.temp"] for k in (*ARM, "gripper"))
        print(
            f"Hottest winding {hottest:.0f} C: parking at the cooling pose until under {args.cool_to:.0f} C"
        )
        go(cool)
        began, last_print = time.monotonic(), 0.0
        while True:
            run.hold(2.0)
            temps = {k: run.last_obs[f"{k}.temp"] for k in (*ARM, "gripper")}
            hottest = max(temps.values())
            waited = time.monotonic() - began
            if hottest < args.cool_to:
                break
            if waited > args.max_cool:
                raise RunAbortError(f"still {hottest:.0f} C after {waited:.0f} s of cooling")
            if waited - last_print >= 15:
                print(f"  cooling {waited:4.0f} s, hottest {max(temps, key=temps.get)} {hottest:.0f} C")
                last_print = waited
        data["cooling"].append({"after_sample": len(data["samples"]), "seconds": waited, "end_c": hottest})
        print(f"Cooled in {waited:.0f} s; continuing")

    try:
        try:
            first = robot.get_observation()
            run.last_obs = first
            run.command = clipped({k: first[f"{k}.pos"] for k in ARM}, robot.config.joint_limits)
            run.move(start_arm, args.approach_speed)
            for i, visit in enumerate(visits[args.start_at :], start=args.start_at):
                for side, approach in enumerate(visit.approaches):
                    go(approach)
                    run.move(visit.pose, args.approach_speed)
                    run.hold(args.settle)
                    frames = run.hold(args.sample)
                    pos, pos_std = summarize(frames, "pos")
                    torque, torque_std = summarize(frames, "torque")
                    temps = {k: max(f[f"{k}.temp"] for f in frames) for k in (*ARM, "gripper")}
                    data["samples"].append(
                        {
                            "pose": i,
                            "approach": "above" if side == 0 else "below",
                            "command_deg": dict(run.command),
                            "position_deg": pos,
                            "position_std_deg": pos_std,
                            "torque_nm": torque,
                            "torque_std_nm": torque_std,
                            "temp_max_c": temps,
                        }
                    )
                    write_json(out, data)
                    print(
                        f"pose {i:2d} {data['samples'][-1]['approach']:5s}  "
                        + "  ".join(
                            f"{k[:5]} {torque[k]:+5.2f}/{model.torque(pos)[k]:+5.2f}" for k in ARM[1:4]
                        )
                        + f"  (measured/CAD N m)  hottest {max(temps.values()):.0f} C"
                    )
                    if max(temps.values()) >= args.cool_at:
                        cool_down()
            data["outcome"] = "complete"
        except KeyboardInterrupt:
            data["outcome"] = "interrupted"
            print(
                "\nInterrupted: returning to the start pose. Ctrl-C again releases torque where the arm is."
            )
        except RunAbortError as e:
            data["outcome"] = f"aborted: {e}"
            print(f"\nAborted: {e}. Returning to the start pose.")
        write_json(out, data)
        # Looser limits on the way home: abandoning the return drops the arm.
        args.max_lag = max(args.max_lag, 40.0)
        # Still under the driver's own 85 C stop, which drops the arm.
        args.max_temp = max(args.max_temp, 84.0)
        try:
            go(start_arm)
            run.hold(1.0)
        except RunAbortError as e:
            print(f"Could not finish the return ({e}); releasing torque where the arm is.")
    finally:
        robot.disconnect()
        print("Torque released.")
    print(f"Saved {len(data['samples'])} samples to {out}")


if __name__ == "__main__":
    main()
