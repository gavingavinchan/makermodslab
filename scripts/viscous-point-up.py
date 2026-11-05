"""Move a single Viscous follower slowly to point straight up, hold, and bring it back.

Straight up is solved on the vendored URDF (configs/viscous/model): with
shoulder_lift -90 the upper arm is vertical, and because the forearm and the
wrist are cranked (~23 deg) the elbow and wrist have to bend back to put the
wrist, the roll joint and tool0 on the vertical line over the shoulder:
elbow_flex 157, wrist_flex 32, wrist_yaw 0 (all within 1 mm). Pan, roll and the
gripper stay where they are. Near-balanced there (CAD gravity ~0.1 N m).

The move is a joint-space line at ``--speed`` deg/s on the joint that travels
furthest, collision-checked on the CAD model before anything is energized.
Trips (the bench Runner's): a joint lagging its command by ``--max-lag`` beyond
the CAD sag it expects at stiffness alone, or a winding at ``--max-temp``.

Stop: SIGINT/SIGTERM (or ``--hold-s`` running out) returns the arm along the
same line to its start pose at the same speed, then releases torque. A second
signal releases torque WHERE THE ARM IS, which drops it. The power switch is
the hard stop. ``--stop-test`` proves the signal path without opening hardware.

The port is passed explicitly rather than read from the robot record, because
records on another machine can point at the wrong adapter.
"""

import argparse
import math
import os
import signal
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO))

from viscous_bench import ARM, RunAbortError, Runner, _interrupt, clipped, read_pose_readonly  # noqa: E402

from makermodslab.arm_gravity import CollisionChecker, GravityModel, interpolate  # noqa: E402

MODEL_DIR = REPO / "configs/viscous/model"
UP = {"shoulder_lift": -90.0, "elbow_flex": 157.0, "wrist_flex": 32.0, "wrist_yaw": 0.0}
GRAVITY_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
# --exercise: each actuator in turn, out and back from the upright pose (deg).
# The gripper goes one way only, more open (its limits are negative-open).
EXERCISE = {
    "shoulder_pan": 15.0,
    "shoulder_lift": 8.0,
    "elbow_flex": 10.0,
    "wrist_flex": 15.0,
    "wrist_yaw": 15.0,
    "wrist_roll": 30.0,
    "gripper": -35.0,
}
# A joint passes when the encoder swing is at least this share of the commanded swing.
FOLLOW_SHARE = 0.7


def build(port, calibration):
    from lerobot.robots import make_robot_from_config

    from makermodslab.arms import registry

    return make_robot_from_config(registry.get("viscous").single_follower_config(port, calibration))


def settled(run, key, seconds=0.6):
    frames = run.hold(seconds)
    return sum(f[key] for f in frames[-5:]) / len(frames[-5:])


def exercise(run, top, gripper, limits, speed):
    """Each actuator out and back; report commanded vs encoder swing per joint."""
    results = {}
    for joint, amp in EXERCISE.items():
        key = f"{joint}.pos"
        if joint == "gripper":
            print(f"PHASE exercise gripper: opening {abs(amp):.0f} deg and back", flush=True)
            before = settled(run, key)
            opened = min(limits["gripper"][1], max(limits["gripper"][0], gripper + amp))
            for value in (opened, gripper):
                start, steps = run.gripper, max(1, math.ceil(abs(value - run.gripper) / (speed * 0.02)))
                for i in range(1, steps + 1):
                    run.gripper = start + (value - start) * i / steps
                    run._tick(run.command)
                if value == opened:
                    after = settled(run, key)
            commanded, measured = opened - gripper, after - before
        else:
            print(f"PHASE exercise {joint}: +{amp:.0f} then -{amp:.0f} deg and back", flush=True)
            reads = {}
            for sign in (1.0, -1.0):
                run.move({**top, joint: top[joint] + sign * amp}, speed)
                reads[sign] = settled(run, key)
            run.move(top, speed)
            commanded, measured = 2 * amp, reads[1.0] - reads[-1.0]
        ok = abs(measured) >= FOLLOW_SHARE * abs(commanded) and measured * commanded > 0
        results[joint] = ok
        print(
            f"RESULT {joint}: commanded {commanded:+.1f} deg, encoder {measured:+.1f} deg -> "
            f"{'OK' if ok else 'DID NOT FOLLOW'}",
            flush=True,
        )
    bad = [k for k, ok in results.items() if not ok]
    print(f"RESULT all actuators: {'OK' if not bad else 'FAILED ' + ', '.join(bad)}", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True)
    p.add_argument("--calibration", default="viscous_waterproof_01")
    p.add_argument("--speed", type=float, default=5.0, help="deg/s on the joint that moves furthest")
    p.add_argument("--hold-s", type=float, default=600.0, help="return home after this long at the top")
    p.add_argument("--max-lag", type=float, default=10.0, help="deg beyond the expected CAD sag")
    p.add_argument("--max-temp", type=float, default=82.0)
    p.add_argument("--floor-z", type=float, default=0.03)
    p.add_argument("--exercise", action="store_true", help="at the top, move every actuator out and back")
    p.add_argument("--dry-run", action="store_true", help="read the pose and check the path; no torque")
    p.add_argument("--stop-test", action="store_true", help="wait for a stop signal; never opens hardware")
    p.add_argument("--pid-file", type=Path, help="write this process's pid here (setsid's $! is not it)")
    args = p.parse_args()
    if args.pid_file:
        args.pid_file.write_text(f"{os.getpid()}\n")
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)

    if args.stop_test:
        print("Stop test: waiting for SIGINT/SIGTERM.", flush=True)
        try:
            while True:
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("Stop signal received: a real run would return home now.", flush=True)
        return

    robot = build(args.port, args.calibration)
    limits = robot.config.joint_limits
    start_full = read_pose_readonly(robot)
    start = clipped({k: start_full[k] for k in ARM}, limits)
    target = clipped({**start, **UP}, limits)
    print("Start (torque off):", {k: round(v, 1) for k, v in start_full.items()}, flush=True)
    print("Target:", {k: round(v, 1) for k, v in target.items()}, flush=True)

    urdf = MODEL_DIR / "viscous_arm.urdf"
    checker = CollisionChecker(urdf, MODEL_DIR, ARM, floor_z=args.floor_z)
    model = GravityModel(urdf, GRAVITY_JOINTS)
    worst = {k: 0.0 for k in GRAVITY_JOINTS}
    for pose in [start, *interpolate(start, target, 2.0)]:
        if problems := checker.problems(pose):
            raise SystemExit(f"Path collides at {pose}: {problems}")
        for k, v in model.torque({k: pose[k] for k in GRAVITY_JOINTS}).items():
            worst[k] = max(worst[k], abs(v))
    if args.exercise:
        for joint, amp in EXERCISE.items():
            if joint == "gripper":
                continue
            for sign in (1.0, -1.0):
                pose = clipped({**target, joint: target[joint] + sign * amp}, limits)
                if pose[joint] != target[joint] + sign * amp:
                    raise SystemExit(f"Exercise of {joint} by {sign * amp:+.0f} deg leaves its soft limits")
                for p in interpolate(target, pose, 2.0):
                    if problems := checker.problems(p):
                        raise SystemExit(f"Exercise of {joint} collides at {p}: {problems}")
        print("Exercise poses clear on the CAD model and inside the soft limits.", flush=True)
    span = max(abs(target[k] - start[k]) for k in ARM)
    print(
        f"Path clear on the CAD model. Peak CAD gravity {({k: round(v, 1) for k, v in worst.items()})} N m. "
        f"{span:.0f} deg at {args.speed:.0f} deg/s = {span / args.speed:.0f} s each way.",
        flush=True,
    )
    if args.dry_run:
        return

    robot = build(args.port, args.calibration)
    robot.connect(calibrate=False)
    gripper = min(limits["gripper"][1], max(limits["gripper"][0], start_full["gripper"]))
    run = Runner(robot, args, gripper)
    kp = {k: robot._resolved_gains[k][0] for k in ARM}

    def expected_lag(obs):
        tau = model.torque({k: obs[f"{k}.pos"] for k in GRAVITY_JOINTS})
        return {k: math.degrees(tau[k] / kp[k]) for k in GRAVITY_JOINTS}

    run.expected_lag = expected_lag
    last_print = [0.0]

    def on_tick(cmd, obs):
        now = time.monotonic()
        if now - last_print[0] >= 2.0:
            last_print[0] = now
            pos = "  ".join(f"{k[:8]} {obs[f'{k}.pos']:+6.1f}" for k in ("shoulder_lift", "elbow_flex", "wrist_flex"))
            hot = max(obs[f"{k}.temp"] for k in (*ARM, "gripper"))
            print(f"  {pos}   hottest {hot:.0f} C", flush=True)

    run.on_tick = on_tick
    try:
        try:
            first = robot.get_observation()
            run.command = clipped({k: first[f"{k}.pos"] for k in ARM}, limits)
            print("PHASE energized: holding the start pose 1 s", flush=True)
            run.hold(1.0)
            print("PHASE moving up", flush=True)
            run.move(target, args.speed)
            if args.exercise:
                exercise(run, target, gripper, limits, args.speed)
            print(f"PHASE at the top: holding up to {args.hold_s:.0f} s; stop signal brings it home", flush=True)
            run.hold(args.hold_s)
            print("PHASE hold time up", flush=True)
        except KeyboardInterrupt:
            print("PHASE stop signal: returning home. A second signal releases torque where the arm is.", flush=True)
        except RunAbortError as e:
            print(f"PHASE aborted: {e}. Returning home.", flush=True)
        # Looser trips on the way home: abandoning the return drops the arm.
        args.max_lag = max(args.max_lag, 25.0)
        args.max_temp = max(args.max_temp, 84.0)
        try:
            if run.command is None:
                raise RunAbortError("stopped before the first reading")
            print("PHASE returning home", flush=True)
            run.move(start, args.speed)
            run.hold(1.0)
            print("PHASE home", flush=True)
        except RunAbortError as e:
            print(f"PHASE could not finish the return ({e}); releasing torque where the arm is.", flush=True)
    finally:
        robot.disconnect()
        print("PHASE torque released", flush=True)


if __name__ == "__main__":
    main()
