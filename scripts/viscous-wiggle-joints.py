"""Wiggle chosen Viscous follower joints one at a time and check each one followed.

The Lab's CAN gripper wiggle (makermodslab/can_wiggle.py), widened to any
joint: a bus carrying ONLY that one motor, opened without the handshake,
captures the motor's position, steps +stroke, -stroke and back, reads the
encoder at each end, and always de-energizes it through de_energize_can_bus.
Every other motor stays off, so run it with the arm resting somewhere it can
stay limp (folded on its stops).

Each joint passes when the encoder swing is at least 70 % of the commanded
swing. ``--one-way JOINT`` strokes that joint only in the + direction (use it
for a joint parked near a physical stop, like J1 near its negative stop).
"""

import argparse
import math
import signal
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from makermodslab.torque import de_energize_can_bus  # noqa: E402

FOLLOW_SHARE = 0.7


def single_motor_bus(robot, joint):
    from lerobot.motors.robstride import RobstrideMotorsBus

    return RobstrideMotorsBus(
        port=robot.config.port,
        motors={joint: robot.bus.motors[joint]},
        calibration={joint: robot.calibration[joint]} if joint in (robot.calibration or {}) else None,
        can_interface=robot.config.can_interface,
        use_can_fd=False,
        bitrate=robot.config.can_bitrate,
        data_bitrate=None,
    )


def settle(bus, joint, seconds=0.6):
    time.sleep(seconds)
    reads = []
    for _ in range(5):
        reads.append(float(bus.read("Present_Position", joint)))
        time.sleep(0.02)
    return sum(reads) / len(reads)


def stroke_ends(rest, limits, stroke, one_way):
    """Both ends clipped to the soft limits (in the motor's raw turn), like plan_can_wiggle."""
    lo, hi = limits
    turn = 360.0 * math.floor((rest - lo) / 360.0)  # raw reading = limit frame + whole turns
    if not lo <= rest - turn <= hi:
        raise ValueError(f"start {rest:.1f} deg is outside the soft limits {limits}")
    lo, hi = lo + turn, hi + turn
    ends = [min(rest + stroke, hi), max(rest - stroke, lo)]
    if one_way:
        ends = [max(ends, key=lambda e: abs(e - rest))]
    return ends


def wiggle(robot, joint, stroke, speed, one_way):
    bus = single_motor_bus(robot, joint)
    kp, kd = robot.config.gains[joint]
    try:
        bus.connect(handshake=False)
        rest = float(bus.read("Present_Position", joint))
        ends = stroke_ends(rest, robot.config.joint_limits[joint], stroke, one_way)
        bus.write("Kp", joint, kp)
        bus.write("Kd", joint, kd)
        goal = [rest]

        def move(target):
            start = goal[0]
            steps = max(1, math.ceil(abs(target - start) / (speed * 0.05)))
            for i in range(1, steps + 1):
                goal[0] = start + (target - start) * i / steps
                bus.write("Goal_Position", joint, goal[0])
                time.sleep(0.05)

        reads = {}
        try:
            bus.enable_torque(joint)
            bus.write("Goal_Position", joint, rest)
            base = settle(bus, joint, 0.3)
            for end in ends:
                move(end)
                reads[end] = settle(bus, joint)
        finally:
            move(rest)
            back = settle(bus, joint)
    finally:
        problems = de_energize_can_bus(bus, f"{joint} wiggle")
        if problems:
            print(f"  WARNING {joint}: {' '.join(problems)}", flush=True)
    if len(ends) == 1:
        commanded, measured = ends[0] - rest, reads[ends[0]] - base
    else:
        commanded, measured = ends[0] - ends[1], reads[ends[0]] - reads[ends[1]]
    ok = abs(measured) >= FOLLOW_SHARE * abs(commanded) and measured * commanded > 0
    print(
        f"RESULT {joint}: commanded {commanded:+.1f} deg, encoder {measured:+.1f} deg, "
        f"back within {back - rest:+.1f} deg of start -> {'OK' if ok else 'DID NOT FOLLOW'}",
        flush=True,
    )
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True)
    p.add_argument("--calibration", default="viscous_waterproof_01")
    p.add_argument("--joints", nargs="+", required=True)
    p.add_argument("--stroke", type=float, default=10.0, help="deg each way (the Lab gripper wiggle's)")
    p.add_argument("--speed", type=float, default=5.0, help="deg/s")
    p.add_argument("--one-way", nargs="*", default=[], help="joints stroked + only")
    args = p.parse_args()
    for sig in (signal.SIGINT, signal.SIGTERM):
        # A detached run starts with SIGINT ignored; make both unwind through the finally blocks.
        signal.signal(sig, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    from lerobot.robots import make_robot_from_config

    from makermodslab.arms import registry

    robot = make_robot_from_config(
        registry.get("viscous").single_follower_config(args.port, args.calibration)
    )
    results = {}
    for joint in args.joints:
        print(f"PHASE wiggling {joint} (only this motor energized)", flush=True)
        try:
            results[joint] = wiggle(robot, joint, args.stroke, args.speed, joint in args.one_way)
        except KeyboardInterrupt:
            print("PHASE stopped by signal (motor de-energized)", flush=True)
            break
        except Exception as e:  # report and carry on: one bad joint must not hide the rest
            print(f"RESULT {joint}: ERROR {e}", flush=True)
            results[joint] = False
    bad = [k for k, ok in results.items() if not ok]
    print(f"RESULT all: {'OK' if not bad else 'FAILED ' + ', '.join(bad)}", flush=True)


if __name__ == "__main__":
    main()
