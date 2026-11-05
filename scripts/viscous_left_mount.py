#!/usr/bin/env python3
"""Unit-1 left-mount preparation. Default/dry-run is entirely offline.

--inspect / --execute are FUTURE operator-only hardware operations; never use
them during the offline investigation. No follower/leader class is connected.
Successful execution stages a release, never installs it into Lab.
"""

import argparse
import copy
import datetime as dt
import hashlib
import json
import math
import os
import signal
import statistics
import subprocess
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BUNDLE = REPO / "configs/viscous/viscous_01"
J1 = "shoulder_pan"
EXPECTED = 90.0  # Hypothesis: identical arm indexed +90 deg CCW at its fixed base.
TOLERANCE = 10.0
OLD_LIMITS = json.loads((BUNDLE / "evidence/commissioning/joint-limits-zeroed-frame.json").read_text())
STAR_J1 = json.loads((BUNDLE / "evidence/commissioning/star-ranges-2026-08-29.json").read_text())["joints"][0]
DIRECTIONS = dict(
    zip(OLD_LIMITS, (1.439742, -0.868697, -0.915867, -1.082952, -1.260963, 0.945426, -1.778452), strict=True)
)


def check_before(samples):
    """Reject rather than silently selecting an encoder whole-turn branch."""
    if len(samples) < 3 or not all(math.isfinite(v) for v in samples):
        raise ValueError("Need at least three finite, fresh J1 readings")
    if max(samples) - min(samples) > 1.0:
        raise ValueError("J1 moved more than 1 degree; support and re-pose it")
    value = statistics.median(samples)
    if not OLD_LIMITS[J1][0] <= value <= OLD_LIMITS[J1][1] or abs(value - EXPECTED) > TOLERANCE:
        raise ValueError(
            f"ABORT: J1 {value:+.2f} deg is not +90 +/-10 in the original unit-1 branch; no auto-wrap"
        )
    return value


def profiles(shift):
    """Actual pre-zero reading, not the nominal quarter turn, translates limits."""
    check_before([shift] * 3)
    limits = copy.deepcopy(OLD_LIMITS)
    limits[J1] = [v - shift for v in limits[J1]]
    ranges = {j: [math.ceil(lo), math.floor(hi)] for j, (lo, hi) in limits.items()}
    # Driver centres its raw +/-180 unwrap window on output midpoint/gain.
    # Translating the entire follower range would wrap raw +110.3 to -249.7!
    # Intersect with actual Star reach, keeping its whole measured raw travel
    # in one unwrap window, with no change to gain/neutral or shared driver.
    ranges[J1][0] = math.ceil(max(limits[J1][0], STAR_J1["min_deg"] * DIRECTIONS[J1]))
    ranges[J1][1] = math.floor(min(limits[J1][1], STAR_J1["max_deg"] * DIRECTIONS[J1]))
    center = sum(ranges[J1]) / (2 * DIRECTIONS[J1])
    if not center - 180 < STAR_J1["min_deg"] < STAR_J1["max_deg"] < center + 180:
        raise ValueError("Prepared leader J1 range cannot unwrap its measured travel safely")
    return {
        "follower": {"joint_limits": limits},
        "leader": {
            "joint_directions": DIRECTIONS,
            "joint_ranges": ranges,
            # Leader neutral stays raw zero. Subtracting shift here would drive
            # the NEW follower away from its new zero. Other joints stay intact.
            "joint_offsets": {j: -33.0 if j == "gripper" else 0.0 for j in limits},
            "max_start_offset_deg": 30.0,
        },
    }


def table_degrees(new_degrees, measured_shift):
    """Single-arm URDF/gravity input after a verified re-zero (same handed arm)."""
    return {**new_degrees, J1: new_degrees[J1] + measured_shift}


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w") as handle:
        handle.write(json.dumps(value, indent=2, allow_nan=False) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)
    directory = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def stage_release(destination, evidence):
    """No profile is produced unless zero and final release were verified."""
    if not evidence.get("activation_ready") or evidence.get("unit") != "viscous_01":
        raise ValueError("Unverified re-zero: refuse to stage profiles")
    if not evidence.get("zero_write_attempted") or evidence.get("final_torque_disabled") is not True:
        raise ValueError("Missing zero/release evidence")
    if evidence.get("physical_mount_verified_by_operator") is not True:
        raise ValueError("Missing physical mount confirmation")
    after = evidence["readings_after_deg"]
    if len(after) < 3 or not all(math.isfinite(v) and abs(v) <= 1.0 for v in after):
        raise ValueError("J1 zero verification failed")
    shift = check_before(evidence["readings_before_deg"])
    profile = profiles(shift)
    profile["mount_frame"] = {
        "unit": "viscous_01",
        "side": "left",
        "old_equals_new_plus_deg": shift,
        # Keep provenance self-contained when these two files are copied to
        # Lab; a staging-relative receipt path would become a broken link.
        "zero_evidence": evidence,
        "physical_mount_verified_by_operator": True,
    }
    directory = destination / "viscous_profiles"
    directory.mkdir()  # Refuse overwriting an existing release.
    for name in ("viscous_01", "star_viscous"):
        write_json(directory / f"{name}.json", profile)
    calibration = json.loads((BUNDLE / "calibration/robots/viscous_follower/viscous_01.json").read_text())
    lo, hi = profile["follower"]["joint_limits"][J1]
    calibration[J1]["range_min"], calibration[J1]["range_max"] = math.ceil(lo), math.floor(hi)
    write_json(destination / "viscous_01-calibration-new-frame.json", calibration)


def fresh(bus):
    values = []
    for _ in range(5):
        bus.last_feedback_time[J1] = None
        values.append(float(bus.read("Present_Position", J1)))
        time.sleep(0.05)
    return values


def interrupted(signum, frame):
    raise KeyboardInterrupt(f"Signal {signum}; release J1 and leave Lab stopped")


def operate(args):
    # These imports are intentionally behind the explicit hardware modes.
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.robstride import RobstrideMotorsBus

    if not args.port or "207E37A55445" not in args.port or not args.port.startswith("/dev/serial/by-id/"):
        raise ValueError("Use unit 1's explicit enrolled CANable by-id path containing 207E37A55445")
    state = subprocess.run(
        ["systemctl", "--user", "is-active", "makermodslab-viscous"], capture_output=True, text=True
    )
    if state.stdout.strip() != "inactive":
        raise ValueError("Lab must be stopped and inactive; this script will not stop it")
    args.output.mkdir(parents=True, exist_ok=False)
    receipt = args.output / "j1-rezero.json"
    evidence = {
        "unit": "viscous_01",
        "date_utc": dt.datetime.now(dt.UTC).isoformat(),
        "joint": J1,
        "motor_id": 1,
        "mode": "execute" if args.execute else "inspect",
        "reason": "Dry arm moved to LEFT ROV mount; operator confirms neutral and encoder branch",
        "expected_old_frame_deg": EXPECTED,
        "tolerance_deg": TOLERANCE,
        "method": "Motor 1 only: connect(handshake=False), disable, fresh reads, SET_ZERO 0xFE, verify, disable",
        "zero_write_attempted": False,
        "activation_ready": False,
        "calibration_sha256": hashlib.sha256(
            (BUNDLE / "calibration/robots/viscous_follower/viscous_01.json").read_bytes()
        ).hexdigest(),
    }
    write_json(receipt, evidence)
    motor = Motor(1, "O0", MotorNormMode.DEGREES)
    motor.recv_id = 1
    motor.motor_type_str = "O0"
    bus = RobstrideMotorsBus(
        args.port, {J1: motor}, can_interface="slcan", use_can_fd=False, bitrate=1000000, data_bitrate=None
    )
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    try:
        bus.connect(handshake=False)
        bus.disable_torque([J1])
        evidence["readings_before_deg"] = fresh(bus)
        shift = check_before(evidence["readings_before_deg"])
        evidence["reading_before_old_frame_deg"] = shift
        write_json(receipt, evidence)
        print(
            f"J1 old frame: {shift:+.2f} deg; proposed new soft limits: {profiles(shift)['follower']['joint_limits'][J1]}"
        )
        if args.execute:
            print(
                "Support arm. Verify physical LEFT neutral and leader J1 neutral by eye. No Star port is opened."
            )
            if input("Type LEFT J1 NEUTRAL VERIFIED to persist motor-1 zero: ") != "LEFT J1 NEUTRAL VERIFIED":
                raise ValueError("Operator did not confirm physical neutral")
            evidence["physical_mount_verified_by_operator"] = True
            # Re-read after the human pause. Any movement invalidates the earlier evidence.
            latest = fresh(bus)
            latest_shift = check_before(latest)
            if abs(latest_shift - shift) > 1:
                raise ValueError("Pose moved while confirming; abort and repeat inspection")
            evidence["readings_before_deg"] = latest
            evidence["reading_before_old_frame_deg"] = latest_shift
            evidence["zero_write_attempted"] = True  # Durable before send; lost ack is ambiguous.
            write_json(receipt, evidence)
            bus.set_zero_position([J1])
            bus.disable_torque([J1])
            evidence["readings_after_deg"] = fresh(bus)
            if not all(math.isfinite(v) and abs(v) <= 1 for v in evidence["readings_after_deg"]):
                raise ValueError(
                    "SET_ZERO may have occurred but verification failed; DO NOT repeat or start Lab"
                )
    except BaseException as error:
        evidence["error"] = str(error) or type(error).__name__
        raise
    finally:
        try:
            if bus.is_connected:
                try:
                    bus.disable_torque([J1])
                    evidence["final_torque_disabled"] = True
                finally:
                    bus.disconnect(disable_torque=False)
        except BaseException as error:
            evidence["final_torque_disabled"] = False
            evidence["cleanup_error"] = str(error)
            raise
        finally:
            write_json(receipt, evidence)
    if args.execute:
        evidence["activation_ready"] = True
        write_json(receipt, evidence)
        stage_release(args.output, evidence)
        print(
            f"Verified zero; release staged in {args.output}. Lab remains stopped; install BOTH profiles before starting it."
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="Offline plan only (also the default)")
    mode.add_argument("--inspect", action="store_true", help="FUTURE: motor-1 disable/read only, no SET_ZERO")
    mode.add_argument(
        "--execute", action="store_true", help="FUTURE: operator-confirmed persistent motor-1 zero"
    )
    parser.add_argument("--port")
    parser.add_argument("--output", type=Path, default=BUNDLE / "evidence/left-mount-2026-11-05/operator-run")
    args = parser.parse_args()
    if args.inspect or args.execute:
        operate(args)
    else:
        print(
            json.dumps(
                {
                    "offline": True,
                    "hardware_opened": False,
                    "expected_reading_deg": EXPECTED,
                    "prediction_status": "conditional on physical +90deg base indexing; CAD does not prove it",
                    "candidate_only_not_installed": profiles(EXPECTED),
                },
                indent=2,
            )
        )


if __name__ == "__main__":
    main()
