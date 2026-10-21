#!/usr/bin/env python3
"""Check the Viscous follower's contact-torque signal and its gravity feedforward on the arm.

Haptics goal A1 (a clean contact signal) and A1b (feedforward), in one powered run:

1. **Feedforward A/B.** At three moderate poses, hold with feedforward at 0 and
   at 100 %, each approached from above and below. Sag is command minus
   position; with an accurate model it should fall from ~torque/kp to ~0.
2. **Free motion.** Every move is logged at 50 Hz. Contact torque (reported
   minus fitted gravity, ``makermodslab.contact_torque``) should stay near the
   friction + model-error floor (~0.1-0.3 N m) while nothing touches the arm.
3. **Push test.** The arm holds one pose with feedforward on and prints contact
   torque live while someone pushes on it by hand: a push should stand clear of
   the floor from step 2.

    viscous-contact-check.py --robot viscous_01 --dry-run     # plan only, reads the arm read-only
    viscous-contact-check.py --robot viscous_01               # energizes; asks first
    viscous-contact-check.py --analyze <run.json>             # re-print the summary

Stop: Ctrl-C, or SIGINT/SIGTERM to a detached run, returns the arm to its start
pose, then releases torque. A second signal releases torque WHERE THE ARM IS,
which drops it. A joint lagging its command by ``--max-lag`` (default 10 deg,
about 5 N m of push on the shoulder) also ends the run and returns home, so
push gently and let go when the log says it is returning.
"""

import argparse
import datetime
import json
import math
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
    GRAVITY_JOINTS,
    HUB,
    TICK_S,
    RunAbortError,
    Runner,
    _interrupt,
    build_robot,
    clipped,
    load_record,
    read_pose_readonly,
    write_json,
)
from viscous_contact_analysis import compensate, motion_arrays  # noqa: E402

from makermodslab import gravity_feedforward  # noqa: E402
from makermodslab.arm_gravity import CollisionChecker, path_clear, stored_path  # noqa: E402
from makermodslab.contact_torque import ContactEstimator  # noqa: E402

DEFAULT_FIT = "configs/viscous/{robot}/gravity-air-fit.json"
DEFAULT_MODEL_DIR = REPO / "configs/viscous/model"
# (shoulder_lift, elbow_flex, wrist_flex). Picked with the unit 1 air fit for
# 1-2 N m on the gravity joints, both signs, every joint under 3.5 N m also at
# the 0 % sag of both approaches, which is what limits the shoulder to ~1.5 N m.
POSES = {
    "reach": (-125.0, 210.0, 10.0),  # lift +1.4, elbow -0.9, wrist 0.0
    "forward": (-80.0, 130.0, -10.0),  # lift +1.4, elbow +2.1, wrist +1.3
    "back": (-105.0, 190.0, 70.0),  # lift -1.2, elbow -2.3, wrist -1.4
}
PUSH_POSE = "forward"
APPROACH_DEG = 6.0
SCALE_RAMP_S = 1.0
LIVE_PERIOD_S = 0.5


def pose_of(values, base):
    return {**base, **dict(zip(GRAVITY_JOINTS, values, strict=True))}


def plan(args, robot, start, model):
    """Every pose and leg, collision- and load-checked with the arm sagging as it would at 0 %."""
    checker = CollisionChecker(args.urdf, args.model_dir, ARM, floor_z=args.floor_z)
    limits = robot.config.joint_limits
    gains = robot.config.gains
    start_arm = clipped({k: start[k] for k in ARM}, limits)
    base = {"shoulder_pan": start_arm["shoulder_pan"], "wrist_yaw": 0.0, "wrist_roll": 0.0}
    hub, cool = {**base, **HUB}, {**base, **COOL}

    def sagged(pose):
        tau = model.torque(pose)
        return clipped({k: pose[k] - math.degrees(tau[k] / gains[k][0]) for k in ARM}, limits)

    def load(pose):
        return max(abs(v) for v in model.torque(pose).values())

    def is_clear(pose):
        return (
            checker.is_clear(pose)
            and checker.is_clear(sagged(pose))
            and load(pose) <= args.max_torque
            and load(sagged(pose)) <= args.max_torque
        )

    def geometry_clear(pose):
        return checker.is_clear(pose) and checker.is_clear(sagged(pose))

    def route(a, b, capped=True):
        # Uncapped legs (out of and back into the fold, and any return home)
        # still avoid collisions: leaving the fold carries ~4 N m on any route,
        # as every Lab session's unfold does, and getting home beats the cap.
        ok = is_clear if capped else geometry_clear
        if path_clear(a, b, ok):
            return [b]
        if path_clear(a, hub, ok) and path_clear(hub, b, ok):
            return [hub, b]
        return None

    poses = {name: pose_of(v, base) for name, v in POSES.items()}
    for name, pose in {"cooling": cool, **poses}.items():
        if not is_clear(pose):
            raise SystemExit(
                f"{name} pose is not clear or over {args.max_torque} N m: {checker.problems(pose)}"
            )
    approaches = {}
    for name, pose in poses.items():
        approaches[name] = []
        for sign in (1.0, -1.0):
            a = {**pose, **{k: pose[k] + sign * APPROACH_DEG for k in GRAVITY_JOINTS}}
            if not (is_clear(a) and path_clear(a, pose, is_clear)):
                raise SystemExit(f"{name} approach {'above' if sign > 0 else 'below'} is not clear")
            approaches[name].append(a)
    # The legs the run takes, in order: start -> cooling -> each pose's approaches -> push -> cooling -> start.
    stops = [start_arm, cool]
    for name, pose in poses.items():
        stops += [approaches[name][0], pose, approaches[name][1], pose] * 2
    stops += [poses[PUSH_POSE], cool, start_arm]
    for i, (a, b) in enumerate(zip(stops, stops[1:], strict=False)):
        if route(a, b, capped=0 < i < len(stops) - 2) is None:
            raise SystemExit(f"No clear path from {a} to {b}")
    return start_arm, cool, poses, approaches, route


def describe(model, robot, start, start_arm, cool, poses, args):
    gains = robot.config.gains
    print("Joints that move: shoulder_lift, elbow_flex, wrist_flex (all three, together).")
    moved = [k for k in ("wrist_yaw", "wrist_roll") if abs(start_arm[k]) > 0.5]
    print(
        f"  shoulder_pan holds at {start_arm['shoulder_pan']:.1f} deg; gripper holds at its start; "
        + (f"{' and '.join(moved)} move to 0 on the first leg." if moved else "wrist_yaw/roll hold at 0.")
    )
    print(
        f"{'pose':>9} {'lift':>7} {'elbow':>7} {'wflex':>7}   fitted N m (lift elbow wflex)   sag at 0 % (deg)"
    )
    for name, pose in {"cooling": cool, **poses}.items():
        tau = model.torque(pose)
        print(
            f"{name:>9} "
            + " ".join(f"{pose[k]:7.1f}" for k in GRAVITY_JOINTS)
            + "   "
            + " ".join(f"{tau[k]:+6.2f}" for k in GRAVITY_JOINTS)
            + "            "
            + " ".join(f"{math.degrees(tau[k] / gains[k][0]):5.1f}" for k in GRAVITY_JOINTS)
        )
    holds = len(poses) * 4 * (args.settle + args.sample + SCALE_RAMP_S)
    print(
        f"Feedforward A/B: {len(poses)} poses x (0 %, 100 %) x (above, below) = {len(poses) * 4} holds, "
        f"~{holds / 60:.1f} min holding"
    )
    print(f"Push test: {args.push_seconds:.0f} s at '{PUSH_POSE}' with feedforward at 100 %")
    print(
        f"Trips: lag > {args.max_lag:.0f} deg beyond expected sag, winding >= {args.max_temp:.0f} C. Start pose: {start_arm}"
    )


def ramp_scale(run, status, target):
    """Hold where the arm is while the feedforward scale ramps to ``target``."""
    begin = status["scale"]
    steps = max(1, round(SCALE_RAMP_S / TICK_S))
    for i in range(1, steps + 1):
        status["scale"] = begin + (target - begin) * i / steps
        run._tick(run.command)


def live_line(contact, lag):
    return "  ".join(f"{k[:5]} {contact[k]:+5.2f} Nm ({lag[k]:+4.1f} deg)" for k in GRAVITY_JOINTS)


def run_check(args, record, start, model, estimator):
    start_arm, cool, poses, approaches, route = plan(args, build_robot(record), start, model)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = args.out or REPO / f"configs/viscous/{args.robot}/evidence/contact-check-{stamp}.json"
    out.parent.mkdir(parents=True, exist_ok=True)

    robot = build_robot(record)
    robot.connect(calibrate=False)
    limits = robot.config.joint_limits
    gripper = min(limits["gripper"][1], max(limits["gripper"][0], start["gripper"]))
    status = gravity_feedforward.install(robot, model, scale=0.0)
    run = Runner(robot, args, gripper)
    columns = ["t", "phase", "scale"] + [f"{k}.{f}" for f in ("cmd", "pos", "torque", "ff") for k in ARM]
    columns.append("temp_max")
    data = {
        "robot": args.robot,
        "fit": stored_path(args.fit),
        "started": stamp,
        "gains": {k: list(v) for k, v in robot._resolved_gains.items()},
        "start": start,
        "holds": [],
        "push": None,
        "ticks": {"columns": columns, "rows": []},
        "outcome": "running",
    }
    phase = {"name": "start"}
    t0 = time.monotonic()

    def on_tick(target, obs):
        data["ticks"]["rows"].append(
            [round(time.monotonic() - t0, 4), phase["name"], round(status["scale"], 3)]
            + [round(target[k], 3) for k in ARM]
            + [round(obs[f"{k}.pos"], 3) for k in ARM]
            + [round(obs[f"{k}.torque"], 4) for k in ARM]
            + [round(status["feedforward_nm"].get(k, 0.0), 4) for k in ARM]
            + [max(obs[f"{k}.temp"] for k in (*ARM, "gripper"))]
        )

    run.on_tick = on_tick
    kp = {k: robot._resolved_gains[k][0] for k in ARM}

    def expected_lag(obs):
        # Sag the feedforward leaves: (1 - scale) x gravity / kp. Without this
        # the 0 % unfold (~4 N m on the elbow, ~8 deg at kp 30) tripped 10 deg.
        tau = model.torque({k: obs[f"{k}.pos"] for k in ARM})
        return {k: (1.0 - status["scale"]) * math.degrees(tau[k] / kp[k]) for k in GRAVITY_JOINTS}

    run.expected_lag = expected_lag

    def go(target, label, capped=True):
        phase["name"] = f"move:{label}"
        legs = route(run.command, target, capped)
        if legs is None:
            raise RunAbortError(f"no clear path to {label}")
        for waypoint in legs:
            run.move(waypoint, args.speed)

    print(f"Recording to {out}")
    try:
        try:
            first = robot.get_observation()
            run.last_obs = first
            run.command = clipped({k: first[f"{k}.pos"] for k in ARM}, limits)
            # Feedforward on before leaving the fold: stiffness alone cannot lift
            # the forearm off its stop (the driver's startup sync stalls), and
            # the unfold then runs without sag. Held past install's own 2 s ramp.
            phase["name"] = "ramp:fold"
            ramp_scale(run, status, 1.0)
            run.hold(gravity_feedforward.RAMP_S)
            phase["name"] = "move:start"
            run.move(start_arm, args.approach_speed)
            go(cool, "cooling", capped=False)
            for name, pose in poses.items():
                for scale in (0.0, 1.0):
                    for side, approach in zip(("above", "below"), approaches[name], strict=True):
                        go(approach, f"{name}-{side}")
                        phase["name"] = f"ramp:{name}"
                        ramp_scale(run, status, scale)
                        phase["name"] = f"approach:{name}"
                        run.move(pose, args.approach_speed)
                        phase["name"] = f"settle:{name}"
                        run.hold(args.settle)
                        phase["name"] = f"hold:{name}"
                        frames = run.hold(args.sample)
                        data["holds"].append(hold_summary(name, side, scale, run.command, frames, estimator))
                        h = data["holds"][-1]
                        print(
                            f"{name:>8} {side:5s} ff {100 * scale:3.0f} %  sag "
                            + " ".join(f"{h['sag_deg'][k]:+5.2f}" for k in GRAVITY_JOINTS)
                            + " deg   contact "
                            + " ".join(f"{h['contact_nm'][k]:+5.2f}" for k in GRAVITY_JOINTS)
                            + f" N m   hottest {h['temp_max_c']:.0f} C",
                            flush=True,
                        )
                        write_json(out, data)
            go(poses[PUSH_POSE], PUSH_POSE)
            phase["name"] = "push-settle"
            run.hold(args.settle)
            print(f"PUSH NOW: {args.push_seconds:.0f} s window. Live contact torque (and lag):", flush=True)
            phase["name"] = "push"
            began, last = time.monotonic(), 0.0
            while (elapsed := time.monotonic() - began) < args.push_seconds:
                run.hold(TICK_S)
                if elapsed - last >= LIVE_PERIOD_S:
                    last = elapsed
                    obs = run.last_obs
                    contact = estimator.from_observation(obs)
                    lag = {k: run.command[k] - obs[f"{k}.pos"] for k in GRAVITY_JOINTS}
                    print(f"  {elapsed:4.1f} s  {live_line(contact, lag)}", flush=True)
            data["push"] = {"pose": PUSH_POSE, "seconds": args.push_seconds}
            print("Push window closed: hands off, the arm is going home.", flush=True)
            data["outcome"] = "complete"
        except KeyboardInterrupt:
            data["outcome"] = "interrupted"
            print(
                "\nInterrupted: returning to the start pose. Ctrl-C again releases torque where the arm is."
            )
        except RunAbortError as e:
            data["outcome"] = f"aborted: {e}"
            print(f"\nAborted: {e}. Hands off: returning to the start pose.")
        write_json(out, data)
        # Looser limits on the way home: abandoning the return drops the arm.
        args.max_lag = max(args.max_lag, 25.0)
        args.max_temp = max(args.max_temp, 84.0)
        try:
            if run.command is None:
                raise RunAbortError("stopped before the first reading")
            # Route from the held command, not the measured pose: snapping the
            # command to a sagged reading drops the arm by its sag at once.
            go(cool, "cooling", capped=False)
            # Cooling is near-balanced, so the feedforward can ramp out there
            # without moving the arm; the last leg home runs on stiffness alone.
            phase["name"] = "ramp:cooling"
            ramp_scale(run, status, 0.0)
            go(start_arm, "home", capped=False)
            phase["name"] = "home"
            run.hold(1.0)
        except RunAbortError as e:
            print(f"Could not finish the return ({e}); releasing torque where the arm is.")
    finally:
        robot.disconnect()
        print("Torque released.")
        write_json(out, data)
    print(f"Saved to {out}")
    analyze(data, estimator)


def hold_summary(name, side, scale, command, frames, estimator):
    pos = {k: float(np.mean([f[f"{k}.pos"] for f in frames])) for k in ARM}
    torque = {k: float(np.mean([f[f"{k}.torque"] for f in frames])) for k in ARM}
    contact = [estimator.from_observation(f) for f in frames]
    return {
        "pose": name,
        "approach": side,
        "scale": scale,
        "command_deg": dict(command),
        "position_deg": pos,
        "sag_deg": {k: command[k] - pos[k] for k in ARM},
        "torque_nm": torque,
        "contact_nm": {k: float(np.mean([c[k] for c in contact])) for k in ARM},
        "temp_max_c": max(f[f"{k}.temp"] for f in frames for k in (*ARM, "gripper")),
    }


def analyze(data, estimator):
    gains = {k: v[0] for k, v in data["gains"].items()}
    # The saved contact_nm was computed with the collection-time fit. Recompute
    # it so --fit actually changes the entire analysis, including static holds.
    data = {
        **data,
        "holds": [
            {**h, "contact_nm": estimator.estimate(h["position_deg"], h["torque_nm"])} for h in data["holds"]
        ],
    }
    print(f"\n=== {data['robot']} contact check {data['started']}: {data['outcome']} ===")
    if data["holds"]:
        print("\nFeedforward A/B (mean of above/below; friction band = half their difference):")
        print(
            f"{'pose':>8} {'joint':>13} {'fitted':>7} {'sag@0%':>7} {'pred':>6} {'sag@100%':>8} "
            f"{'band':>5} {'contact@0':>9} {'@100':>6}"
        )
        for name in dict.fromkeys(h["pose"] for h in data["holds"]):
            holds = [h for h in data["holds"] if h["pose"] == name]
            pos = holds[0]["position_deg"]
            tau = estimator.model.torque(pos)
            tau = {k: tau[k] - estimator.model.offsets[i] for i, k in enumerate(estimator.joints)}
            for k in GRAVITY_JOINTS:
                by = {(h["scale"], h["approach"]): h for h in holds}

                def mean(scale, key, k=k, by=by):
                    vals = [by[(scale, s)][key][k] for s in ("above", "below") if (scale, s) in by]
                    return float(np.mean(vals)) if vals else math.nan

                band = [
                    by[(s, "above")]["sag_deg"][k] - by[(s, "below")]["sag_deg"][k]
                    for s in (0.0, 1.0)
                    if (s, "above") in by and (s, "below") in by
                ]
                print(
                    f"{name:>8} {k:>13} {tau[k]:+7.2f} {mean(0.0, 'sag_deg'):+7.2f} "
                    f"{math.degrees(tau[k] / gains[k]):+6.2f} {mean(1.0, 'sag_deg'):+8.2f} "
                    f"{np.mean(np.abs(band)) / 2 if band else math.nan:5.2f} "
                    f"{mean(0.0, 'contact_nm'):+9.2f} {mean(1.0, 'contact_nm'):+6.2f}"
                )
    if not data["ticks"]["rows"]:
        return
    phases, col, vel, moving = motion_arrays(data)
    t = col("t")
    pos = {k: col(f"{k}.pos") for k in ARM}
    torque = {k: col(f"{k}.torque") for k in ARM}
    contact = {k: np.empty(len(t)) for k in ARM}
    for i in range(len(t)):
        c = estimator.estimate({k: pos[k][i] for k in ARM}, {k: torque[k][i] for k in ARM})
        for k in ARM:
            contact[k][i] = c[k]
    stiff_ff = {k: gains[k] * np.radians(col(f"{k}.cmd") - pos[k]) + col(f"{k}.ff") for k in GRAVITY_JOINTS}
    still = np.array([p.startswith(("hold:", "settle:")) for p in phases])
    on = col("scale") > 0.99
    for label, mask in (("ff 0 %", still & ~on), ("ff 100 %", still & on)):
        if mask.sum() > 10:
            print(f"\nTelemetry vs kp*error + feedforward at static holds, {label} (slope, r):")
            for k in GRAVITY_JOINTS:
                x, y = stiff_ff[k][mask], torque[k][mask]
                slope = float(np.polyfit(x, y, 1)[0]) if np.ptp(x) > 0.2 else math.nan
                r = float(np.corrcoef(x, y)[0, 1]) if np.ptp(x) > 0.2 else math.nan
                print(f"  {k:>13} slope {slope:5.2f} r {r:5.2f}")
    print("\nFree-motion contact torque (N m): the floor a real contact must clear")
    print(
        f"{'joint':>13} {'n':>5} {'rms':>6} {'p95':>6} {'p99':>6} {'max':>6} {'v>0':>6} {'v<0':>6}  still rms"
    )
    floor = {}
    for k in ARM:
        c = contact[k][moving]
        if len(c) < 10:
            continue
        a = np.abs(c)
        up, down = c[vel[k][moving] > 1.0], c[vel[k][moving] < -1.0]
        floor[k] = float(np.percentile(a, 99))
        print(
            f"{k:>13} {len(c):5d} {np.sqrt(np.mean(c**2)):6.2f} {np.percentile(a, 95):6.2f} "
            f"{floor[k]:6.2f} {a.max():6.2f} {np.mean(up) if len(up) else math.nan:+6.2f} "
            f"{np.mean(down) if len(down) else math.nan:+6.2f}  {np.sqrt(np.mean(contact[k][still] ** 2)):9.2f}"
        )
    print("  Selection: move/approach, lift < -15 deg, elbow > 15 deg; acceleration <= 50 deg/s^2,")
    print("             excluding 0.15 s around high acceleration and phase boundaries.")
    if estimator.friction:
        corrected = compensate(contact, vel, estimator.friction)
        print("\nFree-motion contact torque WITH friction compensation (N m):")
        print(f"{'joint':>13} {'n':>5} {'rms':>6} {'p95':>6} {'p99':>6} {'max':>6}  still rms")
        for k in ARM:
            c = corrected[k][moving]
            if len(c) < 10:
                continue
            a = np.abs(c)
            floor[k] = float(np.percentile(a, 99))
            print(
                f"{k:>13} {len(c):5d} {np.sqrt(np.mean(c**2)):6.2f} {np.percentile(a, 95):6.2f} "
                f"{floor[k]:6.2f} {a.max():6.2f}  {np.sqrt(np.mean(corrected[k][still] ** 2)):9.2f}"
            )
    push = np.array([p == "push" for p in phases])
    if push.sum() > 10:
        print("\nPush test (contact torque, N m): baseline = first 2 s of the window")
        tp = t[push] - t[push][0]
        base = tp < 2.0
        for k in ARM:
            c = (corrected if estimator.friction else contact)[k][push]
            above = np.abs(c) > floor.get(k, math.inf)
            print(
                f"  {k:>13} baseline rms {np.sqrt(np.mean(c[base] ** 2)):5.2f}  range {c.min():+5.2f}..{c.max():+5.2f}"
                f"  above free-motion p99 {100 * above.mean():4.0f} % of the window"
            )


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robot", default="viscous_01", help="robot record name")
    p.add_argument("--fit", help=f"gravity fit (default {DEFAULT_FIT})")
    p.add_argument("--urdf", type=Path, default=DEFAULT_MODEL_DIR / "viscous_arm.urdf")
    p.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR, help="collision mesh root")
    p.add_argument("--floor-z", type=float, default=0.03, help="clearance above the base plate underside, m")
    p.add_argument("--max-torque", type=float, default=3.5, help="fitted holding-torque cap per joint, N m")
    p.add_argument("--speed", type=float, default=15.0, help="travel speed, deg/s")
    p.add_argument("--approach-speed", type=float, default=6.0, help="final approach speed, deg/s")
    p.add_argument("--settle", type=float, default=1.5)
    p.add_argument("--sample", type=float, default=1.0)
    p.add_argument("--push-seconds", type=float, default=60.0)
    p.add_argument(
        "--max-temp", type=float, default=82.0, help="abort and return home at this winding temp, C"
    )
    p.add_argument(
        "--max-lag", type=float, default=10.0, help="abort if a joint lags its command this far, deg"
    )
    p.add_argument("--stop-test", action="store_true", help="wait for a stop signal; never opens hardware")
    p.add_argument("--dry-run", action="store_true", help="plan and print; never enables torque")
    p.add_argument("--analyze", type=Path, help="print the summary of a saved run and exit")
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

    if args.analyze:
        data = json.loads(args.analyze.read_text())
        args.fit = args.fit or data["fit"]
    args.fit = args.fit or DEFAULT_FIT.format(robot=args.robot)
    fit, model = gravity_feedforward.load_model(args.fit)
    estimator = ContactEstimator.from_fit(args.fit)
    if args.analyze:
        analyze(data, estimator)
        return
    if fit["robot"] != args.robot:
        raise SystemExit(f"{args.fit} is a fit for {fit['robot']}, not {args.robot}")

    record = load_record(args.robot)
    start = read_pose_readonly(build_robot(record))
    print("Start pose (read-only):", {k: round(v, 1) for k, v in start.items()})
    start_arm, cool, poses, _, _ = plan(args, build_robot(record), start, model)
    describe(model, build_robot(record), start, start_arm, cool, poses, args)
    if args.dry_run:
        return
    if not args.yes and input("Energize the arm and run this check? Type 'go': ").strip() != "go":
        return
    run_check(args, record, start, model, estimator)


if __name__ == "__main__":
    main()
