# Copyright 2026 MakerMods. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""A3 proof of concept on ONE joint: haptics on the gripper, v2 (a rendered wall).

Only two actuators are energized: the Viscous follower's gripper motor (a CAN
bus carrying just that motor, opened without the energizing handshake) and the
Star leader's gripper servo (id 6). The other six follower motors are never
addressed; the other six Star servos are released at start and on every exit.

v1 (2026-10-27) was sluggish and oscillated; see docs/viscous-haptics.md,
"Gripper haptics proof of concept". v2 changes, each tied to a cause:
- Free motion: the follower target passes straight through, like Lab teleop.
  The squeeze cap (target at most --max-lead-deg ahead of the jaws) applies
  only while the jaws are nearly still, i.e. on an object.
- Contact LATCHES a wall at the jaw position. The Star's push then depends
  continuously on how far the Star is closed past that wall (penetration),
  not on a torque on/off level that the push itself switches off.
- The Star resists, it does not shove: power rises with penetration at
  --mw-per-deg, may only RISE at --slew-mw-s, and each move is planned over
  --move-ms (a slow trajectory acts as damping). At or outside the wall it
  is released at once, so it never pulls the finger in.
- Contact ends when the Star opens past the wall by --release-deg, or the
  follower's squeeze has been gone for 0.3 s.
- The follower is clamped at its real closed stop (--follower-closed-deg), and
  the baseline/threshold come from still, unclamped ticks during calibration.

--follower-only never energizes the Star: free-motion responsiveness alone.
Stops: Ctrl-C / SIGTERM / SIGHUP, or --duration. Every exit releases all seven
Star servos and disables the follower gripper. A SIGKILL leaves both holding
their last command: the power switches are the hard stop.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import signal
import statistics
import sys
import time
from datetime import UTC, datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from makermodslab.fashionstar_raw import (  # noqa: E402
    IDS,
    MONITOR,
    RELEASE,
    Guard,
    RawBus,
    SafetyError,
    Travel,
)

STAR_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
CAN_PORT = "/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207E37A55445-if00"
EVIDENCE = REPO / "configs/viscous/viscous_01/evidence"
STAR_GRIPPER = 6
GRIPPER = "gripper"
FOLLOWER_MARGIN_DEG = 2.0
TEMP_ABORT_C = 55.0  # the Star servo; RawBus's own guard trips at 60
FOLLOWER_TEMP_ABORT_C = 82.0


def interrupt(signum, _frame):
    raise KeyboardInterrupt(signal.Signals(signum).name)


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--star-port", default=STAR_PORT)
    p.add_argument("--can-port", default=CAN_PORT)
    p.add_argument("--dry-run", action="store_true", help="read both grippers only; energize nothing")
    p.add_argument("--follower-only", action="store_true", help="drive the follower; never energize the Star")
    p.add_argument("--duration", type=float, default=60.0, help="total run seconds, including calibration")
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--calib-s", type=float, default=8.0, help="minimum free-motion seconds before haptics")
    p.add_argument(
        "--calib-travel-deg", type=float, default=20.0, help="Star travel needed to end calibration"
    )
    p.add_argument("--calib-timeout-s", type=float, default=40.0, help="give up waiting for that travel")
    p.add_argument("--follower-closed-deg", type=float, default=-33.0, help="real closed stop is -32.4")
    p.add_argument("--max-lead-deg", type=float, default=8.0, help="squeeze cap while the jaws are still")
    p.add_argument("--still-deg-s", type=float, default=40.0, help="jaws slower than this count as still")
    p.add_argument("--threshold-nm", type=float, help="override the measured contact threshold")
    p.add_argument("--min-mw", type=int, default=100, help="Star power just past the wall")
    p.add_argument("--max-mw", type=int, default=600)
    p.add_argument("--mw-per-deg", type=float, default=60.0, help="power per Star degree past the wall")
    p.add_argument("--slew-mw-s", type=float, default=1500.0, help="fastest power RISE")
    p.add_argument("--move-ms", type=int, default=150, help="Star move duration toward the wall (damping)")
    p.add_argument("--on-deg", type=float, default=1.0, help="penetration that starts the push")
    p.add_argument("--release-deg", type=float, default=3.0, help="opening past the wall that ends contact")
    return p


def follower_bus(port):
    """A RobStride bus with only the Viscous gripper on it (can_common._build_gripper_bus's pattern)."""
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.robstride import RobstrideMotorsBus
    from lerobot.robots.viscous_follower import ViscousFollowerConfig
    from lerobot.robots.viscous_follower.viscous_follower import MOTOR_MODELS

    config = ViscousFollowerConfig(port=port)
    ids = config.motor_can_ids[GRIPPER]
    send_id, recv_id = ids if isinstance(ids, tuple) else (ids, ids)
    model = MOTOR_MODELS[GRIPPER]
    motor = Motor(send_id, model, MotorNormMode.DEGREES)
    motor.recv_id = recv_id
    motor.motor_type_str = model
    bus = RobstrideMotorsBus(
        port=port,
        motors={GRIPPER: motor},
        can_interface="slcan",
        use_can_fd=False,
        bitrate=config.can_bitrate,
        data_bitrate=None,
    )
    return bus, config.joint_limits[GRIPPER], config.gains[GRIPPER]


def leader_scale():
    from lerobot.teleoperators.rebot_102_leader.config_rebot_102_leader_viscous import (
        RebotArm102LeaderViscousConfig,
    )

    config = RebotArm102LeaderViscousConfig(port="unused")
    if config.joint_ids[GRIPPER] != STAR_GRIPPER:
        raise ValueError("Star gripper is not servo id 6")
    return float(config.joint_directions[GRIPPER]), config.baudrate


def star_read(star):
    m = star.request(MONITOR, STAR_GRIPPER)
    if m.temperature_c >= TEMP_ABORT_C:
        raise SafetyError(f"Star gripper at {m.temperature_c:.1f} C")
    return m


def follower_state(bus, turn=0.0):
    s = bus._last_known_states[GRIPPER]
    return float(s["position"]) + turn, float(s["velocity"]), float(s["torque"]), float(s["temp_mos"])


def turn_offset(raw, lo, hi, grace=20.0):
    """MakerFollower._detect_full_turn_offsets for one joint: a RobStride can come back from a
    power cycle a whole turn off; correct only when a +-360 shift alone lands inside the limits."""
    if lo - grace <= raw <= hi + grace:
        return 0.0
    for shift in (-360.0, 360.0):
        if lo - grace <= raw + shift <= hi + grace:
            return shift
    raise SafetyError(
        f"follower gripper reads {raw:.1f}, not a whole turn from {lo:.1f}..{hi:.1f}: stale zero"
    )


def clamp(x, lo, hi):
    return min(hi, max(lo, x))


def event(t, text):
    print(f"EVENT {t:6.2f}s {text}", flush=True)


def main():
    args = parser().parse_args()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    if not 100 <= args.min_mw <= args.max_mw <= 1000:
        raise SystemExit("need 100 <= --min-mw <= --max-mw <= 1000")
    if not 0 < args.max_lead_deg <= 15:
        raise SystemExit("--max-lead-deg must be in (0, 15]")
    if not 20 <= args.move_ms <= 1000:
        raise SystemExit("--move-ms must be in [20, 1000]")

    import serial

    scale, baudrate = leader_scale()
    # Measured on unit 1: the Star closes toward its LOW end (closed stop near -2 deg),
    # and with a negative scale the follower closes toward its HIGH end (stop at -32.4).
    star_close = -1.0
    f_close = star_close * math.copysign(1.0, scale)
    if f_close != 1.0:
        raise SystemExit("this proof of concept assumes the follower closes toward +deg")
    bus, (f_lo, f_hi), (kp, kd) = follower_bus(args.can_port)
    f_lo, f_hi = f_lo + FOLLOWER_MARGIN_DEG, f_hi - FOLLOWER_MARGIN_DEG
    f_cmd_hi = min(f_hi, args.follower_closed_deg)
    star_drive = not (args.dry_run or args.follower_only)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    tag = "-dry" if args.dry_run else "-follower" if args.follower_only else ""
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    log_path = EVIDENCE / f"gripper-haptics-v2-{stamp}{tag}.csv.gz"
    summary = {"version": 2, "args": vars(args), "scale": scale, "kp_kd": [kp, kd], "f_cmd": [f_lo, f_cmd_hi]}
    print(f"follower cmd range {f_lo:.1f}..{f_cmd_hi:.1f} deg, kp {kp} kd {kd}; leader scale {scale:+.3f}")

    port = serial.Serial(args.star_port, baudrate, timeout=0.05, write_timeout=0.05, exclusive=True)
    star = RawBus(port, read_only=not star_drive, max_power_mw=args.max_mw)
    energized = False
    rows = []
    try:
        with star:
            if star_drive:
                for motor_id in IDS:
                    star.stop(motor_id, RELEASE)
            bus.connect(handshake=False)
            f0_raw = float(bus.read("Present_Position", GRIPPER))
            turn = turn_offset(f0_raw, f_lo, f_hi)
            f0 = f0_raw + turn
            if turn:
                print(
                    f"follower gripper reads {f0_raw:+.1f} raw; correcting by {turn:+.0f} deg (power-cycle turn reset)"
                )
            l0 = star_read(star).angle_deg
            print(f"start: star gripper {l0:+.1f} deg, follower gripper {f0:+.1f} deg")
            if not f_lo <= f0 <= f_hi:
                raise SafetyError(f"follower gripper {f0:.1f} outside {f_lo:.1f}..{f_hi:.1f}")
            summary.update(l0=l0, f0=f0, f0_raw=f0_raw, turn_offset=turn)

            if not args.dry_run:
                bus.write("Kp", GRIPPER, kp)
                bus.write("Kd", GRIPPER, kd)
                bus.enable_torque(GRIPPER)
                energized = True
                bus.write("Goal_Position", GRIPPER, clamp(f0, f_lo, f_cmd_hi) - turn)

            period = 1.0 / args.hz
            slew = args.slew_mw_s * period
            t0 = time.monotonic()
            next_t = t0
            phase = "calib"
            still_tau, l_seen = [], [l0, l0]
            threshold = baseline = travel = None
            latched, wall_f, last_squeeze_t, power, star_on = False, None, 0.0, 0.0, False
            last_print = -1.0
            event(0, "CALIBRATION: open and close the Star gripper freely, nothing in the jaws")
            while (now := time.monotonic()) - t0 < args.duration:
                t = now - t0
                lm = star_read(star)
                lead = lm.angle_deg
                target = clamp(f0 + scale * (lead - l0), f_lo, f_cmd_hi)
                f_prev, fv_prev, _, _ = follower_state(bus, turn)
                if args.dry_run:
                    bus._batch_refresh([GRIPPER])
                    cmd = None
                else:
                    cap = args.max_lead_deg if abs(fv_prev) < args.still_deg_s else 180.0
                    cmd = clamp(target, f_prev - cap, f_prev + cap)
                    bus.write("Goal_Position", GRIPPER, cmd - turn)
                f, fv, tau, ftemp = follower_state(bus, turn)
                if ftemp >= FOLLOWER_TEMP_ABORT_C:
                    raise SafetyError(f"follower gripper at {ftemp:.1f} C")
                still = abs(fv) < args.still_deg_s
                at_limit = cmd is not None and not f_lo + 0.5 < cmd < f_cmd_hi - 0.5

                squeeze, pen, wall_l = 0.0, None, None
                if phase == "calib":
                    l_seen = [min(l_seen[0], lead), max(l_seen[1], lead)]
                    if still and not at_limit:
                        still_tau.append(tau)
                    ready = (
                        t >= args.calib_s
                        and l_seen[1] - l_seen[0] >= args.calib_travel_deg
                        and len(still_tau) >= 25
                    )
                    if ready or t >= args.calib_timeout_s:
                        if len(still_tau) < 25 or l_seen[1] - l_seen[0] < 10:
                            raise SafetyError("calibration saw too little motion; not rendering a wall")
                        baseline = statistics.median(still_tau)
                        dev = sorted(abs(x - baseline) for x in still_tau)
                        p99 = dev[min(len(dev) - 1, int(0.99 * len(dev)))]
                        threshold = args.threshold_nm or max(0.3, 1.5 * p99 + 0.1)
                        travel = Travel(l_seen[0], l_seen[1], margin=1.0)
                        summary.update(
                            baseline_nm=baseline,
                            p99_nm=p99,
                            threshold_nm=threshold,
                            star_travel=l_seen,
                            still_ticks=len(still_tau),
                        )
                        print(
                            f"still torque median {baseline:+.3f} N m over {len(still_tau)} ticks, p99 dev {p99:.3f}; "
                            f"threshold {threshold:.2f} N m; star travel {l_seen[0]:+.1f}..{l_seen[1]:+.1f} deg"
                        )
                        if star_drive:
                            mid, half = (l_seen[0] + l_seen[1]) / 2, (l_seen[1] - l_seen[0]) / 2
                            star.guard = Guard(STAR_GRIPPER, mid, half)
                            phase = "haptic"
                            event(t, "HAPTICS ON: close the follower on the cube and squeeze")
                        else:
                            phase = "free"
                            event(t, "CALIBRATED (Star never energized in this mode): keep moving freely")
                elif phase == "haptic":
                    squeeze = (tau - baseline) * f_close
                    if not latched and still and squeeze >= threshold:
                        latched, wall_f, last_squeeze_t = True, f, t
                        event(t, f"CONTACT at follower {f:+.1f} deg (squeeze {squeeze:.2f} N m)")
                    if latched:
                        if still and squeeze >= 0.3 * threshold:
                            wall_f = 0.8 * wall_f + 0.2 * f
                        wall_l = round(
                            clamp(
                                l0 + (wall_f - f0) / scale,
                                travel.low + travel.margin + 0.1,
                                travel.high - travel.margin - 0.1,
                            ),
                            1,
                        )
                        pen = (lead - wall_l) * star_close
                        if squeeze >= 0.3 * threshold:
                            last_squeeze_t = t
                        if pen < -args.release_deg or t - last_squeeze_t > 0.3:
                            latched = False
                            event(t, f"CONTACT OFF (penetration {pen:+.1f} deg, squeeze {squeeze:.2f} N m)")
                    want = 0.0
                    if latched and pen > (args.on_deg if not star_on else 0.0):
                        want = clamp(args.min_mw + args.mw_per_deg * pen, args.min_mw, args.max_mw)
                    if want > 0:
                        power = min(want, max(power, args.min_mw) + slew)
                        star.angle(STAR_GRIPPER, wall_l, args.move_ms, round(power), travel)
                        star_on = True
                    elif star_on:
                        star.stop(STAR_GRIPPER, RELEASE)
                        star_on, power = False, 0.0

                rows.append(
                    [
                        round(t, 4),
                        phase,
                        lead,
                        cmd,
                        target,
                        f,
                        fv,
                        tau,
                        squeeze,
                        int(latched),
                        None if pen is None else round(pen, 2),
                        None if wall_l is None else round(wall_l, 2),
                        round(power) if star_on else 0,
                        lm.current_ma,
                    ]
                )
                if t - last_print >= 1.0:
                    last_print = t
                    print(
                        f"{t:5.1f}s {phase:6s} star {lead:+6.1f} | follower {f:+6.1f} target {target:+6.1f} "
                        f"tau {tau:+5.2f} | {'WALL pen ' + format(pen, '+.1f') + ' ' + str(round(power)) + 'mW' if latched else ''}",
                        flush=True,
                    )
                next_t += period
                time.sleep(max(0.0, next_t - time.monotonic()))
                if time.monotonic() - next_t > 5 * period:
                    next_t = time.monotonic()
            event(time.monotonic() - t0, "DURATION REACHED")
    except KeyboardInterrupt as e:
        print(f"stopped ({e})")
    finally:
        if energized:
            try:
                bus.disable_torque(GRIPPER)
                print("follower gripper disabled")
            except Exception as e:
                from makermodslab.torque import de_energize_can_bus

                print(f"follower disable failed ({e}); broadcasting release", flush=True)
                print(de_energize_can_bus(bus, "gripper haptics") or "released")
        try:
            if bus.is_connected:
                bus.disconnect(disable_torque=False)
        except Exception as e:
            print(f"follower bus close: {e}")
        if rows:
            write_log(log_path, rows, summary)


def write_log(log_path, rows, summary):
    with gzip.open(log_path, "wt", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(
            [
                "t",
                "phase",
                "star_deg",
                "cmd_deg",
                "target_deg",
                "f_deg",
                "f_vel",
                "tau_nm",
                "squeeze_nm",
                "latched",
                "pen_deg",
                "wall_deg",
                "mw",
                "star_ma",
            ]
        )
        w.writerows(rows)
    # Free-motion responsiveness: how far the jaws trail the Star's (clamped) target.
    free = [abs(r[4] - r[5]) for r in rows if not r[9] and r[3] is not None]
    if free:
        free.sort()
        summary["free_track_err_deg"] = {"p50": free[len(free) // 2], "p95": free[int(0.95 * len(free))]}
    summary.update(
        ticks=len(rows),
        latched_ticks=sum(r[9] for r in rows),
        max_tau=max(abs(r[7]) for r in rows),
        max_star_ma=max(r[13] for r in rows),
    )
    log_path.with_suffix("").with_suffix(".json").write_text(json.dumps(summary, indent=2, default=str))
    print(
        f"SUMMARY {json.dumps({k: summary[k] for k in ('ticks', 'latched_ticks', 'max_tau', 'max_star_ma', 'free_track_err_deg') if k in summary}, default=str)}"
    )
    print(f"log {log_path.relative_to(REPO)}")


if __name__ == "__main__":
    main()
