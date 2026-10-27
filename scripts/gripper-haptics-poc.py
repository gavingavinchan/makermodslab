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
"""A3 proof of concept on ONE joint: input-gated haptics on the gripper.

Only two actuators are energized: the Viscous follower's gripper motor (a CAN
bus carrying just that motor, opened without the energizing handshake) and the
Star leader's gripper servo (id 6). The other six follower motors are never
addressed; the other six Star servos are released at start and on every exit.

Loop (50 Hz):
- The Star gripper drives the follower gripper RELATIVE to where both were at
  start, through the stock preset's scale (follower deg per leader deg). The
  follower target may lead its actual position by at most --max-lead-deg, which
  caps the squeeze at kp x lead.
- Contact = |follower torque - its free-motion median|, counted only while the
  jaws are nearly still (a closing gripper's own effort is not contact).
- Gate off: the Star is released (0x10), free in the hand.
  Gate on: the Star is driven toward where the follower's jaws actually are,
  at a power cap rising with contact (--min-mw .. --max-mw). The pilot feels a
  wall where the follower met the object.

The first --calib-s seconds are free motion with nothing in the jaws: they set
the contact threshold from the noise floor and record the Star's travel, and
the Star is never commanded outside that travel.

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
CAN_PORT = (
    "/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207E37A55445-if00"
)
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
    p.add_argument("--duration", type=float, default=60.0, help="total run seconds, including calibration")
    p.add_argument("--calib-s", type=float, default=8.0, help="free-motion seconds before haptics turn on")
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--calib-travel-deg", type=float, default=20.0, help="Star travel needed to end calibration")
    p.add_argument("--calib-timeout-s", type=float, default=40.0, help="give up waiting for that travel")
    p.add_argument("--max-lead-deg", type=float, default=8.0, help="follower target lead cap (squeeze cap)")
    p.add_argument("--min-mw", type=int, default=150, help="Star power at the contact threshold")
    p.add_argument("--max-mw", type=int, default=800, help="Star power at threshold + --span-nm")
    p.add_argument("--span-nm", type=float, default=1.5, help="contact above threshold that reaches --max-mw")
    p.add_argument("--threshold-nm", type=float, help="override the measured contact threshold")
    p.add_argument("--still-deg-s", type=float, default=40.0, help="contact counts only below this jaw speed")
    p.add_argument("--hold-s", type=float, default=0.1, help="minimum time the gate stays on")
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
    raise SafetyError(f"follower gripper reads {raw:.1f}, not a whole turn from {lo:.1f}..{hi:.1f}: stale zero")


def main():
    args = parser().parse_args()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    if not 100 <= args.min_mw <= args.max_mw <= 1000:
        raise SystemExit("need 100 <= --min-mw <= --max-mw <= 1000")
    if not 0 < args.max_lead_deg <= 15:
        raise SystemExit("--max-lead-deg must be in (0, 15]")

    import serial

    scale, baudrate = leader_scale()
    bus, (f_lo, f_hi), (kp, kd) = follower_bus(args.can_port)
    f_lo, f_hi = f_lo + FOLLOWER_MARGIN_DEG, f_hi - FOLLOWER_MARGIN_DEG
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    log_path = EVIDENCE / f"gripper-haptics-{stamp}{'-dry' if args.dry_run else ''}.csv.gz"
    summary = {"args": vars(args), "scale": scale, "kp_kd": [kp, kd], "follower_limits": [f_lo, f_hi]}
    print(f"follower gripper limits {f_lo:.1f}..{f_hi:.1f} deg, kp {kp} kd {kd}; leader scale {scale:+.3f}")

    port = serial.Serial(args.star_port, baudrate, timeout=0.05, write_timeout=0.05, exclusive=True)
    star = RawBus(port, read_only=args.dry_run, max_power_mw=args.max_mw)
    energized = False
    rows = []
    try:
        with star:
            if not args.dry_run:
                for motor_id in IDS:
                    star.stop(motor_id, RELEASE)
            bus.connect(handshake=False)
            f0_raw = float(bus.read("Present_Position", GRIPPER))
            turn = turn_offset(f0_raw, f_lo, f_hi)
            f0 = f0_raw + turn
            if turn:
                print(f"follower gripper reads {f0_raw:+.1f} raw; correcting by {turn:+.0f} deg (power-cycle turn reset)")
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
                bus.write("Goal_Position", GRIPPER, f0 - turn)
                print("follower gripper ENERGIZED (holding where it is)")

            period = 1.0 / args.hz
            t0 = time.monotonic()
            next_t = t0
            phase = "calib"
            calib_tau, l_seen = [], [l0, l0]
            threshold = baseline = travel = None
            gate_on, gate_since, last_print = False, 0.0, 0.0
            print(f"CALIBRATION {args.calib_s:.0f} s: open and close the Star gripper freely, NOTHING in the jaws")
            while (now := time.monotonic()) - t0 < args.duration:
                t = now - t0
                lm = star_read(star)
                lead = lm.angle_deg
                target = f0 + scale * (lead - l0)
                if args.dry_run:
                    bus._batch_refresh([GRIPPER])
                    f, fv, tau, ftemp = follower_state(bus, turn)
                    cmd = None
                else:
                    f_prev = follower_state(bus, turn)[0]
                    cmd = min(f_hi, max(f_lo, target))
                    cmd = min(f_prev + args.max_lead_deg, max(f_prev - args.max_lead_deg, cmd))
                    bus.write("Goal_Position", GRIPPER, cmd - turn)
                    f, fv, tau, ftemp = follower_state(bus, turn)
                if ftemp >= FOLLOWER_TEMP_ABORT_C:
                    raise SafetyError(f"follower gripper at {ftemp:.1f} C")

                contact, power, wall = 0.0, 0, None
                if phase == "calib":
                    calib_tau.append(tau)
                    l_seen = [min(l_seen[0], lead), max(l_seen[1], lead)]
                    moved = l_seen[1] - l_seen[0] >= args.calib_travel_deg
                    if (t >= args.calib_s and moved) or t >= args.calib_timeout_s:
                        baseline = statistics.median(calib_tau)
                        dev = sorted(abs(x - baseline) for x in calib_tau)
                        p99 = dev[min(len(dev) - 1, int(0.99 * len(dev)))]
                        threshold = args.threshold_nm or max(0.25, 1.5 * p99 + 0.1)
                        travel = Travel(l_seen[0], l_seen[1], margin=1.0)
                        summary.update(baseline_nm=baseline, p99_nm=p99, threshold_nm=threshold, star_travel=l_seen)
                        print(
                            f"free-motion torque median {baseline:+.3f} N m, p99 dev {p99:.3f}; "
                            f"threshold {threshold:.2f} N m; star travel seen {l_seen[0]:+.1f}..{l_seen[1]:+.1f} deg"
                        )
                        if args.dry_run:
                            phase = "dry"
                        elif l_seen[1] - l_seen[0] < 10:
                            raise SafetyError("Star gripper moved < 10 deg in calibration; travel unknown, not driving it")
                        else:
                            mid, half = (l_seen[0] + l_seen[1]) / 2, (l_seen[1] - l_seen[0]) / 2
                            star.guard = Guard(STAR_GRIPPER, mid, half)
                            phase = "haptic"
                            print("HAPTICS ON: close the follower on an object (e.g. a roll of tape) and squeeze")
                elif phase == "haptic":
                    still = abs(fv) < args.still_deg_s
                    contact = abs(tau - baseline) if still else 0.0
                    if gate_on:
                        if contact < 0.5 * threshold and t - gate_since >= args.hold_s:
                            gate_on = False
                            star.stop(STAR_GRIPPER, RELEASE)
                    elif contact >= threshold:
                        gate_on, gate_since = True, t
                    if gate_on:
                        frac = min(1.0, max(0.0, (contact - threshold) / args.span_nm))
                        power = round(args.min_mw + frac * (args.max_mw - args.min_mw))
                        wall = min(travel.high - travel.margin, max(travel.low + travel.margin, l0 + (f - f0) / scale))
                        wall = round(wall, 1)
                        star.angle(STAR_GRIPPER, wall, max(1, round(period * 1000)), power, travel)

                rows.append([round(t, 4), phase, lead, cmd, f, fv, tau, contact, int(gate_on), power, wall, lm.current_ma])
                if t - last_print >= 0.25:
                    last_print = t
                    print(
                        f"{t:5.1f}s {phase:6s} star {lead:+6.1f} | follower {f:+6.1f} cmd "
                        f"{'-' if cmd is None else f'{cmd:+6.1f}'} tau {tau:+5.2f} | contact {contact:4.2f} "
                        f"{'GATE ' + str(power) + 'mW' if gate_on else ''}",
                        flush=True,
                    )
                next_t += period
                time.sleep(max(0.0, next_t - time.monotonic()))
                if time.monotonic() - next_t > 5 * period:
                    next_t = time.monotonic()
            print("duration reached")
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
            with gzip.open(log_path, "wt", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t", "phase", "star_deg", "cmd_deg", "f_deg", "f_vel", "tau_nm", "contact_nm", "gate", "mw", "wall_deg", "star_ma"])
                w.writerows(rows)
            gated = [r for r in rows if r[8]]
            summary.update(ticks=len(rows), gated_ticks=len(gated), max_tau=max(abs(r[6]) for r in rows))
            log_path.with_suffix("").with_suffix(".json").write_text(json.dumps(summary, indent=2, default=str))
            print(f"log {log_path.relative_to(REPO)} ({len(rows)} ticks, {len(gated)} gated)")


if __name__ == "__main__":
    main()
