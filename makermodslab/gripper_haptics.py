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
"""Unilateral gripper wall with a dissipative brake ahead of delayed contact.

Only the Star gripper is energized, through the exclusive raw transport.
Free slow motion is released. Fast closure that outruns the follower gets
bounded damping (0x12), not a speculative position wall. Confirmed contact
anchors a wall at the jaw position; a short capped position command supplies
effort immediately. Depth increases effort, opening velocity reduces it.
There is no slow trajectory or power ramp delaying an impact.

The mW setting is NOT calibrated torque or a current bound. The 800 mW cap
was exercised on 2026-10-21 (136 mA stalled, 29-31 C); 10-27 backdrive reached
660 mA below that setting. See docs/viscous-haptics.md for evidence and risks.
Faults disable haptics and release without stopping follower teleop. Close
releases before return to rest; bus disconnect releases all seven twice,
without RawBus.close() or worker-thread signal handlers. Bus loss/SIGKILL
can leave effort active: the Star power switch remains the hard stop.
"""

from __future__ import annotations

import csv
import gzip
import logging
import math
import os
import statistics
import time
from datetime import UTC, datetime

from lerobot.teleoperators.rebot_102_leader.rebot_102_leader import RebotArm102Leader
from lerobot.utils.decorators import check_if_already_connected

from .fashionstar_raw import DAMPING, IDS, MONITOR, PING, RELEASE, Guard, RawBus, SafetyError, Travel

logger = logging.getLogger(__name__)

GRIPPER = "gripper"

# A3: effort must arrive during impact, not 0.1-0.3 s afterwards.
MAX_LEAD_DEG = 8.0
MOVING_LEAD_DEG = 12.0  # closing only; bounds the formerly unlimited impact command
STILL_DEG_S = 40.0
MIN_MW = 200
MAX_MW = 800  # 10-21 actuation evidence: 800 mW -> 136 mA stalled, 29-31 C
MW_PER_DEG = 100.0
MW_PER_DEG_S = 3.0  # closing adds effort; recoil subtracts it
MOVE_MS = 1  # request an immediate setpoint; actual motion remains servo/effort limited
RESEND_MW = 25
ON_DEG = 0.5
# A brake, not contact inference: lead/lag alone also occurs in empty fast closes.
BRAKE_ON_LAG_DEG = 6.0  # Star degrees (about 10.7 follower degrees)
BRAKE_OFF_LAG_DEG = 4.0
BRAKE_CLOSE_DEG_S = 100.0
BRAKE_MIN_MW = 200
BRAKE_MAX_MW = 400
BRAKE_MW_PER_DEG = 40.0
# Empty-motion torque reached 3.1 N m on 10-27. Require both a larger load
# and a sudden deceleration for a moving-jaw impact, not just acceleration.
IMPACT_NM = 3.5
IMPACT_DROP_DEG_S = 80.0
RELEASE_DEG = 3.0
SQUEEZE_GONE_S = 0.3
CALIB_S = 8.0
CALIB_TRAVEL_DEG = 20.0
CALIB_STILL_TICKS = 25
BASELINE_ERR_DEG = 3.0
THRESHOLD_MIN_NM = 0.5
THRESHOLD_MAX_NM = 1.5
# Run B fix.
CLOSED_STOP_IGNORE_DEG = 4.0
# Thermal: the Star servo (RawBus's own guard is 60 C) and the follower
# gripper (the project's 82 C abort, below the driver's 85 C stop).
STAR_TEMP_LIMIT_C = 55.0
FOLLOWER_TEMP_LIMIT_C = 82.0
# Follower feedback older than this is not used for contact.
FEEDBACK_FRESH_S = 0.1
# motorbridge's default: this many misses in a row is a lost servo.
LOSS_THRESHOLD = 20
# The Star closes toward its LOW end (closed stop near -2 deg, measured on
# unit 1); the follower's closing direction follows from the mapping's sign.
STAR_CLOSE = -1.0


def _clamp(x, lo, hi):
    return min(hi, max(lo, x))


class RawStarBus:
    """The motorbridge ``FashionStarServo`` calls ``RebotArm102Leader`` makes, on ``fashionstar_raw``.

    Reads are one monitor request per servo. A servo that misses a tick keeps
    its previous reading (as motorbridge's ``sync_monitor`` does) but is not
    marked fresh, and haptics never acts on a stale gripper reading.
    """

    def __init__(self, port, baudrate, *, serial_factory=None):
        if serial_factory is None:
            from serial import Serial

            serial_factory = Serial
        self.port = port
        self._serial = serial_factory(
            port, baudrate=baudrate, timeout=0.02, write_timeout=0.05, exclusive=True
        )
        self.raw = RawBus(self._serial, read_only=False, max_power_mw=MAX_MW)
        self.monitors = {}
        self.fresh = {}
        self._misses = dict.fromkeys(IDS, 0)

    @property
    def is_open(self):
        return not self.raw.closed

    def ping(self, servo_id):
        try:
            self.raw.request(PING, servo_id)
            return True
        except SafetyError:
            return False

    def sync_monitor(self, servo_ids):
        for servo_id in servo_ids:
            try:
                self.monitors[servo_id] = self.raw.request(MONITOR, servo_id)
                self.fresh[servo_id] = True
                self._misses[servo_id] = 0
            except SafetyError:
                self.fresh[servo_id] = False
                self._misses[servo_id] += 1
                if self._misses[servo_id] >= LOSS_THRESHOLD:
                    raise
        return {servo_id: self.monitors.get(servo_id) for servo_id in servo_ids}

    def set_stop_mode(self, servo_id, mode, power):
        if mode != RELEASE:
            # Only the lock button holds servos, and a haptic session refuses it.
            raise SafetyError("The haptic Star bus only releases servos")
        self.raw.stop(servo_id, RELEASE)

    def unlock(self, servo_id):
        self.set_stop_mode(servo_id, RELEASE, 0)

    def reset_multi_turn(self, servo_id):
        # Not in the raw protocol. The leader's unwrap absorbs whole turns; the
        # gripper's commands are refused unless its reading sits in the
        # single-turn frame (Travel.check).
        pass

    def read_angle(self, servo_id, multi_turn=True):
        raise NotImplementedError("The haptic Star bus does not read the lock button")

    def close(self):
        """Release all seven servos (two passes), then close the port.

        Never ``RawBus.close()``: it installs signal handlers, which only the
        main thread may do, and teleop runs in a worker thread, where it would
        close the port without releasing anything.
        """
        if self.raw.closed:
            return
        try:
            self.raw.release_all()
        finally:
            try:
                self._serial.close()
            finally:
                self.raw.closed = True


class HapticStarLeader(RebotArm102Leader):
    """The Star leader on ``RawStarBus``, so haptics can command its gripper."""

    def __init__(self, config, *, serial_factory=None):
        super().__init__(config)
        self._serial_factory = serial_factory
        self.gripper_haptics_status = {"phase": "connecting"}

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        if self.config.button.enabled:
            raise RuntimeError("Gripper haptics does not support the Star lock button")
        logger.info(f"Connecting {self} on {self.config.port} (raw transport, gripper haptics)...")
        bus = RawStarBus(self.config.port, self.config.baudrate, serial_factory=self._serial_factory)
        try:
            for motor_name, motor_id in self.config.joint_ids.items():
                if not bus.ping(motor_id):
                    raise RuntimeError(f"Servo not found for {motor_name} (id={motor_id}).")
                self._last_raw_positions[motor_name] = 0.0
            self.bus = bus
            if not self.is_calibrated and calibrate:
                raise RuntimeError(f"{self} has no calibration; calibrate it in MakerMods Lab first")
            self.configure()
            if calibrate and self.config.max_start_offset_deg is not None:
                self._check_start_pose(self.config.max_start_offset_deg)
        except Exception:
            bus.close()
            self.bus = None
            raise
        logger.info(f"{self} connected.")


def _innermost_bus(robot):
    # Same unwrapping as gravity_feedforward.install: can_speed_limit's wrapper
    # delegates to ``_base``, and only the base bus holds the feedback cache.
    bus = robot.bus
    while getattr(type(bus), "__getattr__", None) is not None and "_base" in vars(bus):
        bus = bus._base
    return bus


class GripperHaptics:
    """One session's haptic loop. Call ``before_send`` on the leader's action,
    ``after_send`` once the follower has been commanded, ``close`` when the
    loop ends (before any return to rest)."""

    def __init__(self, robot, leader, *, clock=time.monotonic, wall_clock=time.time, log_dir=None):
        cfg = leader.config
        self.leader = leader
        self.robot = robot
        self.bus = _innermost_bus(robot)
        self.clock = clock
        self.wall_clock = wall_clock
        self.star_id = cfg.joint_ids[GRIPPER]
        self.scale = float(cfg.joint_directions[GRIPPER])
        self.offset = float(cfg.joint_offsets.get(GRIPPER, 0.0))
        if (
            self.star_id != 6
            or not math.isfinite(self.scale)
            or self.scale == 0
            or not math.isfinite(self.offset)
        ):
            raise ValueError("Haptics requires Star gripper id 6 and a finite nonzero mapping")
        self.f_close = STAR_CLOSE * math.copysign(1.0, self.scale)
        lo, hi = robot.config.joint_limits[GRIPPER]
        # The leader maps the Star's closed reading (0) onto the follower's
        # real closed stop: that offset is where closing ends.
        self.closed_f = self.offset
        if not lo <= self.closed_f <= hi:
            raise ValueError(f"Gripper closed stop {self.closed_f} is outside the follower limits {lo}..{hi}")
        self.f_lo, self.f_hi = (lo, self.closed_f) if self.f_close > 0 else (self.closed_f, hi)
        self.log_dir = log_dir
        self.rows = []
        self.started = None
        self.phase = "calibrating"
        self.reason = None
        self._last_t = None
        self._last_lead = self._last_fv = None
        self._cmd = None
        self._still_tau = []
        self._star_seen = None
        self.baseline = self.threshold = self.travel = None
        self.latched = False
        self.wall_f = None
        self._squeeze_t = 0.0
        self.power = 0.0
        self.star_on = False
        self._sent = None
        self._publish()
        logger.info(
            "Gripper haptics: calibrating. Follower command %.1f..%.1f deg (closed stop %.1f), "
            "leader scale %+.3f offset %+.1f",
            self.f_lo,
            self.f_hi,
            self.closed_f,
            self.scale,
            self.offset,
        )

    # --- follower side --------------------------------------------------------

    def _follower(self):
        """Latest follower gripper state in joint degrees, or None when stale."""
        sent = (self.bus.last_feedback_time or {}).get(GRIPPER)
        if sent is None or not 0 <= self.wall_clock() - sent <= FEEDBACK_FRESH_S:
            return None
        state = self.bus._last_known_states[GRIPPER]
        turn = getattr(self.robot, "_turn_offset", {}).get(GRIPPER, 0.0)
        values = (
            float(state["position"]) + turn,
            float(state["velocity"]),
            float(state["torque"]),
            float(state["temp_mos"]),
        )
        if not all(math.isfinite(x) for x in values):
            raise SafetyError("Non-finite follower gripper feedback")
        return values

    def before_send(self, action):
        """Clamp the closed stop and closing lead; leave opening free."""
        key = f"{GRIPPER}.pos"
        if key not in action:
            return action
        target = _clamp(float(action[key]), self.f_lo, self.f_hi)
        try:
            state = self._follower()
            # Calibration must catch up freely, or startup error becomes its
            # torque baseline. Afterwards cap only CLOSING lead, even at speed.
            # Opening must escape an object freely.
            if self.phase == "on" and state is not None:
                cap = MAX_LEAD_DEG if abs(state[1]) < STILL_DEG_S else MOVING_LEAD_DEG
                if (target - state[0]) * self.f_close > cap:
                    target = state[0] + self.f_close * cap
                target = _clamp(target, self.f_lo, self.f_hi)
        except Exception as exc:
            self._turn_off(f"error: {exc}")
        self._cmd = target
        return {**action, key: target}

    # --- haptic loop ----------------------------------------------------------

    def after_send(self):
        if self.phase == "off":
            return
        try:
            self._step()
        except Exception as exc:
            logger.exception("Gripper haptics failed; releasing the Star gripper and turning haptics off")
            self._turn_off(f"error: {exc}")

    def _step(self):
        now = self.clock()
        if self.started is None:
            self.started = now
        dt = 0.0 if self._last_t is None else now - self._last_t
        self._last_t = now
        t = now - self.started

        state = self._follower()
        star = self.leader.bus.monitors.get(self.star_id) if self.leader.bus.fresh.get(self.star_id) else None
        if state is None or star is None:
            self._turn_off("stale gripper feedback")
            return
        if not 0 <= dt <= FEEDBACK_FRESH_S:
            self._turn_off("haptic loop timing gap")
            return
        if not all(math.isfinite(x) for x in (star.angle_deg, star.temperature_c)):
            raise SafetyError("Non-finite Star gripper feedback")
        if not -180 <= star.angle_deg <= 180:
            raise SafetyError("Star gripper outside the single-turn frame")
        f, fv, tau, f_temp = state
        if star.temperature_c >= STAR_TEMP_LIMIT_C:
            self._turn_off(f"Star gripper at {star.temperature_c:.1f} C")
            return
        if f_temp >= FOLLOWER_TEMP_LIMIT_C:
            self._turn_off(f"follower gripper at {f_temp:.1f} C")
            return
        lead = star.angle_deg
        closing_speed = 0.0 if self._last_lead is None or dt <= 0 else (self._last_lead - lead) / dt
        previous_fv = self._last_fv
        self._last_lead, self._last_fv = lead, fv
        still = abs(fv) < STILL_DEG_S
        cmd = self._cmd
        at_limit = cmd is None or not self.f_lo + 0.5 < cmd < self.f_hi - 0.5
        squeeze = pen = wall_l = None

        if self.phase == "calibrating":
            seen = self._star_seen or (lead, lead)
            self._star_seen = (min(seen[0], lead), max(seen[1], lead))
            # Still AND near the command: a jaw straining toward a far target
            # is not free motion.
            if still and not at_limit and abs(cmd - f) < BASELINE_ERR_DEG:
                self._still_tau.append(tau)
            travel = self._star_seen[1] - self._star_seen[0]
            if t >= CALIB_S and travel >= CALIB_TRAVEL_DEG and len(self._still_tau) >= CALIB_STILL_TICKS:
                self._arm(t)
        else:
            squeeze = (tau - self.baseline) * self.f_close
            near_closed = (self.closed_f - f) * self.f_close < CLOSED_STOP_IGNORE_DEG
            jaw_pen = (f - self.offset) / self.scale - lead
            impact = (
                squeeze >= max(IMPACT_NM, 2 * self.threshold)
                and previous_fv is not None
                and (previous_fv - fv) * self.f_close >= IMPACT_DROP_DEG_S
                and jaw_pen > ON_DEG
            )
            if not self.latched and not near_closed and ((still and squeeze >= self.threshold) or impact):
                self.latched, self.wall_f, self._squeeze_t = True, f, t
                logger.info(
                    "Gripper haptics: %s at follower %+.1f deg (squeeze %.2f N m)",
                    "impact" if impact else "contact",
                    f,
                    squeeze,
                )
            if self.latched:
                # Freeze the wall. Following a compliant/stalled jaw lets the
                # wall creep closed precisely while the user is pushing it.
                wall_l = round(
                    _clamp(
                        (self.wall_f - self.offset) / self.scale,
                        self.travel.low + self.travel.margin + 0.1,
                        self.travel.high - self.travel.margin - 0.1,
                    ),
                    1,
                )
                pen = (lead - wall_l) * STAR_CLOSE
                if squeeze >= 0.3 * self.threshold:
                    self._squeeze_t = t
                if pen < -RELEASE_DEG or t - self._squeeze_t > SQUEEZE_GONE_S:
                    self.latched = False
                    logger.info("Gripper haptics: contact off (penetration %+.1f deg)", pen)

            mode, target, want = None, None, 0
            if self.latched and pen > (0.0 if self._sent and self._sent[0] == "wall" else ON_DEG):
                # A unilateral spring + velocity term. Opening subtracts
                # effort; power zero must RELEASE, never encode ceiling.
                want = round(_clamp(MIN_MW + MW_PER_DEG * pen + MW_PER_DEG_S * closing_speed, 0, MAX_MW))
                mode, target = "wall", wall_l
            elif not self.latched and not near_closed:
                braking = self._sent is not None and self._sent[0] == "brake"
                lag_gate = BRAKE_OFF_LAG_DEG if braking else BRAKE_ON_LAG_DEG
                if closing_speed >= BRAKE_CLOSE_DEG_S and jaw_pen > lag_gate:
                    mode = "brake"
                    want = round(
                        _clamp(
                            BRAKE_MIN_MW + BRAKE_MW_PER_DEG * (jaw_pen - BRAKE_ON_LAG_DEG),
                            BRAKE_MIN_MW,
                            BRAKE_MAX_MW,
                        )
                    )
            if want > 0:
                self._drive(mode, target, want)
            else:
                self._release_star()

        self.rows.append(
            [
                round(t, 4),
                self.phase,
                lead,
                cmd,
                f,
                fv,
                tau,
                None if squeeze is None else round(squeeze, 3),
                int(self.latched),
                None if pen is None else round(pen, 2),
                wall_l,
                round(self.power) if self.star_on else 0,
                star.current_ma,
                None if self._sent is None else self._sent[0],
                round(closing_speed, 2),
            ]
        )
        self._publish()

    def _arm(self, t):
        low, high = self._star_seen
        if not (low >= -180.0 and high <= 180.0):
            self._turn_off(
                f"Star gripper reads {low:+.1f}..{high:+.1f} deg, outside its single-turn frame; "
                "power-cycle the Star to reset its turn count"
            )
            return
        self.baseline = statistics.median(self._still_tau)
        dev = sorted(abs(x - self.baseline) for x in self._still_tau)
        # p95, bounded: on 2026-10-28 three ticks of a squeeze being let go
        # (out of 59) set p99 and a 5.7 N m threshold no grasp reached.
        p95 = dev[min(len(dev) - 1, int(0.95 * len(dev)))]
        self.threshold = _clamp(1.5 * p95 + 0.1, THRESHOLD_MIN_NM, THRESHOLD_MAX_NM)
        self.travel = Travel(low, high, margin=1.0)
        self.leader.bus.raw.guard = Guard(self.star_id, (low + high) / 2, (high - low) / 2)
        self.phase = "on"
        logger.info(
            "Gripper haptics ON at %.1f s: still torque %+.3f N m over %d ticks (p95 dev %.3f), "
            "threshold %.2f N m, Star travel %+.1f..%+.1f deg",
            t,
            self.baseline,
            len(self._still_tau),
            p95,
            self.threshold,
            low,
            high,
        )

    def _drive(self, mode, target, power):
        command = (mode, target, power)
        # Mark pending effort BEFORE writing: a partial/failed write may have
        # energized the servo, so the exception path must attempt a release.
        self.star_on = True
        if self._sent is None or command[:2] != self._sent[:2] or abs(power - self._sent[2]) >= RESEND_MW:
            if mode == "brake":
                self.leader.bus.raw.stop(self.star_id, DAMPING, power)
            else:
                self.leader.bus.raw.angle(self.star_id, target, MOVE_MS, power, self.travel)
            self._sent = command
        self.power = self._sent[2]

    def _release_star(self):
        if self.star_on:
            # Keep pending state if this fails, so close/disconnect can retry.
            self.leader.bus.raw.stop(self.star_id, RELEASE)
        self.star_on, self.power, self._sent = False, 0.0, None

    def _turn_off(self, reason):
        self.reason = reason
        self.phase = "off"
        self.latched = False
        try:
            self._release_star()
        except Exception:
            logger.exception("Could not release the Star gripper; disconnect will retry")
        logger.warning("Gripper haptics off: %s", reason)
        self._publish()

    def _publish(self):
        self.leader.gripper_haptics_status = {
            "phase": self.phase,
            "contact": self.latched,
            "power_mw": round(self.power) if self.star_on else 0,
            "reason": self.reason,
        }

    def close(self):
        """Release the Star gripper and write the session's tick log."""
        try:
            self._release_star()
        except Exception:
            logger.exception("Could not release the Star gripper; disconnect will retry")
        self.phase, self.latched = "off", False
        self._publish()
        if self.rows and self.log_dir:
            try:
                self._write_log()
            except Exception:
                logger.exception("Could not write the gripper haptics log")

    def _write_log(self):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
        path = os.path.join(self.log_dir, f"gripper-haptics-{stamp}.csv.gz")
        with gzip.open(path, "wt", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(
                [
                    "t",
                    "phase",
                    "star_deg",
                    "cmd_deg",
                    "f_deg",
                    "f_vel",
                    "tau_nm",
                    "squeeze_nm",
                    "latched",
                    "pen_deg",
                    "wall_deg",
                    "mw",
                    "star_ma",
                    "mode",
                    "star_close_deg_s",
                ]
            )
            writer.writerows(self.rows)
        logger.info(
            "Gripper haptics log %s (%d ticks, baseline %s N m, threshold %s N m)",
            path,
            len(self.rows),
            self.baseline,
            self.threshold,
        )
