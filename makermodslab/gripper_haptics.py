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
"""Gripper haptics in Lab teleop: the Star gripper pushes back on a grasp.

Opt-in per teleop session (the Teleop "Gripper haptics" checkbox), Viscous
only. The loop is ``scripts/gripper-haptics-poc.py`` v2, the version Gavin
called acceptable on 2026-10-27 (docs/viscous-haptics.md, "Gripper haptics
proof of concept"), moved into the teleop loop:

- The follower gripper follows the Star through the session's normal leader
  mapping. Its target may lead the jaws by at most MAX_LEAD_DEG while the
  jaws are nearly still (on an object: caps the squeeze at about kp x 8 deg),
  and never closes past the real closed stop (the leader's gripper offset).
- Calibration first: the operator opens and closes the gripper freely; the
  follower's still-jaw torque gives the free-motion baseline and contact
  threshold, the Star's travel bounds every later command.
- Contact latches a wall at the jaw position. The Star's power then rises
  with how far the hand closes the Star past that wall, only RISES at
  SLEW_MW_S and every move is planned over MOVE_MS (damping). At or outside
  the wall the Star is released, so it never pulls the finger in.

Two fixes from Run B's list are included: contact must persist
CONTACT_PERSIST_S, and none is detected within CLOSED_STOP_IGNORE_DEG of the
closed stop (the jaws closing on their own stop read as a squeeze).

Only the Star GRIPPER servo is ever energized. motorbridge 0.0.4 cannot send
a capped-power move (its power 0 is the protection ceiling,
docs/star-servo-protocol.md), so a haptic session opens the Star through
``fashionstar_raw`` instead of motorbridge, never both: ``HapticStarLeader``
is the stock leader on a ``RawStarBus`` that answers the calls it makes.
Every haptics fault releases the Star gripper and turns haptics off; teleop
itself carries on. Disconnect releases all seven Star servos twice. A
SIGKILL leaves the last command active: the Star's power switch is the hard
stop.
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

from .fashionstar_raw import IDS, MONITOR, PING, RELEASE, Guard, RawBus, SafetyError, Travel

logger = logging.getLogger(__name__)

GRIPPER = "gripper"

# PoC v2 values (Run B, "acceptable").
MAX_LEAD_DEG = 8.0
STILL_DEG_S = 40.0
MIN_MW = 100
MAX_MW = 600
MW_PER_DEG = 60.0
SLEW_MW_S = 1500.0
MOVE_MS = 150
ON_DEG = 1.0
RELEASE_DEG = 3.0
SQUEEZE_GONE_S = 0.3
CALIB_S = 8.0
CALIB_TRAVEL_DEG = 20.0
CALIB_STILL_TICKS = 25
BASELINE_ERR_DEG = 3.0
THRESHOLD_MIN_NM = 0.5
THRESHOLD_MAX_NM = 1.5
# Run B fixes.
CONTACT_PERSIST_S = 0.06
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

    def __init__(self, robot, leader, *, clock=time.monotonic, log_dir=None):
        cfg = leader.config
        self.leader = leader
        self.robot = robot
        self.bus = _innermost_bus(robot)
        self.clock = clock
        self.star_id = cfg.joint_ids[GRIPPER]
        self.scale = float(cfg.joint_directions[GRIPPER])
        self.offset = float(cfg.joint_offsets.get(GRIPPER, 0.0))
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
        self._cmd = None
        self._still_tau = []
        self._star_seen = None
        self.baseline = self.threshold = self.travel = None
        self.latched = False
        self.wall_f = None
        self._candidate_t = None
        self._squeeze_t = 0.0
        self.power = 0.0
        self.star_on = False
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
        if sent is None or time.time() - sent > FEEDBACK_FRESH_S:
            return None
        state = self.bus._last_known_states[GRIPPER]
        turn = getattr(self.robot, "_turn_offset", {}).get(GRIPPER, 0.0)
        return (
            float(state["position"]) + turn,
            float(state["velocity"]),
            float(state["torque"]),
            float(state["temp_mos"]),
        )

    def before_send(self, action):
        """Clamp the follower gripper target: closed stop always, squeeze cap while the jaws are still."""
        key = f"{GRIPPER}.pos"
        if key not in action:
            return action
        target = _clamp(float(action[key]), self.f_lo, self.f_hi)
        state = self._follower()
        # Only once haptics is on. During calibration the jaws are empty, and
        # Lab's absolute mapping can start them far from the Star's pose (the
        # PoC's relative mapping never did): the cap then stalled them on
        # friction and the stall was measured as free-motion torque
        # (2026-10-28, baseline 2.75 N m).
        if self.phase == "on" and state is not None and abs(state[1]) < STILL_DEG_S:
            target = _clamp(target, state[0] - MAX_LEAD_DEG, state[0] + MAX_LEAD_DEG)
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
            # Never push on stale data; resume when both are fresh again.
            self._release_star()
            self._publish()
            return
        f, fv, tau, f_temp = state
        if star.temperature_c >= STAR_TEMP_LIMIT_C:
            self._turn_off(f"Star gripper at {star.temperature_c:.1f} C")
            return
        if f_temp >= FOLLOWER_TEMP_LIMIT_C:
            self._turn_off(f"follower gripper at {f_temp:.1f} C")
            return
        lead = star.angle_deg
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
            if not self.latched:
                if still and squeeze >= self.threshold and not near_closed:
                    self._candidate_t = t if self._candidate_t is None else self._candidate_t
                    if t - self._candidate_t >= CONTACT_PERSIST_S:
                        self.latched, self.wall_f, self._squeeze_t = True, f, t
                        logger.info(
                            "Gripper haptics: contact at follower %+.1f deg (squeeze %.2f N m)", f, squeeze
                        )
                else:
                    self._candidate_t = None
            if self.latched:
                if still and squeeze >= 0.3 * self.threshold:
                    self.wall_f = 0.8 * self.wall_f + 0.2 * f
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
                    self.latched, self._candidate_t = False, None
                    logger.info(
                        "Gripper haptics: contact off (penetration %+.1f deg, squeeze %.2f N m)", pen, squeeze
                    )
            want = 0.0
            if self.latched and pen > (0.0 if self.star_on else ON_DEG):
                want = _clamp(MIN_MW + MW_PER_DEG * pen, MIN_MW, MAX_MW)
            if want > 0:
                self.power = min(want, max(self.power, MIN_MW) + SLEW_MW_S * dt)
                self.leader.bus.raw.angle(self.star_id, wall_l, MOVE_MS, round(self.power), self.travel)
                self.star_on = True
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

    def _release_star(self):
        if self.star_on:
            self.star_on, self.power = False, 0.0
            self.leader.bus.raw.stop(self.star_id, RELEASE)

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
        if self.phase != "off":
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
