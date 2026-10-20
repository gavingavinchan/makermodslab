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
"""Drive the purple Star Arm 102 leader to follow the Viscous follower.

Groundwork for a Viscous DAgger handover: while a policy drives the follower,
the leader has to sit where the follower is so the operator can take over
without a jump. The FashionStar servos in the 102-HD are real servos (its lock
button already holds them with stop mode 0x11), so ``set_angle`` can position
them; the leader only ever runs them released (0x10) during teleop.

The mapping inverts the 1:1 posture trial (joint_posture_mapping.py) for the
six arm joints and the stock linear preset for the gripper. Targets are
clamped to the Star's own measured travel, which is narrower than the
follower's on several joints (pan, wrist yaw): a 1:1 target past it would
stall a servo against its end stop at full power.

Commands use the servo's multi-turn frame, which ``configure()`` resets at
connect. That frame only matches the mapping's unwrap windows when the reset
happened at the folded pose, so ``StarFollower.engage`` refuses a leader whose
multi-turn reading disagrees with the unwrapped one.
"""

from __future__ import annotations

import contextlib
import logging
import math
import time

from .joint_posture_mapping import JOINTS, LEADER_CENTERS

logger = logging.getLogger(__name__)

STOP_RELEASE = 0x10  # torque off: free and backdrivable
TRAVEL_MARGIN_DEG = 5.0
FRAME_TOLERANCE_DEG = 5.0
DEFAULT_SPEED_DEG_S = 30.0
MAX_SPEED_DEG_S = 90.0


def _wrap(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


class StarFollowMapper:
    """Follower joint degrees -> Star raw servo degrees, inside the Star's travel."""

    def __init__(self, directions, offsets, ranges, *, margin_deg: float = TRAVEL_MARGIN_DEG):
        self.directions, self.offsets, self.travel, self.centers = {}, {}, {}, {}
        for name, (lo, hi) in ranges.items():
            direction, offset = float(directions[name]), float(offsets.get(name, 0.0))
            if not math.isfinite(direction) or direction == 0 or not math.isfinite(offset):
                raise ValueError(f"Invalid leader mapping for {name}")
            # The leader preset's ranges are follower limits; divided back by the
            # captured scale they are the Star's own measured travel.
            a, b = sorted(((lo - offset) / direction, (hi - offset) / direction))
            if b - a <= 2 * margin_deg:
                raise ValueError(f"Star travel for {name} is narrower than the safety margin")
            self.directions[name], self.offsets[name] = direction, offset
            self.travel[name] = (a + margin_deg, b - margin_deg)
            self.centers[name] = LEADER_CENTERS.get(name, (a + b) / 2.0)

    def unwrap(self, name: str, raw: float) -> float:
        """Place a raw reading in the same window the targets are computed in."""
        center = self.centers[name]
        return center + _wrap(raw - center)

    def leader_targets(self, follower: dict[str, float]) -> dict[str, float]:
        targets = {}
        for name, direction in self.directions.items():
            position = follower[name]
            if isinstance(position, bool) or not math.isfinite(position):
                raise ValueError(f"Follower {name} has no finite position")
            if name in JOINTS:
                # Inverse of JointPostureMapper.map: unit gain, carried sign.
                raw = math.copysign(1.0, direction) * (position - self.offsets[name])
            else:
                raw = (position - self.offsets[name]) / direction
            # Clamp in this linear frame, never after a wrap: a 1:1 pan target
            # past the Star's reach would otherwise wrap to the opposite stop.
            # The travel already lies inside every unwrap window.
            lo, hi = self.travel[name]
            targets[name] = min(hi, max(lo, raw))
        return targets


def slew(reference: float, target: float, max_step: float) -> float:
    return reference + max(-max_step, min(max_step, target - reference))


class StallWatch:
    """Trip when a joint lags its command for too long: a stop, a snag or a hand on the leader."""

    def __init__(self, *, threshold_deg: float = 20.0, hold_s: float = 0.75, clock=time.monotonic):
        self.threshold_deg, self.hold_s, self.clock = threshold_deg, hold_s, clock
        self._since: dict[str, float] = {}

    def check(self, errors: dict[str, float]) -> str | None:
        now = self.clock()
        for name, error in errors.items():
            if abs(error) < self.threshold_deg:
                self._since.pop(name, None)
            elif now - self._since.setdefault(name, now) >= self.hold_s:
                return name
        return None


class StarFollower:
    """Position the leader's servos toward the follower at a bounded speed."""

    def __init__(self, leader, mapper: StarFollowMapper, *, speed_deg_s=DEFAULT_SPEED_DEG_S, clock=None):
        if not 0 < speed_deg_s <= MAX_SPEED_DEG_S:
            raise ValueError(f"Leader follow speed must be in (0, {MAX_SPEED_DEG_S:.0f}] deg/s")
        self.leader, self.mapper, self.speed = leader, mapper, float(speed_deg_s)
        self.clock = clock or time.monotonic
        self.ids = {name: leader.config.joint_ids[name] for name in mapper.directions}
        self.reference: dict[str, float] = {}
        self._last_t: float | None = None

    def read(self) -> dict[str, float]:
        raw = self.leader._read_raw_positions()
        return {name: self.mapper.unwrap(name, raw[name]) for name in self.ids}

    def frame_errors(self) -> dict[str, float]:
        """Unwrapped reading minus the servo's own multi-turn reading, per joint."""
        unwrapped = self.read()
        bus = self.leader.bus
        return {
            name: unwrapped[name] - float(bus.read_angle(motor_id, multi_turn=True).raw_deg)
            for name, motor_id in self.ids.items()
        }

    def engage(self) -> None:
        errors = self.frame_errors()
        bad = {n: e for n, e in errors.items() if abs(e) > FRAME_TOLERANCE_DEG}
        if bad:
            raise RuntimeError(
                "Star multi-turn frame does not match the mapping window "
                f"({', '.join(f'{n} {e:+.1f} deg' for n, e in bad.items())}); "
                "reconnect the leader with it folded at its zero pose"
            )
        self.reference = self.read()
        self._last_t = self.clock()

    def step(self, follower: dict[str, float]) -> dict[str, float]:
        """Advance every joint's command one tick toward the follower; return the targets."""
        if self._last_t is None:
            raise RuntimeError("StarFollower.step before engage")
        now = self.clock()
        dt = min(max(now - self._last_t, 0.0), 0.1)
        self._last_t = now
        targets = self.mapper.leader_targets(follower)
        max_step = self.speed * dt
        bus = self.leader.bus
        for name, motor_id in self.ids.items():
            self.reference[name] = slew(self.reference[name], targets[name], max_step)
            bus.set_angle(motor_id, self.reference[name], multi_turn=True, interval_ms=round(dt * 1000))
        return targets

    def release(self) -> bool:
        """Torque off every joint. True only if every servo acknowledged the write."""
        ok = True
        for name, motor_id in self.ids.items():
            try:
                self.leader.bus.set_stop_mode(motor_id, mode=STOP_RELEASE, power=0)
            except Exception:
                ok = False
                logger.exception("Star %s (id=%s) did not release; power-cycle the leader", name, motor_id)
            time.sleep(0.005)
        self._last_t = None
        return ok


def release_quietly(follower: StarFollower | None) -> None:
    if follower is not None:
        with contextlib.suppress(Exception):
            follower.release()
