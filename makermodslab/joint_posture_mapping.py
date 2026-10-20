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

"""Opt-in 1:1 Viscous posture trial with a five-second startup indicator.

Absolute references remain fixed across starts. Starting mismatch never becomes
a calibration offset. Physical reference offsets still require operator review.
"""

from __future__ import annotations

import logging
import math
import time

logger = logging.getLogger(__name__)
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
# Independent LEADER-frame windows from the archived commissioning capture.
# Never derive these from shifted follower limits or direction magnitudes.
LEADER_CENTERS = dict(zip(JOINTS, (0.0, 99.25, -135.0, -2.95, -1.85, 2.05), strict=True))


def finite_angles(values):
    if any(
        name not in values
        or isinstance(values[name], bool)
        or not isinstance(values[name], (int, float))
        or not math.isfinite(values[name])
        for name in JOINTS
    ):
        raise ValueError("Joint posture mapping requires six fresh finite encoder angles")
    return {name: float(values[name]) for name in JOINTS}


class JointPostureMapper:
    def __init__(self, directions, offsets, limits):
        self.signs, self.offsets, self.limits = {}, {}, {}
        for name in JOINTS:
            direction, offset = directions[name], offsets.get(name, 0.0)
            lo, hi = limits[name]
            if not math.isfinite(direction) or direction == 0 or not math.isfinite(offset):
                raise ValueError(f"Invalid mapping for {name}")
            if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
                raise ValueError(f"Invalid limits for {name}")
            self.signs[name] = math.copysign(1.0, direction)
            self.offsets[name], self.limits[name] = offset, (lo, hi)

    def map(self, raw, action):
        raw = finite_angles(raw)
        result, requested, limited = dict(action), {}, []
        for name in JOINTS:
            center = LEADER_CENTERS[name]
            relative = (raw[name] - center + 180.0) % 360.0 - 180.0
            if math.isclose(abs(relative), 180.0, abs_tol=1e-9):
                raise ValueError(f"Ambiguous raw encoder turn for {name}")
            requested[name] = self.signs[name] * (center + relative) + self.offsets[name]
            lo, hi = self.limits[name]
            target = min(hi, max(lo, requested[name]))
            if target != requested[name]:
                limited.append(name)
            result[f"{name}.pos"] = target
        return result, requested, limited


class PostureWarmup:
    def __init__(self, *, clock=time.monotonic):
        self.clock = clock
        self.started_at = None
        self.status = {"phase": "warming_up", "max_error_deg": None, "limited_joints": []}

    def action(self, targets, actual, requested, limited):
        actual = finite_angles(actual)
        errors = {n: abs(targets[f"{n}.pos"] - actual[n]) for n in JOINTS}
        worst_joint = max(errors, key=errors.get)
        error = errors[worst_joint]
        now = self.clock()
        if self.started_at is None:
            self.started_at = now
        # This is a startup indicator, not an alignment gate. Gravity can
        # leave a persistent tracking error. After five seconds stay in
        # tracking, with the error and active limits still visible.
        phase = "warming_up" if now - self.started_at < 5.0 else "tracking"
        # Do not keep resetting the target to measured position + a tiny
        # error: that also caps proportional torque and can prevent lifting.
        # RateLimitedRobstrideBus already seeds its reference at the fresh
        # engage pose and slews toward this target at the session speed cap.
        # The reference must be able to advance even while the arm lags it.
        self.status = {
            "phase": phase,
            "max_error_deg": error,
            "worst_joint": worst_joint,
            "joint_errors_deg": errors,
            "max_requested_error_deg": max(abs(requested[n] - actual[n]) for n in JOINTS),
            "limited_joints": list(limited),
        }
        return dict(targets)


def prepare_test(request, robot):
    from .arms import registry as arm_registry

    if (
        not arm_registry.get(request.arm_type).supports_joint_posture_mapping
        or request.mode != "single"
        or request.leader_kind not in (None, "", "star")
        or robot.config.id != "viscous_waterproof_01"
    ):
        raise ValueError("This posture trial is for the commissioned waterproof Viscous and Star pair")
    request.speed_limit_deg_s = min(request.speed_limit_deg_s or 15.0, 15.0)
    # Recorded 2026-10-11: old = current - 94.48. Convert the measured J1
    # stops in RAM only, before startup checks. No motor zero or disk rewrite.
    old = (-126.85285584196426, 187.52908634631845)
    corrected = tuple(v + 94.48 for v in old)
    bounds = robot.config.joint_limits["shoulder_pan"]
    if all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(bounds, old, strict=True)):
        robot.config.joint_limits = {**robot.config.joint_limits, "shoulder_pan": corrected}
    elif not all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(bounds, corrected, strict=True)):
        raise ValueError("Waterproof J1 limits use an unrecognized frame; posture trial refused")


def install(robot, leader):
    original_action = leader.get_action
    mapper = JointPostureMapper(
        leader.config.joint_directions, leader.config.joint_offsets, robot.config.joint_limits
    )
    warmup = PostureWarmup()
    last_status_log = [float("-inf")]

    def get_action():
        # Let the existing leader driver own polling, transient cached reads
        # and button handling. Reuse the raw angles it already read; do not
        # add a second poll, reliability gate, or new leader shutdown policy.
        action = original_action()
        targets, requested, limited = mapper.map(dict(leader._last_raw_positions), action)
        if leader.is_locked:
            for name in leader.lockable_motor_names:
                if name in JOINTS and name in last_targets:
                    targets[f"{name}.pos"] = last_targets[name]
                    requested[name] = last_targets[name]
        observation = robot.get_observation()
        actual = {name: observation.get(f"{name}.pos") for name in JOINTS}
        last_targets.update({name: targets[f"{name}.pos"] for name in JOINTS})
        output = warmup.action(targets, actual, requested, limited)
        leader.posture_mapping_status = dict(warmup.status)
        now = time.monotonic()
        if now - last_status_log[0] >= 2.0:
            logger.info(
                "Posture %s: worst %s %.2f deg; joint errors %s; limited %s",
                warmup.status["phase"],
                warmup.status["worst_joint"],
                warmup.status["max_error_deg"],
                warmup.status["joint_errors_deg"],
                limited,
            )
            last_status_log[0] = now
        return output

    last_targets = {}
    # Use the persistent bus slew limit for soft startup, not the driver's
    # measured-position step cap / stall-release path. Normal gains apply
    # throughout; phase changes do not change strength or speed limits.
    robot.config.startup_sync_speed_deg = None
    leader.get_action = get_action
    leader.posture_mapping_status = dict(warmup.status)
    logger.info(
        "1:1 posture trial: fixed offsets %s; directions %s; limits %s; warm-up lasts five seconds",
        mapper.offsets,
        mapper.signs,
        mapper.limits,
    )
