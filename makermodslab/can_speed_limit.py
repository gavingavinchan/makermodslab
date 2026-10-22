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

"""Persistent local RobStride target slew limits, installed before connect.

Arm Goal_Position writes, including startup and return helpers, share the
same limiter. The gripper retains its normal driver behavior. This bounds
commanded arm angles; gravity and tracking dynamics can exceed target speed.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from .remote_can import limit_action

logger = logging.getLogger(__name__)


class RateLimitedRobstrideBus:
    def __init__(self, robot, speed_deg_s: float, *, clock=time.monotonic):
        if not math.isfinite(speed_deg_s) or not 0 < speed_deg_s <= 30:
            raise ValueError("Teleop target speed must be greater than zero and at most 30 deg/s")
        self._robot = robot
        self._base = robot.bus
        self.speed_limit_deg_s = float(speed_deg_s)
        self._clock = clock
        self._last = {}
        self._last_time = {}
        self._lock = threading.RLock()
        self._last_log = -math.inf
        self.peak_measured_speed_deg_s = 0.0

    def __getattr__(self, name):
        return getattr(self._base, name)

    def _name(self, motor):
        return self._base._get_motor_name(motor)

    def connect(self, handshake=True):
        from .monitored_can import MonitorCanClient

        proxy = MonitorCanClient.discover(
            self._robot.config.port,
            self._robot.config.id,
            self.speed_limit_deg_s,
        )
        if proxy is None:
            return self._base.connect(handshake=handshake)
        self._base.canbus = proxy
        self._base._is_connected = True
        # A local socket hop and the monitor poll are longer than the SDK's
        # 3 ms default. This is a maximum wait, not a fixed added delay.
        original_query = self._base._query_status_via_clear_fault

        def query(motor, timeout=0.05):
            return original_query(motor, timeout=max(0.05, timeout))

        self._base._query_status_via_clear_fault = query
        try:
            if handshake:
                self._base._handshake()
        except BaseException:
            proxy.shutdown()
            self._base.canbus = None
            self._base._is_connected = False
            raise

    def _fresh_position(self, name):
        # The usual driver can serve stale cached values. Startup must get
        # an actual reply before we preload or enable ANY joint.
        fault, message = self._base._query_status_via_clear_fault(name, timeout=0.05)
        if fault or message is None or len(message.data) != 8:
            raise ConnectionError(f"No fresh valid starting pose for {name}; refusing torque enable")
        self._base._decode_motor_state(message.data)
        value = float(self._base.read("Present_Position", name))
        if not math.isfinite(value):
            raise ValueError(f"Invalid starting position for {name}")
        offset = getattr(self._robot, "_turn_offset", {}).get(name, 0.0)
        lo, hi = self._robot.config.joint_limits[name]
        position = value + offset
        # Starting outside the limits is allowed (e.g. resting on the table):
        # torque is seeded at the measured pose and the clamped targets ease
        # the joint inside at the slew limit, so there is no jump.
        if not lo <= position <= hi:
            logger.warning(
                "%s starts at %.2f deg, outside its limits (%.1f, %.1f); easing it inside at the slew limit",
                name,
                position,
                lo,
                hi,
            )
        return value

    def enable_torque(self, motors=None, num_retry=0):
        with self._lock:
            names = self._base._get_motors_list(motors)
            # Validate the complete selected set BEFORE sending any preload
            # or enable. Then seed at exactly the measured motor coordinates.
            present = {self._name(m): self._fresh_position(self._name(m)) for m in names}
            for name, value in present.items():
                self._base.write("Goal_Position", name, value)
                self._last[name] = value
                self._last_time[name] = self._clock()
            self._base.enable_torque(motors, num_retry=num_retry)
            logger.info(
                "Teleop arm target speed limited to %.2f deg/s; gripper uses normal speed; motors %s",
                self.speed_limit_deg_s,
                names,
            )

    def _zero_effort(self, name):
        gains = self._base._gains[name]
        return gains["kp"] == 0 and gains["kd"] == 0

    def write(self, data_name, motor, value):
        with self._lock:
            if data_name == "Goal_Velocity":
                raise ValueError("Velocity-mode commands cannot bypass the teleop position speed limit")
            if data_name != "Goal_Position":
                return self._base.write(data_name, motor, value)
            name = self._name(motor)
            target = float(value)
            if not math.isfinite(target):
                raise ValueError("Non-finite teleop position target")
            # Cleanup's zero-gain MIT packets are not motion commands and
            # must remain available even after a failed setup or write.
            if self._zero_effort(name):
                return self._base.write(data_name, motor, value)
            if name not in self._last:
                raise RuntimeError("Slow teleop was not initialized from a fresh measured pose")
            now = self._clock()
            if now < self._last_time[name]:
                raise RuntimeError("Monotonic clock moved backwards")
            # The gripper uses the normal follower's clamped position target.
            # It still requires the same fresh pre-enable initialization.
            if name == "gripper":
                next_value = target
            else:
                next_value = limit_action(
                    {name: self._last[name]},
                    {name: target},
                    now - self._last_time[name],
                    speed_deg_s=self.speed_limit_deg_s,
                )[name]
            self._base.write(data_name, motor, next_value)
            # Only a successful command advances the last-sent reference.
            self._last[name], self._last_time[name] = next_value, now
            states = getattr(self._base, "_last_known_states", {})
            velocities = {
                m: float(s["velocity"])
                for m, s in states.items()
                if isinstance(s.get("velocity"), (int, float)) and math.isfinite(s["velocity"])
            }
            if velocities:
                peak = max(abs(v) for v in velocities.values())
                self.peak_measured_speed_deg_s = max(self.peak_measured_speed_deg_s, peak)
                if now - self._last_log >= 1:
                    logger.info(
                        "Slow teleop: target cap %.2f deg/s; measured velocities %s; peak %.2f deg/s",
                        self.speed_limit_deg_s,
                        velocities,
                        self.peak_measured_speed_deg_s,
                    )
                    self._last_log = now

    def sync_write(self, data_name, values, *args, **kwargs):
        if data_name == "Goal_Position":
            with self._lock:
                # Validate all fields before the first write of the batch.
                for motor, value in values.items():
                    self._name(motor)
                    if not math.isfinite(float(value)):
                        raise ValueError("Non-finite teleop position target")
                for motor, value in values.items():
                    self.write(data_name, motor, value)
        elif data_name == "Goal_Velocity":
            raise ValueError("Velocity mode cannot bypass the teleop speed limit")
        else:
            return self._base.sync_write(data_name, values, *args, **kwargs)


def install(robot, speed_deg_s: float) -> None:
    from .maker_rest_pose import maker_follower_arms

    for arm, _label in maker_follower_arms(robot):
        arm.bus = RateLimitedRobstrideBus(arm, speed_deg_s)
