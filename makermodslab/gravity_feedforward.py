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
"""Gravity feedforward on a CAN follower's MIT commands (bench switch).

Every follower command is an MIT frame: ``kp * (target - q) + kd * (0 - dq) +
tau_ff``, and the stock bus always sends ``tau_ff = 0``, so the arm holds its
own weight through position error alone (sag). With a gravity fit from
``scripts/viscous-gravity-fit.py`` this fills ``tau_ff`` with the torque the
fitted model says holds the arm at its MEASURED pose, so position error is left
for contact.

Opt-in from the environment, read when a session builds its follower:

    MAKERMODSLAB_GRAVITY_FF=configs/viscous/viscous_01/gravity-air-fit.json   # repo-relative or absolute
    MAKERMODSLAB_GRAVITY_FF_SCALE=0.5   # 0..1, default 1

The fit must name this robot record. The torque ramps in over RAMP_S at
session start and is clamped to CLAMP_NM per joint. The fit's per-joint
offsets are telemetry bias and are not commanded.
"""

from __future__ import annotations

import json
import logging
import math
import os
import time
from pathlib import Path

logger = logging.getLogger(__name__)

ENV_FIT = "MAKERMODSLAB_GRAVITY_FF"
ENV_SCALE = "MAKERMODSLAB_GRAVITY_FF_SCALE"
RAMP_S = 2.0
CLAMP_NM = 6.0
LOG_PERIOD_S = 2.0
FRESH_S = 0.005  # one model evaluation per send_action tick


def load_model(path: str | Path):
    from .arm_gravity import GravityModel, repo_path

    fit = json.loads(repo_path(path).read_text())
    model = GravityModel(repo_path(fit["urdf"]), fit["joints"], params=fit["params"])
    if model.urdf_sha256 != fit["urdf_sha256"]:
        raise ValueError(f"The URDF at {fit['urdf']} changed since this fit was made")
    return fit, model


def install(robot, model, *, scale: float = 1.0, clock=time.monotonic) -> dict:
    """Wrap ``robot.bus.write`` so Goal_Position frames carry gravity torque.

    Returns a live status dict (latest feedforward and stiffness torque per
    joint) that the wrapper keeps updated.
    """
    if not 0.0 <= scale <= 1.0:
        raise ValueError("Gravity feedforward scale must be within 0..1")
    # Hook the innermost bus. A wrapper such as can_speed_limit's rate limiter
    # sits on robot.bus and delegates to ``_base``: patching the wrapper's write
    # would send frames straight to the motors and bypass its slew limit, while
    # patching the base lets the wrapper's limited target flow through here.
    bus = robot.bus
    while getattr(type(bus), "__getattr__", None) is not None and "_base" in vars(bus):
        bus = bus._base
    original_write = bus.write
    joints = set(model.joints)
    state = {"started": None, "computed": -math.inf, "tau": {}, "logged": -math.inf}
    status: dict = {"scale": scale, "feedforward_nm": {}, "stiffness_nm": {}}

    def measured_degrees():
        # The follower's latest reading, in joint coordinates (raw + whole-turn correction).
        return {m: robot._last_positions[m] + robot._turn_offset.get(m, 0.0) for m in model.joints}

    def write(data_name, motor, value):
        if data_name != "Goal_Position":
            return original_write(data_name, motor, value)
        name = bus._get_motor_name(motor)
        torque = 0.0
        if name in joints:
            now = clock()
            if state["started"] is None:
                state["started"] = now
            if now - state["computed"] > FRESH_S:
                try:
                    state["tau"] = model.torque(measured_degrees())
                except Exception:
                    # No reading yet (first tick) or a non-finite one: hold with stiffness alone.
                    state["tau"] = {}
                state["computed"] = now
            ramp = min(1.0, (now - state["started"]) / RAMP_S)
            torque = max(-CLAMP_NM, min(CLAMP_NM, scale * ramp * state["tau"].get(name, 0.0)))
            kp = bus._gains[name]["kp"]
            raw = robot._last_positions.get(name)
            status["feedforward_nm"][name] = torque
            if raw is not None:
                status["stiffness_nm"][name] = kp * math.radians(value - raw)
            if now - state["logged"] >= LOG_PERIOD_S and name == model.joints[-1]:
                state["logged"] = now
                logger.info(
                    "Gravity feedforward x%.2f (ramp %.0f%%): %s",
                    scale,
                    100 * ramp,
                    "  ".join(
                        f"{j} ff {status['feedforward_nm'].get(j, 0.0):+.2f} kp*err "
                        f"{status['stiffness_nm'].get(j, float('nan')):+.2f}"
                        for j in ("shoulder_lift", "elbow_flex", "wrist_flex")
                        if j in joints
                    ),
                )
        gains = bus._gains[name]
        bus._mit_control(motor, gains["kp"], gains["kd"], value, 0, torque)

    bus.write = write
    return status


def install_from_env(robot, robot_name: str) -> dict | None:
    """Install from MAKERMODSLAB_GRAVITY_FF if it is set and names this robot; else do nothing.

    The fit names one robot record, which already pins the arm; the check here
    is only that this follower speaks MIT frames through a bus we can wrap.
    """
    path = os.environ.get(ENV_FIT)
    if not path:
        return None
    bus = getattr(robot, "bus", None)
    if not all(hasattr(bus, a) for a in ("_mit_control", "_gains")) or not hasattr(robot, "_last_positions"):
        logger.warning(
            "%s is set but %s is not a single MIT-mode CAN follower; not applied", ENV_FIT, robot_name
        )
        return None
    fit, model = load_model(path)
    if fit.get("robot") != robot_name:
        logger.warning("Gravity fit %s is for %s, not %s; not applied", path, fit.get("robot"), robot_name)
        return None
    scale = float(os.environ.get(ENV_SCALE, "1.0"))
    status = install(robot, model, scale=scale)
    logger.info(
        "Gravity feedforward ON for %s from %s at x%.2f (ramps in over %.0f s)",
        robot_name,
        path,
        scale,
        RAMP_S,
    )
    return status
