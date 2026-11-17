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
"""Opt-in dry-air fit validation shared by the plan and torque-off start gate."""

import json
import math
from pathlib import Path

import dream_real_core as core
import numpy as np

from makermodslab.arm_gravity import file_sha256, stored_path
from makermodslab.gravity_feedforward import CLAMP_NM, RAMP_S, load_model, support_weight

DEFAULT_FIT = core.ROOT / "configs/viscous/viscous_waterproof_01/gravity-dry-air-fit.json"
MAX_OFFSET_NM = 0.3  # RS00 telemetry bias measured on unit 1 was <= 0.05 Nm


def check(scale, environment, sweep=False):
    if not math.isfinite(scale) or not 0 <= scale <= 1:
        raise ValueError("--gravity-comp must be in [0, 1]")
    if scale and environment != "dry_air":
        raise ValueError("--gravity-comp requires --environment dry_air")
    if scale and sweep:
        raise ValueError("--gravity-comp cannot be combined with --gravity-sweep (raw torque required)")


def load(settings, robot, environment):
    check(settings.gravity_comp, environment, settings.sweep)
    path = Path(settings.gravity_fit or DEFAULT_FIT).expanduser().resolve()
    before = file_sha256(path)
    fit, model = load_model(path)
    if file_sha256(path) != before:
        raise ValueError("gravity fit changed while being loaded")
    if fit.get("fake"):
        raise ValueError("fake gravity fit refused for real use")
    if robot not in [fit.get("robot"), *fit.get("robot_aliases", [])]:
        raise ValueError(f"gravity fit robot {fit.get('robot')!r} does not match record {robot!r}")
    if fit.get("condition") != environment:
        raise ValueError("gravity fit condition must equal the run environment")
    if fit.get("provisional") and not settings.allow_provisional_fit:
        raise ValueError("provisional gravity fit refused; use --allow-provisional-fit explicitly")
    # The physical fit uses motor_to_urdf, not the tank IK pan shift.
    expected_map = {
        n: {k: v[k] for k in ("sign", "offset_deg")}
        for n, v in json.loads((core.ROOT / "configs/viscous/model/motor_to_urdf.json").read_text())[
            "joints"
        ].items()
    }
    if tuple(model.joints) != tuple(core.ARM) or model.joint_map != expected_map:
        raise ValueError("gravity fit must use the six arm joints and commissioned physical joint map")
    if model.urdf_sha256 != file_sha256(core.ROOT / "configs/viscous/model/viscous_arm.urdf"):
        raise ValueError("gravity fit is for another arm URDF")
    if not np.isfinite(model.params).all() or not np.isfinite(model.offsets).all():
        raise ValueError("gravity fit parameters must be finite")
    region = fit.get("region_deg")
    if not isinstance(region, dict) or set(region) != set(core.ARM[1:]):
        raise ValueError("gravity fit region_deg must name the five gravity joints (pan ungated)")
    for bounds in region.values():
        if len(bounds) != 2 or not all(math.isfinite(v) for v in bounds) or bounds[0] > bounds[1]:
            raise ValueError("invalid gravity fit region_deg bounds")
    fade = fit.get("fade_deg", 10.0)
    if not math.isfinite(fade) or fade <= 0:
        raise ValueError("gravity fit fade_deg must be positive and finite")
    model.region_deg, model.fade_deg = region, fade
    # The feedforward commands the gravity terms only (never the fit's offsets or friction). On narrow
    # data a large fitted offset is gravity the fit misfiled: 2026-11-17 the logs-only fit carried
    # -6.4/-7.8/-5.5 Nm offsets and would have commanded 2-2.5x the measured holding torque.
    if np.max(np.abs(model.offsets)) > MAX_OFFSET_NM or any(
        abs(float(v)) > MAX_OFFSET_NM for v in (fit.get("offsets_nm") or [])
    ):
        raise ValueError(
            f"gravity fit offsets exceed {MAX_OFFSET_NM} Nm: the commanded gravity terms would not match "
            "the measured holding torque; refit with --no-offsets"
        )
    support = fit.get("support")
    if fit.get("provisional") and not support:
        raise ValueError("a provisional gravity fit needs a measured-pose support gate")
    if support is not None:
        if list(support.get("joints", [])) != list(core.ARM[1:]) or not support.get("poses"):
            raise ValueError("gravity fit support must list measured poses of the five gravity joints")
        if not all(len(p) == 5 and all(math.isfinite(v) for v in p) for p in support["poses"]):
            raise ValueError("invalid gravity fit support poses")
        support_weight(dict(zip(core.ARM[1:], support["poses"][0], strict=True)), support)  # validates
    identity = {
        "scale": settings.gravity_comp,
        "region_deg": region,
        "fade_deg": fade,
        "weight": "product of cosine factors; measured motor degrees; pan ungated",
        "support": None
        if support is None
        else {
            "poses": len(support["poses"]),
            "full_deg": support["full_deg"],
            "fade_deg": support["fade_deg"],
            "weight": "x cosine of the distance to the nearest measured pose",
        },
        "max_offset_nm": MAX_OFFSET_NM,
        "fit": stored_path(path),
        "fit_sha256": before,
        "robot": robot,
        "condition": environment,
        "provisional": bool(fit.get("provisional")),
        "allow_provisional_fit": settings.allow_provisional_fit,
        "sag_mm_per_m2": core.SAG_MM_PER_M2,
        "ramp_in_s": RAMP_S,
        "ramp_out_s": 1.0,
        "clamp_nm": CLAMP_NM,
        "limp_reference": "cmd + degrees(ff/kp) (effort equivalent)",
    }
    return identity, model


class FeedforwardLimp(core.LimpMonitor):
    """Test zero measured effort against the combined MIT effort demand.

    A negative ff can cancel kp*error at a healthy joint more than 3 degrees
    from its goal. Express the commanded effort as an equivalent positional
    error for this monitor only; ordinary lag and HoldError keep raw goals.
    """

    def __init__(self, limits, arm):
        super().__init__(limits.limp_torque_nm, limits.limp_lag_deg, limits.limp_ticks)
        self.arm = arm

    def update(self, measured, commanded, torque):
        from gravity_fit import KP

        ff = self.arm.gravity_status["feedforward_nm"]
        demand = {**commanded, **{n: commanded[n] + math.degrees(ff.get(n, 0.0) / KP[n]) for n in core.ARM}}
        trips = super().update(measured, demand, torque)
        return [core.Trip(t.name, t.detail + " [combined kp*error + ff demand]") for t in trips]
