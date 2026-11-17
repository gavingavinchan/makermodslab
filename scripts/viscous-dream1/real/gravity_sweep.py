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
"""Pure offline STAR sweep planner. Execution belongs exclusively to Episode.

All retained legs are checked at 20 Hz, including predicted sag and a proved
straight return to the hub from every substep. No hardware or Dream imports.
"""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import dream_real_core as core
import numpy as np
from gravity_fit import DEFAULT_OUT, KP, joint_map

from makermodslab.arm_gravity import VISCOUS_URDF, GravityModel, file_sha256, stored_path
from makermodslab.gravity_feedforward import load_model

MOVING = core.ARM[1:]
APPROACH = MOVING[:3]
# Broad motor-degree exploration around the hub, narrowed by soft limits,
# post, desk, sag, return proofs and the empirical load cap.
HALF_WIDTH = dict(zip(MOVING, (65.0, 85.0, 70.0, 40.0, 90.0), strict=True))
REFERENCE = DEFAULT_OUT.parent / "evidence/gravity-dry-air-selected.json"


@dataclasses.dataclass(frozen=True)
class SweepSettings:
    seed: int = 0
    poses: int = 30
    hold_s: float = 2.0
    max_torque_nm: float = 6.0
    hot_c: float = 65.0
    start_at: int = 0
    box: dict | None = None  # absolute motor degrees, otherwise hub +/- HALF_WIDTH
    fit: Path = DEFAULT_OUT

    def check(self):
        if not 1 <= self.poses <= 200 or not 0 <= self.start_at <= self.poses:
            raise ValueError("sweep poses must be 1..200 and start-at 0..poses")
        if not (math.isfinite(self.hold_s) and 1.0 <= self.hold_s <= 60.0):
            raise ValueError("sweep hold must be finite and in [1, 60] s")
        if not (0 < self.max_torque_nm <= 6.0 and 20 <= self.hot_c <= 65.0):
            raise ValueError("sweep max torque must be in (0, 6] Nm and hot in [20, 65] C")
        if self.box is not None:
            if set(self.box) != set(MOVING):
                raise ValueError("sweep box must give exactly the five moving joints")
            if any(
                len(v) != 2 or not all(math.isfinite(x) for x in v) or v[0] >= v[1] for v in self.box.values()
            ):
                raise ValueError("sweep box bounds must be finite increasing pairs")

    def identity(self):
        self.check()
        fit = Path(self.fit).expanduser()
        return {
            **dataclasses.asdict(self),
            "fit": stored_path(fit),
            "fit_sha256": file_sha256(fit) if fit.is_file() else "missing",
            "reference_sha256": file_sha256(REFERENCE) if REFERENCE.is_file() else "missing",
            "cad_sha256": file_sha256(VISCOUS_URDF),
            "joint_map": joint_map(),
            "kp_nm_rad": KP,
            "delta_deg": 6.0,
            "soft_margin_deg": 8.0,
            "half_width_deg": HALF_WIDTH,
            "candidate_pool": max(96, self.poses * 24),
        }


class Predictor:
    def __init__(self, settings, hub):
        self.friction = dict.fromkeys(core.ARM, 0.0)
        self.scale = dict.fromkeys(core.ARM, 1.0)
        try:
            fit, model = load_model(settings.fit, telemetry_offsets=True)
            if fit.get("robot") != "viscous_waterproof_01" or fit.get("condition") != "dry_air":
                raise ValueError("wrong robot/condition")
            if model.joint_map != joint_map():
                raise ValueError("joint map differs")
            if not np.isfinite(model.params).all() or not np.isfinite(model.offsets).all():
                raise ValueError("nonfinite parameters")
            if model.urdf_sha256 != file_sha256(VISCOUS_URDF):
                raise ValueError("fit is for another URDF")
            errors = list(fit["held_out_rms_nm"].values())
            if not errors or not np.isfinite(errors).all():
                raise ValueError("invalid held-out errors")
            if fit["identification"]["rank"] < 6 or max(errors) > 1.0:
                raise ValueError("rank <6 or held-out RMS >1 Nm")
            friction = fit.get("friction", self.friction)
            if set(friction) != set(core.ARM) or not np.isfinite(list(friction.values())).all():
                raise ValueError("invalid friction estimates")
            self.model, self.friction = model, friction
            self.description = "provisional empirical fit (planning only): " + stored_path(settings.fit)
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.model = GravityModel(VISCOUS_URDF, core.ARM, joint_map=joint_map())
            reference = json.loads(REFERENCE.read_text())["rows"]
            if not reference:
                raise ValueError("CAD fallback needs measured start-pose holding torques") from error
            row = min(reference, key=lambda r: sum((r["cmd"][n] - hub[n]) ** 2 for n in MOVING))
            cad = self.model.torque(row["pos"])
            self.scale = {n: row["torque"][n] / cad[n] if abs(cad[n]) >= 0.1 else 1.0 for n in core.ARM}
            self.description = (
                f"CAD scaled to nearest measured start hold {row['source']} "
                f"(fallback: {error}); near-zero CAD axes retain scale 1"
            )

    def torque(self, pose):
        base = self.model.torque(pose)
        # Worst Coulomb magnitude, so either direction fits the planning cap.
        return {n: self.scale[n] * base[n] + math.copysign(abs(self.friction[n]), base[n]) for n in core.ARM}

    def sagged(self, pose):
        tau = self.torque(pose)
        return {**pose, **{n: pose[n] - math.degrees(tau[n] / KP[n]) for n in core.ARM}}


def plan(
    settings, hub, limits, box, guard, speed, *, predictor=None, say=None, pool=None, stop_path_clear=None
):
    settings.check()
    if not 0 < speed <= 15:
        raise ValueError("sweep speed must be <=15 deg/s")
    if not guard.available:
        raise ValueError("sweep requires the mesh guard")
    predictor = predictor or Predictor(settings, hub)
    mesh = guard.guard
    bounds = {}
    for n in MOVING:
        lo, hi = (settings.box or {}).get(n, (hub[n] - HALF_WIDTH[n], hub[n] + HALF_WIDTH[n]))
        bounds[n] = (max(lo, limits[n][0] + 8), min(hi, limits[n][1] - 8))
        if bounds[n][0] >= bounds[n][1]:
            raise ValueError(f"empty sweep box for {n}")

    def pose_ok(p):
        return (
            all(limits[n][0] <= p[n] <= limits[n][1] for n in core.ARM)
            and not box.violations(p)
            and mesh.check_pose(p).ok
        )

    def checked(p):
        if not pose_ok(p):
            return False
        tau = predictor.torque(p)
        if not all(math.isfinite(v) and abs(v) <= settings.max_torque_nm for v in tau.values()):
            return False
        sag = predictor.sagged(p)
        if not (pose_ok(sag) and mesh.check_return_segment(sag, hub).ok):
            return False
        # The existing stop may skip the hub when already within its lag band.
        # Episode also proves that first leg, without changing the stop itself.
        return stop_path_clear is None or (stop_path_clear(p) and stop_path_clear(sag))

    if not checked(hub):
        raise ValueError("sweep hub fails limits/box/mesh/sag/torque/return checks")

    def leg(a, b):
        # Reject long unsafe segments before doing every substep's return proof.
        if not mesh.check_segment(a, b).ok:
            return None
        steps = core.retime(a, b, arm_deg_s=speed, grip_deg_s=60.0)
        previous = a
        for q in steps:
            if not checked(q):
                return None
            decision = mesh.limit(previous, q)
            if decision.status != "ok" or decision.target != q:
                return None
            if not mesh.check_return_segment(q, hub).ok:
                return None
            previous = q
        return steps

    rng = np.random.default_rng(settings.seed)
    candidates = []
    attempts = pool if pool is not None else max(96, settings.poses * 24)
    for attempt in range(attempts):
        if say and attempt % 24 == 0:
            say(f"sweep planning: candidate {attempt}/{attempts}, {len(candidates)} accepted")
        pose = {**hub, **{n: float(rng.uniform(*bounds[n])) for n in MOVING}}
        if not checked(pose):
            continue
        routes, here = [], hub
        for sign in (1, -1):
            approach = {**pose, **{n: pose[n] + sign * 6.0 for n in APPROACH}}
            if not all(limits[n][0] + 8 <= approach[n] <= limits[n][1] - 8 for n in MOVING):
                continue
            into, settle = leg(here, approach), leg(approach, pose)
            if into is not None and settle is not None:
                routes.append({"direction": "+" if sign == 1 else "-", "into": into, "settle": settle})
                here = pose
        if not routes:
            continue
        back = leg(pose, hub)
        if back is not None:
            candidates.append({"pose": pose, "routes": routes, "back": back})
    if len(candidates) < settings.poses:
        raise ValueError(
            f"only {len(candidates)} checked poses for {settings.poses} requested; narrow box or reduce count"
        )

    def gap(a, b):
        return sum((a[n] - b[n]) ** 2 for n in MOVING)

    # All candidates are already fully proved before farthest-point thinning.
    chosen = [max(candidates, key=lambda c: gap(c["pose"], hub))]
    candidates.remove(chosen[0])
    while len(chosen) < settings.poses:
        pick = max(candidates, key=lambda c: min(gap(c["pose"], v["pose"]) for v in chosen))
        chosen.append(pick)
        candidates.remove(pick)
    ordered, here = [], hub
    while chosen:
        pick = min(chosen, key=lambda c: gap(c["pose"], here))
        ordered.append(pick)
        chosen.remove(pick)
        here = pick["pose"]
    if settings.start_at > len(ordered):
        raise ValueError("start-at exceeds checked pose count")
    worst = dict.fromkeys(core.ARM, 0.0)
    duration, holds = 0.0, 0
    for visit in ordered[settings.start_at :]:
        for route in visit["routes"]:
            holds += 1
            for q in route["into"] + route["settle"]:
                tau = predictor.torque(q)
                worst = {n: max(worst[n], abs(tau[n])) for n in core.ARM}
            duration += (len(route["into"]) + len(route["settle"])) * core.ROW_DT_S + settings.hold_s
        duration += len(visit["back"]) * core.ROW_DT_S
    all_steps = (
        [hub]
        + [q for v in ordered for r in v["routes"] for q in r["into"] + r["settle"]]
        + [q for v in ordered for q in v["back"]]
    )
    worst = {n: max(abs(predictor.torque(q)[n]) for q in all_steps) for n in core.ARM}
    duration += (1 + len(ordered) - settings.start_at) * core.ROW_DT_S
    report = {
        "planned_substeps": len(all_steps),
        "poses": len(ordered),
        "remaining_poses": len(ordered) - settings.start_at,
        "holds": holds,
        "coverage_deg": {
            n: [min(v["pose"][n] for v in ordered), max(v["pose"][n] for v in ordered)] for n in MOVING
        },
        "predicted_max_torque_nm": worst,
        "sweep_duration_s": duration,
        "predictor": predictor.description,
        "box_deg": bounds,
        "one_direction_poses": [i for i, v in enumerate(ordered) if len(v["routes"]) == 1],
    }
    return ordered, report


def actions(visits, settings):
    """(phase, target, hold metadata) queue; every pose ends at the hub."""
    for i, visit in enumerate(visits[settings.start_at :], settings.start_at):
        prefix = f"sweep: pose {i + 1}/{len(visits)}"
        for route in visit["routes"]:
            direction = route["direction"]
            for q in route["into"] + route["settle"]:
                yield f"{prefix} approach from {direction}", q, None
            count = math.ceil(settings.hold_s / core.ROW_DT_S)
            for tick in range(count):
                yield (
                    f"{prefix} hold from {direction}",
                    visit["pose"],
                    {"pose": i, "direction": direction, "last": tick == count - 1},
                )
        for q in visit["back"]:
            yield f"{prefix} return hub", q, None
        yield f"sweep: hub after pose {i + 1}/{len(visits)}", visit["back"][-1], {"hub": i + 1}
