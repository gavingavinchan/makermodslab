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

import collections
import copy
import dataclasses
import hashlib
import json
import math
import time
from pathlib import Path

import dream_real_core as core
import numpy as np
from gravity_fit import DEFAULT_OUT, KP, joint_map

from makermodslab.arm_gravity import VISCOUS_URDF, GravityModel, file_sha256, stored_path
from makermodslab.gravity_feedforward import load_model

MOVING = core.ARM[1:]
APPROACH = MOVING[:3]
# Logged Dream pocket plus the modest lift requested after a grasp. The post,
# sag and load proofs, rather than this sampling box, bound achieved coverage.
DEFAULT_BOX = dict(
    zip(MOVING, ((-86.0, -50.0), (36.0, 85.0), (-55.0, -20.0), (-10.0, 10.0), (-10.0, 10.0)), strict=True)
)
REFERENCE = DEFAULT_OUT.parent / "evidence/gravity-dry-air-selected.json"


@dataclasses.dataclass(frozen=True)
class SweepSettings:
    seed: int = 0
    poses: int = 14
    hold_s: float = 2.0
    max_torque_nm: float = 6.0
    hot_c: float = 65.0
    start_at: int = 0
    box: dict | None = None  # None selects DEFAULT_BOX
    fit: Path = DEFAULT_OUT
    plan_file: Path | None = None

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
        plan_file = Path(self.plan_file).expanduser() if self.plan_file else None
        return {
            **dataclasses.asdict(self),
            "fit": stored_path(fit),
            "fit_sha256": file_sha256(fit) if fit.is_file() else "missing",
            "reference_sha256": file_sha256(REFERENCE) if REFERENCE.is_file() else "missing",
            "cad_sha256": file_sha256(VISCOUS_URDF),
            "joint_map": joint_map(),
            "kp_nm_rad": KP,
            "delta_deg": 4.0,
            "approach": "radial then per-joint signs; require both",
            "selection": "cheap endpoints, six pocket, farthest then full proof v4",
            "hub_planning_cap_exempt": True,
            "soft_margin_deg": 8.0,
            "box": self.box or DEFAULT_BOX,
            "plan_file": stored_path(plan_file) if plan_file else None,
            "plan_sha256": file_sha256(plan_file) if plan_file else None,
            "candidate_pool": max(64, self.poses * 8),
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


def saved_identity(settings):
    # Resume changes execution only; all visits must still be proved.
    return {k: v for k, v in settings.identity().items() if k not in ("plan_file", "plan_sha256", "start_at")}


class Proof:
    """Run-local proof cache. Never changes the tank guard or its tolerances."""

    def __init__(self, settings, hub, limits, box, guard, speed, predictor, stop_path_clear, return_hub=None):
        settings.check()
        if not math.isfinite(speed) or not 0 < speed <= 15:
            raise ValueError("sweep speed must be <=15 deg/s")
        if not guard.available:
            raise ValueError("sweep requires the mesh guard")
        self.settings, self.hub, self.limits = settings, hub, limits
        self.box, self.speed, self.predictor = box, speed, predictor
        self.stop_path_clear = stop_path_clear
        self.return_hub = hub if return_hub is None else return_hub
        # _scan often revisits endpoints. Cache exact coordinates and relaxation,
        # not rounded geometry, and discard the cache after this start gate.
        self.mesh = copy.copy(guard.guard)
        raw = self.mesh.clearances
        clearance_cache = {}

        def clearances(q, relax=None):
            key = (tuple(q[n] for n in core.MOTORS), tuple(sorted((relax or {}).items())))
            if key not in clearance_cache:
                clearance_cache[key] = raw(q, relax)
            return clearance_cache[key]

        self.mesh.clearances = clearances
        self.cache = {}
        self.bounds = {
            n: (max(lo, limits[n][0] + 8), min(hi, limits[n][1] - 8))
            for n, (lo, hi) in (settings.box or DEFAULT_BOX).items()
        }
        if any(lo >= hi for lo, hi in self.bounds.values()):
            raise ValueError("empty sweep box after soft margin")
        self.checked(hub)

    def pose(self, q):
        if not isinstance(q, dict) or set(q) != set(core.MOTORS):
            raise ValueError("pose must contain exactly seven motors")
        if any(not isinstance(v, (float, int)) or not math.isfinite(v) for v in q.values()):
            raise ValueError("nonfinite pose")
        if any(not self.limits[n][0] <= q[n] <= self.limits[n][1] for n in core.MOTORS):
            raise ValueError("joint limit")
        if self.box.violations(q):
            raise ValueError("workspace box")
        verdict = self.mesh.check_pose(q)
        if not verdict.ok:
            raise ValueError("guard:" + str(verdict.face))

    def endpoint(self, q):
        # Cheap filters precede mesh FK and all path/return scans.
        if any(not self.limits[n][0] <= q[n] <= self.limits[n][1] for n in core.MOTORS):
            raise ValueError("joint limit")
        tau = self.predictor.torque(q)
        if not all(math.isfinite(v) for v in tau.values()):
            raise ValueError("nonfinite predicted torque")
        if q != self.hub:
            for n, value in tau.items():
                if abs(value) > self.settings.max_torque_nm:
                    raise ValueError("cap:" + n)
        self.pose(q)
        self.pose(self.predictor.sagged(q))

    def checked(self, q):
        key = tuple(q[n] for n in core.MOTORS)
        if key in self.cache:
            return
        self.endpoint(q)
        sag = self.predictor.sagged(q)
        for pose in (q, sag):
            if not self.mesh.check_return_segment(pose, self.return_hub).ok:
                raise ValueError("return to hub")
            if self.stop_path_clear is not None and not self.stop_path_clear(pose):
                raise ValueError("stop_path")
        self.cache[key] = True

    def steps(self, a, steps, endpoint):
        if not isinstance(steps, list) or not steps:
            raise ValueError("empty leg")
        previous = a
        for q in steps:
            self.pose(q)
            if q["shoulder_pan"] != self.hub["shoulder_pan"] or q["gripper"] != self.hub["gripper"]:
                raise ValueError("sweep pan/gripper must stay fixed at hub")
            if any(abs(q[n] - previous[n]) > self.speed * core.ROW_DT_S + 1e-8 for n in core.ARM):
                raise ValueError("substep speed")
            self.checked(q)
            decision = self.mesh.limit(previous, q)
            if decision.status != "ok" or decision.target != q:
                raise ValueError("path:limit_or_guard")
            previous = q
        if any(abs(previous[n] - endpoint[n]) > 1e-9 for n in core.MOTORS):
            raise ValueError("leg endpoint differs")
        return steps

    def leg(self, a, b):
        if not self.mesh.check_segment(a, b).ok:
            raise ValueError("path:guard")
        return self.steps(a, core.retime(a, b, arm_deg_s=self.speed, grip_deg_s=60.0), b)


def report_visits(visits, settings, hub, proof, failures=None):
    steps = [hub] + [q for v in visits for r in v["routes"] for q in r["into"] + r["settle"]]
    steps += [q for v in visits for q in v["back"]]
    remaining = visits[settings.start_at :]
    duration = (
        sum(
            sum((len(r["into"]) + len(r["settle"])) * core.ROW_DT_S + settings.hold_s for r in v["routes"])
            + len(v["back"]) * core.ROW_DT_S
            for v in remaining
        )
        + (1 + len(remaining)) * core.ROW_DT_S
    )
    return {
        "rejections": dict(sorted((failures or {}).items())),
        "pocket_poses": sum(in_pocket(v["pose"]) for v in visits),
        "both_direction_poses": len(visits),
        "one_direction_poses": [],
        "planned_substeps": len(steps),
        "poses": len(visits),
        "remaining_poses": len(remaining),
        "holds": 2 * len(remaining),
        "coverage_deg": {
            n: [min(v["pose"][n] for v in visits), max(v["pose"][n] for v in visits)] for n in MOVING
        },
        "predicted_max_torque_nm": {
            n: max(abs(proof.predictor.torque(q)[n]) for q in steps) for n in core.ARM
        },
        "sweep_duration_s": duration,
        "predictor": proof.predictor.description,
        "box_deg": proof.bounds,
        "hub_planning_cap_exempt": True,
        "coverage_bound": "requested box, soft margin, post/workspace mesh, predicted sag/load, both approaches and stop-return proofs",
    }


def in_pocket(p):
    return -76 <= p["shoulder_lift"] <= -57 and 37 <= p["elbow_flex"] <= 60 and -45 <= p["wrist_flex"] <= -29


def plan(
    settings, hub, limits, box, guard, speed, *, predictor=None, say=None, pool=None, stop_path_clear=None
):
    started = time.perf_counter()
    predictor = predictor or Predictor(settings, hub)
    try:
        proof = Proof(settings, hub, limits, box, guard, speed, predictor, stop_path_clear)
    except ValueError as error:
        raise ValueError(f"sweep hub fails: {error}") from error
    rng = np.random.default_rng(settings.seed)
    failures = collections.Counter()
    candidates = []
    attempts = pool if pool is not None else max(64, settings.poses * 8)
    # Reserve local logged-pocket measurements before spreading over the box.
    for i in range(attempts):
        bounds = proof.bounds
        if i < attempts // 3:
            pocket = {
                "shoulder_lift": (-76.0, -70.0),
                "elbow_flex": (48.0, 59.0),
                "wrist_flex": (-42.0, -32.0),
            }
            bounds = {
                n: (max(lo, pocket.get(n, (lo, hi))[0]), min(hi, pocket.get(n, (lo, hi))[1]))
                for n, (lo, hi) in bounds.items()
            }
            if any(lo >= hi for lo, hi in bounds.values()):
                continue
        pose = {**hub, **{n: float(rng.uniform(*bounds[n])) for n in MOVING}}
        radius = max(abs(pose[n] - hub[n]) for n in APPROACH)
        vectors = [
            {n: 4.0 * (pose[n] - hub[n]) / max(radius, 1.0) for n in APPROACH},
            {n: 4.0 * (1 if pose[n] >= hub[n] else -1) for n in APPROACH},
            dict.fromkeys(APPROACH, 4.0),
        ]
        viable = []
        for delta in vectors:
            try:
                for q in [
                    pose,
                    *[{**pose, **{n: pose[n] + sign * delta[n] for n in APPROACH}} for sign in (1, -1)],
                ]:
                    if any(not limits[n][0] + 8 <= q[n] <= limits[n][1] - 8 for n in MOVING):
                        raise ValueError("side:margin")
                    proof.endpoint(q)
                viable.append(delta)
            except ValueError as error:
                failures[str(error)] += 1
        if viable:
            candidates.append((pose, viable))
    chosen = []

    def gap(a, b):
        return sum((a[n] - b[n]) ** 2 for n in MOVING)

    while candidates and len(chosen) < settings.poses:
        pocket_count = sum(in_pocket(v["pose"]) for v in chosen)
        local = [c for c in candidates if in_pocket(c[0])] if pocket_count < min(6, settings.poses) else []
        pick = max(
            local or candidates,
            key=lambda c: min(gap(c[0], v["pose"]) for v in chosen) if chosen else gap(c[0], hub),
        )
        candidates.remove(pick)
        pose, vectors = pick
        if say:
            say(
                f"sweep planning: full proof {len(chosen)}/{settings.poses}; {len(candidates)} endpoint survivors remain"
            )
        for delta in vectors:
            try:
                routes, here = [], hub
                for sign in (1, -1):
                    approach = {**pose, **{n: pose[n] + sign * delta[n] for n in APPROACH}}
                    routes.append(
                        {
                            "direction": "+" if sign == 1 else "-",
                            "delta_deg": delta,
                            "into": proof.leg(here, approach),
                            "settle": proof.leg(approach, pose),
                        }
                    )
                    here = pose
                chosen.append({"pose": pose, "routes": routes, "back": proof.leg(pose, hub)})
                break
            except ValueError as error:
                failures[str(error)] += 1
    if len(chosen) != settings.poses:
        raise ValueError(f"only {len(chosen)} checked poses for {settings.poses} requested: {dict(failures)}")
    chosen.sort(key=lambda v: (not in_pocket(v["pose"]), gap(v["pose"], hub)))
    report = report_visits(chosen, settings, hub, proof, failures)
    report["planning_time_s"] = time.perf_counter() - started
    return chosen, report


def save_plan(path, settings, hub, visits, report, speed):
    core.atomic_json(
        path,
        {
            "_copyright": "Copyright 2026 MakerMods. All rights reserved.",
            "_license": json.loads(REFERENCE.read_text())["_license"],
            "schema": 1,
            "hub": hub,
            "settings": saved_identity(settings),
            "speed_deg_s": speed,
            "visits": visits,
            "report": report,
        },
    )


def load_plan(
    settings, hub, limits, box, guard, speed, *, stop_path_clear=None, say=None, expected_sha256=None
):
    """Treat the file as untrusted input: recompute all proofs and report before engage."""
    started = time.perf_counter()
    raw = Path(settings.plan_file).expanduser().read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != (expected_sha256 or settings.identity()["plan_sha256"]):
        raise ValueError("sweep plan changed while loading")
    data = json.loads(raw)
    if data.get("schema") != 1 or data.get("settings") != json.loads(json.dumps(saved_identity(settings))):
        raise ValueError("saved sweep settings/predictor differ")
    saved_hub = data["hub"]
    if set(saved_hub) != set(core.MOTORS) or any(not math.isfinite(v) for v in saved_hub.values()):
        raise ValueError("invalid saved hub")
    if any(abs(saved_hub[n] - hub[n]) > 0.5 for n in core.ARM):
        raise ValueError("saved sweep hub differs by more than 0.5 deg from inputs hub")
    # The gripper is never driven: bind it to this start gate's measured value.
    # Re-prove the resulting mesh geometry at every step, including the hub.
    visits = data["visits"]
    saved_hub["gripper"] = hub["gripper"]
    # Input-hub drift within the tolerance still gets all hub checks. Every
    # saved step must return to the actual inputs hub used by Episode.stop().
    if saved_hub != hub:
        Proof(settings, hub, limits, box, guard, speed, Predictor(settings, hub), stop_path_clear)
    proof = Proof(
        settings,
        saved_hub,
        limits,
        box,
        guard,
        speed,
        Predictor(settings, saved_hub),
        stop_path_clear,
        return_hub=hub,
    )
    if data.get("speed_deg_s") != speed or len(visits) != settings.poses:
        raise ValueError("saved sweep speed/pose count differs")
    for visit in visits:
        pose = visit["pose"]
        if any(not proof.bounds[n][0] <= pose[n] <= proof.bounds[n][1] for n in MOVING):
            raise ValueError("saved pose outside sweep box")
        routes = visit["routes"]
        if [r["direction"] for r in routes] != ["+", "-"]:
            raise ValueError("saved sweep requires both directions")
        here = hub
        for route, sign in zip(routes, (1, -1), strict=True):
            delta = route["delta_deg"]
            if (
                set(delta) != set(APPROACH)
                or any(not math.isfinite(v) or abs(v) > 4.0 + 1e-9 for v in delta.values())
                or not any(delta.values())
            ):
                raise ValueError("invalid approach delta")
            approach = {**pose, **{n: pose[n] + sign * delta[n] for n in APPROACH}}
            for q in [pose, approach, *route["into"], *route["settle"], *visit["back"]]:
                q["gripper"] = hub["gripper"]
            approach["gripper"] = pose["gripper"] = hub["gripper"]
            if any(not limits[n][0] + 8 <= approach[n] <= limits[n][1] - 8 for n in MOVING):
                raise ValueError("saved approach outside soft margin")
            proof.steps(here, route["into"], approach)
            proof.steps(approach, route["settle"], pose)
            here = pose
        if routes[0]["delta_deg"] != routes[1]["delta_deg"]:
            raise ValueError("approaches must oppose each other")
        proof.steps(pose, visit["back"], saved_hub)
    report = report_visits(visits, settings, saved_hub, proof)
    report.update(loaded_plan_sha256=digest, reproof_time_s=time.perf_counter() - started)
    if say:
        say(f"sweep plan re-proved in {report['reproof_time_s']:.3f} s (sha256 {digest})")
    return visits, report


def actions(visits, settings):
    """(phase, target, hold metadata) queue; every pose ends at the hub."""
    for i, visit in enumerate(visits[settings.start_at :], settings.start_at):
        prefix = f"sweep: pose {i + 1}/{len(visits)}"
        for route in visit["routes"]:
            direction = route["direction"]
            for q in route["into"] + route["settle"]:
                yield f"{prefix} approach from {direction}", q, {"pose": i, "motion": True}
            count = math.ceil(settings.hold_s / core.ROW_DT_S)
            for tick in range(count):
                yield (
                    f"{prefix} hold from {direction}",
                    visit["pose"],
                    {"pose": i, "direction": direction, "last": tick == count - 1},
                )
        for q in visit["back"]:
            yield f"{prefix} return hub", q, {"pose": i, "motion": True}
        yield f"sweep: hub after pose {i + 1}/{len(visits)}", visit["back"][-1], {"hub": i + 1}
