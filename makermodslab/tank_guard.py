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
"""Aquarium keep-out: gate every joint target before it reaches the arm.

Gavin (2026-11-07): "can we now build a bounding box to run dream1?" — the
glass must never be hit, and "the floor of the aquarium is the only part of
the aquarium the arm is allowed to touch anyways". This module is pure (numpy
plus tank_geometry's CAD FK), touches no hardware, and is the one check a
real-arm executor or bus wrapper calls before writing a target.

What it checks, for a target AND the straight joint-space path to it:
- every vertex of every moving link's collision mesh against the walls and
  the rim, keeping margin + per-link cable padding (link rules may exempt the
  base-riding links, see the keep-out config);
- the floor as an allowed contact: a vertex may go at most floor_press_mm
  below the floor bound, padding ignored;
- joint limits (the target is clipped into them);
- a jump filter: a joint target more than max_jump_deg from the last
  accepted one is refused, never slid toward (J1 wrap, IK branch flips).

The path is proven clear, not sampled hopefully: each joint's motion is
bounded by the furthest downstream vertex's distance from its axis, so the
segment is subdivided until no point moves more than sweep_tolerance_mm
between samples, and each sample must clear by that much.

The path proof holds for the straight joint-space path between consecutive
targets. A per-joint slew limiter downstream bends that path (joints finish
at different times), so either the executor interpolates synchronously at
the control rate, or the guard sits after the limiter on the target stream.

Positions are motor degrees after the full-turn unwrap (what the follower
reports and the tank-measure trace records), all seven joints with gripper.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import tank_geometry as geo

ARM = geo.JOINTS
MOTORS = (*ARM, "gripper")
FACES = ("near", "far", "left", "right", "floor", "rim")
WALL_FACES = ("near", "far", "left", "right", "rim")
DEFAULT_CONFIG = geo.ENV_DIR / "keepout_viscous_waterproof_01.json"
BISECT_STEPS = 14  # 1/16384 of a segment: well under a millimetre at any reach


def _value(entry):
    return entry["value"] if isinstance(entry, dict) and "value" in entry else entry


@dataclass(frozen=True)
class Verdict:
    ok: bool
    slack_mm: float  # worst clearance minus requirement; negative = violation
    link: str | None = None
    face: str | None = None
    samples: int = 1


@dataclass(frozen=True)
class Decision:
    status: str  # "ok" | "clamped" | "rejected"
    target: dict[str, float]  # what to send; the previous target when rejected
    reason: str = ""
    fraction: float = 1.0  # of the requested segment actually granted
    verdict: Verdict | None = None
    clipped_joints: tuple[str, ...] = field(default_factory=tuple)


class TankGuard:
    def __init__(self, config: dict, model: geo.ArmModel | None = None):
        self.config = config
        tank = json.loads((geo.ROOT / config["tank"]).read_text())
        self.model = model or geo.load_model(tank)
        self.yaw_deg = float(_value(config["yaw_deg"]))
        self.planes = {f: float(_value(config["planes_mm"][f])) for f in FACES}
        margins = _value(config["margins_mm"])
        press = float(_value(config["floor_press_mm"]))
        padding = _value(config["padding_mm"])
        rules = config.get("link_rules", {})
        self.limits = {m: tuple(map(float, v)) for m, v in _value(config["joint_limits_deg"]).items()}
        self.max_jump_deg = float(_value(config["max_jump_deg"]))
        self.sweep_tol_mm = float(_value(config["sweep_tolerance_mm"]))
        self.tracking_allowance_mm = float(_value(config["tracking_allowance_mm"]))
        self.start_tolerance_deg = float(_value(config.get("start_limit_tolerance_deg", 0.0)))
        if set(self.limits) != set(MOTORS) or any(lo >= hi for lo, hi in self.limits.values()):
            raise ValueError("joint_limits_deg must give a positive range for all seven motors")
        values = [*self.planes.values(), *margins.values(), press, self.max_jump_deg, self.sweep_tol_mm]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Keep-out numbers must be finite")
        floors = (*margins.values(), press, self.tracking_allowance_mm, self.start_tolerance_deg)
        if min(floors) < 0 or not math.isfinite(self.start_tolerance_deg) or self.sweep_tol_mm <= 0:
            raise ValueError("Margins, press and allowances must be nonnegative; sweep tolerance positive")
        p = self.planes
        if not (p["far"] < 0 < p["near"] and p["right"] < 0 < p["left"] and p["floor"] < p["rim"]):
            raise ValueError("J1 must lie inside the keep-out planes")
        if unknown := set(rules) - set(self.model.vertices):
            raise ValueError(f"Link rules name unknown links: {sorted(unknown)}")

        self.links = tuple(sorted(n for n in self.model.moving_links if n in self.model.vertices))
        # required[link][face]: clearance the link's mesh must keep; None = exempt.
        self.required: dict[str, dict[str, float | None]] = {}
        for link in self.links:
            rule = rules.get(link, {})
            pad = float(padding.get(link, padding["default"]))
            req = {}
            for face in FACES:
                if face in rule.get("exempt_faces", ()):
                    req[face] = None
                elif face == "floor":
                    req[face] = -press
                else:
                    over = rule.get("faces", {}).get(face, {})
                    req[face] = float(over.get("margin", margins[face])) + float(over.get("padding", pad))
            self.required[link] = req
        self._axes = geo.tank_axes(self.yaw_deg)
        self._reach = self._joint_reach_mm()

    @classmethod
    def from_file(cls, path: Path | str = DEFAULT_CONFIG) -> TankGuard:
        return cls(json.loads(Path(path).read_text()))

    # --- geometry -------------------------------------------------------

    def _joint_reach_mm(self) -> dict[str, float]:
        """Upper bound on any downstream vertex's distance from each joint's axis.

        The axis passes through the joint origin, so origin distance bounds
        it. Along the chain: the sum of the joint offsets below this joint,
        plus the link's furthest vertex, plus the finger slide's full travel.
        For the gripper, a slide of one motor degree moves a finger by at most
        the mapping's mm/deg.
        """
        joints = {j.child: j for j in self.model.joints}
        slide_mm = self.model.gripper_slide_open_m * 1000
        reach = {}
        for name in ARM:
            start = next(j.child for j in self.model.joints if j.name == name)
            best = 0.0
            for link in self.links:
                chain, node, below = 0.0, link, False
                while node in joints:
                    if node == start:
                        below = True
                        break
                    joint = joints[node]
                    chain += float(np.linalg.norm(joint.t)) * 1000
                    if joint.kind == "prismatic":
                        chain += slide_mm
                    node = joint.parent
                if below:
                    best = max(best, chain + float(np.linalg.norm(self.model.vertices[link], axis=1).max()) * 1000)
            reach[name] = best
        span = abs(self.model.gripper_motor_open_deg - self.model.gripper_motor_closed_deg)
        reach["gripper"] = slide_mm / span  # mm per motor degree, not a radius
        return reach

    def motion_bound_mm(self, q0: dict, q1: dict) -> float:
        """No point on the arm moves further than this along the straight path."""
        bound = sum(self._reach[n] * math.radians(abs(q1[n] - q0[n])) for n in ARM)
        return bound + self._reach["gripper"] * abs(q1["gripper"] - q0["gripper"])

    def clearances(self, q: dict) -> dict[tuple[str, str], float]:
        """Slack (clearance minus requirement, mm) for each checked link/face."""
        _check_positions(q)
        transforms = geo.forward_kinematics(self.model, q)
        o, axes, out = self.planes, self._axes, {}
        for link in self.links:
            r, t = transforms[link]
            # Project straight onto the tank axes: the plane distances only
            # need each axis's min/max over the link, so no full transform.
            proj = np.einsum("ij,kj->ik", self.model.vertices[link], axes @ r) * 1000 + (axes @ t) * 1000
            lo, hi = proj.min(axis=0), proj.max(axis=0)
            dist = {
                "near": o["near"] - hi[0],
                "far": lo[0] - o["far"],
                "left": o["left"] - hi[1],
                "right": lo[1] - o["right"],
                "floor": lo[2] - o["floor"],
                "rim": o["rim"] - hi[2],
            }
            for face, need in self.required[link].items():
                if need is not None:
                    out[(link, face)] = float(dist[face] - need)
        return out

    def check_pose(self, q: dict, *, need_mm: float = 0.0) -> Verdict:
        slack = self.clearances(q)
        (link, face), worst = min(slack.items(), key=lambda kv: kv[1])
        return Verdict(worst >= need_mm, worst - need_mm, link, face)

    def check_segment(self, q0: dict, q1: dict) -> Verdict:
        """Prove the straight joint-space path q0 -> q1 clear, or say where not."""
        bound = self.motion_bound_mm(q0, q1)
        steps = max(1, math.ceil(bound / (2 * self.sweep_tol_mm)))
        need = bound / (2 * steps)  # <= sweep tolerance; covers between samples
        worst = None
        for k in range(steps + 1):
            v = self.check_pose(_lerp(q0, q1, k / steps), need_mm=need)
            if worst is None or v.slack_mm < worst.slack_mm:
                worst = v
            if not v.ok:
                break
        return Verdict(worst.ok, worst.slack_mm, worst.link, worst.face, steps + 1)

    # --- decisions ------------------------------------------------------

    def outside_limits(self, q: dict, tolerance_deg: float = 0.0) -> list[str]:
        return [
            m
            for m in MOTORS
            if not self.limits[m][0] - tolerance_deg <= q[m] <= self.limits[m][1] + tolerance_deg
        ]

    def start_gate(self, measured: dict) -> Verdict:
        """A run may start only from a pose inside the box and (nearly) inside limits.

        The arm rests in the tank a few degrees outside some soft limits, as
        the Lab's slow mode already allows; limit() then clips the first
        target inside along a checked path.
        """
        _check_positions(measured)
        if out := self.outside_limits(measured, self.start_tolerance_deg):
            return Verdict(False, -math.inf, None, "joint_limit:" + ",".join(out))
        return self.check_pose(measured)

    def monitor(self, measured: dict) -> Verdict:
        """The measured arm may eat into the margin by the tracking allowance only."""
        return self.check_pose(measured, need_mm=-self.tracking_allowance_mm)

    def limit(self, previous: dict, requested: dict) -> Decision:
        """Grant as much of previous -> requested as is provably clear.

        previous must be a target this guard already accepted (or a pose that
        passed start_gate). Joint limits clip; a jump is refused; a path that
        reaches the keep-out is cut at the last clear fraction.
        """
        _check_positions(previous)
        _check_positions(requested)
        target = {m: min(max(requested[m], self.limits[m][0]), self.limits[m][1]) for m in MOTORS}
        clipped = tuple(m for m in MOTORS if target[m] != requested[m])
        jumps = [m for m in ARM if abs(target[m] - previous[m]) > self.max_jump_deg]
        if jumps:
            return Decision("rejected", dict(previous), "jump:" + ",".join(jumps), 0.0, None, clipped)
        start = self.check_pose(previous)
        if not start.ok:
            return Decision("rejected", dict(previous), "previous_unsafe", 0.0, start, clipped)
        whole = self.check_segment(previous, target)
        if whole.ok:
            status = "clamped" if clipped else "ok"
            return Decision(status, target, "joint_limit" if clipped else "", 1.0, whole, clipped)
        lo, hi = 0.0, 1.0
        for _ in range(BISECT_STEPS):
            mid = (lo + hi) / 2
            if self.check_segment(previous, _lerp(previous, target, mid)).ok:
                lo = mid
            else:
                hi = mid
        if lo == 0.0:
            return Decision("rejected", dict(previous), "keepout", 0.0, whole, clipped)
        return Decision("clamped", _lerp(previous, target, lo), "keepout", lo, whole, clipped)


class GuardedTargets:
    """Stateful wrapper an executor holds: the last accepted target and counts."""

    def __init__(self, guard: TankGuard, start: dict):
        gate = guard.start_gate(start)
        if not gate.ok:
            raise ValueError(f"Start pose refused: {gate}")
        self.guard = guard
        self.target = {m: float(start[m]) for m in MOTORS}
        self.counts = {"ok": 0, "clamped": 0, "rejected": 0}

    def step(self, requested: dict) -> Decision:
        decision = self.guard.limit(self.target, requested)
        self.counts[decision.status] += 1
        self.target = decision.target
        return decision


def _check_positions(q: dict) -> None:
    if set(q) != set(MOTORS):
        raise ValueError("Supply exactly the seven motor positions (six arm joints plus gripper)")
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in q.values()):
        raise ValueError("Positions must be finite motor degrees")


def _lerp(q0: dict, q1: dict, s: float) -> dict:
    return {m: q0[m] + (q1[m] - q0[m]) * s for m in MOTORS}
