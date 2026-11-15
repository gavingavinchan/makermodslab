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
- joint limits (the target is clipped into them, except that a joint already
  outside a soft limit at an accepted target may hold or move back inward, never
  further out, so a start a few degrees outside never snaps to the limit);
- a jump filter: a joint target more than max_jump_deg from the last
  accepted one is refused, never slid toward (J1 wrap, IK branch flips).

The path is proven clear, not sampled hopefully: each joint's motion is
bounded by the furthest downstream vertex's distance from its axis, so a
sample's slack proves the path clear for as far either side as no point can
move that far, and the samples walk the path until those intervals cover it
(_scan has the argument). A pose that sits on a line can therefore leave it;
it cannot be moved closer.

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
# _scan: a walk that has not proven the whole path ends at a sample this close to a line (and closer than its
# start), or after this many samples (~0.35 ms each). A Dream substep is ~10 mm of motion: a few samples clear, a
# few dozen leaving a line; a long stop leg sliding 1 mm off a wall needs a few hundred.
SETTLE_MM = 0.05
RESOLUTION_MM = 1e-9  # a step no point can move further than this proves nothing (FK noise is ~1e-13 mm)
MAX_SAMPLES = 1000
# check_return_segment: a link/face that starts inside its requirement must stay within this much of
# where it started. It gives the walk a positive slack to start from (a slack of exactly 0 proves nothing).
RETURN_EPS_MM = 0.5


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
        # Validated, no longer used: the grid it spaced became _scan's walk (2026-11-08).
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
                    best = max(
                        best, chain + float(np.linalg.norm(self.model.vertices[link], axis=1).max()) * 1000
                    )
            reach[name] = best
        span = abs(self.model.gripper_motor_open_deg - self.model.gripper_motor_closed_deg)
        reach["gripper"] = slide_mm / span  # mm per motor degree, not a radius
        return reach

    def motion_bound_mm(self, q0: dict, q1: dict) -> float:
        """No point on the arm moves further than this along the straight path."""
        bound = sum(self._reach[n] * math.radians(abs(q1[n] - q0[n])) for n in ARM)
        return bound + self._reach["gripper"] * abs(q1["gripper"] - q0["gripper"])

    def clearances(self, q: dict, relax: dict | None = None) -> dict[tuple[str, str], float]:
        """Slack (clearance minus requirement, mm) for each checked link/face; `relax` lowers the
        requirement of the link/faces it names by that many mm (check_return_segment only)."""
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
                    out[(link, face)] = float(dist[face] - need + (relax or {}).get((link, face), 0.0))
        return out

    def check_pose(self, q: dict, *, need_mm: float = 0.0, relax: dict | None = None) -> Verdict:
        slack = self.clearances(q, relax)
        (link, face), worst = min(slack.items(), key=lambda kv: kv[1])
        return Verdict(worst >= need_mm, worst - need_mm, link, face)

    def check_segment(self, q0: dict, q1: dict) -> Verdict:
        """Prove the straight joint-space path q0 -> q1 clear, or say where not."""
        return self._scan(q0, q1, to_violation=False)[0]

    def check_return_segment(self, q0: dict, q1: dict, *, eps_mm: float = RETURN_EPS_MM) -> Verdict:
        """check_segment for a STOP's return leg, whose start may sit slightly inside a requirement.

        Why (live 20261114T084532Z): Dream's last target sat on the right-wall line (+0.04 mm, accepted);
        the stop re-anchored the goal at the measured pose plus its standing error, which then lay 1.84 mm
        over that line (wrist_pitch_link), so the first return leg failed at sample 0 although along it
        the clearance only grew (-1.8 -> +4.9 -> +81 mm). The arm held mid-air, and the shoulder cut out
        during the hold. Such a start was never an accepted target; it is where the arm already is.

        The rule. A start clear of every requirement is check_segment exactly. A start inside some by d_k
        (per link/face k), all within the tracking allowance (what monitor() already tolerates for a
        measured pose), is walked with each such requirement lowered by d_k + eps_mm and every other one
        unchanged. The start then has slack >= eps_mm everywhere, so _scan's proof applies as is, and
        what it proves is: along the whole path every link/face that started clear stays clear, and
        every one that started inside never ends up more than eps_mm further inside than it started.
        Nothing gets meaningfully closer to the glass than it already is. A start inside by more than
        the allowance is refused, as before (the arm is not where anything thinks it is).

        For the stop path only: a Dream substep always starts from an accepted target (slack >= 0), where
        this is check_segment anyway, and limit() keeps refusing an unsafe previous target.
        """
        _check_positions(q0)
        _check_positions(q1)
        start = self.clearances(q0)
        over = {k: -v for k, v in start.items() if v < 0}
        if not over:
            return self.check_segment(q0, q1)
        (link, face), worst = max(over.items(), key=lambda kv: kv[1])
        if worst > self.tracking_allowance_mm:
            return Verdict(False, -worst, link, face, 1)
        return self._scan(q0, q1, to_violation=False, relax={k: d + eps_mm for k, d in over.items()})[0]

    def _scan(
        self, q0: dict, q1: dict, *, to_violation: bool = True, relax: dict | None = None
    ) -> tuple[Verdict, float]:
        """Prove q0 -> q1 clear by a walk along it; also return the proven-clear fraction.

        Why a walk (2026-11-08, live 215535Z): the fixed grid this replaced asked
        every sample, sample 0 (the accepted previous pose) included, to clear by
        the gap it covers (up to the 2 mm sweep tolerance). A target the guard
        had clamped onto a line has ~0 slack, so every segment from it failed at
        sample 0, even one moving straight away, and the arm froze on the line.

        The proof. q(s) = q0 + s (q1 - q0), s in [0, 1]; B = motion_bound_mm
        (q0, q1). Joint i turns by |dq_i| |s - s'| between q(s) and q(s'), which
        moves a point at most its distance from the axis (<= reach_i) times that
        angle, plus the finger slide's mm/deg times the gripper's change; the
        sum is B |s - s'|, and no mesh vertex moves further. Each slack (one
        link against one face) is a plane distance of the link's extreme vertex
        along a unit tank axis minus a constant, so it moves at most as far as
        the furthest vertex does:

            |slack(q(s)) - slack(q(s'))| <= B |s - s'|   for every link/face.

        So a sample at s whose worst slack is m >= 0 proves every link/face
        clear on [s - m/B, s + m/B]. The walk keeps a proven prefix [0, c]
        (sample 0 proves [0, m0/B]) and extends it with a sample at s > c only
        when that sample's interval reaches back to c (m(s) >= B (s - c)), then
        c = s + m(s)/B; otherwise it tries again halfway to c. The fixed grid is
        the special case of spacing 1/N with m >= B/(2N) everywhere.

        What this grants: the requirement is the guard's own everywhere (every
        slack >= 0 along the whole path), so a pose on a line can leave it when
        its slack grows, and a mixed move (away from one face, toward another
        that stays clear) passes; a move closer to the line it sits on cannot
        be proven and is cut there. It is also "never closer than the start":
        a link/face whose start slack is s0 >= 0 must stay >= min(0, s0) = 0.
        A start INSIDE a requirement (slack < 0, never an accepted target; the
        stop path's re-anchored goal can be) is refused as before: the bound
        lets any vertex move toward the glass at B from s = 0 on, so no finite
        set of samples can prove that slack does not first get worse. (The stop
        path's check_return_segment lowers those requirements to just below
        where the start is, so the walk proves "never worse than the start".)
        `relax` passes those lowered requirements to every sample.

        Ends: clear as soon as the prefix covers [0, 1]; cut, before that, when
        a connecting sample comes within SETTLE_MM of a line and closer than
        the start (an approach: proving the last few hundredths of a mm toward
        a line costs samples and buys nothing), when halving would leave a step
        of under RESOLUTION_MM of motion, or after MAX_SAMPLES. With
        `to_violation` (limit()) a violating sample only narrows the search
        toward c, whose prefix is still granted; without it (check_segment) the
        first violating sample ends the walk. The proven fraction returned is
        the furthest SAMPLE whose interval connected (its pose was evaluated
        exactly, slack >= 0), so a clamp lands on a checked pose.
        """
        bound = self.motion_bound_mm(q0, q1)
        v = self.check_pose(q0, relax=relax)
        if not v.ok or bound <= 0:
            return Verdict(v.ok, v.slack_mm, v.link, v.face, 1), 1.0 if v.ok else 0.0
        settle = min(SETTLE_MM, v.slack_mm)  # never a cut at the slack the start already has
        worst, samples = v, 1
        granted, proven = 0.0, v.slack_mm / bound  # the furthest connected sample; the proven prefix
        probe = proven + v.slack_mm / bound  # where an equal slack would just reach back to `proven`
        while proven < 1.0 and samples < MAX_SAMPLES:
            probe = min(1.0, probe)
            v = self.check_pose(_lerp(q0, q1, probe), relax=relax)
            samples += 1
            if v.slack_mm < worst.slack_mm:
                worst = v
            if not v.ok and not to_violation:
                break
            if v.ok and probe - v.slack_mm / bound <= proven:  # its interval reaches back to the prefix
                reach = probe + v.slack_mm / bound
                if reach < 1.0 and v.slack_mm < settle:
                    break  # approaching a line: cut at the last connected sample
                granted, proven = probe, reach
                probe = proven + v.slack_mm / bound
            elif (probe - proven) * bound / 2 < RESOLUTION_MM:
                break  # no provable step left (a pose on a line moving closer): cut where proven
            else:
                probe = proven + (probe - proven) / 2
        if proven >= 1.0:
            return Verdict(True, worst.slack_mm, worst.link, worst.face, samples), 1.0
        cut = worst if worst.slack_mm < 0 else v  # a violation, else where the walk stopped
        return Verdict(False, cut.slack_mm, cut.link, cut.face, samples), granted

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

    def limit(self, previous: dict, requested: dict, allowed: dict | None = None) -> Decision:
        """Grant as much of previous -> requested as is provably clear.

        previous must be a target this guard already accepted (or a pose that
        passed start_gate). Joint limits clip; a jump is refused; a path that
        reaches the keep-out is cut at the last clear fraction. The limits
        stretch to cover previous (and `allowed`, the run's start-gated pose), so
        a joint resting outside one moves back in at the requested pace instead
        of snapping (Gavin 2026-11-08: every motion <= 15 deg/s), and a return
        may end exactly at the start pose.
        """
        _check_positions(previous)
        _check_positions(requested)
        # Judge a jump on the REQUEST: a wild target (a J1 wrap, an IK flip)
        # is refused outright, never clipped to a limit and partly executed.
        jumps = [m for m in ARM if abs(requested[m] - previous[m]) > self.max_jump_deg]
        if jumps:
            return Decision("rejected", dict(previous), "jump:" + ",".join(jumps), 0.0)
        poses = (previous, allowed) if allowed is not None else (previous,)
        bounds = {
            m: (
                min(self.limits[m][0], *(q[m] for q in poses)),
                max(self.limits[m][1], *(q[m] for q in poses)),
            )
            for m in MOTORS
        }
        target = {m: min(max(requested[m], bounds[m][0]), bounds[m][1]) for m in MOTORS}
        clipped = tuple(m for m in MOTORS if target[m] != requested[m])
        start = self.check_pose(previous)
        if not start.ok:
            return Decision("rejected", dict(previous), "previous_unsafe", 0.0, start, clipped)
        whole, lo = self._scan(previous, target)
        if whole.ok:
            status = "clamped" if clipped else "ok"
            return Decision(status, target, "joint_limit" if clipped else "", 1.0, whole, clipped)
        # [0, lo] is proven and the walk already ran up to the line (_scan), so
        # the cut needs no refining. Nothing (or no motion a point could show)
        # provable: moving closer to a line the previous target sits on.
        if lo * self.motion_bound_mm(previous, target) < RESOLUTION_MM:
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
        self.start = dict(self.target)
        self.counts = {"ok": 0, "clamped": 0, "rejected": 0}

    def step(self, requested: dict) -> Decision:
        decision = self.guard.limit(self.target, requested, allowed=self.start)
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
