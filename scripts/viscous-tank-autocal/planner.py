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
"""The probe plan: where to press, from which pose, how deep, and whether each press is safe.

Every probe is a straight line along the surface normal: a lift pose above the standoff (where the
standing sag is measured and compensated), the standoff, then nodes every NODE_MM through the
expected plane to the deepest command the search window allows. Each line is solved, guard-checked
and force-bounded BEFORE the arm moves; the session re-plans the next probe from the planes measured
so far, so later probes start closer and press less blindly.

Verdict per probe (all must pass):
- IK for every node, joint-continuous, inside the soft limits;
- the transit hub -> lift and lift -> hub passes the astra harness's own route checks (tank guard,
  15 deg/s, slow floor zone) on the normal keep-out;
- the press line passes the PROBE keep-out: the normal one with only the two fingers allowed past the
  probed face, by at most the window plus the deep press;
- the fingers lead every other link plus its cable padding toward the surface by LEAD_MIN_MM, and
  stay OTHER_MIN_MM off every other face (a wall probe must not find the floor);
- the arm is soft enough along the normal: the deep press force stays under F_MAX_N with a press of
  at least D2_MIN_MM (a stretched arm pushing along itself is refused, see geometry.normal_stiffness).
"""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import numpy as np
from geometry import (
    ARM,
    FINGERS,
    Estimate,
    IKError,
    approach_axis,
    contact_link,
    contact_point,
    geo,
    lead_margin,
    normal_stiffness,
    point_jacobian,
    probe_tip_mm,
    solve,
    solve_jaw_position,
    tank_guard,
    tilted_axes,
    torque_per_newton,
)

# --- the probe's numbers (each one is a design choice; see the README) -----------------------------

# Probe-session MIT gains (kp Nm/rad, kd), applied at connect for this tool only. Pan and the two
# heavy joints are softened from the runner's 60 / 90 / 90 so a press bends the arm more per newton;
# the wrists keep the Lab's 30. Lift/elbow 60/3 is a gain Gavin already ran in water (rehearsal
# 20261108T211903Z); pan 30/2 has NOT run on hardware.
PROBE_GAINS = {
    "shoulder_pan": (30.0, 2.0),
    "shoulder_lift": (60.0, 3.0),
    "elbow_flex": (60.0, 3.0),
    "wrist_flex": (30.0, 2.0),
    "wrist_yaw": (30.0, 2.0),
    "wrist_roll": (30.0, 2.0),
    "gripper": (20.0, 0.5),
}
KP = {m: PROBE_GAINS[m][0] for m in ARM}

APPROACH_MM_S = 3.0  # tip speed from the standoff until the press is held (Gavin: "a few mm/s")
RETREAT_MM_S = 8.0  # backing off the glass
SLEW_DEG_S = 5.0  # sag-compensation slews and the lift<->standoff descent (the harness's slow zone)
NODE_MM = 4.0
LIFT_MM = 60.0  # the lift pose sits this far above the standoff: the free-space sag is measured there
STANDOFF_MIN_MM = 40.0
WINDOW_PAD_MM = 15.0  # standoff and window = uncertainty + this
WINDOW_MIN_MM = 8.0  # a line the arm can follow less than this past the expected plane is refused
D1_MM = 8.0  # first press, past the contact onset
D2_MM = 14.0  # second, deeper press
D2_MIN_MM = 6.0  # a pose that cannot press this deep under F_MAX_N is refused
F_MAX_N = 8.0  # design force of the deep press
LEAD_MIN_MM = 15.0
OTHER_MIN_MM = 15.0
TORQUE_BACKSTOP_NM = 1.5  # any arm joint this far over its free-space torque stops the advance

# Initial plane uncertainty (Gavin's brief for the sim: J1 off centre up to +-30 mm, yaw up to +-3
# deg, floor +-10 mm) plus 3 mm of model slack. Shrinks as touches arrive (uncertainty()).
U_CENTRE_MM = 30.0
U_YAW_DEG = 3.0
U_FLOOR_MM = 10.0
U_MODEL_MM = 3.0


@dataclasses.dataclass(frozen=True)
class ProbeSpec:
    label: str  # fit_tank label: floor/left/right or check:<face>
    face: str
    u: float  # tank x (mm)
    v: float  # floor: tank y; walls: height above the floor
    spare: bool = False


# The first -y probe at x=-470 takes the full uncertainty window. Further -y observations
# estimate yaw and predict +y from the nominal width. J1's +187.529 degree soft limit
# constrains the available search on +y, so the longer blind approach is unsuitable there.
PROBES = (
    ProbeSpec("floor", "floor", -380, 0),
    ProbeSpec("floor", "floor", -380, -80),
    ProbeSpec("floor", "floor", -420, -40),
    ProbeSpec("floor", "floor", -380, 40),
    ProbeSpec("right", "right", -470, 135),
    ProbeSpec("right", "right", -330, 100),
    ProbeSpec("right", "right", -500, 110),
    ProbeSpec("left", "left", -360, 120),
    ProbeSpec("left", "left", -440, 140),
    ProbeSpec("left", "left", -280, 140),
    ProbeSpec("check:floor", "floor", -400, -60),
    ProbeSpec("check:right", "right", -430, 120),
    ProbeSpec("check:left", "left", -400, 160),
)
SPARES = (
    ProbeSpec("floor", "floor", -400, 28, True),
    ProbeSpec("floor", "floor", -413, -65, True),
    ProbeSpec("floor", "floor", -360, -25, True),
    ProbeSpec("right", "right", -338, 90, True),
    ProbeSpec("right", "right", -350, 115, True),
    ProbeSpec("right", "right", -505, 110, True),
    ProbeSpec("right", "right", -522, 115, True),
    ProbeSpec("right", "right", -460, 135, True),
    ProbeSpec("left", "left", -400, 80, True),
    ProbeSpec("left", "left", -320, 160, True),
    ProbeSpec("left", "left", -360, 160, True),
)
TILTS_DEG = (0, 15, 30, 45, 60)  # approach tilt off the surface normal, least first; the contact point is
# the deepest finger vertex, so a tilted jaw still measures the plane
AZIMUTHS_DEG = (0, 45, 90, 135, 180, 225, 270, 315)
BANK_TILT_DEG = 65.0
BANK_AXES = 16


def face_of(label: str) -> str:
    return label.removeprefix("check:")


# --- the running estimate ----------------------------------------------------------------------


def update_estimate(prior: Estimate, touches: list[dict]) -> Estimate:
    """Planes from the touches so far (not the final fit; that is fit_tank's job).

    Shared wall yaw by least squares over both walls with separate intercepts once a wall has two
    touches 100 mm apart; a wall with no touch follows the other at the nominal inner width; the
    floor is the mean height of its touches."""
    est = prior.copy()
    groups = {
        f: [np.asarray(t["point_mm"]) for t in touches if t["label"] == f] for f in ("floor", "left", "right")
    }
    walls = {f: np.array(p) for f, p in groups.items() if f != "floor" and p}
    sxx = sxy = 0.0
    for p in walls.values():
        dx, dy = p[:, 0] - p[:, 0].mean(), p[:, 1] - p[:, 1].mean()
        sxx += float(dx @ dx)
        sxy += float(dx @ dy)
    if sxx >= 100.0**2 / 2:
        est.yaw_deg = math.degrees(math.atan(sxy / sxx))
    axes = geo.tank_axes(est.yaw_deg)
    for f, p in walls.items():
        setattr(est, f, float((p @ axes.T)[:, 1].mean()))
        est.known.add(f)
    width = est.dims["inner_width"]
    if "left" in walls and "right" not in walls:
        est.right = est.left - width
    if "right" in walls and "left" not in walls:
        est.left = est.right + width
    if groups["floor"]:
        floor = np.array(groups["floor"])
        plausible = np.abs(floor[:, 2] - prior.floor) <= U_FLOOR_MM + U_MODEL_MM
        if plausible.any():
            est.floor = float(np.median(floor[plausible, 2]))
            est.known.add("floor")
        else:
            # A rock gives rigid, repeatable FK too. Keep the original search window
            # until an independently located touch confirms a surface outside the prior.
            for point in floor:
                agree = np.abs(floor[:, 2] - point[2]) <= U_MODEL_MM
                if np.any(agree & (np.linalg.norm(floor[:, :2] - point[:2], axis=1) >= 70.0)):
                    est.floor = float(np.median(floor[agree, 2]))
                    est.known.add("floor")
                    break
            if "floor" not in est.known:
                est.unconfirmed_floor_high = float(max(floor[:, 2]))
    est.source = "touches"
    return est


def uncertainty(est: Estimate, touches: list[dict], face: str, u: float) -> float:
    """How far (mm) the true plane at tank x = u may lie from the estimate, either way."""
    if face == "floor":
        return U_MODEL_MM if "floor" in est.known else U_FLOOR_MM + U_MODEL_MM
    walls = [t for t in touches if t["label"] in ("left", "right")]
    xs = [est.to_tank(t["point_mm"])[0] for t in walls]
    yaw_known = any(
        len(group) >= 2 and max(group) - min(group) >= 100.0
        for label in ("left", "right")
        if (group := [x for x, t in zip(xs, walls, strict=True) if t["label"] == label])
    )
    yaw_u = math.tan(math.radians(0.5 if yaw_known else U_YAW_DEG))
    if not walls:
        return U_CENTRE_MM + abs(u) * yaw_u + U_MODEL_MM
    own = [x for t, x in zip(walls, xs, strict=True) if t["label"] == face]
    ref = own or xs
    slack = U_MODEL_MM if own else U_MODEL_MM + 3.0  # the opposite wall: + the width tolerance
    return min(abs(u - x) for x in ref) * yaw_u + slack


# --- guards ------------------------------------------------------------------------------------


def sync_guard(guard: tank_guard.TankGuard, est: Estimate, nominal: dict, conservative: bool) -> None:
    """Point a TankGuard at the estimated planes. `conservative`: never further from J1 than the
    nominal keep-out (the transit guard only ever tightens on a measurement)."""
    o = est.offsets()
    for face in ("left", "right", "floor", "far"):
        value = o[face]
        if conservative:
            value = min(value, nominal[face]) if face == "left" else max(value, nominal[face])
        guard.planes[face] = float(value)
    if conservative and est.unconfirmed_floor_high is not None:
        guard.planes["floor"] = max(guard.planes["floor"], est.unconfirmed_floor_high)
    guard.planes["rim"] = float(min(o["rim"], nominal["rim"]) if conservative else o["rim"])
    guard.yaw_deg = float(est.yaw_deg)
    guard._axes = geo.tank_axes(guard.yaw_deg)


def probe_guard(config: dict, model: geo.ArmModel, est: Estimate, face: str, depth_mm: float):
    """The normal keep-out, relaxed for the probed face only, by what a press needs and no more.

    The fingers may be commanded up to depth_mm (window + deep press) past the face. Every other
    link, against the probed face, keeps its cable padding plus LEAD_MIN_MM in the REAL arm: the
    glass stops the jaw, so at a command depth_mm past the plane the real arm sits depth_mm short
    of the command, and the requirement on the command is padding + LEAD_MIN_MM - depth_mm. (With
    the jaw on the glass the wrist roll link plus its 40 mm unmeasured cable padding is necessarily
    inside the normal 30 mm margin: no wall touch is possible without this.) The planner's lead
    check proves the same thing on the planned poses; this makes the guard enforce it at run time,
    including in the monitor and the stop path. Every other face is untouched for every link."""
    guard = tank_guard.TankGuard(config, model)
    sync_guard(guard, est, guard.planes.copy(), conservative=True)
    guard.planes[face] = getattr(est, face)
    padding = tank_guard._value(config["padding_mm"])
    for link, required in guard.required.items():
        if required.get(face) is None:
            continue
        if link in FINGERS:
            required[face] = -float(depth_mm)
        else:
            pad = float(padding.get(link, padding["default"]))
            required[face] = min(required[face], pad + LEAD_MIN_MM - float(depth_mm))
    return guard


# --- the seed bank and IK ----------------------------------------------------------------------


class SeedBank:
    """Random in-tank configurations (J1 toward the long axis, 100..187.5 deg, every other joint over
    its soft range), with their tip and approach axis, to seed IK near the right branch."""

    def __init__(self, model, limits, n: int = 80000, seed: int = 7):
        rng = np.random.default_rng(seed)
        lim = dict(limits)
        lim["shoulder_pan"] = (max(120.0, lim["shoulder_pan"][0]), lim["shoulder_pan"][1])
        rows, poses = [], []
        from geometry import approach_axis

        for _ in range(n):
            q = {m: float(rng.uniform(*lim[m])) for m in ARM}
            q["gripper"] = float(limits["gripper"][1])
            tf = geo.forward_kinematics(model, q)
            p = geo.probe_point(model, tf)
            if not (-560 < p[0] < 0 and -230 < p[1] < 240 and -40 < p[2] < 260):
                continue
            rows.append(np.r_[p, approach_axis(model, tf)])
            poses.append(q)
        self.rows, self.poses = np.array(rows), poses
        self.angles = np.array([[q[m] for m in ARM] for q in poses])

    def axes_near(self, target, normal, max_tilt_deg, k) -> list[np.ndarray]:
        """Approach axes this arm actually reaches near `target`, within max_tilt of the normal,
        least tilted first, at least 8 deg apart (the bank's reachable orientations, not a grid)."""
        d = np.linalg.norm(self.rows[:, :3] - target, axis=1)
        cos = self.rows[:, 3:] @ normal
        ok = np.flatnonzero((d < 90.0) & (cos >= math.cos(math.radians(max_tilt_deg))))
        order = ok[np.argsort(-cos[ok] + d[ok] / 400.0)]
        out = []
        for i in order:
            a = self.rows[i, 3:]
            if all(float(a @ b) < math.cos(math.radians(8.0)) for b in out):
                out.append(a / np.linalg.norm(a))
            if len(out) >= k:
                break
        return out

    def near(self, target, axis, k: int = 10) -> list[dict]:
        d = np.linalg.norm(self.rows[:, :3] - target, axis=1) / 60.0
        a = np.degrees(np.arccos(np.clip(self.rows[:, 3:] @ axis, -1, 1))) / 25.0
        return [self.poses[i] for i in np.argsort(d + a)[:k]]


@dataclasses.dataclass
class ProbePlan:
    spec: ProbeSpec
    ok: bool
    reasons: list
    normal: list | None = None
    expected_point_mm: list | None = None
    tilt_deg: float | None = None
    standoff_mm: float | None = None
    window_mm: float | None = None
    uncertainty_mm: float | None = None
    d1_mm: float | None = None
    d2_mm: float | None = None
    stiffness_n_per_m: float | None = None
    press_force_n: float | None = None
    backstop_force_n: float | None = None
    undetected_force_n: float | None = None
    lead_mm: float | None = None
    lead_link: str | None = None
    other_face_mm: float | None = None
    guard_slack_mm: float | None = None
    line_cut: str | None = None  # why the window ends early (the arm's reach), if it does
    lift: dict | None = None
    nodes: list | None = None  # joint targets, NODE_MM apart, from the standoff (s = 0)
    node_s_mm: list | None = None

    def summary(self) -> dict:
        out = dataclasses.asdict(self)
        out.pop("nodes")
        out.pop("node_s_mm")
        out["spec"] = dataclasses.asdict(self.spec)
        return out


class Planner:
    def __init__(self, model, keepout: dict, limits: dict, rest: dict, bank: SeedBank | None = None):
        """`model` is the astra harness Model (IK branch data, the normal guard, plan())."""
        self.model = model
        self.pm = model.guard.model
        self.keepout = keepout
        self.limits = limits
        self.rest = rest
        self.nominal = dict(model.guard.planes)
        self.padding = tank_guard._value(keepout["padding_mm"])
        self.bank = bank or SeedBank(self.pm, limits)
        self.poll = lambda: None
        self.transit_floor = tank_guard.TankGuard(keepout, self.pm)
        for link, requirements in self.transit_floor.required.items():
            if requirements["floor"] is not None and link in (
                "gripper_left",
                "gripper_right",
                "wrist_roll_link",
                "wrist_pitch_link",
                "wrist_yaw_link",
            ):
                # Before lift compensation the archived wet sag reaches 53 mm.
                requirements["floor"] = 60.0
        self.sag = {}
        self.good: list[dict] = []  # standoff poses of earlier feasible plans: the best IK seeds

    def seeds(self, target, axis, k: int = 6) -> list[dict]:
        near = sorted(self.good, key=lambda q: float(np.linalg.norm(probe_tip_mm(self.pm, q) - target)))[:3]
        return self.bank.near(target, axis, k) + near

    def _solve(self, seeds, target, axis) -> dict:
        problems = []
        for seed in seeds:
            self.poll()
            try:
                return solve(
                    self.pm,
                    {**seed, "gripper": self.rest["gripper"]},
                    target,
                    axis,
                    self.limits,
                    pos_tol_mm=1.0,
                    axis_tol_deg=2.0,
                )
            except IKError as error:
                problems.append(str(error))
        raise IKError(problems[0] if problems else "no seeds")

    def solve_contact(self, seeds, point, axis, normal) -> dict:
        """IK putting the deepest finger vertex (not the fixed tip point) at `point`: the tip target
        is shifted by the contact patch's offset from the tip, re-measured once at the solution."""
        q = self._solve(seeds, point, axis)
        for _ in range(2):
            delta = contact_point(self.pm, q, normal) - probe_tip_mm(self.pm, q)
            q = self._solve([q], np.asarray(point) - delta, axis)
        return q

    def hub(self, est: Estimate) -> dict:
        """Every probe starts and ends at the arm file's rest (the Dream start pose, ~490 mm out over
        the tank centre line): a pose the keep-out already accepts and the stop path already knows.
        A hub nearer the walls does not exist for this arm inside the 30 mm + 40 mm keep-out: tips
        above x = -250..-350 fold the forearm into the rim or the upper arm into the back post."""
        verdict = self.model.guard.check_pose(self.rest)
        if not verdict.ok:
            raise IKError(f"hub (arm file rest) fails the keep-out: {verdict}")
        return dict(self.rest)

    def plan(self, spec: ProbeSpec, est: Estimate, touches: list[dict], hub: dict | None) -> ProbePlan:
        """The least-tilted feasible approach; among equals the largest lead, then the softest press."""
        face = spec.face
        sync_guard(self.transit_floor, est, self.nominal, conservative=True)
        n = est.normal(face)
        point = est.surface_point(face, spec.u, spec.v)
        u_mm = uncertainty(est, touches, face, spec.u)
        standoff = max(STANDOFF_MIN_MM, u_mm + WINDOW_PAD_MM)
        if face in ("right", "left"):
            # The lift and standoff still need the full transit clearance, including when
            # the running estimate places the pane outside the nominal transit envelope.
            outside = (getattr(est, face) - self.nominal[face]) * (1 if face == "left" else -1)
            standoff = max(standoff, 65.0 + max(0.0, outside))
        window = u_mm + WINDOW_PAD_MM
        # Candidate approach axes: a tilt/azimuth grid around the normal, plus the orientations the
        # seed bank shows this arm actually reaches near the standoff (the reachable cone at a given
        # point is narrow and rarely on the grid). Least tilted first.
        stand = point - standoff * n
        reasons, feasible = [], []
        if face != "floor":
            standoffs = [standoff]
            if face in est.known:
                # Once this wall has a real observation the 65 mm starting distance
                # can unnecessarily fold the wrist back over the centre. Search closer
                # only inside the same normal guard and outside the full uncertainty.
                closer = max(42.0, u_mm + WINDOW_PAD_MM)
                if closer < standoff:
                    standoffs.append(closer)
            for distance in standoffs:
                plan = self._free_wrist(spec, est, n, point, distance, window, hub, u_mm, reasons)
                if plan:
                    return plan
        cands = [(float(t), a) for t, _, a in tilted_axes(n, TILTS_DEG, AZIMUTHS_DEG)]
        for a in self.bank.axes_near(stand, n, BANK_TILT_DEG, BANK_AXES):
            cands.append((math.degrees(math.acos(float(np.clip(a @ n, -1, 1)))), a))
        cands.sort(key=lambda c: c[0])
        for tilt, axis in cands:
            if feasible and (tilt > feasible[0].tilt_deg + 10.0 or len(feasible) >= 3):
                break
            try:
                result = self._candidate(spec, est, n, point, axis, standoff, window, hub)
            except IKError as error:
                reasons.append(f"IK: {str(error).split(':')[0]} (")
                continue
            result.tilt_deg, result.uncertainty_mm = tilt, u_mm
            if result.ok:
                feasible.append(result)
            else:
                reasons.extend(result.reasons)
        if feasible:
            best = max(feasible, key=lambda r: (min(r.lead_mm, 60.0), -r.press_force_n))
            self.good.append(best.nodes[0])
            return best
        if face == "floor" and (
            plan := self._free_wrist(spec, est, n, point, standoff, window, hub, u_mm, reasons)
        ):
            return plan
        return ProbePlan(
            spec,
            False,
            _top_reasons(reasons),
            normal=n.tolist(),
            expected_point_mm=point.tolist(),
            uncertainty_mm=u_mm,
        )

    def _free_wrist(self, spec, est, n, point, standoff, window, hub, uncertainty_mm, reasons):
        # Search measured, reachable wrist attitudes first. The line constrains the
        # jaw depth and tangential position; orientation can change while the jaw
        # stays ahead of every padded link. A fixed attitude unnecessarily pins J1.
        rows = self.bank.rows
        dist = np.linalg.norm(rows[:, :3] - (point - standoff * n), axis=1)
        cosine = rows[:, 3:] @ n
        travel = np.max(np.abs(self.bank.angles - np.array([self.rest[m] for m in ARM])), axis=1)
        order = np.argsort(dist + 140 * (1 - cosine) + 0.30 * travel)
        seen = []
        for i in order[:120]:
            self.poll()
            seed = self.bank.poses[i]
            key = np.array([seed[m] for m in ARM])
            if any(np.linalg.norm(key - prev) < 24 for prev in seen):
                continue
            seen.append(key)
            try:
                candidate = self._candidate(spec, est, n, point, rows[i, 3:], standoff, window, hub, seed)
            except IKError as error:
                reasons.append(str(error))
                continue
            if candidate.ok:
                axis = approach_axis(self.pm, geo.forward_kinematics(self.pm, candidate.nodes[0]))
                candidate.tilt_deg = math.degrees(math.acos(float(np.clip(axis @ n, -1, 1))))
                candidate.uncertainty_mm = uncertainty_mm
                self.good.append(candidate.nodes[0])
                return candidate
            reasons.extend(candidate.reasons)

        return None

    def _candidate(self, spec, est, n, point, axis, standoff, window, hub, seed=None) -> ProbePlan:
        """One approach axis. Cheap refusals first (standoff guard, lead), then the whole line."""
        pm, face, reasons = self.pm, spec.face, []
        stand = point - standoff * n
        try:
            if seed is None:
                start = self.solve_contact(self.seeds(stand, axis), stand, axis, n)
            else:
                start = solve_jaw_position(pm, seed, stand, n, self.limits)
        except IKError as error:
            raise IKError(f"standoff unreachable: {error}") from None
        v = self.model.guard.check_pose(start)
        if not v.ok:
            return ProbePlan(spec, False, [f"standoff fails the normal keep-out: {v.link} vs {v.face}"])
        lead, lead_link = lead_margin(pm, start, n, self.padding)
        if lead < LEAD_MIN_MM:
            return ProbePlan(
                spec, False, [f"{lead_link} (+ its padding) only {lead:.0f} mm behind the fingers"]
            )
        # The press line: nodes every NODE_MM from the standoff (s = 0) to the deepest command.
        try:
            q_contact = (
                self.solve_contact([start], point, axis, n)
                if seed is None
                else solve_jaw_position(pm, start, point, n, self.limits)
            )
        except IKError as error:
            raise IKError(f"plane point unreachable from the standoff: {error}") from None
        link = contact_link(pm, q_contact, n)
        c = contact_point(pm, q_contact, n)
        jac = point_jacobian(pm, q_contact, c, link)
        stiffness = normal_stiffness(jac, n, KP)
        tau = np.abs(torque_per_newton(jac, n))
        backstop_force = TORQUE_BACKSTOP_NM / max(float(tau.max()), 1e-9)
        # Leave room for friction and the 0.15 Nm telemetry noise inside the backstop.
        d2 = min(D2_MM, min(F_MAX_N, 0.70 * backstop_force) / stiffness * 1000)
        d1 = min(D1_MM, d2 * 0.6)
        if d2 < D2_MIN_MM:
            return ProbePlan(
                spec,
                False,
                [
                    f"too stiff along the normal ({stiffness:.0f} N/m): a {D2_MIN_MM:g} mm press "
                    f"would push {stiffness * D2_MIN_MM / 1000:.1f} N"
                ],
            )
        # The line runs to the window's end unless the arm runs out first (IK, joint continuity, soft
        # limits): then the window is cut to what is reachable, as long as it still covers the
        # expected plane plus the deep press plus WINDOW_MIN_MM. A plane beyond a cut window is not
        # found (the probe is abandoned there), never pushed for.
        depth = standoff + window + d2
        s_values = [float(x) for x in np.arange(0.0, depth + 1e-9, NODE_MM)]
        if s_values[-1] < depth:
            s_values.append(float(depth))
        nodes, q, cut = [], start, None
        for s in s_values:
            self.poll()
            delta = contact_point(pm, q, n) - probe_tip_mm(pm, q)
            try:
                target = point + (s - standoff) * n
                q = (
                    self._solve([q], target - delta, axis)
                    if seed is None
                    else solve_jaw_position(pm, q, target, n, self.limits)
                )
            except IKError as error:
                cut = f"node s={s:.0f} mm unreachable ({str(error).split(':')[0]})"
            else:
                if nodes and max(abs(q[m] - nodes[-1][m]) for m in ARM) > 4.0:
                    cut = f"not joint-continuous at s={s:.0f} mm"
                elif any(not self.limits[m][0] <= q[m] <= self.limits[m][1] for m in ARM):
                    cut = f"soft limit at s={s:.0f} mm"
            if cut:
                break
            nodes.append(q)
        reach = (len(nodes) - 1) * NODE_MM if cut else depth
        if cut:
            s_values = s_values[: len(nodes)]
            if reach < standoff + d2 + WINDOW_MIN_MM:
                raise IKError(f"press line cut before the plane: {cut}")
            window = reach - standoff - d2
        # A free wrist can change the leverage. Bound depth over the entire reachable
        # line, including when the pane is on the near side of the uncertainty interval.
        max_stiffness = stiffness
        for node in nodes:
            cp = contact_point(pm, node, n)
            node_jac = point_jacobian(pm, node, cp, "gripper_left")
            node_stiffness = normal_stiffness(node_jac, n, KP)
            max_stiffness = max(max_stiffness, node_stiffness)
            lever = np.max(np.abs(node_jac.T @ n))
            backstop_force = min(backstop_force, TORQUE_BACKSTOP_NM / max(lever, 1e-9))
        safe_depth = min(d2, min(F_MAX_N, 0.70 * backstop_force) * 1000 / max_stiffness)
        if safe_depth < D2_MIN_MM:
            return ProbePlan(spec, False, ["too stiff along the planned line for two bounded presses"])
        if safe_depth < d2:
            end = standoff + window + safe_depth
            k = int(np.searchsorted(s_values, end, side="left"))
            fraction = (end - s_values[k - 1]) / (s_values[k] - s_values[k - 1])
            last = {m: nodes[k - 1][m] + fraction * (nodes[k][m] - nodes[k - 1][m]) for m in nodes[k]}
            nodes, s_values = [*nodes[:k], last], [*s_values[:k], end]
            d2, d1 = safe_depth, min(d1, safe_depth * 0.6)
        up = np.array([0.0, 0.0, 1.0])
        try:
            lift = (
                self._solve([start], probe_tip_mm(pm, start) + LIFT_MM * up, axis)
                if seed is None
                else solve_jaw_position(pm, start, stand + LIFT_MM * up, n, self.limits)
            )
        except IKError as error:
            raise IKError(f"lift unreachable: {error}") from None
        for name, pose in (("lift", lift),):
            outside = [m for m in ARM if not self.limits[m][0] <= pose[m] <= self.limits[m][1]]
            if outside:
                reasons.append(f"{name}: outside soft limits {outside}")
                break
        deep_lead, deep_link = min(lead_margin(pm, node, n, self.padding) for node in nodes)
        if deep_lead < lead:
            lead, lead_link = deep_lead, deep_link
        if lead < LEAD_MIN_MM:
            reasons.append(f"{lead_link} (+ its padding) only {lead:.0f} mm behind the fingers")
        other = min(
            -est.signed_depth(f, contact_point(pm, q_contact, est.normal(f)))
            for f in ("floor", "left", "right", "near", "far")
            if f != face
        )
        if other < OTHER_MIN_MM:
            reasons.append(f"fingers only {other:.0f} mm from another face at the touch")
        guard = probe_guard(self.keepout, pm, est, face, window + d2 + 5.0)
        if face in self.sag:
            supported = {
                **lift,
                **{m: float(np.clip(lift[m] + self.sag[face][m], *self.limits[m])) for m in ARM},
            }
            if not guard.check_pose(supported, need_mm=-guard.tracking_allowance_mm).ok:
                return ProbePlan(
                    spec, False, ["lift with measured standing offset exceeds tracking allowance"]
                )
        slack = math.inf
        for a, b in zip(nodes, nodes[1:], strict=False):
            v = guard.check_segment(a, b)
            slack = min(slack, v.slack_mm)
            if not v.ok:
                reasons.append(f"press line: {v.link} vs {v.face} ({v.slack_mm:.1f} mm)")
                break
        v = guard.check_segment(lift, nodes[0])
        if not v.ok:
            reasons.append(f"lift -> standoff: {v.link} vs {v.face} ({v.slack_mm:.1f} mm)")
        v = self.model.guard.check_pose(lift)
        if not v.ok:
            reasons.append(f"lift fails the normal keep-out: {v.link} vs {v.face} ({v.slack_mm:.1f} mm)")
        if hub is not None and not reasons:
            transit = self.transit_floor.check_segment(hub, lift)
            if not transit.ok:
                reasons.append(f"uncompensated transit clearance: {transit.link} vs {transit.face}")
        if hub is not None and not reasons:
            from model import RefusalError

            try:
                self.model.plan(hub, [lift], self.rest, self.poll)
                self.model.plan(lift, [hub], self.rest, self.poll)
            except RefusalError as error:
                reasons.append(f"transit hub <-> lift refused: {error}")
        return ProbePlan(
            spec,
            not reasons,
            reasons,
            normal=n.tolist(),
            expected_point_mm=point.tolist(),
            standoff_mm=standoff,
            window_mm=window,
            line_cut=cut,
            d1_mm=d1,
            d2_mm=d2,
            stiffness_n_per_m=stiffness,
            press_force_n=max_stiffness * d2 / 1000,
            backstop_force_n=backstop_force,
            undetected_force_n=min(stiffness * (2 * window - WINDOW_PAD_MM) / 1000, backstop_force),
            lead_mm=lead,
            lead_link=lead_link,
            other_face_mm=other,
            guard_slack_mm=slack,
            lift=lift,
            nodes=nodes,
            node_s_mm=s_values,
        )


def _top_reasons(reasons: list[str], k: int = 6) -> list[str]:
    """The most common refusal causes (the tilt/azimuth prefix dropped), most frequent first."""
    from collections import Counter

    counts = Counter(r.removeprefix("IK: ").split(" (")[0] for r in reasons)
    return [f"{text} (x{count})" for text, count in counts.most_common(k)] or ["no candidate"]


def plan_all(planner: Planner, est: Estimate, specs=PROBES) -> dict:
    """Preview the same staged estimate, assuming each feasible nominal press finds its pane."""
    hub = planner.hub(est)
    hub_verdict = planner.model.guard.check_pose(hub)
    plans, touches = [], []
    current = est
    for spec in specs:
        plan = planner.plan(spec, current, touches, hub)
        plans.append(plan)
        if plan.ok and not spec.spare and not spec.label.startswith("check:"):
            touches.append({"label": spec.label, "point_mm": plan.expected_point_mm})
            current = update_estimate(est, touches)
    return {"hub": hub, "hub_guard": dataclasses.asdict(hub_verdict), "plans": plans}


def write_plan(path: Path, result: dict) -> None:
    out = {
        "hub_motor_deg": result["hub"],
        "hub_guard": result["hub_guard"],
        "assumption": "earlier feasible presses find the nominal planes",
        "probes": [dataclasses.asdict(p) for p in result["plans"]],
    }
    with path.open("x") as stream:
        json.dump(out, stream, indent=2, default=float)
        stream.write("\n")
