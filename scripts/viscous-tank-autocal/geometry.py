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
"""Tank-autocal geometry: the plane estimate, the contact point, the compliance and the tip IK.

Pure numpy over makermodslab.tank_geometry's CAD FK (the same FK the tank guard and the
hand-guided tank measurement use), in the J1 base frame, millimetres and motor degrees.

Why compliance (Gavin, 2026-11-18): "You don't have to detect the exact point of contact. Press on
it, and since the arm is compliant (MIT position control), it bends against the surface." A rigid
pane stops the probe, so any settled MEASURED-joint pose during a press puts the probe ON the
glass. This module supplies the pieces that argument needs: where the probe touches (the finger
vertex deepest along the surface normal), how stiff the arm is along that normal at a given pose
(normal_stiffness, which bounds the press force), and the probe-pose IK.
"""

from __future__ import annotations

import copy
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from makermodslab import (
    tank_geometry as geo,  # noqa: E402
    tank_guard,  # noqa: E402
)

ARM = geo.JOINTS
MOTORS = (*ARM, "gripper")
FINGERS = ("gripper_left", "gripper_right")
PROBED_FACES = ("floor", "left", "right")

# The mount (CAD from Gavin's 3MF, keep-out file 2026-11-13): an L-bracket whose 12 mm truss post
# stands flush on the back glass. J1 is 106 mm from the post's inner face, so 118 mm from the glass,
# and 50 mm from the post's +y edge / 30 mm from its -y edge (80 mm wide), so the post's centre,
# where it bears on the glass, is at y = +10. Gavin: "the back post will always be flush to the back
# of the glass so you don't have to worry about that". The back plane is therefore never probed:
# it is supplied to fit_tank as one synthetic touch at that bearing point.
POST_FACE_X_MM = 106.0
POST_THICKNESS_MM = 12.0
BACK_GLASS_X_MM = POST_FACE_X_MM + POST_THICKNESS_MM
POST_CENTRE_Y_MM = 10.0
POST_WIDTH_MM = 80.0


def tank_dims(tank: dict) -> dict:
    dims = tank["outer_dimensions_mm"]["value"]
    glass = tank["glass_thickness_mm"]["value"]
    return {
        "inner_length": dims["length"] - 2 * glass,
        "inner_width": dims["width"] - 2 * glass,
        "inner_height": dims["height"] - glass,
        "glass": glass,
    }


@dataclass
class Estimate:
    """Plane offsets in TANK axes (geo.tank_axes(yaw) rows), as fit_tank and the guard use them."""

    yaw_deg: float
    near: float
    left: float
    right: float
    floor: float
    dims: dict
    source: str = "nominal"
    known: set = field(default_factory=set)  # faces measured so far (else nominal)

    @property
    def far(self) -> float:
        return self.near - self.dims["inner_length"]

    @property
    def rim(self) -> float:
        return self.floor + self.dims["inner_height"]

    def offsets(self) -> dict:
        return {
            "near": self.near,
            "far": self.far,
            "left": self.left,
            "right": self.right,
            "floor": self.floor,
            "rim": self.rim,
        }

    def axes(self) -> np.ndarray:
        return geo.tank_axes(self.yaw_deg)

    def normal(self, face: str) -> np.ndarray:
        """Unit normal in the base frame pointing OUT of the water, into the glass."""
        a = self.axes()
        return {"near": a[0], "far": -a[0], "left": a[1], "right": -a[1], "floor": -a[2], "rim": a[2]}[face]

    def to_base(self, tank_xyz) -> np.ndarray:
        return self.axes().T @ np.asarray(tank_xyz, dtype=float)

    def to_tank(self, base_xyz) -> np.ndarray:
        return self.axes() @ np.asarray(base_xyz, dtype=float)

    def signed_depth(self, face: str, base_xyz) -> float:
        """How far a base-frame point is past (positive) or short of (negative) the face plane."""
        p = self.to_tank(base_xyz)
        return {
            "near": p[0] - self.near,
            "far": self.far - p[0],
            "left": p[1] - self.left,
            "right": self.right - p[1],
            "floor": self.floor - p[2],
            "rim": p[2] - self.rim,
        }[face]

    def surface_point(self, face: str, u: float, v: float) -> np.ndarray:
        """A base-frame point ON a face. floor: (u, v) = tank (x, y); walls: u = tank x, v = height
        above the floor."""
        if face == "floor":
            return self.to_base([u, v, self.floor])
        if face in ("left", "right"):
            return self.to_base([u, getattr(self, face), self.floor + v])
        raise ValueError(f"{face} is not probed")

    def copy(self) -> Estimate:
        return copy.deepcopy(self)


def nominal_estimate(tank: dict, keepout: dict) -> Estimate:
    """The prior: the keep-out file's current mount planes (CAD 2026-11-13, floor measured 2026-11-08)
    and the back glass at the post, yaw 0. tank.json supplies only the nominal pane sizes: its own
    placement (near 40 mm) predates the new mount."""
    planes = {f: float(tank_guard._value(keepout["planes_mm"][f])) for f in ("left", "right", "floor")}
    return Estimate(
        yaw_deg=float(tank_guard._value(keepout["yaw_deg"])),
        near=BACK_GLASS_X_MM,
        left=planes["left"],
        right=planes["right"],
        floor=planes["floor"],
        dims=tank_dims(tank),
    )


def back_touch(estimate: Estimate) -> dict:
    """The synthetic near-wall touch fit_tank needs (it requires one near touch): the post's bearing
    point on the back glass. One point fixes the near plane's offset at the fitted yaw; it adds no
    yaw constraint (a single-touch face has zero residual)."""
    return {
        "label": "near",
        "point_mm": [BACK_GLASS_X_MM, POST_CENTRE_Y_MM, estimate.floor + 60.0],
        "positions": None,
        "source": "mount CAD: truss post flush on the back glass (Gavin 2026-11-18); not probed",
    }


# --- the probe ---------------------------------------------------------------------------------

PATCH_MM = 0.3  # vertices within this of the deepest one form the contact patch


def finger_vertices_mm(model: geo.ArmModel, transforms: dict) -> np.ndarray:
    parts = []
    for name in FINGERS:
        r, t = transforms[name]
        parts.append((model.vertices[name] @ r.T + t) * 1000)
    return np.concatenate(parts)


def contact_point(model: geo.ArmModel, q: dict, normal: np.ndarray) -> np.ndarray:
    """Where a closed jaw pressed against a plane with this outward normal touches it: the mean of the
    finger vertices within PATCH_MM of the deepest projection along the normal (mm, base frame).

    Why not tank_geometry.probe_point (the fixed fingertip midpoint): Gavin's first traced
    measurement (2026-11-08) showed a tilted gripper touches with the finger EDGE, 15-22 mm from the
    tip point. The deepest vertex is what the glass actually stops, at any tilt."""
    v = finger_vertices_mm(model, geo.forward_kinematics(model, q))
    d = v @ normal
    return v[d >= d.max() - PATCH_MM].mean(axis=0)


def probe_tip_mm(model: geo.ArmModel, q: dict) -> np.ndarray:
    return geo.probe_point(model, geo.forward_kinematics(model, q))


def link_extremes(model: geo.ArmModel, q: dict, normal: np.ndarray) -> dict:
    """Deepest projection (mm) along `normal` of every moving link's collision mesh."""
    transforms = geo.forward_kinematics(model, q)
    out = {}
    for name in sorted(model.moving_links):
        if name not in model.vertices:
            continue
        r, t = transforms[name]
        out[name] = float(((model.vertices[name] @ r.T + t) * 1000 @ normal).max())
    return out


def lead_margin(model: geo.ArmModel, q: dict, normal: np.ndarray, padding: dict) -> tuple[float, str]:
    """How far the fingers lead every other moving link (plus its cable padding) toward the surface.

    "Only the touch point can make contact": positive and large means nothing but the jaw can reach
    the glass first, even with the 40 mm unmeasured cable padding on the wrist links."""
    ext = link_extremes(model, q, normal)
    finger = max(ext[f] for f in FINGERS)
    worst, link = math.inf, ""
    for name, value in ext.items():
        if name in FINGERS or name == "yaw_link":
            continue
        pad = float(padding.get(name, padding["default"]))
        lead = finger - (value + pad)
        if lead < worst:
            worst, link = lead, name
    return worst, link


def point_jacobian(model: geo.ArmModel, q: dict, point_mm: np.ndarray, link: str) -> np.ndarray:
    """d(point)/d(motor angle), metres per radian, 3 x 6, for a point fixed in `link`."""
    r, t = geo.forward_kinematics(model, q)[link]
    local = r.T @ (np.asarray(point_mm) / 1000 - t)
    jac = np.zeros((3, len(ARM)))
    h = 1e-4  # degrees
    for i, m in enumerate(ARM):
        qp, qm = dict(q), dict(q)
        qp[m] += h
        qm[m] -= h
        rp, tp = geo.forward_kinematics(model, qp)[link]
        rm, tm = geo.forward_kinematics(model, qm)[link]
        jac[:, i] = ((rp @ local + tp) - (rm @ local + tm)) / math.radians(2 * h)
    return jac


def contact_link(model: geo.ArmModel, q: dict, normal: np.ndarray) -> str:
    transforms = geo.forward_kinematics(model, q)
    best = max(
        FINGERS,
        key=lambda n: (((model.vertices[n] @ transforms[n][0].T + transforms[n][1]) * 1000) @ normal).max(),
    )
    return best


def normal_stiffness(jac: np.ndarray, normal: np.ndarray, kp: dict) -> float:
    """N/m along the normal of an MIT position-controlled arm with joint stiffness kp (Nm/rad).

    Joint compliance K^-1 maps to the tip as C = J K^-1 J^T; the stiffness against a frictionless
    plane is 1 / (n^T C n). A press commanded d past the glass therefore pushes with ~ this x d.
    Near a singular direction (the arm stretched along the normal) n^T C n -> 0 and the force a
    small overshoot makes blows up: the planner refuses such poses."""
    c = jac @ np.diag([1.0 / kp[m] for m in ARM]) @ jac.T
    value = float(normal @ c @ normal)
    return math.inf if value <= 1e-12 else 1.0 / value


def torque_per_newton(jac: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """Joint torques (Nm) a 1 N push from the glass along -normal adds (J^T n, sign: as MIT reports)."""
    return jac.T @ normal


# --- IK on the guard's FK ----------------------------------------------------------------------


def approach_axis(model: geo.ArmModel, transforms: dict) -> np.ndarray:
    """The tool approach axis (wrist roll link to fingertip), the axis tank_geometry derives the tip
    from: tool_fixed's translation in the wrist roll frame."""
    axis = next(j.t for j in model.joints if j.name == "tool_fixed")
    r, _ = transforms["wrist_roll_link"]
    return r @ (axis / np.linalg.norm(axis))


STALL_ITERS = 20  # LM iterations without a 0.5 % gain before giving up on a seed


class IKError(ValueError):
    pass


def solve(
    model: geo.ArmModel,
    seed: dict,
    target_mm,
    axis,
    limits: dict,
    *,
    pos_tol_mm: float = 0.3,
    axis_tol_deg: float = 1.0,
    iterations: int = 150,
) -> dict:
    """Damped least squares for the fingertip point and the approach axis (5 constraints, 6 joints:
    wrist roll left free, it only spins the closed jaw about the approach). Motor degrees in and out,
    clipped to `limits`; raises IKError when it does not converge."""
    target = np.asarray(target_mm, dtype=float) / 1000
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    q = {m: float(seed[m]) for m in MOTORS}
    lo = np.array([limits[m][0] for m in ARM])
    hi = np.array([limits[m][1] for m in ARM])
    w = 0.12  # metres per radian of axis error

    def residual(qd):
        tf = geo.forward_kinematics(model, qd)
        tip = geo.probe_point(model, tf) / 1000
        return np.r_[tip - target, w * (approach_axis(model, tf) - axis)]

    def pose(xv):
        return {**q, **dict(zip(ARM, xv, strict=True))}

    x = np.clip(np.array([q[m] for m in ARM]), lo, hi)
    e = residual(pose(x))
    lam = 0.01
    best, stall = float(e @ e), 0
    for _ in range(iterations):
        pos = np.linalg.norm(e[:3]) * 1000
        ang = math.degrees(np.linalg.norm(e[3:]) / w)
        if pos <= pos_tol_mm and ang <= axis_tol_deg:
            return pose(x)
        jac = np.zeros((6, 6))
        h = 0.01
        for i in range(6):
            xp = x.copy()
            xp[i] += h
            jac[:, i] = (residual(pose(xp)) - e) / math.radians(h)
        # Levenberg-Marquardt: a joint pinned at a limit is dropped from the step it would leave by.
        for _ in range(8):
            step = np.degrees(-jac.T @ np.linalg.solve(jac @ jac.T + lam**2 * np.eye(6), e))
            step /= max(1.0, np.max(np.abs(step)) / 8.0)
            trial = np.clip(x + step, lo, hi)
            e_trial = residual(pose(trial))
            if e_trial @ e_trial < e @ e:
                x, e, lam = trial, e_trial, max(lam / 3, 1e-4)
                break
            lam *= 4
        else:
            break
        # Stuck (a limit or a fold the target is out of reach from): stop paying for iterations.
        err = float(e @ e)
        stall = stall + 1 if err > 0.995 * best else 0
        best = min(best, err)
        if stall >= STALL_ITERS:
            break
    pos = np.linalg.norm(e[:3]) * 1000
    ang = math.degrees(np.linalg.norm(e[3:]) / w)
    if pos <= pos_tol_mm and ang <= axis_tol_deg:
        return pose(x)
    raise IKError(f"IK did not converge: {pos:.1f} mm, {ang:.1f} deg")


def solve_multi(model, seeds, target_mm, axis, limits, **kw) -> dict:
    problems = []
    for seed in seeds:
        try:
            return solve(model, seed, target_mm, axis, limits, **kw)
        except IKError as error:
            problems.append(str(error))
    raise IKError("; ".join(problems[:3]))


def solve_from(model, seed: dict, target_mm, axis, limits, steps: int = 12, **kw) -> dict:
    """Continuation: walk the tip and axis from where `seed` puts them to the goal in `steps`
    solves, each seeded by the last, so the solution stays on the seed's IK branch (no wrap or
    elbow flip a random seed could land on). Falls back to a direct solve."""
    tf = geo.forward_kinematics(model, seed)
    p0, a0 = geo.probe_point(model, tf), approach_axis(model, tf)
    p1, a1 = np.asarray(target_mm, dtype=float), np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    q = dict(seed)
    try:
        for k in range(1, steps + 1):
            f = k / steps
            a = (1 - f) * a0 + f * a1
            if np.linalg.norm(a) < 1e-6:
                raise IKError("axis reversal")
            q = solve(
                model,
                q,
                (1 - f) * p0 + f * p1,
                a,
                limits,
                pos_tol_mm=2.0 if k < steps else 0.3,
                axis_tol_deg=3.0 if k < steps else 1.0,
                **kw,
            )
        return q
    except IKError:
        return solve(model, seed, target_mm, axis, limits, **kw)


def tilted_axes(normal: np.ndarray, tilts_deg, azimuths_deg) -> list[tuple[float, float, np.ndarray]]:
    """Approach axes around the outward normal: tilt away from it, at several azimuths."""
    n = np.asarray(normal, dtype=float)
    helper = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(n, helper)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    out = []
    for tilt in tilts_deg:
        for az in azimuths_deg if tilt else (0.0,):
            t, a = math.radians(tilt), math.radians(az)
            out.append((tilt, az, math.cos(t) * n + math.sin(t) * (math.cos(a) * e1 + math.sin(a) * e2)))
    return out


def load_inputs():
    tank = json.loads(geo.DEFAULT_TANK.read_text())
    keepout = json.loads(tank_guard.DEFAULT_CONFIG.read_text())
    return tank, keepout
