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
"""Static gravity model of a CAN arm from its URDF, and its identification on the arm.

Gravity torque is linear in each body's mass and first mass moment (m, m*c):
``tau = Y(q) @ theta``, with ``Y`` Pinocchio's joint-torque regressor at zero
velocity and acceleration. The CAD URDF supplies the kinematics and the starting
guess for ``theta``; ``fit_gravity`` then refits ``theta`` from torques the
motors report while holding static poses, pulled toward the CAD values only
where the data cannot tell them apart. That replaces the hand-copied
per-joint factors the Metal arm's model carries (1.1-1.2x on gravity) with
numbers measured on this arm. In water the same fit absorbs buoyancy.

Pinocchio is the optional ``arm-dynamics`` extra (no Windows wheels), so it is
imported only when a model is built.
"""

from __future__ import annotations

import hashlib
import itertools
import math
import random
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
# The CAD model the Viscous fits are made against, vendored with its collision
# meshes so a fresh clone has everything (provenance in its README).
VISCOUS_URDF = REPO_ROOT / "configs/viscous/model/viscous_arm.urdf"

STATIC_PARAMS = 4  # m, m*cx, m*cy, m*cz per body
DYNAMIC_PARAMS = 10  # Pinocchio's full per-body vector: m, m*c, rotational inertia


def _pinocchio():
    try:
        import pinocchio
    except ImportError as e:
        raise RuntimeError(
            "Gravity compensation needs Pinocchio: install MakerMods Lab with the 'arm-dynamics' extra"
        ) from e
    return pinocchio


def file_sha256(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def repo_path(path: str | Path) -> Path:
    """A path stored in a data file: relative ones are relative to the repo root."""
    path = Path(path)
    return path if path.is_absolute() else REPO_ROOT / path


def stored_path(path: str | Path) -> str:
    """The form to write into a data file: repo-relative when inside the repo."""
    path = Path(path).resolve()
    return str(path.relative_to(REPO_ROOT)) if path.is_relative_to(REPO_ROOT) else str(path)


def _finite(values: dict[str, float], names: Sequence[str], what: str) -> np.ndarray:
    out = []
    for name in names:
        value = values.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{what} for {name} is missing or not finite")
        out.append(float(value))
    return np.array(out)


class GravityModel:
    """Gravity torque (N m) the motors must supply to hold the arm, from joint degrees.

    Joint angles are the follower's own degrees; the URDF's joint zeros and
    positive directions must match them (checked by the fit: a mismatched joint
    does not fit). Every URDF joint not in ``joints`` (the gripper's) is locked
    at zero and its mass carried by the body it hangs from.
    """

    def __init__(self, urdf_path: str | Path, joints: Sequence[str], *, params=None, offsets=None):
        pin = _pinocchio()
        full = pin.buildModelFromUrdf(str(urdf_path))
        missing = [name for name in joints if not full.existJointName(name)]
        if missing:
            raise ValueError(f"URDF {urdf_path} has no joint {missing}")
        lock = [i for i in range(1, full.njoints) if full.names[i] not in joints]
        model = pin.buildReducedModel(full, lock, pin.neutral(full)) if lock else full
        if [model.names[i] for i in range(1, model.njoints)] != list(joints):
            raise ValueError(f"URDF joint order {list(model.names)[1:]} does not match {list(joints)}")
        if model.nq != len(joints) or model.nv != len(joints):
            raise ValueError("Every modelled joint must be a single-axis joint")
        self.joints = tuple(joints)
        self.urdf_sha256 = file_sha256(urdf_path)
        self._pin, self._model, self._data = pin, model, model.createData()
        dynamic = np.concatenate([model.inertias[j].toDynamicParameters() for j in range(1, model.njoints)])
        self._static_columns = np.array(
            [b * DYNAMIC_PARAMS + k for b in range(len(joints)) for k in range(STATIC_PARAMS)]
        )
        self.prior = dynamic[self._static_columns].copy()
        self.params = self.prior.copy() if params is None else np.asarray(params, dtype=float)
        self.offsets = np.zeros(len(joints)) if offsets is None else np.asarray(offsets, dtype=float)
        if self.params.shape != self.prior.shape or self.offsets.shape != (len(joints),):
            raise ValueError("Gravity parameters do not match this URDF")

    def q(self, positions_deg: dict[str, float]) -> np.ndarray:
        return np.radians(_finite(positions_deg, self.joints, "Joint position"))

    def regressor(self, positions_deg: dict[str, float]) -> np.ndarray:
        """``Y`` with ``torque = Y @ params + offsets``; one row per joint."""
        zeros = np.zeros(len(self.joints))
        y = self._pin.computeJointTorqueRegressor(
            self._model, self._data, self.q(positions_deg), zeros, zeros
        )
        return np.array(y)[:, self._static_columns]

    def torque(self, positions_deg: dict[str, float], params=None) -> dict[str, float]:
        theta = self.params if params is None else params
        values = self.regressor(positions_deg) @ theta + self.offsets
        return dict(zip(self.joints, values.tolist(), strict=True))

    def body_summary(self, params=None) -> list[dict]:
        """Per body: mass and centre of mass in its joint frame, for reading a fit."""
        theta = (self.params if params is None else params).reshape(len(self.joints), STATIC_PARAMS)
        prior = self.prior.reshape(len(self.joints), STATIC_PARAMS)
        rows = []
        for name, (m, *mc), (m0, *mc0) in zip(self.joints, theta, prior, strict=True):
            rows.append(
                {
                    "joint": name,
                    "mass_kg": m,
                    "cad_mass_kg": m0,
                    "com_m": [v / m for v in mc] if abs(m) > 1e-9 else None,
                    "cad_com_m": [v / m0 for v in mc0],
                }
            )
        return rows


@dataclass
class Sample:
    positions_deg: dict[str, float]
    torques_nm: dict[str, float]
    group: int = 0  # samples of one pose share a group; cross-validation holds out whole groups


@dataclass
class GravityFit:
    params: np.ndarray
    offsets: np.ndarray
    rms_nm: dict[str, float]
    prior_rms_nm: dict[str, float]


def _rms(model: GravityModel, residuals: np.ndarray) -> dict[str, float]:
    return dict(zip(model.joints, np.sqrt(np.mean(residuals**2, axis=0)).tolist(), strict=True))


def fit_gravity(
    model: GravityModel,
    samples: Sequence[Sample],
    *,
    regularization: float = 1e-2,
    mass_scale_kg: float = 0.3,
    moment_scale_kg_m: float = 0.03,
    fit_offsets: bool = True,
    offset_scale_nm: float = 0.5,
) -> GravityFit:
    """Least squares on measured holding torques, pulled toward the CAD parameters.

    The pull resolves the directions the poses cannot excite (a body's mass
    moment along its own joint axis, say) and is otherwise weak: a 0.3 kg or
    0.03 kg m departure from CAD costs as much as a 0.1 N m residual. Offsets are
    per-joint constant torque biases (sensor zero), pulled toward zero.
    """
    if not samples:
        raise ValueError("No samples to fit")
    n = len(model.joints)
    y = np.vstack([model.regressor(s.positions_deg) for s in samples])
    tau = np.concatenate([_finite(s.torques_nm, model.joints, "Torque") for s in samples])
    columns = [y]
    if fit_offsets:
        columns.append(np.tile(np.eye(n), (len(samples), 1)))
    a = np.hstack(columns)

    scale = np.tile([mass_scale_kg] + [moment_scale_kg_m] * 3, n)
    prior = model.prior
    if fit_offsets:
        scale = np.concatenate([scale, np.full(n, offset_scale_nm)])
        prior = np.concatenate([prior, np.zeros(n)])
    weight = math.sqrt(regularization) / scale
    a_aug = np.vstack([a, np.diag(weight)])
    b_aug = np.concatenate([tau, weight * prior])
    solution = np.linalg.lstsq(a_aug, b_aug, rcond=None)[0]

    params = solution[: len(model.prior)]
    offsets = solution[len(model.prior) :] if fit_offsets else np.zeros(n)
    measured = tau.reshape(len(samples), n)
    fitted = (y @ params).reshape(len(samples), n) + offsets
    prior_pred = (y @ model.prior).reshape(len(samples), n)
    return GravityFit(params, offsets, _rms(model, measured - fitted), _rms(model, measured - prior_pred))


def cross_validate(model: GravityModel, samples: Sequence[Sample], **fit_kwargs) -> dict[str, float]:
    """Held-out RMS per joint, leaving out one pose group at a time."""
    groups = sorted({s.group for s in samples})
    if len(groups) < 2:
        raise ValueError("Cross-validation needs at least two pose groups")
    residuals = []
    for group in groups:
        train = [s for s in samples if s.group != group]
        fit = fit_gravity(model, train, **fit_kwargs)
        for s in samples:
            if s.group == group:
                predicted = model.regressor(s.positions_deg) @ fit.params + fit.offsets
                residuals.append(_finite(s.torques_nm, model.joints, "Torque") - predicted)
    return _rms(model, np.array(residuals))


# --- pose planning for an identification run --------------------------------


class CollisionChecker:
    """Self-collision and floor clearance from the URDF's collision meshes.

    Pairs on the same or adjacent joints are skipped (they touch at every
    interface). The floor is a plane ``floor_z`` metres above the base plate's
    underside; every moving piece's bounding box must stay above it, which is
    conservative for these small convex pieces.
    """

    def __init__(self, urdf_path: str | Path, mesh_dir: str | Path, joints: Sequence[str], floor_z: float):
        pin = _pinocchio()
        # Collision geometry only: the visual meshes are display assets and are not vendored.
        model = pin.buildModelFromUrdf(str(urdf_path))
        collision = pin.buildGeomFromUrdf(
            model, str(urdf_path), pin.GeometryType.COLLISION, package_dirs=[str(mesh_dir)]
        )
        collision.addAllCollisionPairs()
        pairs = []
        for pair in collision.collisionPairs:
            a = collision.geometryObjects[pair.first].parentJoint
            b = collision.geometryObjects[pair.second].parentJoint
            if a == b or model.parents[a] == b or model.parents[b] == a:
                continue
            pairs.append(pair)
        collision.removeAllCollisionPairs()
        for pair in pairs:
            collision.addCollisionPair(pair)
        self._pin, self._model, self._collision = pin, model, collision
        self._data, self._geometry_data = model.createData(), pin.GeometryData(collision)
        self._index = [model.getJointId(name) for name in joints]
        self.joints, self.floor_z = tuple(joints), floor_z
        self._corners = []
        for k, obj in enumerate(collision.geometryObjects):
            joint = obj.parentJoint
            if joint == 0 or model.parents[joint] == 0:
                continue  # the base and the first rotating link sit on the plate by design
            obj.geometry.computeLocalAABB()
            box = obj.geometry.aabb_local
            self._corners.append(
                (k, np.array(list(itertools.product(*zip(box.min_, box.max_, strict=True)))))
            )

    def problems(self, positions_deg: dict[str, float]) -> list[str]:
        q = self._pin.neutral(self._model)
        for name, index in zip(self.joints, self._index, strict=True):
            q[self._model.idx_qs[index]] = math.radians(positions_deg[name])
        self._pin.computeCollisions(self._model, self._data, self._collision, self._geometry_data, q, False)
        found = set()
        for k, result in enumerate(self._geometry_data.collisionResults):
            if result.isCollision():
                pair = self._collision.collisionPairs[k]
                a = self._collision.geometryObjects[pair.first].parentJoint
                b = self._collision.geometryObjects[pair.second].parentJoint
                found.add(f"{self._model.names[a]}/{self._model.names[b]}")
        for k, corners in self._corners:
            placement = self._geometry_data.oMg[k]
            low = float(((placement.rotation @ corners.T).T + placement.translation)[:, 2].min())
            if low < self.floor_z:
                found.add(f"{self._model.names[self._collision.geometryObjects[k].parentJoint]}/floor")
        return sorted(found)

    def is_clear(self, positions_deg: dict[str, float]) -> bool:
        return not self.problems(positions_deg)


def interpolate(a: dict[str, float], b: dict[str, float], step_deg: float) -> list[dict[str, float]]:
    """Joint-space straight line from a to b, every ``step_deg`` on the joint that moves furthest."""
    span = max(abs(b[k] - a[k]) for k in a)
    count = max(1, math.ceil(span / step_deg))
    return [{k: a[k] + (b[k] - a[k]) * i / count for k in a} for i in range(1, count + 1)]


def path_clear(a, b, is_clear: Callable[[dict[str, float]], bool], step_deg: float = 3.0) -> bool:
    return all(is_clear(p) for p in interpolate(a, b, step_deg))


@dataclass
class PoseVisit:
    pose: dict[str, float]
    approaches: list[dict[str, float]]  # visited before settling on ``pose``, one per sample
    via: list[list[dict[str, float]]]  # waypoints into each approach


def plan_poses(
    model: GravityModel,
    *,
    start: dict[str, float],
    limits: dict[str, tuple[float, float]],
    box: dict[str, tuple[float, float]],
    count: int,
    is_clear: Callable[[dict[str, float]], bool],
    hub: dict[str, float],
    max_torque_nm: float,
    approach_deg: float = 6.0,
    approach_joints: Sequence[str] = ("shoulder_lift", "elbow_flex", "wrist_flex"),
    margin_deg: float = 8.0,
    pool: int = 3000,
    seed: int = 0,
) -> list[PoseVisit]:
    """Diverse, reachable, collision-free static poses, each approached from both sides.

    Candidates are drawn inside ``box`` (shrunk to the soft limits less
    ``margin_deg``), kept only if clear and if the CAD model asks no joint for
    more than ``max_torque_nm``, then thinned by farthest-point sampling over the
    joints that load gravity. Visits are ordered nearest-first from ``start``.
    Each pose is approached once from above and once from below on
    ``approach_joints`` so Coulomb friction cancels in the fit. A leg whose
    straight line is not clear is routed through ``hub``; a pose reachable by
    neither is dropped.
    """
    rng = random.Random(seed)
    joints = model.joints
    bounds = {}
    for name in joints:
        lo, hi = box.get(name, (start[name], start[name]))
        if name in box:
            lo, hi = max(lo, limits[name][0] + margin_deg), min(hi, limits[name][1] - margin_deg)
            if lo > hi:
                raise ValueError(f"Pose box for {name} lies outside its soft limits")
        bounds[name] = (lo, hi)

    def inside(p):
        return all(limits[n][0] + margin_deg / 2 <= p[n] <= limits[n][1] - margin_deg / 2 for n in box)

    def loaded_ok(p):
        return max(abs(v) for v in model.torque(p, model.prior).values()) <= max_torque_nm

    candidates = []
    for _ in range(pool):
        p = {n: rng.uniform(*bounds[n]) for n in joints}
        if loaded_ok(p) and is_clear(p):
            candidates.append(p)
    if not candidates:
        raise ValueError("No clear pose inside the box meets the torque cap")

    key = [n for n in approach_joints if n in box] or list(box)

    def gap(p, q):
        return math.sqrt(sum((p[n] - q[n]) ** 2 for n in key))

    chosen = [max(candidates, key=lambda p: gap(p, start))]
    while len(chosen) < min(count, len(candidates)):
        chosen.append(max(candidates, key=lambda p: min(gap(p, c) for c in chosen)))

    def leg(a, b):
        if path_clear(a, b, is_clear):
            return []
        if path_clear(a, hub, is_clear) and path_clear(hub, b, is_clear):
            return [hub]
        return None

    visits, here, remaining = [], start, chosen
    while remaining:
        remaining.sort(key=lambda p: gap(p, here))
        pose = remaining.pop(0)
        approaches, vias, last = [], [], here
        for sign in (1.0, -1.0):
            approach = dict(pose)
            for n in key:
                approach[n] = pose[n] + sign * approach_deg
            if not (inside(approach) and is_clear(approach) and path_clear(approach, pose, is_clear)):
                continue
            route = leg(last, approach)
            if route is None:
                continue
            approaches.append(approach)
            vias.append(route)
            last = pose
        if approaches:
            visits.append(PoseVisit(pose, approaches, vias))
            here = pose
    return visits
