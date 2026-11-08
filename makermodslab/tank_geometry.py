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
"""Pure, millimetre tank measurement and collision geometry.

Gavin removes the arm daily: fit the glass relative to J1 each time rather
than trusting the old +40 mm placement. This module measures and displays;
it installs no teleoperation limit or start gate. FK follows the repo's
offline aquarium model, including the provisional negative J5 sign.
"""

from __future__ import annotations

import json
import math
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ENV_DIR = ROOT / "configs/viscous/environments/aquarium_landen_75"
DEFAULT_TANK = ENV_DIR / "tank.json"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
WALLS = ("near", "far", "left", "right", "floor", "rim")
EPS_MM = 1e-5  # float32 STL vertices: nominal 40 mm becomes 39.9999991 mm


def numbers(value: str | None, default: str = "0 0 0") -> np.ndarray:
    return np.array([float(x) for x in (value or default).split()])


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def origin(element: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    node = element.find("origin")
    if node is None:
        return np.eye(3), np.zeros(3)
    roll, pitch, yaw = numbers(node.get("rpy"))
    r = rotation(np.array([0, 0, 1]), yaw)
    r = r @ rotation(np.array([0, 1, 0]), pitch) @ rotation(np.array([1, 0, 0]), roll)
    return r, numbers(node.get("xyz"))


def stl_vertices(path: Path) -> np.ndarray:
    """Read STL without trimesh. Deduplication removes no geometric information."""
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0] if len(raw) >= 84 else 0
    if len(raw) == 84 + 50 * count:
        dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
        vertices = np.frombuffer(raw, dtype=dtype, offset=84)["vertices"].reshape(-1, 3)
    else:
        vertices = np.array(
            [
                [float(x) for x in line.split()[1:]]
                for line in raw.decode("ascii").splitlines()
                if line.strip().startswith("vertex ")
            ]
        )
    if vertices.size == 0 or not np.isfinite(vertices).all():
        raise ValueError(f"Empty or nonfinite mesh: {path}")
    return np.unique(vertices, axis=0).astype(np.float64)


@dataclass(frozen=True)
class Joint:
    name: str
    parent: str
    child: str
    kind: str
    r: np.ndarray
    t: np.ndarray
    axis: np.ndarray


@dataclass(frozen=True)
class ArmModel:
    path: Path
    joints: tuple[Joint, ...]
    vertices: dict[str, np.ndarray]
    motor_to_urdf: dict[str, dict]
    moving_links: frozenset[str]
    gripper_motor_open_deg: float
    gripper_motor_closed_deg: float
    gripper_slide_open_m: float


def derive_bounds(tank: dict) -> dict[str, float]:
    dims = tank["outer_dimensions_mm"]["value"]
    glass = tank["glass_thickness_mm"]["value"]
    placement = tank["placement"]
    near = placement["near_inner_x_mm"]["value"]
    offset_y = placement["j1_y_from_inner_midline_mm"]["value"]
    floor = placement["floor_z_mm"]["value"]
    if not 0 < 2 * glass < min(dims.values()):
        raise ValueError("Glass thickness must leave a positive inner box")
    if placement["base_rpy_deg"]["value"] != [0, 0, 0]:
        raise ValueError("This axis-aligned plane model requires base_rpy_deg = [0,0,0]")
    half_width = (dims["width"] - 2 * glass) / 2
    return {
        "x_min": near - dims["length"] + 2 * glass,
        "x_max": near,
        "y_min": -half_width - offset_y,
        "y_max": half_width - offset_y,
        "z_min": floor,
        "z_max": floor + dims["height"] - glass,
    }


def load_tank(path: Path = DEFAULT_TANK) -> dict:
    tank = json.loads(Path(path).read_text())
    bounds = derive_bounds(tank)
    if tank["frame"] != "base_link" or tank["units"] != "mm":
        raise ValueError("Tank must use base_link and mm")
    for key, value in bounds.items():
        if not math.isclose(value, tank["derived_inner_bounds_mm"]["value"][key], abs_tol=1e-6):
            raise ValueError(f"Stale derived bound {key}; update tank.json derived values")
    if not math.isclose(bounds["z_max"], tank["rim_z_mm"]["value"], abs_tol=1e-6):
        raise ValueError("Stale rim_z_mm")
    surface_margins(tank)
    return tank


def load_motor_map(path: Path) -> dict[str, dict]:
    mapping = json.loads(path.read_text())["joints"]
    if set(mapping) != set(JOINTS):
        raise ValueError("Motor map must specify exactly J1-J6")
    for entry in mapping.values():
        if entry["sign"] not in (-1, 1) or not math.isfinite(entry["offset_deg"]):
            raise ValueError("Motor map requires sign +/-1 and finite offset_deg")
        if not entry["status"] or not entry["source"]:
            raise ValueError("Motor map requires status and source")
    return mapping


def surface_margins(tank: dict, wall_margin: float | None = None, floor_margin: float | None = None) -> dict:
    margins = {w: tank["surface_margins_mm"][w]["value"] for w in WALLS}
    if wall_margin is not None:
        margins.update({w: wall_margin for w in WALLS if w != "floor"})
    if floor_margin is not None:
        margins["floor"] = floor_margin
    if any(not math.isfinite(v) or v < 0 for v in margins.values()):
        raise ValueError("Margins must be finite and nonnegative")
    return margins


def load_model(tank: dict) -> ArmModel:
    path = ROOT / tank["arm_urdf"]
    robot = ET.parse(path).getroot()
    vertices = {}
    for link in robot.findall("link"):
        parts = []
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                raise ValueError("Expected collision meshes only")
            v = stl_vertices(path.parent / mesh.attrib["filename"])
            v *= numbers(mesh.get("scale"), "1 1 1")
            r, t = origin(collision)
            parts.append(np.einsum("ij,kj->ik", v, r) + t)
        if parts:
            vertices[link.attrib["name"]] = np.unique(np.concatenate(parts), axis=0)
    # Topologically sort rather than depend on XML joint order.
    pending = list(robot.findall("joint"))
    joints, known = [], {"base_link"}
    while pending:
        ready = [j for j in pending if j.find("parent").attrib["link"] in known]
        if not ready:
            raise ValueError("Disconnected or cyclic URDF")
        for j in ready:
            r, t = origin(j)
            axis = j.find("axis")
            joint = Joint(
                j.attrib["name"],
                j.find("parent").attrib["link"],
                j.find("child").attrib["link"],
                j.attrib["type"],
                r,
                t,
                numbers(axis.get("xyz") if axis is not None else "1 0 0"),
            )
            joints.append(joint)
            known.add(joint.child)
            pending.remove(j)
    # Descendants of shoulder_pan, including its child yaw_link and fixed tools.
    moving = set()
    for joint in joints:
        if joint.name == "shoulder_pan" or joint.parent in moving:
            moving.add(joint.child)
    mapping = tank["joint_mapping"]["gripper"]["value"]
    model = ArmModel(
        path,
        tuple(joints),
        vertices,
        load_motor_map(ROOT / tank["joint_mapping"]["arm"]["value"]),
        frozenset(moving),
        mapping["motor_open_deg"],
        mapping["motor_closed_deg"],
        mapping["slide_open_mm"] / 1000,
    )
    return model


def joint_positions(model: ArmModel, motor_deg: dict[str, float]) -> dict[str, float]:
    """Apply the documented sign/offset map; approximate clamped finger slide."""
    if set(motor_deg) != {*JOINTS, "gripper"} or not all(math.isfinite(v) for v in motor_deg.values()):
        raise ValueError("Supply exactly six arm joints plus gripper, all finite motor degrees")
    q = {
        name: math.radians(
            model.motor_to_urdf[name]["sign"] * motor_deg[name] + model.motor_to_urdf[name]["offset_deg"]
        )
        for name in JOINTS
    }
    fraction = (motor_deg["gripper"] - model.gripper_motor_closed_deg) / (
        model.gripper_motor_open_deg - model.gripper_motor_closed_deg
    )
    slide = min(1.0, max(0.0, fraction)) * model.gripper_slide_open_m
    q.update(gripper_left_joint=slide, gripper_right_joint=slide)
    return q


def forward_kinematics(model: ArmModel, motor_deg: dict[str, float]) -> dict:
    q = joint_positions(model, motor_deg)
    transforms = {"base_link": (np.eye(3), np.zeros(3))}
    for joint in model.joints:
        parent_r, parent_t = transforms[joint.parent]
        r, t = parent_r @ joint.r, parent_r @ joint.t + parent_t
        if joint.kind in ("revolute", "continuous"):
            r = r @ rotation(joint.axis, q[joint.name])
        elif joint.kind == "prismatic":
            t = t + r @ joint.axis * q[joint.name]
        elif joint.kind != "fixed":
            raise ValueError(f"Unsupported joint: {joint.kind}")
        transforms[joint.child] = r, t
    return transforms


# Quality gates are deliberately explicit so a future commissioned procedure
# can tune them without silently accepting an ill-conditioned wall fit.
MAX_RESIDUAL_MM = 3.0
WIDTH_TOLERANCE_MM = 10.0
MIN_WALL_SPAN_MM = 150.0
DEFAULT_PADDING_MM = 40.0
FACES = ("near", "far", "left", "right", "floor", "rim")


def probe_local_points(model: ArmModel) -> dict[str, np.ndarray]:
    """Derive the tip from CAD, not the tool0 origin (which is behind it).

    The approach axis is tool_fixed's translation, normalized, in the wrist
    roll frame (+x, slightly -z). For each finger select its collision
    vertices at the greatest projection along that axis (0.01 mm tolerance)
    and average that extreme patch. The probe is the midpoint of those two
    tips after FK, appropriate for the CLOSED gripper Gavin touches with.
    Opening is reported independently; the two slides cancel at the midpoint.
    """
    axis = next(j.t for j in model.joints if j.name == "tool_fixed")
    axis = axis / np.linalg.norm(axis)
    tips = {}
    for name in ("gripper_left", "gripper_right"):
        vertices = model.vertices[name]
        projection = np.einsum("ij,j->i", vertices, axis)
        tips[name] = vertices[projection >= projection.max() - 0.00001].mean(axis=0)
    return tips


def probe_point(model: ArmModel, transforms: dict) -> np.ndarray:
    return (
        np.mean(
            [transforms[n][0] @ v + transforms[n][1] for n, v in probe_local_points(model).items()], axis=0
        )
        * 1000
    )


def gripper_opening_mm(model: ArmModel, positions: dict[str, float]) -> float:
    return joint_positions(model, positions)["gripper_left_joint"] * 1000


def tank_axes(yaw_deg: float) -> np.ndarray:
    """Rows project base coordinates onto tank axes; yaw is tank +x in base.

    Export base_rpy_deg stores this fitted yaw by the measurement contract.
    Older offline readers reject any nonzero yaw rather than misreading it.
    """
    return rotation(np.array([0.0, 0.0, 1.0]), math.radians(yaw_deg)).T


def fit_tank(touches: list[dict], tank: dict, previous: dict | None = None) -> dict:
    """Eliminate plane offsets analytically, then scan/refine the shared yaw.

    Check touches (check:<face>) are held out of the fit, but participate in
    residual gates. Widely spaced touches on BOTH long walls constrain yaw;
    near/floor alone cannot. Far is unreachable and remains estimated.
    """
    for touch in touches:
        point = np.asarray(touch["point_mm"], dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError("Touch coordinates must be finite xyz values")
    groups = {face: [t for t in touches if t["label"] == face] for face in ("left", "right", "near", "floor")}
    missing = [
        f"{face}: need {count}"
        for face, count in (("left", 2), ("right", 2), ("near", 1), ("floor", 1))
        if len(groups[face]) < count
    ]
    if missing:
        return {"valid": False, "failures": missing}
    points = {f: np.array([t["point_mm"] for t in ts], dtype=float) for f, ts in groups.items()}
    if any(p.shape[1:] != (3,) or not np.isfinite(p).all() for p in points.values()):
        raise ValueError("Touch coordinates must be finite xyz values")

    def evaluate(yaw):
        axes = tank_axes(yaw)
        projected = {f: p @ axes.T for f, p in points.items()}
        offsets = {
            f: float(p[:, 2 if f == "floor" else 0 if f == "near" else 1].mean())
            for f, p in projected.items()
        }
        residuals = [
            p[:, 2 if f == "floor" else 0 if f == "near" else 1] - offsets[f] for f, p in projected.items()
        ]
        return float(sum(r @ r for r in residuals)), offsets, projected

    candidates = np.linspace(-20, 20, 401)
    yaw = float(min(candidates, key=lambda x: evaluate(x)[0]))
    # Golden-section bounded refinement of the best coarse interval.
    lo, hi = max(-20, yaw - 0.1), min(20, yaw + 0.1)
    for _ in range(36):
        a, b = lo + (hi - lo) * 0.38196601125, lo + (hi - lo) * 0.61803398875
        if evaluate(a)[0] < evaluate(b)[0]:
            hi = b
        else:
            lo = a
    yaw = (lo + hi) / 2
    _, offsets, projected = evaluate(yaw)
    dims = tank["outer_dimensions_mm"]["value"]
    glass = tank["glass_thickness_mm"]["value"]
    offsets["far"] = offsets["near"] - (dims["length"] - 2 * glass)
    offsets["rim"] = offsets["floor"] + dims["height"] - glass
    residuals = []
    for index, touch in enumerate(touches):
        face = touch["label"].removeprefix("check:")
        if face not in offsets:
            continue
        p = tank_axes(yaw) @ np.array(touch["point_mm"])
        axis = 2 if face in ("floor", "rim") else 0 if face in ("near", "far") else 1
        residuals.append(
            {"index": index, "label": touch["label"], "residual_mm": float(p[axis] - offsets[face])}
        )
    rms = math.sqrt(sum(r["residual_mm"] ** 2 for r in residuals) / len(residuals))
    maximum = max(abs(r["residual_mm"]) for r in residuals)
    width = offsets["left"] - offsets["right"]
    failures = []
    for face in ("left", "right"):
        if np.ptp(projected[face][:, 0]) < MIN_WALL_SPAN_MM:
            failures.append(f"{face}: touches must be at least {MIN_WALL_SPAN_MM:g} mm apart along the wall")
    if abs(yaw) > 19.99:
        failures.append("Yaw reached the ±20° search boundary; reposition and measure again")
    if maximum > MAX_RESIDUAL_MM:
        failures.append(f"Max residual {maximum:.2f} mm exceeds {MAX_RESIDUAL_MM:g} mm")
    if abs(width - (dims["width"] - 2 * glass)) > WIDTH_TOLERANCE_MM:
        failures.append(
            f"Inner width {width:.2f} mm differs from nominal by more than {WIDTH_TOLERANCE_MM:g} mm"
        )
    distances = {
        "near": offsets["near"],
        "far": -offsets["far"],
        "left": offsets["left"],
        "right": -offsets["right"],
        "floor": -offsets["floor"],
    }
    if any(distances[f] <= 0 for f in ("near", "far", "left", "right")):
        failures.append("J1 must be inside all four walls; check touch labels")
    change = None
    if previous and previous.get("measured"):
        old = previous["measured"]
        old_offsets = old["offsets_mm"]
        delta = np.array(
            [
                offsets["near"] - old_offsets["near"],
                (offsets["left"] + offsets["right"] - old_offsets["left"] - old_offsets["right"]) / 2,
            ]
        )
        # Base position relative to the same tank datum is -plane offset.
        moved = -delta
        toward = (
            ("near" if moved[0] >= 0 else "far")
            if abs(moved[0]) >= abs(moved[1])
            else ("left" if moved[1] >= 0 else "right")
        )
        change = {
            "translation_mm": float(np.linalg.norm(delta)),
            "toward": toward,
            "rotation_deg": float(old["yaw_deg"] - yaw),
        }
    return {
        "valid": not failures,
        "failures": failures,
        "yaw_deg": yaw,
        "offsets_mm": offsets,
        "width_mm": width,
        "rms_mm": rms,
        "max_mm": maximum,
        "residuals": residuals,
        "j1_distances_mm": distances,
        "far_status": "estimated",
        "change": change,
    }


def plane_distances(points_mm: np.ndarray, fit: dict) -> dict[str, np.ndarray]:
    p = np.einsum("ij,kj->ik", points_mm, tank_axes(fit["yaw_deg"]))
    o = fit["offsets_mm"]
    return {
        "near": o["near"] - p[:, 0],
        "far": p[:, 0] - o["far"],
        "left": o["left"] - p[:, 1],
        "right": p[:, 1] - o["right"],
        "floor": p[:, 2] - o["floor"],
        "rim": o["rim"] - p[:, 2],
    }


def tower_box(touches: list[dict], fit: dict) -> dict | None:
    """Tower is rigid with the mount: retain a BASE-frame axis-aligned box.

    Gavin needs the tower included because J3 can hit the back of the tank.
    The unprobed rear edge is the near plane at the box's lateral centre;
    the floor is its lower edge. This is an explicit box approximation.
    """
    labels = ("tower_front", "tower_top", "tower_left", "tower_right")
    groups = {n: [t["point_mm"] for t in touches if t["label"] == n] for n in labels}
    if not all(groups.values()) or "offsets_mm" not in fit:
        return None
    front = float(np.mean(groups["tower_front"], axis=0)[0])
    top = float(np.mean(groups["tower_top"], axis=0)[2])
    left = float(np.mean(groups["tower_left"], axis=0)[1])
    right = float(np.mean(groups["tower_right"], axis=0)[1])
    axes = tank_axes(fit["yaw_deg"])
    # Use the furthest near-plane x at either side, enclosing the rear edge.
    rear = max((fit["offsets_mm"]["near"] - axes[0, 1] * y) / axes[0, 0] for y in (left, right))
    lower = [front, right, fit["offsets_mm"]["floor"]]
    upper = [rear, left, top]
    if any(a >= b for a, b in zip(lower, upper, strict=True)):
        raise ValueError("Tower planes do not form a positive box; check labels")
    return {"min_mm": lower, "max_mm": upper, "rear_status": "estimated_near_plane"}


def cable_paddings(model: ArmModel, touches: list[dict], fit: dict, saved: dict) -> dict:
    result = {
        n: dict(saved.get(n, {"padding_mm": DEFAULT_PADDING_MM, "status": "default"}))
        for n in sorted(model.moving_links)
        if n in model.vertices
    }
    for touch in touches:
        if not touch["label"].startswith("envelope:"):
            continue
        _, link, face = touch["label"].split(":")
        r, t = forward_kinematics(model, touch["positions"])[link]
        distances = plane_distances((np.einsum("ij,kj->ik", model.vertices[link], r) + t) * 1000, fit)[face]
        padding = max(0.0, float(distances.min()))
        old = result[link]
        result[link] = {
            "padding_mm": max(padding, old["padding_mm"] if old["status"] == "measured" else 0),
            "status": "measured",
        }
    return result


def box_distances(points: np.ndarray, box: dict) -> np.ndarray:
    """Signed point-to-solid AABB distance; negative inside the tower."""
    lo, hi = np.array(box["min_mm"]), np.array(box["max_mm"])
    q = np.abs(points - (lo + hi) / 2) - (hi - lo) / 2
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(np.max(q, axis=1), 0)


def live_clearances(
    model: ArmModel, tank: dict, positions: dict, fit: dict, paddings: dict, tower: dict | None
) -> dict:
    """Every distinct mesh vertex, no subsampling; measured display at 10 Hz.

    Floor contact is informational: the arm rests on the floor. Rim is
    headroom, not a lid. Tower uses vertex-to-box distance, not an exact
    triangle/box intersection test; it must not be used as a motion guard.
    """
    transforms = forward_kinematics(model, positions)
    result = {}
    for name in sorted(model.moving_links):
        if name not in model.vertices:
            continue
        r, t = transforms[name]
        vertices = (np.einsum("ij,kj->ik", model.vertices[name], r) + t) * 1000
        padding = paddings[name]
        distances = {
            face: float(d.min()) - padding["padding_mm"] for face, d in plane_distances(vertices, fit).items()
        }
        if tower:
            distances["tower"] = float(box_distances(vertices, tower).min()) - padding["padding_mm"]
        result[name] = {
            "padding_mm": padding["padding_mm"],
            "padding_status": padding["status"],
            "distances_mm": distances,
        }
    probe = probe_point(model, transforms)
    probe_distances = {f: float(d[0]) for f, d in plane_distances(probe[None, :], fit).items()}
    if tower:
        probe_distances["tower"] = float(box_distances(probe[None, :], tower)[0])
    return {
        "links": result,
        "probe_mm": probe_distances,
        "margins_mm": surface_margins(tank),
        "floor_informational": True,
        "tower_method": "vertex_to_box",
    }


def export_tank(
    tank: dict, fit: dict, touches: list[dict], tower: dict | None, paddings: dict, timestamp: str
) -> dict:
    """Schema v2 plus measurement evidence; future guards will read this file.

    Nominal far length remains an estimate. A measured width replaces the
    nominal width for regenerated bounds; the original dimensions remain in
    measured.nominal_outer_dimensions_mm. Nonzero yaw is stored even though
    today's offline axis-aligned loader deliberately refuses it.
    """
    import copy

    if not fit["valid"]:
        raise ValueError("; ".join(fit["failures"]))
    out = copy.deepcopy(tank)
    source = f"Lab tank measurement {timestamp}"

    def measured(value):
        return {"value": value, "status": "measured", "source": source}

    o = fit["offsets_mm"]
    out["placement"] = {
        "near_inner_x_mm": measured(o["near"]),
        "j1_y_from_inner_midline_mm": measured(-(o["left"] + o["right"]) / 2),
        "base_rpy_deg": measured([0, 0, fit["yaw_deg"]]),
        "floor_z_mm": measured(o["floor"]),
        "base_spacer_height_mm": measured(-o["floor"]),
    }
    out["outer_dimensions_mm"]["value"]["width"] = fit["width_mm"] + 2 * tank["glass_thickness_mm"]["value"]
    out["outer_dimensions_mm"]["source"] = (
        source + "; width derived from measured inner faces; length/height remain nominal"
    )
    out["rim_z_mm"] = {
        "value": o["rim"],
        "status": "derived",
        "source": source + "; nominal floor-to-rim height",
    }
    out["derived_inner_bounds_mm"] = {
        "value": {
            "x_min": o["far"],
            "x_max": o["near"],
            "y_min": o["right"],
            "y_max": o["left"],
            "z_min": o["floor"],
            "z_max": o["rim"],
        },
        "status": "derived",
        "source": source + "; tank-axis plane offsets, far estimated",
    }
    out["measured"] = {
        **fit,
        "timestamp": timestamp,
        "tower_box": tower,
        "paddings": paddings,
        "touches": copy.deepcopy(touches),
        "nominal_outer_dimensions_mm": copy.deepcopy(tank["outer_dimensions_mm"]),
        "measurement_only": True,
    }
    return out
