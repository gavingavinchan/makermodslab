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
"""Offline aquarium geometry, exact mesh/plane clearance and optional MuJoCo tools.

Core requires only numpy; no Lab/driver imports, hardware, user-state or network.
Load tank/model once, then call clearance(model, tank, motor_deg). The core is
pure (no mutable FK workspace) and retains every distinct collision vertex.
Scene/render require a separate Python with MuJoCo; never imported by the core.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import struct
import time
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
            parts.append(v @ r.T + t)
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
    floor = vertices["base_link"][:, 2].min() * 1000
    floor -= tank["placement"]["base_spacer_height_mm"]["value"]
    if not math.isclose(floor, tank["placement"]["floor_z_mm"]["value"], abs_tol=EPS_MM):
        raise ValueError("floor_z_mm disagrees with base mesh minimum minus spacer height")
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


def clearance(
    model: ArmModel,
    tank: dict,
    motor_deg: dict[str, float],
    *,
    exclude_links: tuple[str, ...] = (),
    static_mount: bool = False,
) -> dict[str, dict]:
    """Minimum signed plane distance over every selected collision vertex (mm).

    Near/far are +x/-x; left/right are +y/-y. Rim is upper headroom,
    water_line is submersion headroom, both positive below the plane. Neither
    is a physical lid. Default selection is everything below shoulder_pan,
    including yaw_link. static_mount=True reports the fixed mount separately;
    its distances are informational, never violations. No input angle clamping.
    """
    transforms = forward_kinematics(model, motor_deg)
    b = tank["derived_inner_bounds_mm"]["value"]
    names = list(WALLS)
    height = tank["water_height_mm"]["value"]
    if height is not None:
        names.append("water_line")
    best = np.full(len(names), np.inf)
    closest = [None] * len(names)
    for name, vertices in model.vertices.items():
        selected = name not in model.moving_links if static_mount else name in model.moving_links
        if name in exclude_links or not selected:
            continue
        r, t = transforms[name]
        v = (vertices @ r.T + t) * 1000
        low, high = v.min(axis=0), v.max(axis=0)
        values = [
            b["x_max"] - high[0],
            low[0] - b["x_min"],
            b["y_max"] - high[1],
            low[1] - b["y_min"],
            low[2] - b["z_min"],
            b["z_max"] - high[2],
        ]
        if height is not None:
            values.append(b["z_min"] + height - high[2])
        values = np.array(values)
        for i in np.flatnonzero(values < best):
            closest[i] = name
        best = np.minimum(best, values)
    if not np.isfinite(best).all():
        raise ValueError("No collision vertices selected")
    return {name: {"distance_mm": float(best[i]), "link": closest[i]} for i, name in enumerate(names)}


def clearance_report(model: ArmModel, tank: dict, pose: dict) -> dict:
    moving = clearance(model, tank, pose)
    margins = surface_margins(tank)
    return {
        "moving_links": {
            w: {
                **entry,
                "margin_mm": margins.get(w),
                "meets_margin": entry["distance_mm"] >= margins[w] - EPS_MM if w in margins else None,
            }
            for w, entry in moving.items()
        },
        "static_mount": {
            "role": "informational only; never a violation",
            "clearance": clearance(model, tank, pose, static_mount=True),
        },
        "violations": [w for w in WALLS if moving[w]["distance_mm"] < margins[w] - EPS_MM],
    }


def benchmark(model: ArmModel, tank: dict, pose: dict, repeat: int = 1000) -> dict:
    if repeat < 1:
        raise ValueError("repeat must be positive")
    for _ in range(20):
        clearance(model, tank, pose)
    durations = []
    for _ in range(repeat):
        start = time.perf_counter_ns()
        clearance(model, tank, pose)
        durations.append((time.perf_counter_ns() - start) / 1e6)
    return {
        "calls": repeat,
        "median_ms": float(np.median(durations)),
        "p95_ms": float(np.percentile(durations, 95)),
        "max_ms": max(durations),
        "distinct_vertices": sum(len(v) for v in model.vertices.values()),
        "evaluated_moving_vertices": sum(
            len(v) for name, v in model.vertices.items() if name in model.moving_links
        ),
        "operation": "clearance() moving-link FK/plane distances; loading and separate static-mount report excluded",
        "decimation": "none; duplicate vertices only removed",
    }


def soft_limits(tank: dict) -> dict:
    return json.loads((ROOT / tank["soft_limits_file"]).read_text())["follower"]["joint_limits"]


def sweep(
    model: ArmModel,
    tank: dict,
    pose: dict,
    margin: float | None = None,
    *,
    exclude_links: tuple[str, ...] = (),
    include_water: bool = False,
    floor_margin: float | None = None,
) -> dict:
    """Contiguous safe samples, anchored at read angle, 1 deg toward both limits.

    All intervals are sampled geometry only, not swept-volume/path guarantees.
    Out-of-limit reference angles are retained and explicitly reported.
    """
    limits = soft_limits(tank)
    margins = surface_margins(tank, margin, floor_margin)
    # Optional water constraint uses the rim margin, never the floor margin.
    margins["water_line"] = margins["rim"]
    reference_clearance = clearance(model, tank, pose, exclude_links=exclude_links)
    constraints = (
        (*WALLS, "water_line") if include_water and tank["water_height_mm"]["value"] is not None else WALLS
    )
    rows = []
    for joint in JOINTS:
        lo, hi = limits[joint]
        center = pose[joint]
        samples = {lo, hi}
        for direction in (-1, 1):
            angle = center
            while lo <= angle <= hi or (direction == 1 and angle < lo) or (direction == -1 and angle > hi):
                if lo <= angle <= hi:
                    samples.add(angle)
                angle += direction
        intervals, active = [], None
        blocking_counts = dict.fromkeys(constraints, 0)
        for angle in sorted(samples):
            p = {**pose, joint: angle}
            c = clearance(model, tank, p, exclude_links=exclude_links)
            failed = [w for w in constraints if c[w]["distance_mm"] < margins[w] - EPS_MM]
            for wall in failed:
                blocking_counts[wall] += 1
            safe = not failed
            if safe:
                if active is None:
                    active = [angle, angle]
                active[1] = angle
            elif active is not None:
                intervals.append(active)
                active = None
        if active is not None:
            intervals.append(active)
        rows.append(
            {
                "joint": joint,
                "limits_deg": [lo, hi],
                "safe_sample_intervals_deg": intervals,
                "contiguous_around_read_deg": next(
                    (interval for interval in intervals if interval[0] <= center <= interval[1]), None
                ),
                "reference_meets_margins": all(
                    reference_clearance[w]["distance_mm"] >= margins[w] - EPS_MM for w in constraints
                ),
                "reference_within_limits": lo <= center <= hi,
                "samples": len(samples),
                "samples_failing_surface": blocking_counts,
            }
        )
    return {
        "limits_source": tank["soft_limits_file"],
        "surface_margins_mm": {w: margins[w] for w in constraints},
        "moving_links": sorted(model.moving_links & model.vertices.keys()),
        "static_mount": "base_link informational only; excluded from sweep",
        "reference_clearance": reference_clearance,
        "reference_violations": [
            w for w in constraints if reference_clearance[w]["distance_mm"] < margins[w] - EPS_MM
        ],
        "excluded_links": list(exclude_links),
        "constraints": list(constraints),
        "rows": rows,
        "reference_outside_limits": {
            j: pose[j] for j in (*JOINTS, "gripper") if not limits[j][0] <= pose[j] <= limits[j][1]
        },
        "warning": "Sampled geometry only, no continuous path proof. Moving links include yaw_link; static base is informational. "
        "Floor has its own margin. Other joints remain at the reference even if outside soft limits. "
        "A contiguous range is null if the reference fails margins or is outside that joint's soft limits.",
    }


def sweep_table(result: dict) -> str:
    lines = [
        "| Joint | Soft limits (motor deg) | Passing samples (motor deg) | Contiguous around read (motor deg) |",
        "|---|---:|---:|---:|",
    ]
    for i, row in enumerate(result["rows"], 1):
        intervals = (
            "; ".join(f"{lo:.2f} to {hi:.2f}" for lo, hi in row["safe_sample_intervals_deg"]) or "none"
        )
        lo, hi = row["limits_deg"]
        around = row["contiguous_around_read_deg"]
        around_text = f"{around[0]:.2f} to {around[1]:.2f}" if around else "none"
        lines.append(f"| J{i} {row['joint']} | {lo:.2f} to {hi:.2f} | {intervals} | {around_text} |")
    return "\n".join(lines)


def fmt(values) -> str:
    return " ".join(f"{float(v):.12g}" for v in values)


def quaternion(r: np.ndarray) -> np.ndarray:
    # Eigenvector method is stable at 180 degrees; MuJoCo uses w,x,y,z.
    k = (
        np.array(
            [
                [r[0, 0] - r[1, 1] - r[2, 2], r[0, 1] + r[1, 0], r[0, 2] + r[2, 0], r[2, 1] - r[1, 2]],
                [r[0, 1] + r[1, 0], r[1, 1] - r[0, 0] - r[2, 2], r[1, 2] + r[2, 1], r[0, 2] - r[2, 0]],
                [r[0, 2] + r[2, 0], r[1, 2] + r[2, 1], r[2, 2] - r[0, 0] - r[1, 1], r[1, 0] - r[0, 1]],
                [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1], r.trace()],
            ]
        )
        / 3
    )
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1][[3, 0, 1, 2]]
    return q if q[0] >= 0 else -q


def glass_boxes(tank: dict) -> dict:
    b = {k: v / 1000 for k, v in tank["derived_inner_bounds_mm"]["value"].items()}
    g = tank["glass_thickness_mm"]["value"] / 1000
    x, y, z = [(b[f"{a}_min"] + b[f"{a}_max"]) / 2 for a in "xyz"]
    hx, hy, hz = [(b[f"{a}_max"] - b[f"{a}_min"]) / 2 for a in "xyz"]
    return {
        "near": ([b["x_max"] + g / 2, y, z], [g / 2, hy + g, hz]),
        "far": ([b["x_min"] - g / 2, y, z], [g / 2, hy + g, hz]),
        "left": ([x, b["y_max"] + g / 2, z], [hx, g / 2, hz]),
        "right": ([x, b["y_min"] - g / 2, z], [hx, g / 2, hz]),
        "floor": ([x, y, b["z_min"] - g / 2], [hx + g, hy + g, g / 2]),
    }


def build_scene(model: ArmModel, tank: dict, output: Path) -> Path:
    """Write MJCF directly from URDF kinematics, inertias and collision meshes."""
    robot = ET.parse(model.path).getroot()
    root = ET.Element("mujoco", model="viscous_waterproof_aquarium")
    root.append(ET.Comment(Path(__file__).read_text().split('"""')[0].replace("# ", "").replace("#\n", "\n")))
    root.append(
        ET.Comment(
            "Generated by scripts/viscous_aquarium_env.py scene from tank.json and repo URDF. "
            "Arm/glass contacts only; no self contacts. CAD joint limits copied verbatim. "
            "Motor signs/offsets from motor_to_urdf.json; static mount masks 0/0. "
            "Read/folded keyframes are kinematic snapshots, NOT dynamic safe targets."
        )
    )
    ET.SubElement(
        root, "compiler", angle="radian", autolimits="true", fusestatic="false", inertiafromgeom="false"
    )
    ET.SubElement(root, "option", gravity="0 0 -9.81")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1100", offheight="850")
    ET.SubElement(visual, "headlight", ambient="0.6 0.6 0.6", diffuse="0.7 0.7 0.7", specular="0.1 0.1 0.1")
    ET.SubElement(visual, "rgba", haze="0.96 0.97 0.98 1")
    assets = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="-0.4 -0.3 1.5", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    bodies = {"base_link": ET.SubElement(world, "body", name="base_link")}
    joint_nodes = {n.attrib["name"]: n for n in robot.findall("joint")}
    for j in model.joints:
        body = ET.SubElement(bodies[j.parent], "body", name=j.child, pos=fmt(j.t), quat=fmt(quaternion(j.r)))
        bodies[j.child] = body
        if j.kind != "fixed":
            limits = joint_nodes[j.name].find("limit")
            ET.SubElement(
                body,
                "joint",
                name=j.name,
                type="slide" if j.kind == "prismatic" else "hinge",
                axis=fmt(j.axis),
                range=f"{limits.get('lower')} {limits.get('upper')}",
            )
    equality = ET.SubElement(root, "equality")
    for j in robot.findall("joint"):
        mimic = j.find("mimic")
        if mimic is not None:
            ET.SubElement(
                equality,
                "joint",
                joint1=j.attrib["name"],
                joint2=mimic.attrib["joint"],
                polycoef=f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0",
            )
    mesh_names = {}
    for link in robot.findall("link"):
        body = bodies[link.attrib["name"]]
        inertial = link.find("inertial")
        if inertial is not None:
            r, t = origin(inertial)
            i = inertial.find("inertia").attrib
            tensor = np.array(
                [
                    [float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                    [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                    [float(i["ixz"]), float(i["iyz"]), float(i["izz"])],
                ]
            )
            tensor = r @ tensor @ r.T
            ET.SubElement(
                body,
                "inertial",
                pos=fmt(t),
                mass=inertial.find("mass").get("value"),
                fullinertia=fmt(tensor[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]),
            )
        for c in link.findall("collision"):
            mesh = c.find("geometry/mesh")
            filename = mesh.attrib["filename"]
            scale = mesh.get("scale", "1 1 1")
            key = (filename, scale)
            if key not in mesh_names:
                mesh_names[key] = f"mesh_{len(mesh_names)}"
                ET.SubElement(
                    assets,
                    "mesh",
                    name=mesh_names[key],
                    scale=scale,
                    file=Path(os.path.relpath(model.path.parent / filename, output.parent)).as_posix(),
                )
            r, t = origin(c)
            ET.SubElement(
                body,
                "geom",
                name=f"arm_{link.attrib['name']}_{c.get('name')}",
                type="mesh",
                mesh=mesh_names[key],
                pos=fmt(t),
                quat=fmt(quaternion(r)),
                rgba="0.32 0.39 0.45 1",
                contype="1" if link.attrib["name"] in model.moving_links else "0",
                conaffinity="2" if link.attrib["name"] in model.moving_links else "0",
                group="0",
            )
    for name, (pos, size) in glass_boxes(tank).items():
        ET.SubElement(
            world,
            "geom",
            name=f"glass_{name}",
            type="box",
            pos=fmt(pos),
            size=fmt(size),
            rgba="0.4 0.75 0.85 0.15",
            contype="2",
            conaffinity="1",
            group="1",
        )
    b = tank["derived_inner_bounds_mm"]["value"]
    height = tank["water_height_mm"]["value"]
    if height is not None:
        ET.SubElement(
            world,
            "geom",
            name="water_line",
            type="box",
            pos=fmt(
                [
                    (b["x_min"] + b["x_max"]) / 2000,
                    (b["y_min"] + b["y_max"]) / 2000,
                    (b["z_min"] + height) / 1000,
                ]
            ),
            size=fmt([(b["x_max"] - b["x_min"]) / 2000, (b["y_max"] - b["y_min"]) / 2000, 0.00015]),
            rgba="0.2 0.6 0.9 0.07",
            contype="0",
            conaffinity="0",
            group="2",
        )
    # Encode the two snapshots by named joint addresses after first compilation.
    import mujoco

    compiled = mujoco.MjModel.from_xml_string(
        ET.tostring(root, encoding="unicode"),
        assets={
            Path(os.path.relpath(model.path.parent / f, output.parent)).as_posix(): (
                model.path.parent / f
            ).read_bytes()
            for f, _ in mesh_names
        },
    )
    keys = ET.SubElement(root, "keyframe")
    for name, entry in tank["reference_poses_motor_deg"].items():
        qpos = np.zeros(compiled.nq)
        for joint, value in joint_positions(model, entry["value"]).items():
            idx = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_JOINT, joint)
            qpos[compiled.jnt_qposadr[idx]] = value
        ET.SubElement(keys, "key", name=name, qpos=fmt(qpos))
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return output


def check_scene(scene: Path, model: ArmModel, tank: dict) -> dict:
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(scene))
    results = {
        "mujoco_version": mujoco.__version__,
        "scene": str(scene.relative_to(ROOT)),
        "nq": m.nq,
        "ngeom": m.ngeom,
        "static_mount_contacts": "disabled by contype=0, conaffinity=0 on base_link geoms",
    }
    for name, entry in tank["reference_poses_motor_deg"].items():
        d = mujoco.MjData(m)
        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, name)
        mujoco.mj_resetDataKeyframe(m, d, key)
        # Validate the saved keyframes too, rather than overwriting their qpos.
        for joint, value in joint_positions(model, entry["value"]).items():
            idx = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if not math.isclose(d.qpos[m.jnt_qposadr[idx]], value, abs_tol=1e-10):
                raise ValueError(f"Stale {name} keyframe for {joint}")
        mujoco.mj_forward(m, d)
        contacts = []
        for c in d.contact:
            a, b = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(i)) for i in c.geom]
            if (a.startswith("arm_") and b.startswith("glass_")) or (
                b.startswith("arm_") and a.startswith("glass_")
            ):
                contacts.append({"geom1": a, "geom2": b, "distance_mm": float(c.dist * 1000)})
        tool = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tool0")
        expected = forward_kinematics(model, entry["value"])["tool0"][1] * 1000
        results[name] = {
            "arm_glass_contacts": contacts,
            "count": len(contacts),
            "tool0_mm": (d.xpos[tool] * 1000).tolist(),
            "fk_disagreement_mm": float(np.linalg.norm(d.xpos[tool] * 1000 - expected)),
        }
    if results["read"]["count"] != 0:
        raise ValueError(f"Read pose unexpectedly contacts glass: {results['read']}")
    if not any("glass_near" in (c["geom1"], c["geom2"]) for c in results["folded"]["arm_glass_contacts"]):
        raise ValueError("Folded target must contact the near wall")
    return results


def render_scene(scene: Path, directory: Path) -> list[str]:
    """MuJoCo EGL previews; hide water face and outline tank for clear comparison."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(scene))
    d = mujoco.MjData(m)
    key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "read")
    mujoco.mj_resetDataKeyframe(m, d, key)
    mujoco.mj_forward(m, d)
    directory.mkdir(parents=True, exist_ok=True)
    from PIL import Image, ImageDraw

    # Derive cameras, labels and outline from panes, so edits to tank.json carry
    # through after scene regeneration without a second geometry parameter source.
    pane_ids = {
        w: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"glass_{w}")
        for w in ("near", "far", "left", "right")
    }
    near, far, left, right = [pane_ids[w] for w in ("near", "far", "left", "right")]
    xs = [m.geom_pos[far][0] + m.geom_size[far][0], m.geom_pos[near][0] - m.geom_size[near][0]]
    ys = [m.geom_pos[right][1] + m.geom_size[right][1], m.geom_pos[left][1] - m.geom_size[left][1]]
    zs = [m.geom_pos[near][2] - m.geom_size[near][2], m.geom_pos[near][2] + m.geom_size[near][2]]
    corners = list(itertools.product(xs, ys, zs))
    inner_length, inner_width = (xs[1] - xs[0]) * 1000, (ys[1] - ys[0]) * 1000
    results = []
    with mujoco.Renderer(m, height=800, width=1000) as renderer:
        for name, azimuth, elevation in [("top", 90, -90), ("side", 90, 0), ("three_quarter", 135, -28)]:
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [np.mean(xs), np.mean(ys), np.mean(zs)]
            camera.distance, camera.azimuth, camera.elevation = 1.27, azimuth, elevation
            camera.orthographic = name != "three_quarter"
            options = mujoco.MjvOption()
            options.geomgroup[2] = 0
            renderer.update_scene(d, camera=camera, scene_option=options)
            # Thin line geoms on the actual glass inner-box edges.
            for geom in list(renderer.scene.geoms)[: renderer.scene.ngeom]:
                if (
                    geom.objtype == mujoco.mjtObj.mjOBJ_GEOM
                    and geom.objid >= 0
                    and m.geom_group[geom.objid] == 1
                ):
                    geom.rgba[3] = 0.06
            for i, a in enumerate(corners):
                for b_corner in corners[i + 1 :]:
                    if sum(x != y for x, y in zip(a, b_corner, strict=True)) != 1:
                        continue
                    idx = renderer.scene.ngeom
                    geom = renderer.scene.geoms[idx]
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_LINE,
                        np.zeros(3),
                        np.zeros(3),
                        np.eye(3).ravel(),
                        np.array([0.15, 0.60, 0.72, 1.0], dtype=np.float32),
                    )
                    mujoco.mjv_connector(
                        geom, mujoco.mjtGeom.mjGEOM_LINE, 2.0, np.array(a), np.array(b_corner)
                    )
                    renderer.scene.ngeom += 1
            picture = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(picture)
            draw.rectangle((0, 0, 1000, 50), fill=(245, 248, 250))
            draw.text(
                (18, 12),
                f"LANDEN 75 | {name.replace('_', ' ')} | READ POSE | J5 sign -1 (photo-inferred) | inner {inner_length:g} x {inner_width:g} mm",
                fill=(25, 40, 50),
            )
            draw.text(
                (18, 30),
                "Placement ESTIMATED. Near +x (base); far -x (gripper). Not filled. Outline = inner glass box.",
                fill=(25, 40, 50),
            )
            path = directory / f"read_{name}.png"
            picture.save(path, optimize=True)
            if path.stat().st_size >= 1_000_000:
                raise ValueError(f"Preview exceeds 1 MB: {path}")
            results.append(str(path.relative_to(ROOT)))
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tank", type=Path, default=DEFAULT_TANK)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("clearance", "sweep", "scene", "render"):
        p = sub.add_parser(name)
        p.add_argument("-o", "--output", type=Path, help="Write the JSON result report here")
        if name in ("clearance", "sweep"):
            p.add_argument("--pose", choices=("read", "folded"), default="read")
            p.add_argument(
                "--angles", help="JSON object, or path to a JSON file, of seven motor angles in deg"
            )
            for joint in (*JOINTS, "gripper"):
                p.add_argument("--" + joint.replace("_", "-"), type=float)
        if name == "clearance":
            p.add_argument("--repeat", type=int, default=1000)
        if name == "sweep":
            p.add_argument("--margin-mm", type=float, help="Override wall/rim margins only")
            p.add_argument(
                "--floor-margin-mm", type=float, help="Override floor margin (default from tank.json: 0)"
            )
            p.add_argument(
                "--exclude-link",
                action="append",
                default=[],
                help="Explicit exploratory exclusion; default none",
            )
            p.add_argument("--include-water", action="store_true", help="Also require submersion headroom")
        if name in ("scene", "render"):
            p.add_argument("--scene-path", type=Path, default=ENV_DIR / "aquarium_scene.xml")
        if name == "render":
            p.add_argument("--preview-dir", type=Path, default=ENV_DIR / "previews")
    args = parser.parse_args()
    tank = load_tank(args.tank)
    model = load_model(tank)
    if args.command in ("clearance", "sweep"):
        pose = tank["reference_poses_motor_deg"][args.pose]["value"].copy()
        if args.angles:
            pose = json.loads(
                args.angles if args.angles.lstrip().startswith("{") else Path(args.angles).read_text()
            )
        for joint in (*JOINTS, "gripper"):
            value = getattr(args, joint)
            if value is not None:
                pose[joint] = value
        if args.command == "clearance":
            result = {
                "motor_deg": pose,
                **clearance_report(model, tank, pose),
                "tool0_mm": (forward_kinematics(model, pose)["tool0"][1] * 1000).tolist(),
                "timing": benchmark(model, tank, pose, args.repeat),
            }
        else:
            if set(args.exclude_link) - set(model.vertices):
                raise ValueError("Unknown excluded collision link")
            result = sweep(
                model,
                tank,
                pose,
                args.margin_mm,
                exclude_links=tuple(args.exclude_link),
                include_water=args.include_water,
                floor_margin=args.floor_margin_mm,
            )
    elif args.command == "scene":
        build_scene(model, tank, args.scene_path)
        result = check_scene(args.scene_path, model, tank)
    else:
        result = {"previews": render_scene(args.scene_path, args.preview_dir)}
    output = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(output)
    if args.command == "sweep":
        print(sweep_table(result))
        print(f"\nMargins (mm): {result['surface_margins_mm']}; limits: {result['limits_source']}")
        print("Reference margin violations:", result["reference_violations"])
        print(result["warning"])
        print("Reference outside limits:", result["reference_outside_limits"])
    else:
        print(output, end="")


if __name__ == "__main__":
    main()
