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
"""Compile the measured tank around the articulated CAD collision model (base_link frame)."""

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "configs/viscous/model"
ENV = ROOT / "configs/viscous/environments/aquarium_landen_75"
SCENE = ENV / "aquarium_scene.xml"
PLANES = {
    k: v["value"] / 1000
    for k, v in json.loads((ENV / "keepout_viscous_waterproof_01.json").read_text())["planes_mm"].items()
}
WATER = PLANES["floor"] + json.loads((ENV / "tank.json").read_text())["water_height_mm"]["value"] / 1000
MAPPING = json.loads((MODEL / "motor_to_urdf.json").read_text())["joints"]
MOTORS = (*MAPPING, "gripper")
PROFILE = json.loads((ENV / "keepout_viscous_waterproof_01.json").read_text())["joint_limits_deg"]["value"]
OPEN, CLOSED = PROFILE["gripper"]
SLIDE = 0.0524
GEAR = np.radians(OPEN - CLOSED) / SLIDE
GAINS = [(60, 4), (90, 4), (90, 4), (30, 2), (30, 2), (30, 2), (20, 0.5)]
DT = 0.002


def fmt(values):
    return " ".join(f"{v:.12g}" for v in values)


def motor_vector(pose):
    if set(pose) != set(MOTORS):
        raise ValueError("supply all seven motor angles")
    v = np.array([pose[k] for k in MOTORS], dtype=float)
    if not np.isfinite(v).all():
        raise ValueError("motor angles must be finite")
    return v


def add_box(parent, name, center, half, **attrs):
    return ET.SubElement(parent, "geom", name=name, type="box", pos=fmt(center), size=fmt(half), **attrs)


def make_tank(world):
    p = PLANES
    near = p["near"] + 0.012
    mid = [(near + p["far"]) / 2, (p["left"] + p["right"]) / 2, (p["rim"] + p["floor"]) / 2]
    half = np.array([(near - p["far"]) / 2, (p["left"] - p["right"]) / 2, (p["rim"] - p["floor"]) / 2])
    for face, axis, sign in [
        ("floor", 2, -1),
        ("left", 1, 1),
        ("right", 1, -1),
        ("near", 0, 1),
        ("far", 0, -1),
    ]:
        geom = world.find(f"geom[@name='glass_{face}']")
        size = half.copy()
        size[axis] = 0.004
        center = mid.copy()
        center[axis] += sign * (half[axis] + 0.004)
        geom.set("pos", fmt(center))
        geom.set("size", fmt(size))
        geom.set("conaffinity", "5")
        if face == "floor":
            geom.set("rgba", ".33 .43 .46 1")
    world.remove(world.find("geom[@name='water_line']"))
    add_box(
        world,
        "post",
        [p["near"] + 0.006, 0.010, mid[2]],
        [0.006, 0.040, half[2]],
        contype="2",
        conaffinity="5",
        rgba=".15 .17 .16 1",
    )


def make_object(world, task, xy):
    """Object frame is the grasp feature (handle midpoint for screwdriver)."""
    h = {"cube": 0.0125, "ball": 0.02135, "screwdriver": 0.014}[task]
    obj = ET.SubElement(world, "body", name="object", pos=fmt([*xy, PLANES["floor"] + h]))
    ET.SubElement(obj, "freejoint", name="object_free")
    shapes = []
    if task == "cube":
        shapes.append(("cube", "box", [h, h, h], [0, 0, 0], h**3 * 8, 1400, ".08 .28 .88 1"))
    elif task == "ball":
        volume = 4 * math.pi * h**3 / 3
        shapes.append(("ball", "sphere", [h], [0, 0, 0], volume, 0.0459 / volume, ".95 .94 .81 1"))
    else:
        for name, r, length, x, density, color in [
            ("handle", 0.014, 0.095, 0, 1150, ".92 .49 .07 1"),
            ("shaft", 0.003, 0.100, -0.0975, 7850, ".68 .71 .74 1"),
        ]:
            shapes.append(
                (name, "cylinder", [r, length / 2], [x, 0, 0], math.pi * r * r * length, density, color)
            )
    elements = []
    for name, kind, size, pos, volume, density, color in shapes:
        part = ET.SubElement(obj, "body", name=f"object_{name}", pos=fmt(pos))
        geom = ET.SubElement(
            part,
            "geom",
            name=f"task_{name}",
            type=kind,
            size=fmt(size),
            mass=str(density * volume),
            rgba=color,
            contype="4",
            conaffinity="3",
            friction=".65 .002 .0002",
            condim="4",
        )
        dims = np.asarray(size) * 2
        if kind == "sphere":
            dims = np.repeat(dims, 3)
        if kind == "cylinder":
            geom.set("euler", fmt([0, math.pi / 2, 0]))
            dims = dims[[1, 0, 0]]
        elements.append((part, volume, np.zeros(3), dims))
    return elements


def add_visuals(root, bodies):
    # Repo-vendored Viscous CAD display meshes. This layer has zero mass and no contact.
    source = ROOT / "scripts/viscous-dream1/assets/propeller_cardinal/arm"
    source_xml = ET.parse(source / "source.xml").getroot()
    for mesh in source_xml.findall("./asset/mesh"):
        if mesh.get("name", "").startswith("visual_"):
            mesh.set("file", str(source / mesh.get("file")))
            root.find("asset").append(mesh)
    for body in source_xml.iter("body"):
        target = bodies.get(body.get("name"))
        if target is None:
            continue
        for collision in target.findall("geom"):
            if collision.get("name", "").startswith("arm_"):
                collision.set("group", "3")
        for geom in body.findall("geom"):
            if geom.get("name", "").startswith("visual_"):
                target.append(geom)


def configure(req, sidecar, drag_scale, added_scale, *, visuals=False):
    if sidecar["schema"] != "viscous-buoyancy/1" or sidecar["variant"] != req["variant"]:
        raise ValueError("buoyancy sidecar schema/variant mismatch")
    if sidecar["urdf_sha256"] != hashlib.sha256((MODEL / "viscous_arm.urdf").read_bytes()).hexdigest():
        raise ValueError("buoyancy sidecar CAD hash mismatch")
    root = ET.parse(SCENE).getroot()
    for mesh in root.iter("mesh"):
        mesh.set("file", str((SCENE.parent / mesh.get("file")).resolve()))
    root.remove(root.find("keyframe"))
    root.find("compiler").set("inertiafromgeom", "auto")
    root.find("option").attrib.update(timestep=str(DT), integrator="implicitfast", cone="elliptic")
    world = root.find("worldbody")
    make_tank(world)
    defs = ET.SubElement(root, "default")
    ET.SubElement(defs, "geom", solref=".008 1", friction=".6 .002 .0002", condim="4")
    arm = json.loads(Path(req["arm_file"]).read_text())
    bodies = {b.get("name"): b for b in root.iter("body")}
    flange = bodies[arm["flange"]]
    ET.SubElement(flange, "site", name="tip", pos=fmt(arm["tcp"]), size=".001", rgba="0 0 0 0")
    # Top-down, arm base at image right. Physical KD mount/height not measured.
    ET.SubElement(world, "camera", name="head", pos="-.248 .01 .71", xyaxes="1 0 0 0 1 0", fovy="46")
    # Maker-style lens offset in the calibrated approach/jaw basis. Use the raw inverted
    # mount here and apply the physical 180-degree image rotation to the wrist stream.
    forward = np.array(arm["approach"])
    forward /= np.linalg.norm(forward)
    jaw = np.array(arm["jaw"])
    up = np.cross(forward, jaw)
    pos = np.array(arm["tcp"]) - 0.08749 * forward + 0.07935 * up
    z = -np.cos(np.radians(20)) * forward + np.sin(np.radians(20)) * up
    x = jaw / np.linalg.norm(jaw)
    y = np.cross(z, x)
    ET.SubElement(flange, "camera", name="left", pos=fmt(pos), xyaxes=fmt([*x, *y]), fovy="76")
    actuators = ET.SubElement(root, "actuator")
    links = sidecar["links"]
    for i, m in enumerate(MOTORS):
        jname = "gripper_left_joint" if m == "gripper" else m
        joint = root.find(f".//joint[@name='{jname}']")
        if m != "gripper":
            mp = MAPPING[m]
            limits = np.sort(np.radians(np.array(PROFILE[m]) * mp["sign"] + mp["offset_deg"]))
            # Published profile is a software limit backed off from mechanical stops.
            joint.set("range", fmt(limits + np.radians([-5, 5])))
        gear = GEAR if m == "gripper" else MAPPING[m]["sign"]
        joint.set("armature", str(0.003 * gear**2))
        joint.set("frictionloss", str([0.05, 0.11, 0.04, 0.07, 0.03, 0.03, 0.015][i] * abs(gear)))
        kp, kd = arm.get("gains", {}).get(m, GAINS[i])
        peak = arm.get("torque_limits_nm", {}).get(m, 2 * 10 / abs(GEAR) if m == "gripper" else 14.0)
        ET.SubElement(
            actuators,
            "position",
            name=m,
            joint=jname,
            group=str(i),
            kp=str(kp),
            kv=str(kd),
            gear=str(gear),
            forcerange=fmt([-peak, peak]),
        )
    # MuJoCo supports actuator groups 0..30. Keep all seven disabled until engage.
    for geom in world.iter("geom"):
        if geom.get("contype") == "1":
            geom.set("conaffinity", "6")
    xy = req.get("object_xy", [-0.325, 0.060])
    target = np.array([*req.get("target_xy", [-0.345, 0.005]), PLANES["floor"]])
    if req["task"] == "cube":
        add_box(
            world,
            "place_mark",
            target + [0, 0, 0.00012],
            [0.020, 0.020, 0.0001],
            contype="0",
            conaffinity="0",
            rgba=".84 .07 .09 1",
        )
    elements = make_object(world, req["task"], xy)
    rgb = ".88 .82 .58 1"
    for i, b in enumerate(sidecar.get("foam_blocks", [])):
        block = ET.SubElement(
            bodies[b["link"]],
            "body",
            name=f"foam_{i}",
            pos=fmt(b["center_m"]),
            quat=fmt(Rotation.from_euler("xyz", b.get("rpy", [0, 0, 0])).as_quat()[[3, 0, 1, 2]]),
        )
        size = np.asarray(b["size_m"])
        vol = float(b.get("volume_m3", b.get("volume", np.prod(size))))
        mass = float(b.get("mass_kg", b.get("mass", vol * b.get("density", 60))))
        add_box(
            block, f"block_{i}", [0, 0, 0], size / 2, mass=str(mass), rgba=rgb, contype="1", conaffinity="6"
        )
        elements.append((block, vol, np.zeros(3), size))
    for name, body in bodies.items():
        if name in ("base_link", "tool0"):
            continue
        info = links[name]
        vol, cob = float(info["volume_m3"]), np.asarray(info["cob_m"])
        # Tight local bounding box of the link's actual collision mesh vertices.
        points = []
        for geom in body.findall("geom"):
            mesh = root.find(f"./asset/mesh[@name='{geom.get('mesh')}']")
            data = Path(mesh.get("file")).read_bytes()
            faces = np.frombuffer(
                data, offset=84, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
            )
            points.append(faces["v"].reshape(-1, 3))
        dims = np.ptp(np.concatenate(points), axis=0)
        elements.append((body, vol, cob, dims))
    rho = float(sidecar.get("water_density_kg_m3", 1000))
    g = float(sidecar.get("g_m_s2", 9.81))
    root.find("option").set("gravity", fmt([0, 0, -g]))
    wet = []
    for body, vol, cob, dims in elements:
        if vol < 0 or not np.isfinite([vol, *cob, *dims]).all():
            raise ValueError(f"invalid displacement for {body.get('name')}")
        # Added water inertia persists for translations/rotations even at zero speed.
        # gravcomp cancels the extra body's weight only. Real dry/foam inertia stays.
        mass = added_scale * 0.5 * rho * vol
        if mass > 0:
            extra = ET.SubElement(body, "body", name=f"water_{body.get('name')}", pos=fmt(cob), gravcomp="1")
            inertia = mass * (np.sum(dims**2) - dims**2) / 20
            ET.SubElement(extra, "inertial", pos="0 0 0", mass=str(mass), diaginertia=fmt(inertia))
        wet.append((body.get("name"), vol, cob, dims))
    if visuals:
        add_visuals(root, bodies)
    model = mj.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    sources = [
        Path(__file__).with_name(n)
        for n in ("scene.py", "physics.py", "thermal.py", "cameras.py", "child.py")
    ]
    metadata = {
        "sim_source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        "sidecar_sha256": hashlib.sha256(json.dumps(sidecar, sort_keys=True).encode()).hexdigest(),
        "arm_file": str(req["arm_file"]),
        "rho": rho,
        "g": g,
        "viscosity_pa_s": 0.001,
        "planes_m": PLANES,
        "water_z_m": WATER,
        "drag_scale": drag_scale,
        "added_scale": added_scale,
        "object_xy": xy,
        "object_dry_mass_kg": sum(
            model.body_mass[model.body(n).id] for n, _, _, _ in wet if n.startswith("object_")
        ),
        "object_volume_m3": sum(v for n, v, _, _ in wet if n.startswith("object_")),
    }
    return model, wet, target, metadata
