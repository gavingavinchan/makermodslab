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
"""Hydrostatics in base_link and motor convention. Distances are metres, torques Nm."""

import itertools
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from makermodslab import tank_geometry as geo  # noqa: E402

URDF = ROOT / "configs/viscous/model/viscous_arm.urdf"
OUT = URDF.parent / "buoyancy"
KEEP = json.loads((geo.ENV_DIR / "keepout_viscous_waterproof_01.json").read_text())
MAP = json.loads((URDF.parent / "motor_to_urdf.json").read_text())
NOTICE = {key: MAP[key] for key in ("_copyright", "_license")}
JOINTS = geo.JOINTS
BODIES = tuple(
    link.get("name") for link in ET.parse(URDF).findall("link") if link.find("inertial") is not None
)
G = 9.81
RHO = 1000.0
SURFACE = (KEEP["planes_mm"]["floor"]["value"] + 283) / 1000
CORNERS = np.array(list(itertools.product((-0.5, 0.5), repeat=3)))


def write_json(name, data):
    (OUT / name).write_text(json.dumps(NOTICE | data, indent=2, allow_nan=False) + "\n")


def frames(model, poses):
    """Batch version of tank_geometry FK. Shape (N,7), motor degrees in JOINTS + gripper order."""
    eye = np.broadcast_to(np.eye(3), (len(poses), 3, 3))
    transforms = {"base_link": (eye, np.zeros((len(poses), 3)))}
    axes = []
    slide = (
        np.clip(
            (poses[:, 6] - model.gripper_motor_closed_deg)
            / (model.gripper_motor_open_deg - model.gripper_motor_closed_deg),
            0,
            1,
        )
        * model.gripper_slide_open_m
    )
    for joint in model.joints:
        parent_r, parent_t = transforms[joint.parent]
        r, t = parent_r @ joint.r, parent_r @ joint.t + parent_t
        if joint.kind == "revolute":
            j = JOINTS.index(joint.name)
            mapping = model.motor_to_urdf[joint.name]
            angle = np.radians(poses[:, j] * mapping["sign"] + mapping["offset_deg"])
            a = joint.axis / np.linalg.norm(joint.axis)
            axes.append((t, (r @ a) * mapping["sign"]))
            x, y, z = a
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            r = r @ (
                eye + np.sin(angle)[:, None, None] * skew + (1 - np.cos(angle))[:, None, None] * (skew @ skew)
            )
        elif joint.kind == "prismatic":
            t = t + slide[:, None] * (r @ joint.axis)
        transforms[joint.child] = (r, t)
    return transforms, axes


def lever(axes, link, points, weights=1):
    """Potential-energy derivative for a vertical unit weight at a world point."""
    depth = min(BODIES.index(link), len(JOINTS))
    result = np.zeros((len(points), len(JOINTS)))
    for i, (pivot, axis) in enumerate(axes[:depth]):
        p = points - pivot
        result[:, i] = (axis[:, 0] * p[:, 1] - axis[:, 1] * p[:, 0]) * G * weights
    return result


def mass_loads(model, poses, links, *, submerged=False):
    """Dry torque and buoyancy per displaced kg by link. Waterline samples carry equal volume."""
    transforms, axes = frames(model, poses)
    dry = np.zeros((len(poses), len(JOINTS)))
    water = []
    for name, link in links.items():
        r, t = transforms[name]
        dry += lever(axes, name, r @ link["com_m"] + t, link["mass_kg"])
        samples = np.asarray(link["volume_samples"])
        if submerged:
            water.append(lever(axes, name, r @ link["cob_m"] + t))
        else:
            points = np.einsum("nij,kj->nki", r, samples) + t[:, None, :]
            wet = points[:, :, 2] < SURFACE
            count = wet.sum(axis=1)
            center = (points * wet[:, :, None]).sum(axis=1) / np.maximum(count[:, None], 1)
            water.append(lever(axes, name, center, count / len(samples)))
    return dry, np.stack(water, axis=-1)


def block_load(model, poses, block, *, submerged=False):
    """Torque change per m³ of this block, including its dry weight."""
    transforms, axes = frames(model, poses)
    name = block["link"]
    r, t = transforms[name]
    center = np.array(block["center_m"])
    points = r @ center + t
    moment = lever(axes, name, points)
    if submerged:
        return (block["density"] - RHO) * moment
    # 4x4x4 midpoint integration, symmetric in every axis.
    grid = np.array(list(itertools.product((-0.375, -0.125, 0.125, 0.375), repeat=3)))
    samples = center + grid * block["size_m"]
    world = np.einsum("nij,kj->nki", r, samples) + t[:, None, :]
    wet = world[:, :, 2] < SURFACE
    count = wet.sum(axis=1)
    cob = (world * wet[:, :, None]).sum(axis=1) / np.maximum(count[:, None], 1)
    return block["density"] * moment - RHO * lever(axes, name, cob, count / len(grid))
