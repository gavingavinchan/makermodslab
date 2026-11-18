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
"""Batched base_link FK and position Jacobian, degrees and millimetres internally."""

import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
REAL = ROOT / "scripts/viscous-dream1/real"
sys.path.insert(0, str(REAL))
import dream_real_core as core  # noqa: E402

MODEL = ROOT / "configs/viscous/model"
URDF = MODEL / "viscous_arm.urdf"
MAP = MODEL / "motor_to_urdf.json"
ARM_FILE = REAL / "tank_model/viscous_waterproof_tank_arm.json"
JOINTS = core.ARM
LINK_JOINTS = (2, 3)
PARAMS = [
    *JOINTS,
    "tcp_x_mm",
    "tcp_y_mm",
    "tcp_z_mm",
    "sheet_x_mm",
    "sheet_y_mm",
    "sheet_yaw_deg",
    "upper_arm_length_mm",
    "forearm_length_mm",
]


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Arm:
    def __init__(self):
        arm = read(ARM_FILE)
        chain = core.chain(core.parse_urdf(URDF.read_bytes()), "base_link", arm["flange"])
        if [j.name for j in chain] != list(JOINTS):
            raise ValueError("Unexpected flange chain")
        self.xyz = np.array([j.xyz for j in chain]) * 1000
        self.axes = np.array([j.axis for j in chain])
        self.axes /= np.linalg.norm(self.axes, axis=1)[:, None]
        self.origins = Rotation.from_euler("xyz", [j.rpy for j in chain]).as_matrix()
        self.jaw = np.array(arm["jaw"])
        self.tcp = np.array(arm["tcp"]) * 1000
        mapping = core.JointMap.load(MAP)
        self.sign = np.array([mapping.sign[j] for j in JOINTS])
        self.offset = np.array([mapping.offset_deg[j] for j in JOINTS])

    def tip(self, motors, p, sign=None, direction=None):
        """Return (N,3) tips and (N,3,14) partials of tip xyz."""
        n = len(motors)
        angles = np.deg2rad(motors * (self.sign if sign is None else sign) + self.offset + p[:6])
        rot = np.broadcast_to(np.eye(3), (n, 3, 3))
        pos = np.zeros((n, 3))
        jac = np.zeros((n, 3, len(PARAMS)))
        centres, axes = [], []
        for i, xyz in enumerate(self.xyz):
            pos = pos + rot @ xyz
            if i in LINK_JOINTS:
                k = 12 + LINK_JOINTS.index(i)
                jac[:, :, k] = rot @ (xyz / np.linalg.norm(xyz))
                pos += p[k] * jac[:, :, k]
            rot = rot @ self.origins[i]
            centres.append(pos)
            axes.append(rot @ self.axes[i])
            rot = rot @ Rotation.from_rotvec(angles[:, i, None] * self.axes[i]).as_matrix()
        tip = pos + rot @ (self.tcp + p[6:9])
        for i in range(6):
            jac[:, :, i] = np.cross(axes[i], tip - centres[i]) * np.pi / 180
        jac[:, :, 6:9] = rot
        if direction is not None:
            tip = rot @ direction
            jac.fill(0)
            for i in range(6):
                jac[:, :, i] = np.cross(axes[i], tip) * np.pi / 180
        return tip, jac
