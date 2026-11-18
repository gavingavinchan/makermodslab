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
"""Fixed-step underwater dynamics, direct contact evidence and task scoring."""

import hashlib
import itertools
import json
import math

import mujoco as mj
import numpy as np
from scene import CLOSED, DT, GEAR, MAPPING, MODEL, MOTORS, PLANES, WATER, configure, motor_vector
from scipy.spatial import ConvexHull
from thermal import Thermal


def motors(values):
    return dict(zip(MOTORS, values.tolist(), strict=True))


def crosses(a, b):
    # np.cross's axis handling dominates these small batches at 500 Hz.
    return np.column_stack(
        (
            a[:, 1] * b[:, 2] - a[:, 2] * b[:, 1],
            a[:, 2] * b[:, 0] - a[:, 0] * b[:, 2],
            a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0],
        )
    )


def local_collision_hulls(model, bodies):
    """Convex hull for each rigid link gives exact extreme points against tank planes."""
    hulls = []
    for bid in bodies:
        parts = []
        for gid in np.where(model.geom_bodyid == bid)[0]:
            if not model.geom_contype[gid]:
                continue
            mid = model.geom_dataid[gid]
            if model.geom_type[gid] == mj.mjtGeom.mjGEOM_MESH:
                start = model.mesh_vertadr[mid]
                v = model.mesh_vert[start : start + model.mesh_vertnum[mid]]
            elif model.geom_type[gid] == mj.mjtGeom.mjGEOM_BOX:
                v = np.array(list(itertools.product([-1, 1], repeat=3))) * model.geom_size[gid]
            else:
                raise ValueError("unexpected arm collision geometry")
            rot = np.empty(9)
            mj.mju_quat2Mat(rot, model.geom_quat[gid])
            parts.append(v @ rot.reshape(3, 3).T + model.geom_pos[gid])
        if parts:
            vertices = np.unique(np.concatenate(parts), axis=0)
            hulls.append((bid, vertices[ConvexHull(vertices).vertices]))
    return hulls


class Tank:
    def __init__(self, req, sidecar=None, *, drag_scale=1.0, added_scale=1.0, visuals=False):
        if req["variant"] not in ("current", "foamed") or req["task"] not in ("cube", "ball", "screwdriver"):
            raise ValueError("unknown variant or task")
        if not all(math.isfinite(v) and v >= 0 for v in (drag_scale, added_scale)):
            raise ValueError("scale factors must be finite and nonnegative")
        sidecar_path = None
        if sidecar is None:
            sidecar_path = MODEL / "buoyancy" / (req["variant"] + ".json")
            sidecar = json.loads(sidecar_path.read_text())
        self.model, wet, self.target, self.metadata = configure(
            req, sidecar, drag_scale, added_scale, visuals=visuals
        )
        if sidecar_path is not None:
            self.metadata["sidecar_file_sha256"] = hashlib.sha256(sidecar_path.read_bytes()).hexdigest()
        m = self.model
        self.data = mj.MjData(m)
        self.task = req["task"]
        self.bids = np.array([m.body(n).id for n, *_ in wet])
        self.vol = np.array([v for _, v, _, _ in wet])
        self.cobs = np.stack([c for _, _, c, _ in wet])
        self.dims = np.stack([d for _, _, _, d in wet])
        # Sampled displaced volumes follow the hydrostatic sidecar exactly. Foam
        # uses the same 4x4x4 midpoint cubature as the static optimizer.
        groups = {}
        for i, (name, _, _, dims) in enumerate(wet):
            if name in sidecar["links"]:
                samples = np.asarray(sidecar["links"][name]["volume_samples"])
            elif name.startswith("foam_"):
                grid = np.array(list(itertools.product((-0.375, -0.125, 0.125, 0.375), repeat=3)))
                samples = grid * dims
            else:
                continue
            groups.setdefault(len(samples), []).append((i, samples))
        self.samples = [
            (np.array([i for i, _ in rows]), np.stack([s for _, s in rows])) for rows in groups.values()
        ]
        self.areas = np.pi / 4 * np.prod(self.dims, axis=1)[:, None] / self.dims
        self.cd = np.array([0.47 if n == "object_ball" else 1.05 for n, *_ in wet])[:, None] * drag_scale
        self.spin_factors = np.zeros_like(self.dims)
        for axis in range(3):
            j, k = (axis + 1) % 3, (axis + 2) % 3
            self.spin_factors[:, axis] = (
                self.dims[:, j] * self.dims[:, k] ** 4 + self.dims[:, k] * self.dims[:, j] ** 4
            ) / 32
        self.radius = np.cbrt(3 * self.vol / (4 * np.pi))
        self.rootids = m.body_rootid[self.bids]
        self.rho = self.metadata["rho"]
        self.g = self.metadata["g"]
        self.thermal = Thermal(DT)
        self.offsets = np.array([MAPPING[k]["offset_deg"] for k in MAPPING] + [CLOSED])
        self.signs = np.array([MAPPING[k]["sign"] for k in MAPPING] + [GEAR])
        self.offsets[:-1] *= -self.signs[:-1]
        self.joints = np.array([m.joint(n).qposadr[0] for n in [*MAPPING, "gripper_left_joint"]])
        self.right = m.joint("gripper_right_joint").qposadr[0]
        self.objectid = m.body("object").id
        self.tipid = m.site("tip").id
        self.oid = m.joint("object_free").dofadr[0]
        self.surfaces = {
            m.geom("glass_" + face).id: "glass_" + face for face in ("floor", "left", "right", "near", "far")
        }
        self.surfaces[m.geom("post").id] = "post"
        self.geoms = [m.geom(i).name for i in range(m.ngeom)]
        self.names = [m.body(i).name for i in range(m.nbody)]
        self.arm_bids = {int(i) for i in self.bids if not self.names[i].startswith("object_")}
        self.hulls = local_collision_hulls(m, self.arm_bids)
        self.mask = 127
        m.opt.disableactuator = self.mask
        self.data.qpos[self.joints] = np.radians((motor_vector(req["start"]) - self.offsets) / self.signs)
        self.data.qpos[self.right] = self.data.qpos[self.joints[-1]] = np.clip(
            self.data.qpos[self.joints[-1]], 0, m.joint("gripper_left_joint").range[1]
        )
        mj.mj_forward(m, self.data)
        self.initial_z = self.data.xpos[self.objectid, 2]
        if self.task != "cube":
            self.target = self.data.xpos[self.objectid] + [0, 0, 0.05]
        self.peak_lift = 0.0
        self.grasped_lift = 0.0
        self.picked = False
        self.success = False
        self.success_duration = 0.0
        self.glass_min = math.inf
        self.peak = np.zeros(7)
        self.integral = np.zeros(7)
        self.hold_peak = np.zeros(7)
        self.hold_integral = np.zeros(7)
        self.hold_time = np.zeros(7)
        self.last_pairs = set()
        self.events = []
        self.object_events = []
        self.object_peak_glass_speed = 0.0
        self.first_floor = None
        self.peak_glass_speed = 0.0
        self.peak_floor_normal = 0.0
        self.grasp = set()
        self.object_floor = False
        self.step_count = 0
        self.label = ""
        self.clearances()

    def pos(self):
        return motors(np.degrees(self.data.qpos[self.joints]) * self.signs + self.offsets)

    def read(self):
        return {
            "pos": self.pos(),
            "torque": motors(self.data.actuator_force),
            "temp": motors(self.thermal.temp),
        }

    def engage(self):
        self.goal(self.pos())
        self.mask = 0
        self.apply_mask()
        return {"pos": self.pos()}

    def goal(self, target):
        # MJ transmissions output sign*q for hinges and GEAR*slide on J7.
        self.data.ctrl[:] = np.radians(motor_vector(target) - self.offsets)

    def apply_mask(self):
        self.model.opt.disableactuator = self.mask | sum(
            1 << int(i) for i in np.flatnonzero(self.thermal.limp)
        )

    def release(self, gripper_only=False):
        self.mask |= (1 << 6) if gripper_only else 127
        self.apply_mask()

    def water_forces(self):
        d = self.data
        ids = self.bids
        rot = d.xmat[ids].reshape(-1, 3, 3)
        center = d.xpos[ids] + (rot @ self.cobs[:, :, None])[:, :, 0]
        # Exact volume fraction and centroid of a clipped ellipsoid. Its nominal
        # centre is the calibrated COB. Axes approximate the collision envelope.
        local_z = rot[:, 2, :]
        a2n = (self.dims * 0.5) ** 2 * local_z
        h = np.sqrt((a2n * local_z).sum(axis=1))
        t = np.clip((WATER - center[:, 2]) / h, -1, 1)
        fraction = (1 + t) ** 2 * (2 - t) * 0.25
        # Conditional mean of ellipsoid slice coordinate for -1 <= z/h <= t.
        shift = -0.75 * (1 - t) ** 2 / (2 - t)
        point = center + (rot @ (a2n * (shift / h)[:, None])[:, :, None])[:, :, 0]
        for idx, local in self.samples:
            points = local @ rot[idx].transpose(0, 2, 1) + d.xpos[ids[idx], None, :]
            immersed = points[:, :, 2] < WATER
            count = immersed.sum(axis=1)
            fraction[idx] = count / local.shape[1]
            centroid = (points * immersed[:, :, None]).sum(axis=1) / np.maximum(count, 1)[:, None]
            point[idx] = np.where(count[:, None] > 0, centroid, center[idx])
        twist = d.cvel[ids]
        omega = twist[:, :3]
        v = twist[:, 3:] + crosses(omega, point - d.subtree_com[self.rootids])
        rt = rot.transpose(0, 2, 1)
        vl = (rt @ v[:, :, None])[:, :, 0]
        wl = (rt @ omega[:, :, None])[:, :, 0]
        viscosity = self.metadata["viscosity_pa_s"]
        fd = (
            -0.5 * self.rho * self.cd * self.areas * np.abs(vl) * vl
            - (6 * np.pi * viscosity * self.radius)[:, None] * vl
        )
        force = (rot @ fd[:, :, None])[:, :, 0] * fraction[:, None]
        force[:, 2] += self.rho * self.g * self.vol * fraction
        # Broadside pressure torque integrated along each of the other two
        # ellipsoid diameters. Box-strip approximation, zero net force for pure spin.
        spin = (
            -0.5 * self.rho * self.cd * self.spin_factors * np.abs(wl) * wl
            - (8 * np.pi * viscosity * self.radius**3)[:, None] * wl
        )
        d.xfrc_applied[ids, :3] = force
        d.xfrc_applied[ids, 3:] = (
            crosses(point - d.xipos[ids], force) + (rot @ spin[:, :, None])[:, :, 0] * fraction[:, None]
        )

    def clearances(self):
        mj.mj_kinematics(self.model, self.data)
        lo = np.full(3, np.inf)
        hi = -lo
        for bid, vertices in self.hulls:
            points = vertices @ self.data.xmat[bid].reshape(3, 3).T + self.data.xpos[bid]
            lo = np.minimum(lo, points.min(axis=0))
            hi = np.maximum(hi, points.max(axis=0))
        out = {
            "floor": lo[2] - PLANES["floor"],
            "far": lo[0] - PLANES["far"],
            "near": PLANES["near"] + 0.012 - hi[0],
            "left": PLANES["left"] - hi[1],
            "right": lo[1] - PLANES["right"],
        }
        self.clearance = {k: float(v) for k, v in out.items()}
        self.glass_min = min(self.glass_min, min(out.values()))

    def contacts(self):
        d, m = self.data, self.model
        pairs = {}
        self.grasp = set()
        self.object_floor = False
        for contact in d.contact:
            g1, g2 = contact.geom
            if contact.dist > 0:
                continue
            a, b = self.geoms[g1], self.geoms[g2]
            if a.startswith("task_") or b.startswith("task_"):
                obj, other = (a, b) if a.startswith("task_") else (b, a)
                self.object_floor |= other == "glass_floor"
                if self.task != "screwdriver" or obj == "task_handle":
                    for finger in ("gripper_left", "gripper_right"):
                        if other.startswith("arm_" + finger + "_"):
                            self.grasp.add(finger)
            if g1 in self.surfaces:
                surface, moving, sign = self.surfaces[g1], g2, -1.0
            elif g2 in self.surfaces:
                surface, moving, sign = self.surfaces[g2], g1, 1.0
            else:
                continue
            bid = m.geom_bodyid[moving]
            is_arm = bid in self.arm_bids
            if not is_arm and not self.geoms[moving].startswith("task_"):
                continue
            # cvel is world-oriented at the root subtree COM before integration.
            twist = d.cvel[bid]
            v = twist[3:] + np.cross(twist[:3], contact.pos - d.subtree_com[m.body_rootid[bid]])
            normal = max(0.0, sign * float(v @ contact.frame[:3]))
            event = {
                "t_s": float(d.time),
                "surface": surface,
                "actor": "arm" if is_arm else "object",
                "link": self.names[bid],
                "geom": self.geoms[moving],
                "normal_m_s": normal,
                "speed_m_s": float(np.linalg.norm(v)),
                "xyz_m": contact.pos.tolist(),
            }
            key = (surface, bid)
            if key not in pairs or normal > pairs[key]["normal_m_s"]:
                pairs[key] = event
            if not is_arm and surface.startswith("glass_"):
                self.object_peak_glass_speed = max(self.object_peak_glass_speed, event["speed_m_s"])
            if is_arm and surface.startswith("glass_"):
                self.glass_min = min(self.glass_min, contact.dist)
                self.peak_glass_speed = max(self.peak_glass_speed, event["speed_m_s"])
                if surface == "glass_floor":
                    self.peak_floor_normal = max(self.peak_floor_normal, normal)
        for key, event in pairs.items():
            if key not in self.last_pairs:
                (self.events if event["actor"] == "arm" else self.object_events).append(event)
        floor = [v for (s, bid), v in pairs.items() if s == "glass_floor" and bid in self.arm_bids]
        if self.first_floor is None and floor:
            self.first_floor = max(floor, key=lambda v: v["normal_m_s"])
        self.last_pairs = set(pairs)

    def step(self):
        d, m = self.data, self.model
        mj.mj_step1(m, d)
        self.water_forces()
        self.contacts()
        lift = d.xpos[self.objectid, 2] - self.initial_z
        self.peak_lift = max(self.peak_lift, lift)
        both = len(self.grasp) == 2
        if both and not self.object_floor:
            self.grasped_lift = max(self.grasped_lift, lift)
            self.picked |= lift >= 0.010
        if self.task == "cube":
            qualifies = (
                self.picked
                and not self.grasp
                and self.object_floor
                and np.linalg.norm(d.xpos[self.objectid, :2] - self.target[:2]) <= 0.020
                and np.linalg.norm(d.qvel[self.oid : self.oid + 3]) < 0.01
            )
        else:
            qualifies = both and lift >= 0.05 and not self.object_floor
        self.success_duration = self.success_duration + DT if qualifies else 0.0
        self.success |= self.success_duration >= 0.20
        mj.mj_step2(m, d)
        torque = d.actuator_force
        magnitude = np.abs(torque)
        self.peak = np.maximum(self.peak, magnitude)
        self.integral += magnitude * DT
        holding = np.abs(d.actuator_velocity) < np.radians(1)
        enabled = np.array([not (m.opt.disableactuator & (1 << i)) for i in range(7)])
        self.hold_peak = np.maximum(self.hold_peak, magnitude * holding * enabled)
        self.hold_integral += magnitude * holding * enabled * DT
        self.hold_time += holding * enabled * DT
        self.thermal.advance(torque)
        if self.thermal.limp.any():
            self.apply_mask()
        self.step_count += 1
        if self.step_count % 25 == 0:
            self.clearances()
        if not np.isfinite(d.qpos).all() or d.warning.number.any():
            raise RuntimeError(f"MuJoCo numerical warning at {d.time:.3f}s: {d.warning.number.tolist()}")

    def truth(self):
        d = self.data
        mj.mj_kinematics(self.model, d)
        return {
            "t_s": float(d.time),
            "frame": "base_link",
            "object": d.xpos[self.objectid].tolist(),
            "tip": d.site_xpos[self.tipid].tolist(),
            "target": self.target.tolist(),
            "task": self.task,
            "success": bool(self.success),
            "bilateral_contact": len(self.grasp) == 2,
            "contact_fingers": sorted(self.grasp),
            "object_on_floor": bool(self.object_floor),
            "object_lift_m": float(d.xpos[self.objectid, 2] - self.initial_z),
            "peak_lift_m": float(self.peak_lift),
            "grasped_lift_m": float(self.grasped_lift),
            "target_error_xy_m": float(np.linalg.norm(d.xpos[self.objectid, :2] - self.target[:2])),
            "object_quat_wxyz": d.xquat[self.objectid].tolist(),
            "clearance_m": self.clearance,
            "min_glass_clearance_m": self.glass_min,
            "first_floor_contact": self.first_floor,
            "peak_glass_contact_speed_m_s": self.peak_glass_speed,
            "peak_floor_normal_m_s": self.peak_floor_normal,
            "object_peak_glass_contact_speed_m_s": self.object_peak_glass_speed,
            "torque_peak_nm": motors(self.peak),
            "torque_abs_integral_nm_s": motors(self.integral),
            "hold_peak_nm": motors(self.hold_peak),
            "hold_abs_integral_nm_s": motors(self.hold_integral),
            "hold_time_s": motors(self.hold_time),
            "overload_nm2_s": motors(self.thermal.dose),
            "limp": [m for m, v in zip(MOTORS, self.thermal.limp, strict=True) if v],
            "enabled_mask": (~self.model.opt.disableactuator) & 127,
            "label": self.label,
        }
