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
"""Tank FK, DLS tip IK and whole-command checks adapted from metal-arm-harness ik and safety."""

import dataclasses
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
REAL = ROOT / "scripts/viscous-dream1/real"
sys.path[:0] = [str(ROOT), str(REAL)]
import dream_real as real  # noqa: E402
from dream_real import core  # noqa: E402

from makermodslab import tank_geometry as geo, tank_guard  # noqa: E402

DT = core.ROW_DT_S
SLOW = 5.0
FAST = real.HARD_ARM_SPEED_DEG_S
ZONE = 76.0
LEG = 35.0
HAND = {"wrist_pitch_link", "wrist_yaw_link", "wrist_roll_link", "gripper_left", "gripper_right"}


class RefusalError(ValueError):
    """A request can be changed and retried without stopping the session."""


def mix(a, b, fraction):
    return {m: a[m] + fraction * (b[m] - a[m]) for m in a} if fraction < 1 else dict(b)


def travel(a, b):
    return max(abs(b[m] - a[m]) for m in core.ARM)


def check_leg(a, b):
    if travel(a, b) > LEG + 1e-8:
        raise RefusalError("excursion exceeds 35 degrees in one leg")


def deepens_floor_contact(height, next_height):
    return next_height < min(0.0, height) - 1e-7


class Model:
    def __init__(self, arm_file, env, folder, limits, lab=False):
        self.in_tank = env.name == "tank"
        self.spec = json.loads(arm_file.read_text())
        path = (arm_file.parent / self.spec["urdf"]).resolve()
        raw = path.read_bytes()
        self.sha = hashlib.sha256(raw).hexdigest()
        self.map = core.JointMap.load()
        self.joints = core.parse_urdf(raw)
        self.tcp = np.asarray(self.spec["tcp"])
        self.axis = np.asarray(self.spec["approach"])
        self.axis = self.axis / np.linalg.norm(self.axis)
        self.jaw = np.asarray(self.spec["jaw"])
        root = self.joints["base_link"]
        if (
            self.spec["base"] != "dream_base"
            or root.parent != "dream_base"
            or root.kind != "fixed"
            or any(root.xyz)
            or not np.allclose(root.rpy, [0, 0, math.pi])
        ):
            raise ValueError("expected colocated dream_base and base_link, rotated 180 degrees")
        if lab:
            sidecar = path.with_suffix(".provenance.json")
            if not sidecar.is_file():
                sidecar = real.DEFAULT_ARM_FILE.with_name("viscous_waterproof_tank.provenance.json")
            problem = core.approval_problem(raw, json.loads(sidecar.read_text())) or core.tool_point_problem(
                self.spec, raw
            )
            if problem:
                raise ValueError(problem)
        cfg = json.loads(env.keepout_file.read_text())
        tank = json.loads((ROOT / cfg["tank"]).read_text())
        # TankGuard works in physical base_link. Use the exact IK variant below that link,
        # including its shifted pan origin and the matching shifted pan coordinate.
        xml = ET.fromstring(raw)
        xml.remove(xml.find(f"joint[@name='{root.name}']"))
        xml.remove(xml.find("link[@name='dream_base']"))
        for mesh in xml.iter("mesh"):
            relative = mesh.attrib["filename"]
            filename = path.parent / relative
            if not filename.exists():
                filename = real.SRC_URDF.parent / relative
            mesh.set("filename", str(filename))
        derived = folder / "guard-base-link.urdf"
        ET.ElementTree(xml).write(derived)
        physical = geo.load_model({**tank, "arm_urdf": str(derived)})
        physical.motor_to_urdf["shoulder_pan"]["offset_deg"] -= self.map.pan_shift_deg
        self.guard = tank_guard.TankGuard(cfg, physical)
        self.limits = {
            m: (max(limits[m][0], self.guard.limits[m][0]), min(limits[m][1], self.guard.limits[m][1]))
            for m in core.MOTORS
        }
        ik_limits = self.map.ik_limits_deg(self.limits)
        joints = {j.name: j for j in self.joints.values()}
        self.lo = np.array([max(math.radians(ik_limits[m][0]), joints[m].lower) for m in core.ARM])
        self.hi = np.array([min(math.radians(ik_limits[m][1]), joints[m].upper) for m in core.ARM])

    def axes(self, angles):
        tf = core.fk(self.joints, "dream_base", self.spec["flange"], dict(zip(core.ARM, angles, strict=True)))
        r = tf[:3, :3]
        return tf[:3, 3] + r @ self.tcp, r @ self.axis, r @ self.jaw

    def pose(self, motor):
        tip, axis, jaw = self.axes(self.map.to_ik(motor))
        pitch = math.degrees(math.atan2(axis[2], np.linalg.norm(axis[:2])))
        yaw = math.degrees(math.atan2(axis[1], axis[0])) if np.linalg.norm(axis[:2]) > 1e-5 else None
        return {
            "xyz_m": tip.tolist(),
            "pitch_deg": pitch,
            "yaw_deg": yaw,
            "jaw_yaw_deg": math.degrees(math.atan2(jaw[1], jaw[0])),
        }

    def solve(self, start, xyz, pitch, yaw=None, poll=lambda: None):
        xyz = np.asarray(xyz, dtype=float)
        if xyz.shape != (3,) or not np.isfinite(xyz).all() or np.linalg.norm(xyz) > 2:
            raise RefusalError("tip must be finite and within reach (metres)")
        if not -90 <= pitch <= 90 or (yaw is not None and not math.isfinite(yaw)):
            raise RefusalError("pitch must be within ±90 degrees and yaw must be finite")
        q = np.clip(self.map.to_ik(start), self.lo, self.hi)
        p = math.radians(pitch)

        # At either pole solve the direction vector; elevation alone has a cusp there.
        def residual(angles):
            pos, axis, jaw = self.axes(angles)
            elevation = math.atan2(axis[2], np.linalg.norm(axis[:2]))
            orient = axis - [0, 0, math.sin(p)] if abs(pitch) == 90 else [elevation - p]
            if yaw is not None:
                heading = math.radians(yaw)
                if abs(pitch) == 90:
                    # At a pole, forward projects to a point. Orient the transverse
                    # opening axis at yaw+90 instead, keeping jaw heading observable.
                    orient = np.append(
                        orient,
                        math.remainder(math.atan2(jaw[1], jaw[0]) - heading - math.pi / 2, 2 * math.pi),
                    )
                else:
                    orient = [
                        elevation - p,
                        math.remainder(math.atan2(axis[1], axis[0]) - heading, 2 * math.pi),
                    ]
            return np.r_[pos - xyz, np.asarray(orient) * 0.18]

        probes = np.eye(6) * 0.001
        for _ in range(150):
            poll()
            e = residual(q)
            if np.linalg.norm(e[:3]) <= 0.002 and np.linalg.norm(e[3:]) / 0.18 <= math.radians(2):
                return {**self.map.to_motor(q), "gripper": start["gripper"]}
            jac = np.column_stack([(residual(q + d) - e) / 0.001 for d in probes])
            step = -jac.T @ np.linalg.solve(jac @ jac.T + 0.015**2 * np.eye(len(e)), e)
            q = np.clip(q + step / max(1, np.max(abs(step)) / 0.15), self.lo, self.hi)
        raise RefusalError(
            f"IK refused: residual {1000 * np.linalg.norm(e[:3]):.1f} mm, orientation {math.degrees(np.linalg.norm(e[3:]) / 0.18):.1f} deg (limits 2 mm / 2 deg)"
        )

    def clearance(self, q):
        slack = self.guard.clearances(q)
        dist = {(n, f): s + self.guard.required[n][f] for (n, f), s in slack.items()}
        post = min(d for (_, face), d in dist.items() if face == "near")
        walls = [d for (_, f), d in dist.items() if f in ("left", "right", "far")]
        if self.in_tank:
            walls.append(post + 12)  # 12 mm mount post sits flush on the back glass.
        return {
            "jaw_floor_mm": min(
                d for (n, f), d in dist.items() if n in ("gripper_left", "gripper_right") and f == "floor"
            ),
            "hand_floor_mm": min(d for (n, f), d in dist.items() if n in HAND and f == "floor"),
            "wall_mm": min(walls),
            "post_mm": post,
        }

    def check_step(self, a, b, speed, floor_a, floor_b, rest):
        cap = SLOW if min(floor_a, floor_b) <= ZONE else FAST
        if travel(a, b) > speed * DT + 1e-8 or not 0 < speed <= cap:
            raise RefusalError("waypoint speed exceeds cap")
        if deepens_floor_contact(floor_a, floor_b):
            raise RefusalError("floor: hand below the measured glass may only stay level or rise")
        if any(not min(lo, rest[m]) <= b[m] <= max(hi, rest[m]) for m, (lo, hi) in self.limits.items()):
            raise RefusalError("joint soft limit")
        decision = self.guard.limit(a, b, allowed=rest)
        if decision.status != "ok":
            raise RefusalError(f"{decision.reason}: {decision.verdict or decision.clipped_joints}")
        return decision.verdict

    def plan(self, start, goals, rest, poll=lambda: None):
        """All waypoints checked before execution. Bound the swept slow-zone crossing too."""
        a, steps = start, []
        height = self.clearance(a)["hand_floor_mm"]
        for goal in goals:
            origin = a
            legs = max(1, math.ceil(travel(origin, goal) / LEG))
            for i in range(1, legs + 1):
                end = mix(origin, goal, i / legs)
                check_leg(a, end)
                leg_start = a
                # One straight path per leg. Use the same fraction for every joint.
                while a != end:
                    poll()
                    t = travel(a, end)
                    grip = abs(a["gripper"] - end["gripper"])
                    frac = min(1, 1 / max(t / (FAST * DT), grip / (FAST * DT), 1))
                    b = mix(a, end, frac)
                    speed = SLOW if height - self.guard.motion_bound_mm(a, b) <= ZONE else FAST
                    if speed == SLOW:
                        b = mix(a, end, 1 / max(t / (speed * DT), grip / (FAST * DT), 1))
                    next_height = self.clearance(b)["hand_floor_mm"]
                    verdict = self.check_step(a, b, speed, height, next_height, rest)
                    steps.append((b, leg_start, speed, dataclasses.asdict(verdict)))
                    a, height = b, next_height
        return steps
