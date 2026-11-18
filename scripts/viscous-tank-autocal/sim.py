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
"""MuJoCo backend: the arm in a randomly placed aquarium, MIT-style joints, honest sensors.

Runs only in the MuJoCo venv (~/dreamtrial/.venv), never the Lab's. The arm comes from the
committed aquarium scene (read, never edited: configs/viscous/environments/aquarium_landen_75/
aquarium_scene.xml, generated from the repo URDF, whose FK equals the guard's to 1e-13 mm); the
glass, the post and any obstacles are this module's, placed from a true tank the session never sees.

Each joint is an MIT position loop at 1 kHz on the MOTOR side of a series structural spring:
    tau = kp (q_target - q_motor) - kd qdot,  q_motor = q_link + tau / K_flex + backlash(tau),
with the RS00's 14 Nm peak as a clip. The encoder reports q_motor, so flex and backlash are exactly
the part of the arm's bend the joint encoders cannot see. Water: buoyancy as a gravity-compensation
fraction per body, joint damping for drag, static friction (frictionloss) 0.04-0.11 Nm as measured
on the RS00s (2026-10-20). Telemetry noise: 0.15 Nm torque, 0.01 deg position.
"""

from __future__ import annotations

import dataclasses
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from context import ROOT, core, real
from geometry import ARM, POST_CENTRE_Y_MM, POST_FACE_X_MM, POST_WIDTH_MM, Estimate, geo

TEMPLATE = ROOT / "configs/viscous/environments/aquarium_landen_75/aquarium_scene.xml"
PHYSICS_DT = 0.001
TORQUE_PEAK_NM = 14.0


@dataclasses.dataclass
class Physics:
    buoyancy: float = 0.25  # fraction of each body's weight the water carries
    damping: float = 0.4  # Nm s/rad per joint (drag)
    friction_nm: dict = dataclasses.field(
        default_factory=lambda: {
            "shoulder_pan": 0.08,
            "shoulder_lift": 0.11,
            "elbow_flex": 0.04,
            "wrist_flex": 0.07,
            "wrist_yaw": 0.07,
            "wrist_roll": 0.07,
        }
    )
    armature: float = 0.02  # reflected rotor inertia, kg m^2
    # Structural stiffness behind each encoder (gearbox, housings, links): UNKNOWN on this arm, so a
    # moderate guess by default and the "flex_severe" scenario at less than half of it.
    flex_nm_per_rad: dict = dataclasses.field(
        default_factory=lambda: {
            "shoulder_pan": 1500.0,
            "shoulder_lift": 1500.0,
            "elbow_flex": 1200.0,
            "wrist_flex": 600.0,
            "wrist_yaw": 600.0,
            "wrist_roll": 600.0,
        }
    )
    backlash_deg: float = 0.05  # +- free play per joint, unseen by the encoder
    torque_noise_nm: float = 0.15
    position_noise_deg: float = 0.01
    encoder_bias_deg: dict = dataclasses.field(default_factory=dict)  # a calibration / URDF error
    glass_friction: float = 0.4


@dataclasses.dataclass
class SimObstacle:
    """A cylinder standing on (or against) `face`: rigid, or on a spring (a cable bundle)."""

    face: str
    centre_mm: tuple  # base frame, on the face
    radius_mm: float
    height_mm: float
    stiffness_n_per_m: float | None = None  # None: rigid
    name: str = "obstacle"


def _f(values) -> str:
    return " ".join(f"{float(v):.9g}" for v in values)


def build_scene(
    truth: Estimate, physics: Physics, motor_map: dict, gripper_slide_m: float, obstacles=(), tip_local_m=None
) -> str:
    root = ET.parse(TEMPLATE).getroot()
    for mesh in root.iter("mesh"):
        mesh.set("file", str((TEMPLATE.parent / mesh.get("file")).resolve()))
    for tag in ("keyframe", "equality"):
        for node in root.findall(tag):
            root.remove(node)
    world = root.find("worldbody")
    for geom in list(world.findall("geom")):
        world.remove(geom)  # the template's nominal glass and water line
    option = root.find("option")
    option.set("timestep", str(PHYSICS_DT))
    option.set("integrator", "implicitfast")
    option.set("gravity", "0 0 -9.81")
    for body in root.iter("body"):
        if body.get("name") != "base_link":
            body.set("gravcomp", f"{physics.buoyancy:.4f}")
        for joint in list(body.findall("joint")):
            name = joint.get("name")
            if name in ("gripper_left_joint", "gripper_right_joint"):
                # The closed jaw: the finger slide at its closed opening, rigid.
                axis = np.array([float(v) for v in joint.get("axis").split()])
                pos = np.array([float(v) for v in body.get("pos", "0 0 0").split()])
                body.set("pos", _f(pos + axis * gripper_slide_m))
                body.remove(joint)
                continue
            joint.attrib.pop("range", None)
            joint.set("limited", "false")
            joint.set("damping", str(physics.damping))
            joint.set("frictionloss", str(physics.friction_nm.get(name, 0.05)))
            joint.set("armature", str(physics.armature))
    for geom in root.iter("geom"):
        if geom.get("name", "").startswith("arm_"):
            geom.set("friction", f"{physics.glass_friction} 0.005 0.0001")
            geom.set("solref", "0.002 1")
    if tip_local_m is not None:
        wrist = next(b for b in root.iter("body") if b.get("name") == "wrist_roll_link")
        ET.SubElement(wrist, "site", name="probe_tip", pos=_f(tip_local_m), size="0.003", rgba="1 0.2 0.2 1")
    axes = geo.tank_axes(truth.yaw_deg)
    o = truth.offsets()
    glass = truth.dims["glass"] / 1000
    length, width, height = (truth.dims[k] / 1000 for k in ("inner_length", "inner_width", "inner_height"))
    cx, cy, cz = (
        (o["near"] + o["far"]) / 2000,
        (o["left"] + o["right"]) / 2000,
        (o["floor"] / 1000 + height / 2),
    )
    quat = _f([math.cos(math.radians(truth.yaw_deg) / 2), 0, 0, math.sin(math.radians(truth.yaw_deg) / 2)])
    panes = {
        "near": ([o["near"] / 1000 + glass / 2, cy, cz], [glass / 2, width / 2 + glass, height / 2]),
        "far": ([o["far"] / 1000 - glass / 2, cy, cz], [glass / 2, width / 2 + glass, height / 2]),
        "left": ([cx, o["left"] / 1000 + glass / 2, cz], [length / 2, glass / 2, height / 2]),
        "right": ([cx, o["right"] / 1000 - glass / 2, cz], [length / 2, glass / 2, height / 2]),
        "floor": (
            [cx, cy, o["floor"] / 1000 - glass / 2],
            [length / 2 + glass, width / 2 + glass, glass / 2],
        ),
    }
    env = dict(
        contype="2",
        conaffinity="1",
        friction=f"{physics.glass_friction} 0.005 0.0001",
        solref="0.002 1",
        solimp="0.95 0.99 0.001",
    )
    for name, (centre_tank, half) in panes.items():
        ET.SubElement(
            world,
            "geom",
            name=f"glass_{name}",
            type="box",
            pos=_f(axes.T @ np.array(centre_tank)),
            size=_f(half),
            quat=quat,
            rgba="0.45 0.75 0.85 0.18",
            group="1",
            **env,
        )
    # The mount's truss post, rigid with the base, flush on the back glass at its bearing point.
    post_z0, post_h = truth.floor / 1000, 0.30
    ET.SubElement(
        world,
        "geom",
        name="post",
        type="box",
        pos=_f([(POST_FACE_X_MM + 6) / 1000, POST_CENTRE_Y_MM / 1000, post_z0 + post_h / 2]),
        size=_f([0.006, POST_WIDTH_MM / 2000, post_h / 2]),
        rgba="0.15 0.15 0.15 1",
        group="1",
        **env,
    )
    for ob in obstacles:
        n = truth.normal(ob.face)
        centre = np.asarray(ob.centre_mm, dtype=float) / 1000 - n * ob.height_mm / 2000
        z = np.array([0.0, 0.0, 1.0])
        axis = np.cross(z, -n)
        angle = math.acos(float(np.clip(z @ -n, -1, 1)))
        q = (
            [1, 0, 0, 0]
            if np.linalg.norm(axis) < 1e-9
            else [math.cos(angle / 2), *(axis / np.linalg.norm(axis) * math.sin(angle / 2))]
        )
        if ob.stiffness_n_per_m is None:
            ET.SubElement(
                world,
                "geom",
                name=ob.name,
                type="cylinder",
                pos=_f(centre),
                quat=_f(q),
                size=_f([ob.radius_mm / 1000, ob.height_mm / 2000]),
                rgba="0.8 0.5 0.2 1",
                group="1",
                **env,
            )
        else:
            body = ET.SubElement(world, "body", name=ob.name, pos=_f(centre))
            ET.SubElement(
                body,
                "joint",
                name=f"{ob.name}_spring",
                type="slide",
                axis=_f(n),
                stiffness=str(ob.stiffness_n_per_m),
                damping="20",
                range="-0.03 0.002",
                limited="true",
            )
            ET.SubElement(
                body,
                "geom",
                name=f"{ob.name}_geom",
                type="cylinder",
                quat=_f(q),
                mass="0.02",
                size=_f([ob.radius_mm / 1000, ob.height_mm / 2000]),
                rgba="0.1 0.1 0.1 1",
                group="1",
                **env,
            )
    ET.SubElement(world, "light", pos="-0.3 0 1.2", dir="0 0 -1", diffuse="0.6 0.6 0.6")
    vis = root.find("visual")
    vis.find("global").set("offwidth", "960")
    vis.find("global").set("offheight", "720")
    return ET.tostring(root, encoding="unicode")


class SimClock:
    def __init__(self, arm=None):
        self.arm = arm
        self.t = 1000.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        if self.arm is not None and dt > 0:
            self.arm.advance(dt)
        else:
            self.t += max(0.0, dt)


class MujocoArm(real.FakeArm):
    """The FakeArm interface (connect/read/engage/send/reanchor/return_to_rest/release) over MuJoCo."""

    def __init__(
        self,
        start,
        truth,
        pm,
        kp_kd: dict,
        physics: Physics | None = None,
        obstacles=(),
        seed=0,
        fault=None,
        fault_tick=10**9,
        recorder=None,
    ):
        self.physics = physics or Physics()
        self.clock = SimClock(self)
        super().__init__(start, [], self.clock, scenario="nominal", sag={}, heat_per_tick=0.0)
        self.pm = pm
        self.map = {m: pm.motor_to_urdf[m] for m in ARM}
        # The scene is the repo URDF; its pan origin is the guard model's pan offset away.
        repo = geo.load_model(_tank())
        self.scene_map = {m: repo.motor_to_urdf[m] for m in ARM}
        slide = geo.joint_positions(pm, start)["gripper_left_joint"]
        axis = next(j.t for j in pm.joints if j.name == "tool_fixed")
        r, t = geo.forward_kinematics(pm, start)["wrist_roll_link"]
        tip_local = r.T @ (geo.probe_point(pm, geo.forward_kinematics(pm, start)) / 1000 - t)
        del axis
        xml = build_scene(truth, self.physics, self.scene_map, slide, obstacles, tip_local)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.jid = {m: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, m) for m in ARM}
        self.qadr = {m: self.model.jnt_qposadr[self.jid[m]] for m in ARM}
        self.vadr = {m: self.model.jnt_dofadr[self.jid[m]] for m in ARM}
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "probe_tip")
        self.kp = {m: kp_kd[m][0] for m in ARM}
        self.kd = {m: kp_kd[m][1] for m in ARM}
        self.rng = np.random.default_rng(seed)
        self.fault, self.fault_tick = fault, fault_tick
        self.env_geoms = {
            i
            for i in range(self.model.ngeom)
            if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith(
                ("glass_", "post", "obstacle", "cable", "rock")
            )
        }
        self.arm_geoms = {
            i
            for i in range(self.model.ngeom)
            if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith("arm_")
        }
        self.finger_bodies = {
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n)
            for n in ("gripper_left", "gripper_right")
        }
        for m in ARM:
            self.data.qpos[self.qadr[m]] = self._urdf(m, start[m])
        mujoco.mj_forward(self.model, self.data)
        self.tau = dict.fromkeys(ARM, 0.0)
        self._held = False
        self.encoder = dict(start)
        self.target = dict(start)
        self.pos = dict(start)
        self.recorder = recorder
        self.contact_peak = []  # per 50 ms tick: (t, max total normal force N, surfaces touched, non-finger?)
        self.first_contacts = []  # (t, surface, probe-tip speed mm/s)
        self.peak_force_n = 0.0
        self.peak_tip_speed_mm_s = 0.0
        self._touching = False
        self._tick_force = 0.0
        self._tick_surfaces = set()
        self._tick_nonfinger = False
        self.kp_override = {}
        self._window = []
        self.peak_force_20ms_n = 0.0
        # The real arm is engaged lying on the floor; this one starts at the hub in mid-water, so it
        # is held for 3 s first (as if it had been engaged a moment ago) to take up its sag.
        self.target = dict(start)
        self._held = True
        self.advance(3.0)
        self.first_contacts.clear()
        self.peak_force_n = self.peak_force_20ms_n = 0.0

    # motor <-> scene joint coordinates (scene = repo URDF map)
    def _urdf(self, m, motor_deg):
        e = self.scene_map[m]
        return math.radians(e["sign"] * motor_deg + e["offset_deg"])

    def _motor(self, m, q_rad):
        e = self.scene_map[m]
        return (math.degrees(q_rad) - e["offset_deg"]) / e["sign"]

    def connect(self):
        self.events.append("connect (MuJoCo: no port opened)")

    def advance(self, dt):
        steps = max(1, round(dt / PHYSICS_DT))
        p = self.physics
        for _ in range(steps):
            for m in ARM:
                q = self.data.qpos[self.qadr[m]]
                qd = self.data.qvel[self.vadr[m]]
                link_motor = self._motor(m, q)
                sign = self.scene_map[m]["sign"]
                if (self.energized or self._held) and m not in self.dead:
                    kp = self.kp_override.get(m, self.kp[m])
                    flex = p.flex_nm_per_rad.get(m, math.inf)
                    prev = self.tau[m]
                    play = math.radians(p.backlash_deg) * math.tanh(prev / 0.05)
                    err = math.radians(self.target[m] - link_motor) - play
                    tau = (kp * err - self.kd[m] * sign * qd) / (1 + kp / flex)
                    tau = max(-TORQUE_PEAK_NM, min(TORQUE_PEAK_NM, tau))
                else:
                    tau, play, flex = 0.0, 0.0, p.flex_nm_per_rad.get(m, math.inf)
                self.tau[m] = tau
                self.data.qfrc_applied[self.vadr[m]] = sign * tau
                self.encoder[m] = (
                    link_motor + math.degrees(tau / flex + play) + p.encoder_bias_deg.get(m, 0.0)
                )
            mujoco.mj_step(self.model, self.data)
            self._contacts()
            self.clock.t += PHYSICS_DT
            if self.recorder is not None:
                self.recorder(self)
        self.pos = {**{m: self.encoder[m] for m in ARM}, "gripper": self.target["gripper"]}

    def _contacts(self):
        total, surfaces, nonfinger = 0.0, set(), False
        force = np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = c.geom1, c.geom2
            if not (
                (g1 in self.arm_geoms and g2 in self.env_geoms)
                or (g2 in self.arm_geoms and g1 in self.env_geoms)
            ):
                continue
            mujoco.mj_contactForce(self.model, self.data, i, force)
            total += abs(force[0])
            env = g2 if g2 in self.env_geoms else g1
            arm = g1 if env == g2 else g2
            surfaces.add(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, env))
            if self.model.geom_bodyid[arm] not in self.finger_bodies:
                nonfinger = True
        self._window.append(total)
        if len(self._window) > 20:
            self._window.pop(0)
        self.peak_force_20ms_n = max(self.peak_force_20ms_n, sum(self._window) / len(self._window))
        if total > 0 and not self._touching:
            vel = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_SITE, self.site, vel, 0)
            self.first_contacts.append(
                (self.clock.t, sorted(surfaces), float(np.linalg.norm(vel[3:]) * 1000))
            )
        self._touching = total > 0
        self.peak_force_n = max(self.peak_force_n, total)
        self._tick_force = max(self._tick_force, total)
        self._tick_surfaces |= surfaces
        self._tick_nonfinger |= nonfinger

    def tip_mm(self):
        return self.data.site_xpos[self.site] * 1000

    read_hook = None

    def read(self):
        if self.read_hook is not None:
            self.read_hook()
        self.ticks += 1
        if self.fault == "stale" and self.ticks > self.fault_tick:
            raise ConnectionError("No fresh feedback from motor 'elbow_flex' within 50 ms (sim)")
        if self.fault == "limp" and self.ticks > self.fault_tick and self.energized:
            self.limp.add("shoulder_lift")
            self.dead.add("shoulder_lift")
        p = self.physics
        pos = {m: self.encoder[m] + float(self.rng.normal(0, p.position_noise_deg)) for m in ARM}
        pos["gripper"] = self.target["gripper"]
        torque = {m: self.scene_sign_torque(m) + float(self.rng.normal(0, p.torque_noise_nm)) for m in ARM}
        torque["gripper"] = 0.0
        self.contact_peak.append(
            (self.clock.t, self._tick_force, sorted(self._tick_surfaces), self._tick_nonfinger)
        )
        self._tick_force, self._tick_surfaces, self._tick_nonfinger = 0.0, set(), False
        return real.Reading(pos, torque, dict.fromkeys(core.MOTORS, 33.0), {})

    def scene_sign_torque(self, m):
        return 0.0 if not self.energized or m in self.dead else self.tau[m]

    def torque(self):
        return {**{m: self.scene_sign_torque(m) for m in ARM}, "gripper": 0.0}

    def engage(self):
        self.energized = True
        self._held = False
        self.target = {**self.encoder, "gripper": self.target["gripper"]}
        self.events.append("engage: torque on at the measured pose")
        return dict(self.pos)

    def send(self, target, previous):
        self.target = dict(target)

    def reanchor(self, offsets):
        if any(abs(offsets[m]) > real.REANCHOR_MAX_OFFSET_DEG for m in ARM):
            raise ValueError("re-anchor offset too large")
        for m in ARM:
            self.target[m] = self.encoder[m] + offsets[m]
        self.events.append("re-anchor: arm goals at the measured pose")
        return {m: self.encoder[m] for m in ARM}

    def return_to_rest(self, pose, abort, speed, watch=None):
        if "gripper" in pose:
            raise AssertionError("the return must leave the (limp) gripper alone")
        start = {m: self.encoder[m] for m in pose}
        travel = max(abs(start[m] - pose[m]) for m in pose)
        self.returns.append((dict(pose), speed))
        steps = max(1, math.ceil(travel / (speed * core.ROW_DT_S)))
        for i in range(1, steps + 1):
            if abort.is_set():
                return False, "cut-short"
            for m, v in pose.items():
                self.target[m] = start[m] + (v - start[m]) * i / steps
            self.clock.sleep(core.ROW_DT_S)
            if watch is not None and (
                trips := watch(
                    dict(self.pos), self.torque(), dict.fromkeys(core.MOTORS, 33.0), dict(self.target)
                )
            ):
                self.events.append(f"return leg: ended by {trips[0].name} after {i}/{steps} steps")
                return False, f"{trips[0].name}: {trips[0].detail}"
        for _ in range(20):
            self.clock.sleep(core.ROW_DT_S)
        self.events.append(f"return leg: {travel:.1f} deg max travel at {speed:.0f} deg/s")
        motor, off = max(((m, abs(self.encoder[m] - v)) for m, v in pose.items()), key=lambda kv: kv[1])
        if off <= 2.0:
            return True, ""
        if off <= 6.0:
            return True, "settled"
        return False, f"{motor} still {off:.1f} deg away"

    def release(self):
        self.energized = False
        self._held = False
        self.events.append("release: torque off on every motor")
        return None


def _tank():
    import json

    return json.loads(geo.DEFAULT_TANK.read_text())


class Recorder:
    """Offscreen frames every `every_s` of sim time (MUJOCO_GL=egl), for the run video."""

    def __init__(self, every_s=0.4, width=960, height=720):
        self.every, self.width, self.height = every_s, width, height
        self.frames = []
        self.next_t = None
        self.renderer = None
        self.label = ""

    def __call__(self, arm):
        t = arm.clock.t
        if self.next_t is None:
            self.next_t = t
        if t < self.next_t:
            return
        self.next_t = t + self.every
        if self.renderer is None:
            self.renderer = mujoco.Renderer(arm.model, self.height, self.width)
            self.cams = []
            for az, el, dist in ((150, -30, 1.15), (90, -80, 1.1)):
                cam = mujoco.MjvCamera()
                cam.lookat[:] = [-0.28, 0.0, 0.05]
                cam.azimuth, cam.elevation, cam.distance = az, el, dist
                self.cams.append(cam)
        views = []
        for cam in self.cams:
            self.renderer.update_scene(arm.data, camera=cam)
            views.append(self.renderer.render())
        frame = np.concatenate([v[:, 120:-120] for v in views], axis=1)
        self.frames.append((t, frame, self.label, arm._touching))

    def write(self, path: Path, fps=10):
        import imageio.v2 as imageio
        from PIL import Image, ImageDraw

        writer = imageio.get_writer(str(path), fps=fps, codec="libx264", quality=7, macro_block_size=8)
        t0 = self.frames[0][0] if self.frames else 0
        for t, frame, label, touching in self.frames:
            img = Image.fromarray(frame)
            draw = ImageDraw.Draw(img)
            draw.rectangle([0, 0, img.width, 30], fill=(0, 0, 0))
            text = f"t={t - t0:6.1f} s  x{self.every * fps:.0f} speed   {label}" + (
                "   CONTACT" if touching else ""
            )
            draw.text((10, 8), text, fill=(255, 255, 0) if touching else (255, 255, 255))
            writer.append_data(np.asarray(img))
        writer.close()
