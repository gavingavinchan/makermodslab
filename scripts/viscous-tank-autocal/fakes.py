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
"""A numpy-only compliant arm against a true tank: the fast backend for tests and `sim --backend fake`.

It is dream_real.FakeArm (first-order tracking, the standing sag, the stop path's re-anchor and
return, limp/stale faults) plus quasi-static contact: when the commanded pose would put a finger
vertex past a true surface, the measured joints settle where MIT stiffness balances the surface
force,

    q = q_free - K^-1 J^T n F,   F = depth / (n^T J K^-1 J^T n + 1 / k_surface),

which is exactly the compliance argument the press relies on (rigid glass: k_surface = inf). Soft
obstacles (a cable) have a finite k and move with the press; rigid ones (an object on the floor)
do not. Frictionless: a press slides freely along the glass. No encoder flex or backlash here;
the MuJoCo backend has those.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
from context import core, real
from geometry import ARM, FINGERS, Estimate, finger_vertices_mm, geo, point_jacobian, probe_tip_mm

TORQUE_NOISE_NM = 0.15
POSITION_NOISE_DEG = 0.01


@dataclasses.dataclass
class Obstacle:
    """A disc of radius_mm on `face`, standing proud of it by height_mm, centred at a point."""

    face: str
    centre_mm: tuple
    radius_mm: float
    height_mm: float
    stiffness_n_per_m: float = math.inf  # inf: rigid; else it gives way (a cable bundle)
    name: str = "obstacle"


def random_truth(nominal: Estimate, rng, centre_mm=30.0, yaw_deg=3.0, floor_mm=10.0) -> Estimate:
    """A true tank: J1 off the centre line by up to +-centre_mm, yawed by up to +-yaw_deg about the
    post's bearing point on the back glass (the post stays flush), floor +-floor_mm."""
    from geometry import BACK_GLASS_X_MM, POST_CENTRE_Y_MM

    truth = nominal.copy()
    truth.yaw_deg = float(rng.uniform(-yaw_deg, yaw_deg))
    axes = geo.tank_axes(truth.yaw_deg)
    bearing = axes @ np.array([BACK_GLASS_X_MM, POST_CENTRE_Y_MM, 0.0])
    truth.near = float(bearing[0])
    # Nominal midline is (left + right) / 2 in J1's frame; the true one is shifted by `shift`.
    mid = (nominal.left + nominal.right) / 2 + float(rng.uniform(-centre_mm, centre_mm))
    half = nominal.dims["inner_width"] / 2
    truth.left, truth.right = mid + half, mid - half
    truth.floor = nominal.floor + float(rng.uniform(-floor_mm, floor_mm))
    truth.source = "truth"
    return truth


class ContactModel:
    """Finger-vertex contact with the true panes and obstacles, solved quasi-statically."""

    def __init__(self, pm, truth: Estimate, kp: dict, obstacles=()):
        self.pm, self.truth, self.kp = pm, truth, kp
        self.obstacles = list(obstacles)
        self.last = []  # [(surface, force N, point mm)] of the last solve

    def _deepest(self, q):
        verts = finger_vertices_mm(self.pm, geo.forward_kinematics(self.pm, q))
        best = (0.0, None, None, None, math.inf)
        for face in ("floor", "left", "right", "near", "far"):
            n = self.truth.normal(face)
            depth = verts @ n - self._offset(face)
            i = int(np.argmax(depth))
            if depth[i] > best[0]:
                best = (float(depth[i]), face, n, verts[i], math.inf)
        for ob in self.obstacles:
            n = self.truth.normal(ob.face)
            centre = np.asarray(ob.centre_mm, dtype=float)
            radial = verts - centre - np.outer((verts - centre) @ n, n)
            inside = np.linalg.norm(radial, axis=1) <= ob.radius_mm
            if not inside.any():
                continue
            surface = float(centre @ n) - ob.height_mm  # the obstacle's top, short of the pane
            depth = np.where(inside, verts @ n - surface, -np.inf)
            i = int(np.argmax(depth))
            if depth[i] > best[0]:
                best = (float(depth[i]), ob.name, n, verts[i], ob.stiffness_n_per_m)
        return best

    def _offset(self, face):
        """n . x = offset for points on the face (base frame), from the tank-axis plane offsets."""
        o = self.truth.offsets()
        return {
            "floor": -o["floor"],
            "left": o["left"],
            "right": -o["right"],
            "near": o["near"],
            "far": -o["far"],
        }[face]

    def solve(self, q_free: dict) -> dict:
        q = dict(q_free)
        self.last = []
        forces = {}
        for _ in range(6):
            depth, name, n, vertex, k_surface = self._deepest(q)
            if name is None or depth < 1e-4:
                break
            link = FINGERS[0]  # either finger: both are rigid with the wrist
            jac = point_jacobian(self.pm, q, vertex, link)
            kinv = np.diag([1.0 / self.kp[m] for m in ARM])
            c = float(n @ jac @ kinv @ jac.T @ n)
            soft = 0.0 if math.isinf(k_surface) else 1.0 / k_surface
            force = (depth / 1000 - soft * forces.get(name, 0.0)) / (c + soft)
            if force < 1e-6:
                break
            dq = -kinv @ jac.T @ n * force
            for i, m in enumerate(ARM):
                q[m] += math.degrees(dq[i])
            forces[name] = forces.get(name, 0.0) + force
        self.last = sorted(forces.items(), key=lambda kv: -kv[1])
        return q


class PlaneFakeArm(real.FakeArm):
    """FakeArm whose energized joints settle on the contact solution of the commanded pose."""

    def __init__(
        self,
        start,
        clock,
        pm,
        truth,
        kp,
        *,
        sag=None,
        obstacles=(),
        rng=None,
        fault=None,
        fault_tick=10**9,
        torque_noise=TORQUE_NOISE_NM,
    ):
        super().__init__(
            start,
            [],
            clock,
            scenario="nominal",
            sag=sag or {"shoulder_lift": 4.0, "elbow_flex": 2.0},
            heat_per_tick=0.0,
        )
        self.pm = pm
        self.kp = kp
        self.contact = ContactModel(pm, truth, kp, obstacles)
        self.rng = rng or np.random.default_rng(0)
        self.fault, self.fault_tick = fault, fault_tick
        self.torque_noise = torque_noise
        self.home = dict(start)
        self.contact_log = []  # per read: (t, force N, surface, measured tip mm)
        self.first_contacts = []  # (t, surface, tip speed mm/s at first contact)
        self._touching = False
        self._last_tip = None

    def _settle(self, m, goal):
        # Held in water, never supported: the full standing sag, phased in over ~3 s after engage
        # (a real arm engaged lying on the floor takes up its load as it lifts).
        return goal - self.sag[m] * min(1.0, self._held_ticks / 60)

    _held_ticks = 0

    read_hook = None

    def read(self):
        if self.read_hook is not None:
            self.read_hook()
        self.ticks += 1
        if self.fault == "stale" and self.ticks > self.fault_tick:
            raise ConnectionError("No fresh feedback from motor 'elbow_flex' within 50 ms (fake)")
        if self.fault == "limp" and self.ticks > self.fault_tick and self.energized:
            self.limp.add("shoulder_lift")
            self.dead.add("shoulder_lift")
        self._dynamics()
        t = self.clock()
        tip = probe_tip_mm(self.pm, self.pos)
        force = sum(f for _, f in self.contact.last)
        surface = self.contact.last[0][0] if self.contact.last else None
        if force > 0 and not self._touching and self._last_tip is not None:
            speed = float(np.linalg.norm(tip - self._last_tip[1]) / max(t - self._last_tip[0], 1e-6))
            self.first_contacts.append((t, surface, speed))
        self._touching = force > 0
        self._last_tip = (t, tip)
        self.contact_log.append((t, force, surface))
        noisy = {m: v + float(self.rng.normal(0, POSITION_NOISE_DEG)) for m, v in self.pos.items()}
        temp = dict.fromkeys(core.MOTORS, 31.0)
        return real.Reading(noisy, self.torque(), temp, {})

    def _dynamics(self):
        if self.energized:
            self._held_ticks += 1
            free = {m: self._settle(m, self.target[m]) for m in core.MOTORS}
            if self.gripper_limp:
                free["gripper"] = self.pos["gripper"]
            eq = self.contact.solve(free)
            for m in core.MOTORS:
                if m in self.limp:
                    continue
                self.pos[m] += 0.6 * (eq[m] - self.pos[m])
            if "shoulder_lift" in self.limp:
                self.pos["shoulder_lift"] -= self.FALL_DEG_PER_TICK

    def reanchor(self, offsets):
        if self.fault == "stale" and self.ticks > self.fault_tick:
            raise ConnectionError("cannot re-anchor without a fresh reading")
        return super().reanchor(offsets)

    def return_to_rest(self, pose, abort, speed, watch=None):
        """The stop path's leg: the goal ramped at `speed` from the measured pose, the arm following
        through the same dynamics (and contacts) as every other tick."""
        if self.fault == "stale" and self.ticks > self.fault_tick:
            return False, "no fresh feedback for the return"
        start = {m: self.pos[m] for m in pose}
        travel = max(abs(start[m] - pose[m]) for m in pose)
        self.returns.append((dict(pose), speed))
        steps = max(1, math.ceil(travel / (speed * core.ROW_DT_S)))
        for i in range(1, steps + 21):
            if self.read_hook:
                self.read_hook()
            if abort.is_set():
                return False, "cut-short"
            f = min(1.0, i / steps)
            for m, v in pose.items():
                self.target[m] = start[m] + (v - start[m]) * f
            self.clock.sleep(core.ROW_DT_S)
            self._dynamics()
            if watch is not None and (
                trips := watch(
                    dict(self.pos), self.torque(), dict.fromkeys(core.MOTORS, 31.0), dict(self.target)
                )
            ):
                return False, f"{trips[0].name}: {trips[0].detail}"
        motor, off = max(((m, abs(self.pos[m] - v)) for m, v in pose.items()), key=lambda kv: kv[1])
        if off <= 2.0:
            return True, ""
        if off <= 6.0:
            return True, "settled"
        return False, f"{motor} still {off:.1f} deg away"

    def torque(self) -> dict:
        out = {}
        for m in core.MOTORS:
            if not self.energized or m in self.dead or (m == "gripper" and self.gripper_limp):
                out[m] = 0.0
                continue
            kp = self.kp.get(m, 20.0)
            out[m] = kp * math.radians(self.target[m] - self.pos[m]) + float(
                self.rng.normal(0, self.torque_noise)
            )
        return out
