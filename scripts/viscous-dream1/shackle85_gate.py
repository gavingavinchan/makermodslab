# Copyright 2026 MakerMods. All rights reserved.
"""Pre-print acceptance gate for the 85-degree shackle layout: the whole left arm, end to end, kinematic.

Offline only: the shackle scene is loaded for the arm model, cameras and kinematics; no server, network, billed
calls, hardware or printing. Fixtures are the holder session's committed `tilt_85deg` exports (0de8facd), placed by
their `layout_*_plane_frame.obj` (already in plane metres; only world = (0.305 + W, 0.240 - S, 0.760 + H) applied).

What it does: plans the task as a sequence of IK waypoints and Cartesian/joint-space segments (cup pickup at roll 0,
axial extraction toward J1, transfer to the line, shank push until the fingertips are 3 mm off the clear ear,
release, head regrip, push to the tapped ear, 90-degree tightening strokes with partial-opening resets, seated,
retract, home), samples every segment finely, and at every sample checks:

- water: the highest point of every left-arm mesh (and of the held pin) against the 283 mm line;
- wall: nothing that moves reaches behind the base's back edge, and the elbow (J3) stays ahead of J1;
- exact triangle-mesh distances (python-fcl BVH, non-convex) from every moving arm mesh, fingers included, to the
  holder, the tie/locking-head envelopes, the cup, the shackle and the floor; from the held pin to the fixtures;
  from open fingers to the pin;
- arm self-collision (MuJoCo's own arm geoms) and joint-limit margins; IK error and joint jumps between samples.

What it does not do: dynamics, grasp forces, thread friction, torque (Gavin 2026-11-06: not gated), or water drag.
The pin is moved kinematically (held = rigid to the hand; released = fixed; screwing = rotation about its axis with
2.5 mm pitch advance). It is a geometric/kinematic gate, not a physics success claim.

    MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python shackle85_gate.py            # the manifest's placement
    GATE_CUP_R_MM=540 GATE_BORE_S_MM=625 MUJOCO_GL=egl ... shackle85_gate.py  # another placement (no reprint)
    ~/dreamtrial/.venv/bin/python shackle85_gate.py --summarise              # table over every run folder
"""

import json
import os
import sys
from pathlib import Path

import fcl
import mujoco
import numpy as np
import trimesh
from scipy.optimize import least_squares

import shackle_scene

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets/shackle"
FIX = HERE / "runs/shackle_tied_holder/tilt_85deg"
OUT_ROOT = Path(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else HERE / "runs/shackle85_gate"
MANIFEST = json.loads((FIX / "layout_manifest.json").read_text())
TABLE = shackle_scene.TABLE

# --- geometry from the committed layout and the shackle model --------------------------------------------------
TILT = np.radians(MANIFEST["cup_axis_tilt_from_vertical_deg"])  # 85 deg; the shackle uses the same tilt
C, S = np.cos(TILT), np.sin(TILT)
PLANE = np.array(MANIFEST["plane_metres_to_sim_world_metres"])
CUP_AZ = np.radians(MANIFEST["cup_azimuth_deg"])


def plane_to_world(p):
    p = np.atleast_2d(p)
    return p @ PLANE[:3, :3].T + PLANE[:3, 3]


# Shackle body frame in world: Z = pin axis toward the head (toward J1 = world +Y, and up), Y = across (world +X),
# X = Y x Z; the bow runs along body -X, i.e. toward J1 and down (orientation session, 2026-11-07).
R_SHACKLE = np.column_stack([[0, -C, S], [1, 0, 0], [0, S, C]])
# GATE_BORE_S_MM moves the shackle, holder and ties together along the line (placement tolerance); default: the
# manifest's bore (600 since 348fefac).
BORE_S_LAYOUT = MANIFEST["shackle_bore_midpoint_mm"][0]
BORE_S = float(os.environ.get("GATE_BORE_S_MM", BORE_S_LAYOUT))
SHACKLE_SHIFT = np.array([0.0, -(BORE_S - BORE_S_LAYOUT) / 1000, 0.0])  # plane S runs along world -Y
BORE = plane_to_world(np.array(MANIFEST["shackle_bore_midpoint_mm"]) / 1000)[0] + SHACKLE_SHIFT
TOP = 0.039925  # clear ear's outer face (the collar seats here), body z
MOUTH = -0.020574  # tapped ear's mouth, body z
SEAT = -0.039925  # pin tip when seated, body z
PITCH = 0.0025
# Pin frame: origin at the threaded tip, +Z toward the head. Collision-relevant radii from shackle_scene.py.
SHANK_R, COLLAR_TOP, PIN_LEN = 0.009845, 0.08385, 0.1079
TAB_HALF = (0.012, 0.004595)  # head tab half-extents along pin x and pin y
FINGERTIP = 0.0343  # closed fingertips ahead of the pad TCP (left_tool0), measured on the model
APPROACH_LOCAL = np.array([0.986991, 0.0, -0.160775])  # viscous_arm.json, wrist_roll_link frame
JAW_LOCAL = np.array([0.0, 1.0, 0.0])

# Grips, as (TCP distance from the pin tip along its axis, jaw direction in the pin frame, holding opening per side).
CUP_GRIP = 0.068  # README: grip/TCP 68 mm from the tip; fingertips 33.7 mm from the tip, 6.7 mm beyond the cup lip
# Head grip: the fingers must stay above the pin's own collar (r 13.8 mm) because, closed on the tab, they sit inside
# that radius. So the fingertips end 2 mm above the collar and the distal ~22 mm of the flat finger faces hold the tab.
HEAD_GRIP = COLLAR_TOP + FINGERTIP + 0.002
NOMINAL_HEAD_GRIP = 0.057 - SEAT  # the orientation session's screening grip: middle of the head (for the report)
OPEN_RESET = 0.018  # per side, for releases/resets near the shackle (holder audit cleared <= 26 mm)
OPEN_CUP = 0.026  # per side, approaching the cup (audit: any opening at roll 0)
FLATS, WIDTH = np.array([0, 1.0, 0]), np.array([1.0, 0, 0])
CLEAR_PAD = 0.0005  # the holder audit's padding


def jsonable(o):
    return o.tolist() if hasattr(o, "tolist") else str(o)


def load_obj(path):
    m = trimesh.load(path, force="mesh", process=False)
    return np.asarray(m.vertices, float), np.asarray(m.faces, np.int32)


def bvh(v, f):
    model = fcl.BVHModel()
    model.beginModel(len(v), len(f))
    model.addSubModel(v, f)
    model.endModel()
    return model


def rot(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


# --- the cup and the pin's pose in it ---------------------------------------------------------------------------
RADIAL_CUP = np.array([np.sin(CUP_AZ), -np.cos(CUP_AZ), 0.0])  # away from J1 along the cup's line, world
# GATE_CUP_R_MM moves the whole cup along its own radial line (same azimuth, height and tilt): a placement change, not
# a reprint. Default: the manifest's cup radius (565 since 348fefac).
CUP_R_LAYOUT = float(np.hypot(*MANIFEST["cup_pin_tip_mm"][:2]))
CUP_R = float(os.environ.get("GATE_CUP_R_MM", CUP_R_LAYOUT))
CUP_SHIFT = RADIAL_CUP * (CUP_R - CUP_R_LAYOUT) / 1000
CUP_TIP = plane_to_world(np.array(MANIFEST["cup_pin_tip_mm"]) / 1000)[0] + CUP_SHIFT
# Every run lands in a folder named by the placement it actually used, defaults included.
OUT = OUT_ROOT / f"cup_r{CUP_R:.0f}_bore_s{BORE_S:.0f}"
CUP_EXTRACT = 0.040  # axial pull toward J1: 27 mm tip-to-mouth plus 13 mm before any sideways motion
CUP_ABOVE = 0.040  # pre-grasp: open hand this far straight above the shank grip, then straight down
ACROSS_CUP = np.array([np.cos(CUP_AZ), np.sin(CUP_AZ), 0.0])
ACROSS_LINE = np.array([1.0, 0, 0])
AXIS_CUP = -S * RADIAL_CUP + C * np.array([0, 0, 1.0])  # head toward J1 and slightly up
# Pin roll in the cup is free (a round bore), so the setup places the tab's flats facing the jaws at roll 0:
# pin y along the cup's across direction. Then the head regrip at roll 0 closes on the flats.
R_PIN_CUP = np.column_stack([np.cross(ACROSS_CUP, AXIS_CUP), ACROSS_CUP, AXIS_CUP])


class Gate:
    def __init__(self):
        self.patch_scene()
        from shackle_env import ShackleEnv

        self.env = ShackleEnv()
        m, d = self.m, self.d = self.env.model, self.env.data
        self.qadr = np.array(self.env.qadr["left"])
        self.fadr = [m.joint(f"left_finger_{k}").qposadr[0] for k in (-1, 1)]
        self.pin_qadr = m.joint("pin_free").qposadr[0]
        mujoco.mj_forward(m, d)
        self.base = d.body("left_base_link").xpos.copy()
        self.lo = np.array([m.jnt_range[m.joint(f"left_joint{i}").id][0] for i in range(1, 7)])
        self.hi = np.array([m.jnt_range[m.joint(f"left_joint{i}").id][1] for i in range(1, 7)])
        self.lo[4], self.hi[4] = -1e-4, 1e-4  # J5 = 0: the hand stays in the arm's vertical plane (orientation session)
        self.geoms = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("left_")
                      and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] == 2]
        self.moving = [g for g in self.geoms if m.body(m.geom_bodyid[g]).name != "left_base_link"]
        self.fingers = {g for g in self.geoms if "gripper_left" in m.body(m.geom_bodyid[g]).name
                        or "gripper_right" in m.body(m.geom_bodyid[g]).name}
        self.verts, self.arm_obj = {}, {}
        for g in self.geoms:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(float)
            f = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]].astype(np.int32)
            self.verts[g] = v
            self.arm_obj[g] = fcl.CollisionObject(bvh(v, f), fcl.Transform())
        self.arm_contact = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("left_")
                            and (m.geom_contype[g] or m.geom_conaffinity[g])}
        # The wall: the base's own back edge (orientation session: 40 mm behind J1)
        mujoco.mj_kinematics(m, d)
        base_geoms = [g for g in self.geoms if g not in self.moving]
        self.wall_y = max(self.world(g)[:, 1].max() for g in base_geoms)
        j2 = d.body("left_upper_arm_link").xpos
        self.water = j2[2] + np.linalg.norm(d.body("left_forearm_link").xpos - j2)  # J3 with J2-J3 vertical
        self.events = []
        self.fixtures = self.load_fixtures()
        pv, pf = load_obj(ASSETS / "shackle_pin.obj")
        self.pin_v = pv
        self.pin_hull = trimesh.Trimesh(pv, pf).convex_hull.vertices
        self.pin_obj = fcl.CollisionObject(bvh(pv, pf), fcl.Transform())
        self.samples, self.q = [], None
        self.pin_R, self.pin_p = R_PIN_CUP.copy(), CUP_TIP.copy()
        self.held, self.grip, self.opening, self.stage = None, None, OPEN_CUP, "start"
        self.in_cup, self.engaged, self.closing = True, False, False

    # -- scene: move the shackle, drop the legacy glued holder, add the committed fixtures (visual only) ----------
    def patch_scene(self):
        OUT.mkdir(parents=True, exist_ok=True)
        import tempfile

        tmp = Path(tempfile.mkdtemp(prefix="shackle85_gate_"))  # placed copies for MuJoCo's renderer; not evidence
        meshes = {}
        for name, colour in (("holder", ".12 .29 .39 1"), ("ties_and_heads", ".95 .55 .15 1"),
                             ("cup_pedestal", ".20 .45 .30 1")):
            v, f = load_obj(FIX / f"layout_{name}_plane_frame.obj")
            path = tmp / f"world_{name}.obj"
            shift = CUP_SHIFT if name == "cup_pedestal" else SHACKLE_SHIFT
            trimesh.Trimesh(plane_to_world(v) + shift, f, process=False).export(path)
            meshes[name] = (path, colour)
        original = shackle_scene.add_scene
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R_SHACKLE.ravel())
        import xml.etree.ElementTree as ET
        from flange_scene import numbers

        def add_scene(root):
            root = original(root)
            w, a = root.find("worldbody"), root.find("asset")
            w.remove(w.find("body[@name='shackle_holder']"))
            body = w.find("body[@name='shackle']")
            body.set("pos", numbers(BORE))
            body.set("quat", numbers(quat))
            for name, (path, colour) in meshes.items():
                ET.SubElement(a, "mesh", name=f"fix_{name}", file=str(path))
                ET.SubElement(w, "geom", name=f"fix_{name}", type="mesh", mesh=f"fix_{name}", rgba=colour,
                              contype="0", conaffinity="0", group="2")
            return root

        shackle_scene.add_scene = add_scene

    def load_fixtures(self):
        fx = {}
        for name in ("holder", "ties_and_heads", "cup_pedestal"):
            v, f = load_obj(FIX / f"layout_{name}_plane_frame.obj")
            v = plane_to_world(v) + (CUP_SHIFT if name == "cup_pedestal" else SHACKLE_SHIFT)
            fx[name] = (fcl.CollisionObject(bvh(v, f), fcl.Transform()), v.min(0), v.max(0), v)
        v, f = load_obj(ASSETS / "shackle_body.obj")
        v = v @ R_SHACKLE.T + BORE
        fx["shackle"] = (fcl.CollisionObject(bvh(v, f), fcl.Transform()), v.min(0), v.max(0), v)
        floor = fcl.CollisionObject(fcl.Box(4, 4, 0.2), fcl.Transform(np.array([0.0, 0, TABLE - 0.1])))
        fx["floor"] = (floor, np.array([-2, -2, TABLE - 0.2]), np.array([2, 2, TABLE]), None)
        self.layout_check(fx)
        names = ["holder", "ties_and_heads", "cup_pedestal", "shackle"]
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                dd = fcl.distance(fx[a][0], fx[b][0], fcl.DistanceRequest(), fcl.DistanceResult())
                self.events.append(dict(check=f"fixture clearance {a}~{b}", mm=round(dd * 1000, 2)))
        return fx

    def layout_check(self, fx):
        """Our shackle pose must reproduce the holder session's placed holder, and the pin must sit in the cup."""
        bv, _ = load_obj(FIX / "shackle_tied_holder_body_frame.obj")
        lv, _ = load_obj(FIX / "layout_holder_plane_frame.obj")
        ours = bv @ R_SHACKLE.T + BORE
        theirs = plane_to_world(lv) + SHACKLE_SHIFT
        same = len(ours) == len(theirs)
        self.events.append(dict(check="holder pose matches layout_holder_plane_frame.obj",
                                max_vertex_error_mm=float(np.abs(ours - theirs).max() * 1000) if same else None,
                                note=None if same else "vertex counts differ; compared bounds",
                                bounds_error_mm=float(np.abs(np.r_[ours.min(0) - theirs.min(0),
                                                                   ours.max(0) - theirs.max(0)]).max() * 1000)))
        sv, _ = load_obj(FIX / "pin_seated_in_cup_floor_frame.obj")
        Rc = np.array(MANIFEST["cup_floor_rotation"])
        tc = np.array(MANIFEST["cup_floor_translation_mm"]) / 1000
        sw = plane_to_world(sv @ Rc.T + tc) + CUP_SHIFT
        along = (sw - CUP_TIP) @ AXIS_CUP
        radial = np.linalg.norm((sw - CUP_TIP) - np.outer(along, AXIS_CUP), axis=1)
        self.events.append(dict(check="pin_seated_in_cup_floor_frame.obj vs our cup tip and axis",
                                along_axis_mm=[float(along.min() * 1000), float(along.max() * 1000)],
                                max_radius_mm=float(radial.max() * 1000)))

    # -- kinematics ----------------------------------------------------------------------------------------------
    def world(self, g):
        return self.verts[g] @ self.d.geom_xmat[g].reshape(3, 3).T + self.d.geom_xpos[g]

    def set(self, q, opening):
        d = self.d
        d.qpos[self.qadr] = q
        for a in self.fadr:
            d.qpos[a] = np.clip(opening, 0, 0.0524125)
        mujoco.mj_kinematics(self.m, d)

    def hand(self):
        d = self.d
        R = d.body("left_wrist_roll_link").xmat.reshape(3, 3)
        return d.body("left_tool0").xpos.copy(), R @ APPROACH_LOCAL, R @ JAW_LOCAL

    def hand_target(self, grip):
        """TCP position, approach and jaw for a grip (distance from tip, jaw in pin frame) on the pin as it is now."""
        g, jaw = grip
        return self.pin_p + self.pin_R @ np.array([0, 0, g]), -self.pin_R[:, 2], self.pin_R @ jaw

    def ik(self, target, approach, jaw, seeds, opening):
        best = None
        for seed in seeds:
            def res(q):
                self.set(q, opening)
                p, a, j = self.hand()
                return np.r_[(p - target) * 1000, (a - approach) * 50, (j - jaw) * 20]

            r = least_squares(res, np.clip(seed, self.lo + 1e-6, self.hi - 1e-6), bounds=(self.lo, self.hi),
                              xtol=1e-10, ftol=1e-10)
            self.set(r.x, opening)
            p, a, j = self.hand()
            err = (np.linalg.norm(p - target) * 1000, np.degrees(np.arccos(np.clip(a @ approach, -1, 1))),
                   np.degrees(np.arccos(np.clip(j @ jaw, -1, 1))))
            if err[0] < 1.0 and err[1] < 1.0 and err[2] < 2.0:
                ok = self.static_ok()
                key = (not ok, self.top())
                if best is None or key < best[0]:
                    best = (key, r.x.copy(), err)
                if ok and len(seeds) == 1:
                    break
        if best is None:
            return None, None
        self.set(best[1], opening)
        return best[1], best[2]

    def top(self):
        return max(self.world(g)[:, 2].max() for g in self.geoms)

    def static_ok(self):
        """Wall and elbow rules plus arm self-collision, for choosing among IK branches."""
        back = max(self.world(g)[:, 1].max() for g in self.moving)
        if back > self.wall_y + 0.0005 or self.d.body("left_forearm_link").xpos[1] > self.base[1]:
            return False
        return not self.self_collision()

    def self_collision(self):
        m, d = self.m, self.d
        mujoco.mj_collision(m, d)
        for c in d.contact[:d.ncon]:
            if c.dist < -0.0005 and c.geom1 in self.arm_contact and c.geom2 in self.arm_contact:
                return True
        return False

    def grid(self, j1):
        return [np.r_[j1, q2, q3, q4, 0, q6] for q2 in (-2.4, -1.6, -0.8) for q3 in (0.8, 2.0, 3.2)
                for q4 in (-1.2, 0.0, 1.0) for q6 in (-1.5, 0.0, 1.5)]

    # -- sampling and checks -------------------------------------------------------------------------------------
    def pin_world(self):
        return self.pin_v @ self.pin_R.T + self.pin_p

    def record(self):
        m, d = self.m, self.d
        tops = {g: self.world(g) for g in self.geoms}
        top_g = max(self.geoms, key=lambda g: tops[g][:, 2].max())
        arm_top = tops[top_g][:, 2].max()
        pin_hull = self.pin_hull @ self.pin_R.T + self.pin_p
        back = max(tops[g][:, 1].max() for g in self.moving)
        j3_ahead = self.base[1] - d.body("left_forearm_link").xpos[1]
        dist, pairs = {}, []
        for g in self.moving:
            v = tops[g]
            lo, hi = v.min(0), v.max(0)
            self.arm_obj[g].setTransform(fcl.Transform(d.geom_xmat[g].reshape(3, 3).copy(), d.geom_xpos[g].copy()))
            part = "finger" if g in self.fingers else m.body(m.geom_bodyid[g]).name.replace("left_", "")
            for name, (obj, flo, fhi, _) in self.fixtures.items():
                gap = np.max(np.r_[flo - hi, lo - fhi])
                if gap > 0.02:
                    continue
                r = fcl.DistanceResult()
                dd = fcl.distance(self.arm_obj[g], obj, fcl.DistanceRequest(enable_nearest_points=True), r)
                if dd <= 0:
                    cr = fcl.CollisionResult()
                    fcl.collide(self.arm_obj[g], obj, fcl.CollisionRequest(num_max_contacts=4, enable_contact=True),
                                cr)
                    dd = -max([c.penetration_depth for c in cr.contacts] or [0.0])
                key = f"{part}~{name}"
                if key not in dist or dd < dist[key][0]:
                    dist[key] = (dd, (r.nearest_points[0] if dd > 0 else (cr.contacts[0].pos if cr.contacts else None)))
        self.pin_obj.setTransform(fcl.Transform(self.pin_R.copy(), self.pin_p.copy()))
        plo, phi = pin_hull.min(0), pin_hull.max(0)
        for name, (obj, flo, fhi, _) in self.fixtures.items():
            if name == "cup_pedestal" and self.in_cup:
                name_key = "pin~cup (seated in its bore)"
            elif name == "shackle" and self.engaged:
                continue  # threaded contact in the tapped ear is by design
            else:
                name_key = f"pin~{name}"
            if np.max(np.r_[flo - phi, plo - fhi]) > 0.02:
                continue
            r = fcl.DistanceResult()
            dd = fcl.distance(self.pin_obj, obj, fcl.DistanceRequest(enable_nearest_points=True), r)
            if dd <= 0:
                cr = fcl.CollisionResult()
                fcl.collide(self.pin_obj, obj, fcl.CollisionRequest(num_max_contacts=4, enable_contact=True), cr)
                dd = -max([c.penetration_depth for c in cr.contacts] or [0.0])
            dist[name_key] = (dd, r.nearest_points[0] if dd > 0 else None)
        if self.held is None and not self.closing:  # open fingers around a pin they are not holding
            for g in self.fingers:
                r = fcl.DistanceResult()
                dd = fcl.distance(self.arm_obj[g], self.pin_obj, fcl.DistanceRequest(), r)
                if "finger~pin (not held)" not in dist or dd < dist["finger~pin (not held)"][0]:
                    dist["finger~pin (not held)"] = (dd, None)
        q = d.qpos[self.qadr].copy()
        limit = np.minimum(q - self.lo, self.hi - q)
        limit[4] = np.inf
        p, a, j = self.hand()
        across = ACROSS_CUP if self.stage.startswith("cup") else ACROSS_LINE
        roll = np.degrees(np.arctan2(j @ np.cross(a, across), j @ across))
        self.samples.append(dict(
            stage=self.stage, q=np.round(q, 5).tolist(), opening_per_side_mm=round(self.opening * 1000, 2),
            held=self.held, roll_deg=round(float(roll), 1),
            arm_top_mm=round((arm_top - TABLE) * 1000, 2), arm_top_part=m.body(m.geom_bodyid[top_g]).name,
            pin_top_mm=round((pin_hull[:, 2].max() - TABLE) * 1000, 2),
            back_beyond_wall_mm=round((back - self.wall_y) * 1000, 2), j3_ahead_of_j1_mm=round(j3_ahead * 1000, 2),
            self_collision=self.self_collision(), min_limit_margin_deg=round(float(np.degrees(limit.min())), 2),
            fingertip_body_z_mm=round(float(((p + FINGERTIP * a) - BORE) @ R_SHACKLE[:, 2] * 1000), 2),
            pin_tip_body_z_mm=round(float((self.pin_p - BORE) @ R_SHACKLE[:, 2] * 1000), 2),
            dist_mm={k: (round(v[0] * 1000, 2), None if v[1] is None else np.round((np.asarray(v[1]) - [0.305, 0.240, TABLE])
                                                                                  * [1000, -1000, 1000], 1).tolist())
                     for k, v in dist.items()}))

    # -- motions -------------------------------------------------------------------------------------------------
    def move_pin_with_hand(self):
        if self.held is not None:
            R = self.d.body("left_wrist_roll_link").xmat.reshape(3, 3)
            p = self.d.body("left_tool0").xpos
            self.pin_R = R @ self.held[0]
            self.pin_p = p + R @ self.held[1]

    def attach(self):
        R = self.d.body("left_wrist_roll_link").xmat.reshape(3, 3)
        p = self.d.body("left_tool0").xpos
        self.held = (R.T @ self.pin_R, R.T @ (self.pin_p - p))

    def joint_move(self, q_goal, opening=None, step_deg=1.5):
        opening = self.opening if opening is None else opening
        q0 = self.q.copy()
        n = max(2, int(np.ceil(np.degrees(np.abs(q_goal - q0).max()) / step_deg)))
        o0 = self.opening
        for u in np.linspace(0, 1, n + 1)[1:]:
            s = u * u * (3 - 2 * u)
            self.q = q0 + (q_goal - q0) * s
            self.opening = o0 + (opening - o0) * s
            self.set(self.q, self.opening)
            self.move_pin_with_hand()
            self.record()

    def solve_grip(self, grip, opening, j1_hint, extra_seeds=()):
        target, approach, jaw = self.hand_target(grip)
        q, err = self.ik(target, approach, jaw, [self.q, *extra_seeds, *self.grid(j1_hint)], opening)
        if q is None:
            raise RuntimeError(f"{self.stage}: no IK solution for {np.round((target - [0.305, 0.240, TABLE]) * 1000, 1)}")
        return q

    def cart(self, pin_motion, n, opening=None, grip=None, offset=None):
        """Move along a path of pin poses (held: the hand follows; free: the hand follows `grip` on the fixed pin)."""
        opening = self.opening if opening is None else opening
        o0 = self.opening
        for i in range(1, n + 1):
            u = i / n
            if pin_motion is not None:
                self.pin_R, self.pin_p = pin_motion(u)
            target, approach, jaw = self.hand_target(grip(u) if callable(grip) else grip)
            if offset is not None:
                target = target + offset(u)
            q, err = self.ik(target, approach, jaw, [self.q], o0 + (opening - o0) * u)
            if q is None or np.degrees(np.abs(q - self.q).max()) > 10:
                q2, err2 = self.ik(target, approach, jaw, [self.q, *self.grid(self.q[0])], o0 + (opening - o0) * u)
                if q2 is None:
                    raise RuntimeError(f"{self.stage}: IK lost at u={u:.2f}")
                if q is None or np.abs(q2 - self.q).max() < np.abs(q - self.q).max():
                    q = q2
                if np.degrees(np.abs(q - self.q).max()) > 10:
                    self.events.append(dict(stage=self.stage, issue="joint jump", deg=float(np.degrees(np.abs(q - self.q).max()))))
            self.q, self.opening = q, o0 + (opening - o0) * u
            self.set(self.q, self.opening)
            if self.held is not None and pin_motion is None:
                self.move_pin_with_hand()
            self.record()

    def hand_line(self, target, approach, jaw, n):
        """Straight-line TCP path with the hand's frame slerped, IK at every step (continuity from the last one)."""
        from scipy.spatial.transform import Rotation, Slerp

        p0, a0, j0 = self.hand()
        frame = lambda a, j: np.column_stack([a, j, np.cross(a, j)])
        slerp = Slerp([0, 1], Rotation.from_matrix(np.stack([frame(a0, j0), frame(approach, jaw)])))
        for u in np.linspace(0, 1, n + 1)[1:]:
            F = slerp(u).as_matrix()
            q, err = self.ik(p0 + (target - p0) * u, F[:, 0], F[:, 1], [self.q], self.opening)
            if q is None or np.degrees(np.abs(q - self.q).max()) > 10:
                raise RuntimeError(f"{self.stage}: straight-line IK lost at u={u:.2f}")
            self.q = q
            self.set(q, self.opening)
            self.move_pin_with_hand()
            self.record()

    def gripper(self, opening, n=6, closing=False):
        self.closing = closing  # closing on or opening off the pin: finger-pin contact is intended
        o0 = self.opening
        for u in np.linspace(0, 1, n + 1)[1:]:
            self.opening = o0 + (opening - o0) * u
            self.set(self.q, self.opening)
            self.record()
        self.closing = False

    def axial(self, start_p, along, distance):
        R = self.pin_R.copy()
        axis = R[:, 2]
        return lambda u: (R, start_p + axis * along * distance * u)

    # -- the task ------------------------------------------------------------------------------------------------
    def start_pose(self):
        """A low start between the cup and the line: lowest arm top among candidates the wrist camera can use."""
        best = None
        # The water line is J3's height with the upper arm vertical, so a folded pose near the base puts the elbow
        # at the line: a start under water has to reach forward.
        for s, h, down, az_deg in ((s, h, down, az) for az in (-7.5, -20.0, -35.0) for s in (0.30, 0.34, 0.38, 0.42, 0.46)
                                   for h in (0.08, 0.12, 0.16, 0.20) for down in (0, 15, 30, 45)):
            if True:
                if True:
                    az = np.radians(az_deg)  # on the far side of the line from the cup, so the hand starts clear
                    fwd = np.array([np.sin(az), -np.cos(az), 0])
                    target = self.base + fwd * s
                    target[2] = TABLE + h
                    a = np.cos(np.radians(down)) * fwd - np.sin(np.radians(down)) * np.array([0, 0, 1.0])
                    jaw = np.array([np.cos(az), np.sin(az), 0])
                    q, err = self.ik(target, a, jaw, self.grid(-az), OPEN_CUP)
                    if q is None or not self.static_ok():
                        continue
                    top = self.top()
                    if top > self.water - 0.015:  # the start pose must itself be 15 mm under the water line
                        continue
                    self.record()
                    clear = min(v[0] for k, v in self.samples.pop()["dist_mm"].items() if not k.startswith("pin~"))
                    if clear < 20 or np.degrees(min(q[3] - self.lo[3], self.hi[3] - q[3])) < 10:
                        continue  # 20 mm from every fixture and the pin, and 10 deg of wrist-pitch margin
                    vis = self.visibility()
                    seen = min(vis["left"]["cup_pedestal"], vis["left"]["shackle"])
                    score = (seen < 0.9, -seen, top)
                    if best is None or score < best[0]:
                        best = (score, q, dict(tcp_s_mm=s * 1000, tcp_h_mm=h * 1000, pitch_down_deg=down,
                                               arm_top_mm=round((top - TABLE) * 1000, 1), visibility=vis))
        if best is None:
            raise RuntimeError("no start pose 15 mm under the water line")
        self.start_info = best[2]
        return best[1]

    def visibility(self):
        """Fraction of each fixture's vertices inside each camera's image, and their pixel bounding box."""
        m, d = self.m, self.d
        mujoco.mj_forward(m, d)  # camera poses come from mj_camlight, which mj_kinematics does not run
        out = {}
        for cam, (w, h, crop) in (("left", (640, 360, (0, 360))), ("top", (640, 480, (60, 420)))):
            cid = m.camera(cam).id
            pos, R = d.cam_xpos[cid], d.cam_xmat[cid].reshape(3, 3)
            f = (h / 2) / np.tan(np.radians(m.cam_fovy[cid]) / 2)
            res = {}
            for name in ("cup_pedestal", "shackle"):
                v = self.fixtures[name][3][::7]
                c = (v - pos) @ R
                front = c[:, 2] < -0.01
                u = w / 2 + f * c[:, 0] / -c[:, 2]
                vv = h / 2 - f * c[:, 1] / -c[:, 2]
                inside = front & (u >= 0) & (u < w) & (vv >= crop[0]) & (vv < crop[1])
                res[name] = round(float(inside.mean()), 3)
                if inside.any():
                    res[name + "_bbox_px"] = [round(float(np.ptp(u[inside])), 1), round(float(np.ptp(vv[inside])), 1)]
            out[cam] = res
        return out

    def run(self):
        self.set(np.zeros(6), OPEN_CUP)
        self.stage = "start pose"
        self.q = self.start_pose()
        self.set(self.q, OPEN_CUP)
        self.home = self.q.copy()
        self.record()
        self.snapshot("01_start")
        try:
            # An axial approach from beyond the pin's head would start ~75 mm back toward J1, past J4's limit, so the
            # open hand comes straight down onto the shank: the fingers straddle it sideways (roll 0, jaws
            # horizontal) and the palm stays 13.5 mm beyond the pin's head end.
            self.stage = f"cup: pre-grasp (open hand {CUP_ABOVE * 1000:.0f} mm straight above the shank grip, roll 0)"
            up = np.array([0, 0, CUP_ABOVE])
            target, approach, jaw = self.hand_target((CUP_GRIP, FLATS))
            self.hand_line(target + up, approach, jaw, 60)
            self.stage = "cup: descend onto the shank"
            g0 = (CUP_GRIP, FLATS)
            self.cart(None, 20, grip=lambda u: g0, offset=lambda u: up * (1 - u))
            self.stage = "cup: close on the shank"
            self.gripper(SHANK_R, closing=True)
            self.attach()
            self.snapshot("02_cup_grasp")
            self.stage = "cup: extract along the axis toward J1"
            self.cart(self.axial(self.pin_p.copy(), 1, CUP_EXTRACT), 30, grip=(CUP_GRIP, FLATS))
            self.in_cup = False
            self.stage = "transfer: to the pre-insert pose (tip 30 mm before the clear ear)"
            R_line = R_SHACKLE.copy()
            pre_p = BORE + R_SHACKLE[:, 2] * (TOP + 0.030)
            saved = (self.pin_R, self.pin_p)
            self.pin_R, self.pin_p = R_line, pre_p
            q = self.solve_grip((CUP_GRIP, FLATS), SHANK_R, 0.0)
            self.pin_R, self.pin_p = saved
            self.joint_move(q, SHANK_R)
            self.snapshot("03_pre_insert")
            self.stage = "insert: shank push until the fingertips are 3 mm off the clear ear"
            tip_goal = TOP + 0.003 - (CUP_GRIP - FINGERTIP)
            self.cart(self.axial(self.pin_p.copy(), -1, (TOP + 0.030) - tip_goal), 40, grip=(CUP_GRIP, FLATS))
            self.snapshot("04_shank_pushed")
            self.stage = "regrip: release the shank"
            self.held = None
            self.gripper(OPEN_RESET, closing=True)
            self.stage = "regrip: slide back along the pin to the head grip"
            self.cart(None, 30, grip=lambda u: (CUP_GRIP + (HEAD_GRIP - CUP_GRIP) * u, FLATS))
            self.stage = "regrip: close on the head (flats)"
            self.gripper(TAB_HALF[1], closing=True)
            self.attach()
            self.snapshot("05_head_regrip")
            self.stage = "push: tip to the tapped ear's mouth"
            tip_now = (self.pin_p - BORE) @ R_SHACKLE[:, 2]
            self.cart(self.axial(self.pin_p.copy(), -1, tip_now - MOUTH), 40, grip=(HEAD_GRIP, FLATS))
            self.engaged = True
            self.snapshot("06_thread_start")
            jaw_dir, stroke = FLATS, 0
            tip = MOUTH
            while tip > SEAT + 1e-6:
                stroke += 1
                turn = min(np.pi / 2, (tip - SEAT) / PITCH * 2 * np.pi)
                self.stage = f"screw: stroke {stroke} ({np.degrees(turn):.0f} deg)"
                R0, p0, axis = self.pin_R.copy(), self.pin_p.copy(), -self.pin_R[:, 2]
                n = max(2, int(np.ceil(np.degrees(turn) / 7.5)))
                self.cart(lambda u: (rot(axis, turn * u) @ R0, p0 + axis * PITCH * turn * u / (2 * np.pi)), n,
                          grip=(HEAD_GRIP, jaw_dir))
                tip = (self.pin_p - BORE) @ R_SHACKLE[:, 2]
                if tip <= SEAT + 1e-6:
                    break
                self.stage = f"screw: reset after stroke {stroke} (open {OPEN_RESET * 1000:.0f} mm/side, roll back)"
                self.held = None
                self.gripper(OPEN_RESET, 3, closing=True)
                # Roll the open hand back by the stroke. Tightening turned the pin by -turn about its own +Z, so in the
                # pin frame the jaw goes back by +turn (world roll returns to where the stroke started).
                j0 = jaw_dir
                self.cart(None, n, grip=lambda u: (HEAD_GRIP, rot([0, 0, 1], turn * u) @ j0))
                jaw_dir = rot([0, 0, 1], turn) @ j0
                self.gripper(TAB_HALF[1] if abs(jaw_dir @ FLATS) > 0.7 else TAB_HALF[0], 3, closing=True)
                self.attach()
            self.snapshot("07_seated")
            self.stage = "finish: release and retract along the axis"
            self.held = None
            self.gripper(OPEN_RESET, closing=True)
            # 30 mm clears the fingertips past the pin's head end (12.4 mm beyond it at the head grip) with margin; a
            # longer pull along the axis raises the wrist toward the water line for no gain.
            self.cart(None, 20, grip=lambda u: (HEAD_GRIP + 0.030 * u, jaw_dir))
            self.stage = "finish: return to the start pose"
            self.joint_move(self.home, OPEN_RESET)
            self.completed = True
        except RuntimeError as error:
            self.events.append(dict(stage=self.stage, issue="stopped", error=str(error)))
            self.completed = False
        self.nominal_head_grip_check()
        return self.summarise()

    def nominal_head_grip_check(self):
        """The orientation session's screening head grip (TCP at the middle of the head), pin seated: where are the
        fingertips relative to the pin's collar and the clear ear, and do the fingers intersect them?"""
        self.pin_R, self.pin_p = R_SHACKLE.copy(), BORE + R_SHACKLE[:, 2] * SEAT
        self.held, self.engaged, self.stage = None, True, "nominal head grip (screening pose), seated"
        try:
            q = self.solve_grip((NOMINAL_HEAD_GRIP, FLATS), TAB_HALF[1], 0.0)
        except RuntimeError as error:
            self.events.append(dict(stage=self.stage, issue="no IK", error=str(error)))
            return
        self.q, self.opening = q, TAB_HALF[1]
        self.set(q, self.opening)
        self.record()
        s = self.samples.pop()
        pin_overlap = 0.0
        for g in self.fingers:
            self.arm_obj[g].setTransform(fcl.Transform(self.d.geom_xmat[g].reshape(3, 3).copy(),
                                                       self.d.geom_xpos[g].copy()))
            cr = fcl.CollisionResult()
            fcl.collide(self.arm_obj[g], self.pin_obj, fcl.CollisionRequest(num_max_contacts=50, enable_contact=True), cr)
            pin_overlap = max([pin_overlap] + [c.penetration_depth for c in cr.contacts])
        s["finger_pin_max_penetration_mm"] = round(pin_overlap * 1000, 2)
        s["fingertip_minus_collar_top_mm"] = round(s["fingertip_body_z_mm"] - (SEAT + COLLAR_TOP) * 1000, 2)
        s["fingertip_minus_clear_ear_face_mm"] = round(s["fingertip_body_z_mm"] - TOP * 1000, 2)
        self.nominal = s
        self.snapshot("08_nominal_head_grip_seated")

    # -- output --------------------------------------------------------------------------------------------------
    def snapshot(self, name):
        if not hasattr(self, "renderer"):
            self.renderer = mujoco.Renderer(self.m, height=480, width=640)
            self.shots = []
            cam = mujoco.MjvCamera()
            cam.lookat[:] = (BORE + CUP_TIP) / 2
            cam.distance, cam.azimuth, cam.elevation = 0.75, 0.0, -12.0
            self.free_cam = cam
            side = mujoco.MjvCamera()
            side.lookat[:] = (BORE + CUP_TIP) / 2 + [0.0, 0.08, 0.05]
            side.distance, side.azimuth, side.elevation = 0.95, 180.0, -2.0
            self.side_cam = side
        d = self.d
        d.qpos[self.pin_qadr:self.pin_qadr + 3] = self.pin_p
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self.pin_R.ravel())
        d.qpos[self.pin_qadr + 3:self.pin_qadr + 7] = quat
        mujoco.mj_forward(self.m, d)
        views = []
        for cam in (self.side_cam, self.free_cam, "left", "top"):
            self.renderer.update_scene(d, camera=cam)
            views.append(self.renderer.render().copy())
        self.shots.append((name, views))

    def contact_sheet(self):
        from PIL import Image, ImageDraw

        water_px = None
        rows = []
        for name, views in self.shots:
            tiles = [Image.fromarray(v).resize((480, 360)) for v in views]
            row = Image.new("RGB", (480 * 4, 380), "#15212a")
            for i, t in enumerate(tiles):
                row.paste(t, (480 * i, 20))
            ImageDraw.Draw(row).text((6, 4), name + "   [side, 3/4, wrist 'left', overhead 'top' (old station cam)]",
                                     fill="white")
            rows.append(row)
        sheet = Image.new("RGB", (480 * 4, 380 * len(rows)))
        for i, r in enumerate(rows):
            sheet.paste(r, (0, 380 * i))
        sheet.save(OUT / "contact_sheet.jpg", quality=88)
        del water_px

    def summarise(self):
        water_mm = (self.water - TABLE) * 1000
        worst = {}

        def keep(key, value, sample, lower_is_worse=True):
            if key not in worst or (value < worst[key][0] if lower_is_worse else value > worst[key][0]):
                worst[key] = (value, sample["stage"], sample)

        stages = {}
        for s in self.samples:
            keep("water_margin_mm", water_mm - s["arm_top_mm"], s)
            keep("pin_water_margin_mm", water_mm - s["pin_top_mm"], s)
            keep("wall_margin_mm", -s["back_beyond_wall_mm"], s)
            keep("j3_ahead_of_j1_mm", s["j3_ahead_of_j1_mm"], s)
            keep("joint_limit_margin_deg", s["min_limit_margin_deg"], s)
            for k, (v, _) in s["dist_mm"].items():
                keep("clearance_mm:" + k, v, s)
            st = stages.setdefault(s["stage"].split(":")[0], dict(samples=0, min_water_margin_mm=1e9, min_clearance_mm=1e9,
                                                                  self_collision=False))
            st["samples"] += 1
            st["min_water_margin_mm"] = round(min(st["min_water_margin_mm"], water_mm - s["arm_top_mm"]), 1)
            arm_d = [v for k, (v, _) in s["dist_mm"].items() if not k.startswith("pin~cup")]
            if arm_d:
                st["min_clearance_mm"] = round(min(st["min_clearance_mm"], min(arm_d)), 2)
            st["self_collision"] |= s["self_collision"]
        failures = []
        for key, (value, stage, sample) in worst.items():
            need = 0.0 if key == "clearance_mm:pin~shackle" else CLEAR_PAD * 1000  # the shackle's own bore fit
            if key.startswith("clearance_mm:") and value < need and "cup (seated" not in key:
                failures.append(dict(check=key, value_mm=value, stage=stage, nearest_point_plane_mm=sample["dist_mm"][key[13:]][1]))
        if worst["water_margin_mm"][0] < 0:
            failures.append(dict(check="arm above the water line", value_mm=worst["water_margin_mm"][0],
                                 stage=worst["water_margin_mm"][1]))
        if worst["wall_margin_mm"][0] < -0.5:
            failures.append(dict(check="arm behind the wall", value_mm=worst["wall_margin_mm"][0], stage=worst["wall_margin_mm"][1]))
        if worst["j3_ahead_of_j1_mm"][0] < 0:
            failures.append(dict(check="elbow behind J1", value_mm=worst["j3_ahead_of_j1_mm"][0], stage=worst["j3_ahead_of_j1_mm"][1]))
        if any(s["self_collision"] for s in self.samples):
            failures.append(dict(check="arm self-collision", stages=sorted({s["stage"] for s in self.samples if s["self_collision"]})))
        if not self.completed:
            failures.append(dict(check="sequence did not complete", events=[e for e in self.events if e.get("issue")]))
        summary = dict(
            assets_commit="0de8facd", bore_s_mm=BORE_S, cup_tip_radius_mm=round(CUP_R, 1), cup_tip_radius_layout_mm=round(CUP_R_LAYOUT, 1),
            water_line_mm=round(water_mm, 1), wall_y_mm_behind_j1=round((self.wall_y - self.base[1]) * 1000, 1),
            head_grip_tcp_from_tip_mm=round(HEAD_GRIP * 1000, 2), cup_grip_tcp_from_tip_mm=CUP_GRIP * 1000,
            completed=self.completed, samples=len(self.samples), start_pose=self.start_info,
            worst={k: dict(value=round(v[0], 2), stage=v[1]) for k, v in sorted(worst.items())},
            stages=stages, failures=failures, nominal_head_grip_seated=getattr(self, "nominal", None),
            events=self.events, verdict="PASS" if not failures else "FAIL")
        (OUT / "gate_report.json").write_text(json.dumps(summary, indent=1, default=jsonable))
        (OUT / "gate_samples.json").write_text(json.dumps(self.samples, default=jsonable))
        self.contact_sheet()
        return summary


def summarise_sweep(root):
    """One row per run folder under `root`: placement, verdict, the binding margins and the first failure."""
    rows = []
    for report in sorted(root.glob("cup_r*/gate_report.json")):
        r = json.loads(report.read_text())
        w = r["worst"]
        rows.append(dict(
            cup_tip_radius_mm=r["cup_tip_radius_mm"], bore_s_mm=r["bore_s_mm"], verdict=r["verdict"],
            completed=r["completed"], water_margin_mm=w["water_margin_mm"]["value"],
            water_margin_stage=w["water_margin_mm"]["stage"], joint_limit_margin_deg=w["joint_limit_margin_deg"]["value"],
            joint_limit_stage=w["joint_limit_margin_deg"]["stage"], wall_margin_mm=w["wall_margin_mm"]["value"],
            first_failure=(r["failures"][0] if r["failures"] else None)))
    (root / "placement_sweep.json").write_text(json.dumps(rows, indent=1, default=jsonable))
    for x in rows:
        f = x["first_failure"]
        why = "" if f is None else f"{f['check']} @ {f.get('stage') or f.get('events', [{}])[0].get('stage', '')}"
        print(f"cup {x['cup_tip_radius_mm']:5.0f}  bore {x['bore_s_mm']:5.0f}  {x['verdict']}  water {x['water_margin_mm']:6.1f} mm"
              f"  joints {x['joint_limit_margin_deg']:5.1f} deg  {why}")


if __name__ == "__main__":
    if sys.argv[1:] == ["--summarise"]:
        summarise_sweep(OUT_ROOT)
        sys.exit(0)
    result = Gate().run()
    print(json.dumps({k: v for k, v in result.items() if k not in ("stages",)}, indent=1, default=jsonable)[:6000])
    print("verdict", result["verdict"])
