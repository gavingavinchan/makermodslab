# Copyright 2026 MakerMods. All rights reserved.
"""How high does the left arm reach while working the tilted shackle, versus the pin's tilt and the shackle's distance?

Gavin will run the task in an aquarium (2026-11-06): the water line is J3's height with the J2-J3 link straight up,
and no part of the arm may come out of the water during the task. J1 stands at the very edge of the tank, so the
elbow (J3) must stay ahead of J1 and nothing that moves may reach past the base's back edge (the wall). The tilt
need not be 45 degrees. Everything stays on the left arm's forward plane: J1 and wrist yaw (J5) are held at 0.

For each pin tilt and shackle distance along the forward line, this solves IK on the real arm model (offline: the
shackle scene is only loaded, no server, no network, no hardware) with the gripper approaching along the pin axis,
for the grips of a regrip plan:

- insert start: the pin tip at the clear ear's mouth, held by the shank SHANK_GRIP above the tip;
- head regrip: pushed until the fingertips reach the clear ear, then regripped at the head (the highest moment);
- thread start: the tip at the tapped ear's mouth, held at the head;
- seated: the pin fully home, held at the head (the last tightening stroke).

Wrist roll is swept all the way round (the 120-degree tightening strokes use part of that). Per roll the lowest
collision-free solution is kept (arm against itself or the table), and each grip reports the highest arm-mesh
point over the best 120-degree window, with the water margin and the arm part that is highest. This screens
poses only: it does not check that one continuous motion links them, the pickup of the loose pin, or torque.
The shackle sits with its bow on the tank floor (CLEARANCE), bow toward the arm and down.

    MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python shackle_tilt45_water_check.py [runs/shackle_tilt45]
"""

import json
import os
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares

from shackle_env import ShackleEnv
from shackle_scene import TABLE

HERE = Path(__file__).resolve().parent
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "runs/shackle_tilt45"
APPROACH_LOCAL = np.array([0.986991, 0.0, -0.160775])  # viscous_arm.json, wrist_roll_link frame
JAW_LOCAL = np.array([0.0, 1.0, 0.0])
ASSETS = HERE / "assets/shackle"
CLEARANCE = 0.0  # under the lowest point: Gavin 2026-11-06 accepted the bow resting on the tank floor (was 15 mm)
PIN_SEATED_Z = -0.039925
HEAD_GRIP = 0.057  # along the pin axis from the bore midpoint to the middle of the head, pin seated
UNDERHEAD = 0.07985  # shackle_scene.py: pin travel from the clear ear's mouth to seated
TILTS_DEG = (70, 75, 80, 85)  # Gavin 2026-11-06: need not be exactly 45
DISTANCES_MM = {t: range(400, 576, 25) for t in TILTS_DEG}
THREAD_TRAVEL = 0.020  # tip from the tapped ear's mouth to seated: the head grip is needed only for this last part
# The gripper holds the pin coaxially, so the pin behind the grip point goes up between the fingers: the palm is
# 53.4 mm behind the TCP (measured on the model), so at most ~50 mm of the 107.9 mm pin may sit behind the grip.
SHANK_GRIP = 0.062  # grip centre above the threaded tip on the shank (the collar starts 79.85 mm above the tip)
HEAD_ABOVE_TIP = HEAD_GRIP - PIN_SEATED_Z  # the head grip measured from the tip (96.9 mm)
FINGERTIP = 0.0343 + 0.003  # closed fingertips ahead of the TCP, plus a 3 mm gap to the clear ear's outer face
# Regrip plan (Gavin 2026-11-07: regripping is fine): start on the shank, push until the fingertips reach the clear
# ear (tip ~25 mm through it), regrip on the head, push to the threads and screw. The head regrip is the highest
# moment. Each grip is (name, distance from the bore midpoint along the pin axis toward the head).
PUSHED_TIP = -PIN_SEATED_Z + FINGERTIP - SHANK_GRIP  # tip position (along the axis) when the shank push stops
GRIPS = (("insert start", -PIN_SEATED_Z + SHANK_GRIP), ("head regrip", PUSHED_TIP + HEAD_ABOVE_TIP),
         ("thread start", HEAD_GRIP + THREAD_TRAVEL), ("seated", HEAD_GRIP))
RESULT = f"water_check_clear{CLEARANCE * 1000:.0f}mm_wall_regrip2.json"
# Optional overrides so several sweeps can run side by side: WATER_TILTS="60,65" WATER_DIST="550,701,25" WATER_TAG=far
if os.environ.get("WATER_TILTS"):
    TILTS_DEG = tuple(float(t) for t in os.environ["WATER_TILTS"].split(","))
if os.environ.get("WATER_DIST"):
    DISTANCES_MM = {t: range(*(int(v) for v in os.environ["WATER_DIST"].split(","))) for t in TILTS_DEG}
# WATER_MODE=pickup checks the loose pin's pickup instead: Gavin wants it in a ~30 mm cup (2026-11-06). Below the
# water line the cup must hold it at the shackle's tilt, head toward the arm, on the same line nearer the arm, so the
# pickup grip is the insertion grip. distance = the pin tip's distance ahead of J1; one roll is enough (no window).
CUP_TIP_H = 0.015  # pin axis height at the tip: the tilted cup's lowest edge on the floor (2.5 mm wall, ~20.6 bore)
CUP_TIP_H = float(os.environ.get("WATER_CUP_H_MM", CUP_TIP_H * 1000)) / 1000  # raised on a pedestal
CUP_GRIP = 0.068  # above the tip: the fingertips (34.3 mm ahead of the TCP) then clear a 30 mm cup's rim by ~3 mm
PICKUP = os.environ.get("WATER_MODE") == "pickup"
if PICKUP:
    GRIPS = (("cup pickup", CUP_GRIP),)
    RESULT = RESULT.replace(".json", f"_pickup_h{CUP_TIP_H * 1000:.0f}.json")
if os.environ.get("WATER_TAG"):
    RESULT = RESULT.replace(".json", f"_{os.environ['WATER_TAG']}.json")
ROLLS_DEG = range(0, 360, 15)
WINDOW_DEG = 0 if PICKUP else 120


def load_verts(path):
    return np.array([[float(x) for x in line.split()[1:4]] for line in path.read_text().splitlines()
                     if line.startswith("v ")])


def center_height(tilt):
    """Bore-midpoint height that leaves CLEARANCE under the lowest point, bow toward the arm and down."""
    t = np.radians(tilt)
    body = load_verts(ASSETS / "shackle_body.obj")
    pin = load_verts(ASSETS / "shackle_pin.obj") + [0, 0, PIN_SEATED_Z]
    # height of body-frame X (= minus the bow direction, which points down at sin t) and Z (the pin axis, cos t)
    h = np.vstack([body, pin]) @ np.array([np.sin(t), 0.0, np.cos(t)])
    return CLEARANCE - h.min()


def main():
    env = ShackleEnv()
    m, d = env.model, env.data
    mujoco.mj_forward(m, d)
    qadr = np.array(env.qadr["left"])
    start_q = d.qpos[qadr].copy()
    base = d.body("left_base_link").xpos.copy()
    lo = np.array([m.jnt_range[m.joint(f"left_joint{i}").id][0] for i in range(1, 7)])
    hi = np.array([m.jnt_range[m.joint(f"left_joint{i}").id][1] for i in range(1, 7)])
    # Everything stays on the forward plane: base (J1) and wrist yaw (J5) held at 0, so J2-J4 place the gripper in
    # the plane and J6 rolls it. Left free, the solver escaped sideways (J1 68 deg) to lower the arm top.
    for i in (0, 4):
        lo[i], hi[i] = -1e-4, 1e-4
    arm_geoms = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("left_")}
    table = m.geom("table").id

    def collides():
        """Arm against itself or the table (the scene's own shackle, pin and holder sit elsewhere: ignored)."""
        mujoco.mj_collision(m, d)
        for c in d.contact[:d.ncon]:
            pair = {c.geom1, c.geom2}
            if c.dist < -0.5e-3 and pair <= arm_geoms | {table} and pair & arm_geoms:
                return True
        return False
    geoms = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("left_")
             and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] == 2]
    moving = [g for g in geoms if m.body(m.geom_bodyid[g]).name != "left_base_link"]
    verts = {g: m.mesh_vert[m.mesh_vertadr[m.geom_dataid[g]]:][:m.mesh_vertnum[m.geom_dataid[g]]] for g in geoms}

    def pose(q):
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        R = d.body("left_wrist_roll_link").xmat.reshape(3, 3)
        return d.body("left_tool0").xpos.copy(), R @ APPROACH_LOCAL, R @ JAW_LOCAL

    def back_mm():
        """How far behind J1's axis the moving arm reaches (mm, positive = behind), and J3's own offset."""
        behind = max((verts[g] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 1].max() - base[1]
                     for g in moving)  # forward is world -Y, so behind is +Y
        return behind * 1000, (d.body("left_forearm_link").xpos[1] - base[1]) * 1000

    def top_mm():
        return max((verts[g] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].max() for g in geoms)

    def heights_mm(q):
        pose(q)
        return (top_mm() - TABLE) * 1000, (d.body("left_forearm_link").xpos[2] - TABLE) * 1000

    def top_body():
        return max(geoms, key=lambda g: (verts[g] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].max())

    pose(start_q)
    start_top, start_j3 = heights_mm(start_q)
    # Gavin 2026-11-06: J1 stands at the very edge of a long aquarium, so the wall is right behind the base. Nothing
    # that moves may reach further back than the base's own back edge, and the elbow (J3) stays ahead of J1.
    wall = max((verts[g] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 1].max() - base[1]
               for g in geoms if m.body(m.geom_bodyid[g]).name == "left_base_link") * 1000

    def clear_of_wall(q):
        pose(q)
        back, j3_behind = back_mm()
        return back <= wall + 0.5 and j3_behind <= 0.0
    # Gavin 2026-11-06: the water line is J3's height with the J2-J3 link straight up
    j2 = d.body("left_upper_arm_link").xpos
    water = (j2[2] + np.linalg.norm(d.body("left_forearm_link").xpos - j2) - TABLE) * 1000
    across = np.array([1.0, 0.0, 0.0])

    def solve(target, approach, jaw, seeds):
        """All distinct collision-free solutions from these seeds, lowest arm top first."""
        found = []
        for seed in seeds:
            def res(q):
                p, a, j = pose(q)
                return np.r_[(p - target) * 1000, (a - approach) * 50, (j - jaw) * 20]

            r = least_squares(res, np.clip(seed, lo + 1e-6, hi - 1e-6), bounds=(lo, hi), xtol=1e-10, ftol=1e-10)
            p, a, j = pose(r.x)
            err = (np.linalg.norm(p - target) * 1000, np.degrees(np.arccos(np.clip(a @ approach, -1, 1))),
                   np.degrees(np.arccos(np.clip(j @ jaw, -1, 1))))
            if err[0] < 1.0 and err[1] < 1.0 and err[2] < 2.0 and not collides() and clear_of_wall(r.x):
                if all(np.abs(r.x - f[0]).max() > 0.05 for f in found):
                    found.append((r.x, heights_mm(r.x)[0]))
        return sorted(found, key=lambda f: f[1])

    grid = [np.r_[0, q2, q3, q4, 0, q6] for q2 in (-2.4, -1.6, -0.8) for q3 in (0.8, 2.0, 3.2)
            for q4 in (-1.2, 0.0, 1.0) for q6 in (-1.5, 1.5)]
    rows = []
    for tilt, dist in ((t, dist) for t in TILTS_DEG for dist in DISTANCES_MM[t]):
        axis = np.array([0.0, np.sin(np.radians(tilt)), np.cos(np.radians(tilt))])  # toward the head and the base
        center = base + np.array([0.0, -dist / 1000, 0.0])
        center[2] = TABLE + (CUP_TIP_H if PICKUP else center_height(tilt))  # the pin tip when picking up
        for grip, along in GRIPS:
            target = center + axis * along
            approach = -axis
            per_roll = {}
            branches = []  # the elbow-up / elbow-down solutions, carried from one roll to the next
            for roll in ROLLS_DEG:
                phi = np.radians(roll)
                jaw = np.cos(phi) * across + np.sin(phi) * np.cross(approach, across)
                step = np.radians(ROLLS_DEG.step)
                seeds = [start_q] + [q + np.r_[0, 0, 0, 0, 0, sign * step] for q in branches for sign in (-1, 1)]
                found = solve(target, approach, jaw, seeds if branches else seeds + grid)
                if not found and branches:
                    found = solve(target, approach, jaw, grid)
                if found:
                    branches = [f[0] for f in found]
                    best = found[0]
                    top, j3 = heights_mm(best[0])
                    per_roll[roll] = dict(top_mm=round(top, 1), j3_mm=round(j3, 1), q=np.round(best[0], 4).tolist(),
                                          top_part=m.body(m.geom_bodyid[top_body()]).name)
            window = None
            for r0 in ROLLS_DEG:
                inside = [(r0 + k) % 360 for k in range(0, WINDOW_DEG + 1, 15)]
                if all(r in per_roll for r in inside):
                    worst = max(per_roll[r]["top_mm"] for r in inside)
                    if window is None or worst < window[1]:
                        top_roll = max(inside, key=lambda r: per_roll[r]["top_mm"])
                        window = (r0, worst, max(per_roll[r]["j3_mm"] for r in inside), per_roll[top_roll]["top_part"])
            rows.append(dict(tilt_deg=tilt, distance_mm=dist, grip=grip, center_height_mm=round(
                (center[2] - TABLE) * 1000, 1), top_part=None if window is None else window[3], reachable_rolls=len(per_roll),
                             best_window_start_deg=None if window is None else window[0],
                             arm_top_mm=None if window is None else round(window[1], 1),
                             j3_mm=None if window is None else round(window[2], 1), per_roll=per_roll))
            w = rows[-1]
            w["margin_mm"] = None if w["arm_top_mm"] is None else round(water - w["arm_top_mm"], 1)
            print(f"tilt {tilt}  {dist:4d} mm  {grip:12s}  rolls reached {len(per_roll):2d}/24  "
                  + ("no full 120-deg window" if window is None else
                     f"arm top {w['arm_top_mm']:6.1f} mm, {w['margin_mm']:+6.1f} under water ({w['top_part']})"), flush=True)

    summary = dict(water_line_mm=round(water, 1), wall_behind_j1_mm=round(wall, 1), start_pose=dict(arm_top_mm=round(start_top, 1),
                   j3_mm=round(start_j3, 1)), rows=rows)
    (OUT / RESULT).write_text(json.dumps(summary, indent=1))
    print(f"water line {water:.1f} mm; start pose: arm top {start_top:.1f} mm, J3 {start_j3:.1f} mm -> {OUT / RESULT}")
    env.close()


if __name__ == "__main__":
    main()
