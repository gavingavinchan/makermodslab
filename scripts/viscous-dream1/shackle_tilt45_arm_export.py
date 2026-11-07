# Copyright 2026 MakerMods. All rights reserved.
"""Export the left Viscous arm's visual meshes, posed at the shackle task's start pose, for shackle_tilt45_sketch.py.

Loads the shackle scene offline (no server, no network, no hardware), takes the left arm's group-2 visual mesh geoms
at the start pose and writes their triangles (to .cache/shackle_tilt45/, regenerable, kept out of git) in the sketch's plane frame: s = mm ahead of the left base along its
forward direction (world -Y), w = mm across (world +X), h = mm above the table. Run with the sim venv (MuJoCo); the
sketch itself only needs numpy + matplotlib.

If WATER_RESULT (from shackle_tilt45_water_check.py) is present, also exports the arm at the critical pose of each case
in WATER_CASES (the highest roll inside its best 120-degree window) to .cache/shackle_tilt45/, for
shackle_tilt45_water_figure.py. Those are regenerable and stay out of git.

    MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python shackle_tilt45_arm_export.py [runs/shackle_tilt45]
"""

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "runs/shackle_tilt45"
CACHE = HERE / ".cache/shackle_tilt45"
WATER_RESULT = "water_check_clear0mm_wall_regrip2_t85.json"  # the layout chosen 2026-11-07: 85 deg, 575 mm
WATER_CASES = [(85, 575, "insert start"), (85, 575, "head regrip"), (85, 575, "thread start"), (85, 575, "seated")]


def case_tag(tilt, dist, grip):
    return f"tilt{float(tilt):g}_{dist}mm_{grip.replace(' ', '_')}"


def main():
    import mujoco  # here, not at module level: shackle_tilt45_water_figure.py imports this module without MuJoCo

    from shackle_env import ShackleEnv
    from shackle_scene import TABLE

    OUT.mkdir(parents=True, exist_ok=True)
    env = ShackleEnv()
    m, d = env.model, env.data
    mujoco.mj_forward(m, d)
    base = d.body("left_base_link").xpos.copy()
    to_plane = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # (x, y, z) -> (s, w, h), proper

    def arm_tris():
        tris = []
        for g in range(m.ngeom):
            body = m.body(m.geom_bodyid[g]).name
            if not body.startswith("left_") or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_group[g] != 2:
                continue
            mid = m.geom_dataid[g]
            v0, nv = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            f0, nf = m.mesh_faceadr[mid], m.mesh_facenum[mid]
            world = m.mesh_vert[v0:v0 + nv] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            tris.append(world[m.mesh_face[f0:f0 + nf]])
        return (np.concatenate(tris) - [base[0], base[1], TABLE]) @ to_plane.T * 1000.0

    plane = arm_tris()
    CACHE.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(CACHE / "left_arm_start_pose.npz", tris=plane.astype(np.float32),
                        base_height_mm=np.float32((base[2] - TABLE) * 1000.0),
                        qpos=d.qpos[env.qadr["left"]].copy())
    print(f"{len(plane)} triangles, s {plane[..., 0].min():.0f}..{plane[..., 0].max():.0f}, "
          f"h {plane[..., 2].min():.0f}..{plane[..., 2].max():.0f} mm -> {CACHE}")

    check = OUT / WATER_RESULT
    if check.exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        rows = json.loads(check.read_text())["rows"]
        qadr = env.qadr["left"]
        for tilt, dist, grip in WATER_CASES:
            row = next(r for r in rows
                       if (float(r["tilt_deg"]), r["distance_mm"], r["grip"]) == (float(tilt), dist, grip))
            inside = [str((row["best_window_start_deg"] + k) % 360) for k in range(0, 121, 15)]
            roll = max(inside, key=lambda r: row["per_roll"][r]["top_mm"])
            d.qpos[qadr] = row["per_roll"][roll]["q"]
            mujoco.mj_kinematics(m, d)
            tris = arm_tris()
            np.savez_compressed(CACHE / f"{case_tag(tilt, dist, grip)}.npz", tris=tris.astype(np.float32),
                                roll_deg=int(roll), center_height_mm=row["center_height_mm"],
                                arm_top_mm=row["arm_top_mm"])
            print(f"{case_tag(tilt, dist, grip)}: roll {roll}, top {tris[..., 2].max():.1f} mm")
    env.close()


if __name__ == "__main__":
    main()
