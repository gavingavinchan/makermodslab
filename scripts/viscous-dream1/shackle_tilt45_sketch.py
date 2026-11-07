# Copyright 2026 MakerMods. All rights reserved.
"""Orientation sketch for Gavin to confirm: shackle and pin tilted 45 deg, everything on the arm's forward plane.

Proposal only: nothing in the scene, holder or prints changes. The plane is the vertical plane through the left
(active) arm's base along its start-pose forward direction (world -Y). The U of the shackle lies in that plane, the
pin axis is 45 deg from vertical with its head (the end that gets turned) leaning back toward the arm, and the bow
points toward the arm and down (Gavin, 2026-11-06: flipped 180 deg about the pin axis from the first draft, whose bow
pointed away and up). The shackle's distance along the line and its height are
placeholders, to be chosen later with the reach and torque checks. The left arm is drawn from its real visual meshes
at the task's start pose, exported by shackle_tilt45_arm_export.py (run that first, with the sim venv).

    python3 shackle_tilt45_sketch.py [assets/shackle] [runs/shackle_tilt45]
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection
from matplotlib.patches import Arc

HERE = Path(__file__).resolve().parent
ASSETS = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "assets/shackle"
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE / "runs/shackle_tilt45"
PIN_SEATED_Z_MM = -39.925  # shackle_side_view.py: pin origin (threaded tip) when seated, body frame
TILT = np.radians(45.0)
CENTER_S_MM = 430.0  # Gavin 2026-11-06: 150 mm further out than the 280 mm draft (bore midpoint from the J1 axis)
XMAX = max(440.0, CENTER_S_MM + 170.0)
CLEARANCE_MM = 15.0  # placeholder gap under the lowest point, room for a holder


def load_obj(path):
    verts, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
            faces.extend([idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1))
    return np.array(verts) * 1000.0, np.array(faces)  # mm


# Body frame (shackle_scene.py): +Z is the pin axis toward the head, -X is the bow, origin at the bore midpoint.
# Plane frame: s = distance ahead of the base along the forward line, w = across the plane, h = height.
# Body +Z -> back toward the arm and up; body -X (bow) -> toward the arm and down; body Y -> across.
AXIS = np.array([-np.sin(TILT), 0.0, np.cos(TILT)])  # head direction in (s, w, h)
VARIANTS = {  # file stem -> (bow direction, bow label); the first draft is kept for a side-by-side comparison
    "shackle_tilt45_arm_bow_toward": (np.array([-np.cos(TILT), 0.0, -np.sin(TILT)]),  # chosen 2026-11-06
                                      "bow (C) points toward the arm\nand down, under the pin head"),
    "shackle_tilt45_arm_bow_away": (np.array([np.cos(TILT), 0.0, np.sin(TILT)]),  # first draft, rejected
                                "bow (C) points away from the arm\nand up"),
}


ARM_RGB = np.array([0.93, 0.80, 0.55])
LABEL_BOX = dict(facecolor="white", alpha=0.85, edgecolor="none", pad=1.5)


def draw(ax, parts, view, light):
    """Project (triangles, rgb) parts along `view` (unit, toward the viewer), painter-sorted together so the arm
    and the shackle hide each other correctly."""
    tri = np.concatenate([t for t, _ in parts])
    rgb = np.concatenate([np.tile(c, (len(t), 1)) for t, c in parts])
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    shade = 0.35 + 0.65 * np.abs(n @ (np.array(light) / np.linalg.norm(light)))
    order = np.argsort(tri.mean(axis=1) @ view)  # far first
    keep = [0, 2] if abs(view[1]) > 0.5 else [0, 1]  # side view -> (s, h); top view -> (s, w)
    ax.add_collection(PolyCollection(tri[order][:, :, keep], facecolors=np.clip(shade[:, None] * rgb, 0, 1)[order],
                                     edgecolors="none", zorder=3, rasterized=True))


def render(stem, BOW, bow_label):
    R = np.column_stack([-BOW, np.cross(AXIS, -BOW), AXIS])  # columns: body X, Y, Z in plane frame (proper)
    body_v, body_f = load_obj(ASSETS / "shackle_body.obj")
    pin_v, pin_f = load_obj(ASSETS / "shackle_pin.obj")
    pin_v = pin_v + np.array([0.0, 0.0, PIN_SEATED_Z_MM])
    body_p, pin_p = body_v @ R.T, pin_v @ R.T
    center = np.array([CENTER_S_MM, 0.0, CLEARANCE_MM - min(body_p[:, 2].min(), pin_p[:, 2].min())])
    body_p, pin_p = body_p + center, pin_p + center
    head = pin_p[np.argmax(pin_p @ AXIS)]
    head_end = center + AXIS * ((head - center) @ AXIS)

    arm = np.load(HERE / ".cache/shackle_tilt45/left_arm_start_pose.npz")
    arm_tris = arm["tris"].astype(float)
    fig = plt.figure(figsize=(10.5, 12.5))
    side = fig.add_axes([0.06, 0.25, 0.9, 0.67])
    top = fig.add_axes([0.06, 0.02, 0.9, 0.19])
    steel, blue = np.array([0.80, 0.82, 0.85]), np.array([0.45, 0.62, 0.95])
    fig.text(0.04, 0.965, "Shackle tilted 45°, all on the arm's forward plane (proposal, please confirm)",
             fontsize=13, weight="bold")
    fig.text(0.04, 0.945, "All to scale from real meshes: the left Viscous arm at the task's start pose, shackle and pin. "
             "The shackle's distance\nalong the line and its height are placeholders.", fontsize=8.5, color="#555")

    # --- side view: looking across the plane
    ax = side
    ax.axhline(0, color="#6b5b45", lw=2)
    ax.fill_between([-80, XMAX], -14, 0, color="#e8dfd0", zorder=0)
    ax.text(XMAX - 5, -9, "table", ha="right", fontsize=8, color="#6b5b45")
    ax.plot([0, 0], [0, 420], color="#7f8c8d", ls="-.", lw=1, zorder=2)
    ax.text(-75, 395, "left arm, start pose\n(J1 axis dash-dot)", fontsize=8.5)
    ax.annotate("", xy=(140, 432), xytext=(45, 432),
                arrowprops=dict(arrowstyle="-|>", color="#7f8c8d", lw=1.2))
    ax.text(50, 437, "arm's forward direction", fontsize=8, color="#555")
    parts = ((arm_tris, ARM_RGB), (body_p[body_f], steel), (pin_p[pin_f], blue))
    draw(ax, parts, view=np.array([0, -1.0, 0]), light=(-0.4, -0.8, 0.5))

    # the distance Gavin set: J1 axis to the bore midpoint
    ax.annotate("", xy=(center[0], -6), xytext=(0, -6), arrowprops=dict(arrowstyle="<->", color="#8e44ad", lw=1.3),
                zorder=6)
    ax.text(center[0] / 2, -11, f"{CENTER_S_MM:.0f} mm from the J1 axis to the shackle centre (bore midpoint)", ha="center",
            fontsize=8, color="#8e44ad", zorder=6)
    ax.plot([center[0], center[0]], [-6, center[2]], color="#8e44ad", lw=0.6, ls=":", zorder=4)

    # pin axis, vertical reference and the 45 deg angle between them
    tip_far = center - AXIS * 70
    ax.plot([tip_far[0], head_end[0] + AXIS[0] * 25], [tip_far[2], head_end[2] + AXIS[2] * 25],
            color="#c0392b", lw=0.9, ls=(0, (6, 3, 1, 3)), zorder=4)
    ax.plot([center[0], center[0]], [center[2], center[2] + 115], color="#2c3e50", lw=0.9, ls="--", zorder=4)
    ax.add_patch(Arc((center[0], center[2]), 2 * 95, 2 * 95, theta1=90, theta2=135, color="#c0392b", lw=1.4,
                     zorder=5))
    ax.text(center[0] - 22, center[2] + 103, "45°", color="#c0392b", fontsize=13, weight="bold", zorder=5,
            bbox=LABEL_BOX)
    ax.text(center[0] + 4, center[2] + 80, "vertical", fontsize=7.5, color="#2c3e50")

    # gripper approach along the pin axis (drawn beside it, clear of the angle)
    a0, a1 = head_end + AXIS * 110 + BOW * 28, head_end + AXIS * 18 + BOW * 28
    ax.annotate("", xy=(a1[0], a1[2]), xytext=(a0[0], a0[2]),
                arrowprops=dict(arrowstyle="-|>", color="#16a085", lw=2))
    ax.text(a0[0] + 8, a0[2] + 10, "gripper comes in along the pin axis;\nturning the pin = wrist roll",
            fontsize=8, color="#16a085", va="bottom", bbox=LABEL_BOX, zorder=6)

    ax.annotate("pin head = the top\n(the end that is turned)\nleans back toward the arm",
                xy=(head_end[0] - 4, head_end[2] - 4), xytext=(-70, 112), fontsize=8.5, bbox=LABEL_BOX, zorder=6,
                arrowprops=dict(arrowstyle="->", lw=0.8))
    upper, lower = center + AXIS * 30, center - AXIS * 30
    ax.annotate("clear ear (upper)", xy=(upper[0] + 10, upper[2] + 10), xytext=(center[0] + 40, center[2] + 60),
                fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax.annotate("tapped ear (lower)", xy=(lower[0] + 10, lower[2] + 8), xytext=(center[0] + 70, center[2]),
                fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    crown = body_p[np.argmax(body_p @ BOW)]
    ax.annotate(bow_label, xy=(crown[0], crown[2]),
                xytext=(center[0] + 45, 40), fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax.text(center[0] + 65, 5, "holder: to be redesigned\nfor this pose (not drawn)", fontsize=7.5, color="#888",
            style="italic")
    ax.text(-75, 455, "SIDE VIEW: looking across the plane", fontsize=9.5, weight="bold")
    ax.set_xlim(-80, XMAX)
    ax.set_ylim(-14, 470)
    ax.set_aspect("equal")
    ax.set_xlabel("mm ahead of the left base (world -Y)", fontsize=8)
    ax.set_ylabel("mm above table", fontsize=8)
    ax.tick_params(labelsize=7)

    # --- top view: everything sits on the forward line
    ax = top
    ax.axhline(0, color="#c0392b", lw=1, ls=(0, (6, 3, 1, 3)))
    ax.text(XMAX - 5, 22, "forward line = the plane seen from above", ha="right", va="bottom", fontsize=8,
            color="#c0392b")
    ax.text(-75, -70, "left arm, start pose", fontsize=7.5, bbox=LABEL_BOX, zorder=6)
    draw(ax, parts, view=np.array([0, 0, 1.0]), light=(-0.3, 0.4, 0.85))

    ax.text(center[0] - 55, -50, "shackle + pin seen edge-on: a thin strip along the line", fontsize=7.5)
    ax.text(-75, 70, "TOP VIEW", fontsize=9.5, weight="bold")
    ax.set_xlim(-80, XMAX)
    ax.set_ylim(-80, 85)
    ax.set_aspect("equal")
    ax.set_yticks([])
    ax.tick_params(labelsize=7)
    for a in (side, top):
        for sp in ("top", "right"):
            a.spines[sp].set_visible(False)

    fig.savefig(OUT / f"{stem}_{CENTER_S_MM:.0f}mm.png", dpi=150)
    plt.close(fig)
    print(f"{stem}: bore center s={center[0]:.0f} h={center[2]:.1f} mm, head end s={head_end[0]:.1f}"
          f" h={head_end[2]:.1f}")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for stem, (bow, label) in VARIANTS.items():
        render(stem, bow, label)
    imgs = [plt.imread(OUT / f"{stem}_{CENTER_S_MM:.0f}mm.png") for stem in VARIANTS]
    fig, axes = plt.subplots(1, 2, figsize=(16, 4.1))
    for ax, img, title in zip(axes, imgs, ("NEW: bow toward the arm and down", "OLD (first draft): bow away and up")):
        ax.imshow(img[int(img.shape[0] * 0.08):int(img.shape[0] * 0.74)])
        ax.set_title(title, fontsize=13, weight="bold", color="#1e8449" if title.startswith("NEW") else "#999")
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(OUT / f"shackle_tilt45_arm_compare_{CENTER_S_MM:.0f}mm.png", dpi=110)


if __name__ == "__main__":
    main()
