# Copyright 2026 MakerMods. All rights reserved.
"""Side views of the left arm at the shackle task's critical poses against the aquarium water line.

Gavin, 2026-11-06: the water line is J3's height with the J2-J3 link straight up, and no part of the arm may come out
of the water during the task. Each panel shows one case from shackle_tilt45_water_check.py (the arm posed by its IK,
exported by shackle_tilt45_arm_export.py) with the shackle at that tilt, the pin seated or at the start of insertion.

    python3 shackle_tilt45_water_figure.py
"""

import json
import sys
from pathlib import Path

sys.argv = sys.argv[:1]  # the sketch module reads optional paths from argv
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import shackle_tilt45_sketch as sketch  # noqa: E402
from shackle_tilt45_arm_export import CACHE, WATER_CASES, WATER_RESULT, case_tag  # noqa: E402

OUT = sketch.OUT
# How far the tip is from its seated place along the axis (shackle_tilt45_water_check.py's grip plan)
PIN_RAISE_MM = {"seated": 0.0, "thread start": 20.0, "head regrip": 55.2, "insert start": 79.85}
GRIP_TEXT = {"seated": "last tightening stroke (pin seated, head grip)",
             "thread start": "threads about to catch (head grip)",
             "head regrip": "regrip on the head after the shank push",
             "insert start": "starting the insertion (shank grip)"}


def shackle_parts(tilt, dist, center_h, grip):
    t = np.radians(tilt)
    axis = np.array([-np.sin(t), 0.0, np.cos(t)])
    bow = np.array([-np.cos(t), 0.0, -np.sin(t)])
    R = np.column_stack([-bow, np.cross(axis, -bow), axis])
    body_v, body_f = sketch.load_obj(sketch.ASSETS / "shackle_body.obj")
    pin_v, pin_f = sketch.load_obj(sketch.ASSETS / "shackle_pin.obj")
    pin_v = pin_v + [0.0, 0.0, sketch.PIN_SEATED_Z_MM + PIN_RAISE_MM[grip]]
    center = np.array([dist, 0.0, center_h])
    return (((body_v @ R.T + center)[body_f], np.array([0.80, 0.82, 0.85])),
            ((pin_v @ R.T + center)[pin_f], np.array([0.45, 0.62, 0.95])))


def main():
    result = json.loads((OUT / WATER_RESULT).read_text())
    water, wall = result["water_line_mm"], result["wall_behind_j1_mm"]
    xmax = max(dist for _, dist, _ in WATER_CASES) + 130
    ncol = 2 if len(WATER_CASES) == 4 else len(WATER_CASES)
    nrow = -(-len(WATER_CASES) // ncol)
    fig, axes = plt.subplots(nrow, ncol, figsize=(7.2 * ncol * (xmax + 80) / 710, 5.2 * nrow + 0.8), squeeze=False)
    fig.suptitle(f"Water line {water:.0f} mm above the table (J3 with the J2-J3 link straight up): "
                 "the arm at the task's highest moments", fontsize=13, weight="bold", x=0.02, ha="left")
    for ax, (tilt, dist, grip) in zip(axes.ravel(), WATER_CASES):
        pose = np.load(CACHE / f"{case_tag(tilt, dist, grip)}.npz")
        arm = pose["tris"].astype(float)
        ax.axhspan(0, water, color="#d6eaf8", zorder=0)
        ax.axhline(water, color="#2e86c1", lw=1.5, zorder=5)
        ax.text(-75, water + 4, "water line", color="#2e86c1", fontsize=8.5, zorder=6)
        ax.axhline(0, color="#6b5b45", lw=2, zorder=1)
        parts = ((arm, sketch.ARM_RGB), *shackle_parts(tilt, dist, float(pose["center_height_mm"]), grip))
        sketch.draw(ax, parts, view=np.array([0, -1.0, 0]), light=(-0.4, -0.8, 0.5))
        top = arm[..., 2].max()
        over = top - water
        ax.axhline(top, color="#c0392b" if over > 0 else "#1e8449", lw=0.9, ls="--", zorder=5)
        verdict = f"{over:.0f} mm OUT of the water" if over > 0 else f"{-over:.0f} mm under the water"
        ax.text(xmax - 5, top + 4, f"arm top {top:.0f} mm: {verdict}", ha="right", fontsize=8.5, weight="bold",
                color="#c0392b" if over > 0 else "#1e8449", zorder=6, bbox=sketch.LABEL_BOX)
        ax.set_title(f"pin {float(tilt):g}° from vertical, shackle {dist} mm out: {GRIP_TEXT[grip]}", fontsize=10,
                     loc="left")
        ax.set_xlim(-80, xmax)
        ax.plot([0, 0], [0, 420], color="#7f8c8d", ls="-.", lw=1, zorder=2)
        ax.axvline(-wall, color="#5d6d7e", lw=3, zorder=2)
        ax.text(-wall - 4, 380, "tank wall", rotation=90, ha="right", fontsize=8, color="#5d6d7e")
        ax.set_ylim(-10, 440)
        ax.set_aspect("equal")
        ax.tick_params(labelsize=7)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    for ax in axes[-1]:
        ax.set_xlabel("mm ahead of the left base", fontsize=8)
    for ax in axes[:, 0]:
        ax.set_ylabel("mm above table", fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(OUT / f"shackle_water_line_{water:.0f}mm_planar.png", dpi=120)
    print(OUT / f"shackle_water_line_{water:.0f}mm_planar.png")


if __name__ == "__main__":
    main()
