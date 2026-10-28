# Copyright 2026 MakerMods. All rights reserved.
"""Side view of the shackle (bow plane face-on, so the bow reads as a C) for Gavin to sketch the holder on.

Orthographic, looking along the body's Y axis: pin axis vertical, clear ear on top, tapped ear below, bow to the
left. Writes a PNG with a mm grid and a 1:1-scale PDF that can be printed and drawn on.

    python shackle_side_view.py [assets/shackle] [runs/shackle_side_view]
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection

HERE = Path(__file__).resolve().parent
ASSETS = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "assets/shackle"
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE / "runs/shackle_side_view"
PIN_SEATED_Z_M = -0.039925  # shackle_model_report.md: pin origin (threaded tip) when seated, body frame


def load_obj(path):
    verts, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
            faces.extend([idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1))
    return np.array(verts) * 1000.0, np.array(faces)  # mm


def shaded_tris(verts, faces, base_rgb, light=(-0.4, -0.8, 0.5)):
    tri = verts[faces]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    light = np.array(light) / np.linalg.norm(light)
    shade = 0.35 + 0.65 * np.abs(n @ light)
    depth = tri[:, :, 1].mean(axis=1)  # viewer at -Y looking +Y: larger Y is farther
    order = np.argsort(-depth)
    colors = np.clip(np.outer(shade, base_rgb), 0, 1)[order]
    polys = tri[order][:, :, [0, 2]]  # project to (X, Z)
    return polys, colors


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    body_v, body_f = load_obj(ASSETS / "shackle_body.obj")
    pin_v, pin_f = load_obj(ASSETS / "shackle_pin.obj")
    pin_v = pin_v + np.array([0.0, 0.0, PIN_SEATED_Z_M * 1000.0])

    xmin, xmax = body_v[:, 0].min(), max(body_v[:, 0].max(), pin_v[:, 0].max())
    zmin, zmax = body_v[:, 2].min(), pin_v[:, 2].max()
    ear_bottom = body_v[(np.abs(body_v[:, 0]) < 20.5), 2].min()

    # Page extents (mm): room below for the stand and around for notes.
    x0, x1 = np.floor(xmin / 10) * 10 - 40, np.ceil(xmax / 10) * 10 + 50
    z0, z1 = np.floor(zmin / 10) * 10 - 70, np.ceil(zmax / 10) * 10 + 30
    fig = plt.figure(figsize=((x1 - x0) / 25.4, (z1 - z0) / 25.4))  # 1 mm on the page = 1 mm on the part
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(x0, x1)
    ax.set_ylim(z0, z1)
    ax.set_aspect("equal")
    ax.axis("off")

    for x in np.arange(x0, x1 + 1, 5):
        ax.axvline(x, color="#e4e8ee" if x % 10 else "#c9d1dc", lw=0.3 if x % 10 else 0.6, zorder=0)
    for z in np.arange(z0, z1 + 1, 5):
        ax.axhline(z, color="#e4e8ee" if z % 10 else "#c9d1dc", lw=0.3 if z % 10 else 0.6, zorder=0)
    for x in np.arange(x0, x1 + 1, 20):
        ax.text(x + 0.6, z0 + 1.5, f"{x:.0f}", fontsize=5, color="#8a96a8")
    for z in np.arange(z0, z1 + 1, 20):
        ax.text(x0 + 1, z + 0.6, f"{z:.0f}", fontsize=5, color="#8a96a8")

    polys, colors = shaded_tris(body_v, body_f, np.array([0.80, 0.82, 0.85]))
    ax.add_collection(PolyCollection(polys, facecolors=colors, edgecolors="none", zorder=2))
    polys, colors = shaded_tris(pin_v, pin_f, np.array([0.62, 0.74, 0.95]))
    ax.add_collection(PolyCollection(polys, facecolors=colors, edgecolors="none", alpha=0.45, zorder=3))

    ax.plot([0, 0], [zmin - 25, zmax + 15], ls=(0, (6, 3, 1, 3)), color="#c0392b", lw=0.7, zorder=4)
    ax.annotate("pin goes in from above", xy=(0, zmax + 8), xytext=(14, zmax + 18), fontsize=7, color="#c0392b",
                arrowprops=dict(arrowstyle="->", color="#c0392b", lw=0.7))
    ax.text(24, 30, "clear ear (top)", fontsize=7, va="center")
    ax.text(24, -30, "tapped ear (bottom)", fontsize=7, va="center")
    ax.text(24, ear_bottom - 9, "keep clear below the tapped hole:\nthe pin tip ends here (longer if the\nreal pin is longer)",
            fontsize=5.5, va="top", color="#c0392b")
    ax.axhline(zmin, xmin=(xmin - x0) / (x1 - x0), xmax=(xmax - x0) / (x1 - x0), color="#2c3e50", lw=0.5, ls="--")
    ax.text(xmin, zmin - 4, f"lowest point of the bow: z = {zmin:.1f} mm "
            f"(the bottom ear's face is at {ear_bottom:.1f})", fontsize=6, va="top")
    ax.text(x0 + 3, z1 - 4,
            "Shackle side view, bow plane face-on. Scale 1:1, grid 5 / 10 mm, origin at the pin-axis midpoint.\n"
            "Blue: the pin, seated. Dimensions partly estimated (ear gap, pin length); see shackle_params.json.",
            fontsize=6, va="top")
    # 50 mm scale bar to check the print scale
    ax.plot([x1 - 60, x1 - 10], [z0 + 8, z0 + 8], color="black", lw=1.2)
    ax.text(x1 - 35, z0 + 10, "50 mm (check your print)", fontsize=6, ha="center")

    fig.savefig(OUT / "shackle_side_view.pdf")
    fig.savefig(OUT / "shackle_side_view.png", dpi=12 * 25.4)  # 12 px per mm
    print(f"extent x {xmin:.1f}..{xmax:.1f}, z {zmin:.1f}..{zmax:.1f}, bottom ear face {ear_bottom:.1f} mm -> {OUT}")


if __name__ == "__main__":
    main()
