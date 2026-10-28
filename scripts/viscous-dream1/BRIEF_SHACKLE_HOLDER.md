# Task: 3D-printable holder that keeps the shackle upright (Gavin's design), NO billed calls

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python` (trimesh, manifold3d).
Read first: `runs/shackle_model/shackle_model_report.md`, `assets/shackle/shackle_params.json`,
`shackle_side_view.py` and its output `runs/shackle_side_view/shackle_side_view.png`, and **Gavin's sketch
`shackle/holder_sketch_gavin.png`** (a crop of that side view with his design drawn in blue).

## Why (Gavin, 2026-10-28)
Task 2 of goal B4: the shackle must stand upright on the table (pin axis vertical, clear ear on top, tapped ear
below) so the arm can drop the pin in from above and screw it in, first in sim, then for real. Gavin: "It's just
basically a triangle on that blue annotation there. Of course it's not just a simple triangle: it should also hug
the curvature of the hook ... the bottom should be flat, because it's on the table, and I'm going to use hot glue
to stick this on it." He will hot-glue the shackle to the holder and hot-glue or tape the holder to the table.

## Gavin's design, in the shackle body frame (mm; X right toward the ears, Z up, pin axis at X=0)
His sketch is drawn on the side view (the bow plane face-on, so the bow reads as a C). Claude mapped the blue
triangle with the grid (10 mm = 69.4 px in the crop, pin axis at x = 796 px, the dashed line z = -49.2 at
y = 885 px); **check this against the image yourself**:
- vertical outer face at X ~ -89, from the table (Z = -49.2, the bow's lowest point) up to Z ~ -28;
- flat bottom on the table plane Z = -49.2, from X ~ -89 to X ~ -60.5;
- the hypotenuse runs along the lower-left outside of the bow crown: that face must instead be a **cradle that
  follows the bow's outer surface**, in the bow plane AND across the rod's round section (a saddle), so the
  shackle sits in it and cannot slide sideways or along the curve.

## Build it
- Parametric script `shackle_holder.py`: derive the cradle from the shackle body mesh itself (e.g. the holder's
  solid minus the body offset outward by a glue gap), so it follows any later change to the shackle params. All
  design numbers named at the top: the triangle corners above, glue gap (start at ~0.5 mm; hot glue fills it),
  depth across the bow plane (Y), how far the saddle wraps around the rod (enough to locate it sideways, still
  drop-in from above), corner rounding.
- Depth in Y: wide enough for a solid glue footprint and stability on the table (say 40 mm, centred on the bow
  plane), unless the geometry argues otherwise.
- Printable on its flat bottom with no supports: no overhangs past ~45 deg, minimum wall ~2.5 mm, no knife
  edges (truncate the top tip of the triangle to a small flat), small chamfer on the bottom edges against
  elephant's foot. Watertight, single body.
- Exports to `assets/shackle/`: `shackle_holder.stl` and `shackle_holder.3mf` **in millimetres, in print
  orientation (flat face on Z = 0)** for the slicer; `shackle_holder_sim.obj` **in metres, in the shackle body
  frame** for MuJoCo later; holder dimensions added to `shackle_params.json` (or a `holder_params.json` beside it).

## Check it
- Fit: minimum and maximum gap between the holder and the shackle body mesh over the contact area, no
  interpenetration. The bow crown dimensions are measured (m291, m294, m295), but the bow's exact curve is
  partly an estimate: say how far off it could be and whether hot glue covers that.
- Statics: compute the shackle's centre of mass (stainless steel, ~7.9 g/cm^3, with and without the pin seated)
  and its contact points (the bow's lowest point and the holder). Then report what happens when the arm pushes
  the pin down at X = 0 (say 20 N) and when it applies a screwing torque about the pin axis (say 2 N m): does it
  tip, and how much the hot glue joints must carry. Gavin chose this design; do not redesign it. If it clearly
  will not hold, say so with the numbers and sketch the smallest change (e.g. a second block) as an option.
- The pin path must stay clear: nothing within the pin's bore circle above the top ear or below the tapped ear.

## Deliverables (under `runs/shackle_holder/`)
1. `side_view_holder.png`: the same 1:1 side view as `shackle_side_view.png` with the holder drawn in, to put
   beside Gavin's sketch (render it with `shackle_side_view.py`'s approach; add the holder, do not change the
   existing outputs). Plus a 1:1 PDF.
2. `holder_3q.png`, `holder_with_shackle_3q.png`, `holder_print_orientation.png`.
3. `shackle_holder_report.md` (short): dimensions, glue gap, fit numbers, statics, print settings (orientation,
   no supports, suggested layer height / infill / walls, material: PLA or PETG), and anything Gavin must check
   with the real part before gluing.

## Hard rules
- No billed calls, no Astra or Dream-1 episodes, no hardware. Only write inside `~/dreamtrial/viscous_sim`:
  `shackle_holder.py`, new files in `assets/shackle/`, `runs/shackle_holder/`. Do not modify `shackle_assets.py`
  or the shackle meshes: **another Codex job is adding threads to them right now**; read them, don't write them.
  Another job (`astra_learn/`) may also be running: don't touch it or any sim server.
