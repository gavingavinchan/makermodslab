# Task: 3D model of Gavin's bow shackle and screw pin (model only, NO sim run, NO billed calls)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python` (has trimesh and
manifold3d; `/usr/bin/blender` is available for renders). Model the parts the same way the flange parts were
built: read `flange_assets_v2.py` first and follow its style.

## Why (Gavin, 2026-10-27)
This is task 1 of goal B4 (Lab repo `docs/shackle-sim2real.md`). Later tasks: a printed holder that keeps the
shackle standing vertically, a MuJoCo scene, then Astra picks up the pin, inserts it down through the top
(clear) ear and screws it into the bottom (threaded) ear, then the same on the real Viscous arm (sim to real).
So the model must be dimensionally faithful: the real part will be put in front of the real arm later.

## The part (photos in `shackle/photos/`, open every one before modelling)
A stainless steel **bow (anchor) shackle** with an **eye-head screw pin**. One ear has a clearance hole, the
other is tapped. The pin has a threaded end, a plain shank, a collar/shoulder, and a flat tab with a small
cross hole at the head. `m288` shows the pin standing upright on its threaded end on the table (that is
how it will start in the task). `m289`, `m296` show the overall shape.

Caliper readings, mm. "What it measures" is Claude's reading of each photo: **check each against the photo
yourself**, and say where you disagree. Several displays are upside down or sideways.

| photo | reading | what it measures (best guess) | confidence |
|---|---|---|---|
| m283 | 7.92 | pin head tab: thickness or cross-hole diameter (upside-down display) | low |
| m284 | 27.68 | pin head seen end-on: collar/shoulder diameter, or tab width | medium |
| m285 | 9.19 | pin head tab: thickness, or the other of m283 | low |
| m286 | 24.05 | pin head length along the axis, collar shoulder to tab tip (upside-down display) | medium |
| m287 | 22.23 | pin shank diameter (7/8" = 22.2) OR threaded length (upside-down display) | medium |
| m290 | 79.85 | assembled, along the pin axis: outer face of one ear to the outer face of the other / collar | medium |
| m291 | 19.41 | bow rod (body) diameter | high |
| m292 | 40.07 | ear boss outer diameter, looking down the pin axis | high |
| m293 | 19.69 | ear boss thickness, or bow diameter near the ear | medium |
| m294 | 118.71 | overall length: bow crown to the outside of the pin's ear | high |
| m295 | 98.5x | bow maximum outer width (sideways display) | medium |

These numbers look like a 3/4" (19 mm) bow shackle with a 7/8" pin. Cross-check against a published
dimension table for that size (US Federal / G-209-style bow shackle, or the metric stainless equivalent) to fill
what was not measured (inside width at the pin, inside length, thread length, pin total length) and to decide
the ambiguous rows. Where the table and the photos disagree, the photos win. **Thread pitch** is unknown:
estimate it by counting crests in `m286`/`m287`/`m288` against a known length (7/8"-9 UNC = 2.82 mm and
M22x2.5 are the likely candidates) and say how sure you are.

## Model
- Parametric Python script `shackle_assets.py`: every dimension is a named constant at the top with a comment
  saying where it came from (photo id, table, or estimate). Gavin will send corrections, and a correction must be
  a one-line edit.
- Two separate bodies: **shackle body** (bow + two ears, the clear hole in one ear, the tapped hole in the other
  as a plain bore at the minor/pitch diameter; real threads are not needed) and **pin** (threaded section can be
  a plain cylinder at the major diameter, plus shank, collar, tab with cross hole). Smooth shading, proper
  normals, watertight meshes.
- Frame convention: millimetres in the parameter file, metres in the exported meshes (MuJoCo). Shackle origin
  at the pin axis midpoint between the ears, pin axis = +Z with the **clear ear on top, tapped ear below** (the
  task orientation), bow extending along -X. Pin origin at the tip of the threaded end, axis +Z toward the head.
- For MuJoCo collision later, also export a convex decomposition (or a set of primitive approximations) of the
  shackle body that keeps both holes open with real clearance, and say the clearance you used.
- Export to `assets/shackle/`: `shackle_body.stl`, `shackle_pin.stl` (visual), the collision parts, and
  `shackle_params.json` (all dimensions, mm, with sources).

## Deliverables (under `runs/shackle_model/`)
1. `compare_*.png`: renders of the model from roughly the same viewpoints as `m289`, `m296` and `m288`, each
   placed side by side with the photo, so Gavin can judge the shape at a glance. Plus `assembled.png` (pin
   seated through both ears) and `exploded.png`.
2. `dimension_check.md`: for every caliper reading, the same measurement taken on the model (state how), the
   difference, and your call on what each photo measures.
3. `shackle_model_report.md` (short): the size you identified and why, every estimated (not measured)
   dimension, the thread pitch estimate, and **the exact list of measurements Gavin should take** to remove the
   remaining guesses (tell him which jaw of the caliper and where to put it).
4. Grip check: the Viscous gripper's maximum jaw opening (from `viscous_arm.json` / the MJCF) versus the pin
   shank, collar and tab, and versus the shackle body. Say which features the gripper can hold.

## Hard rules
- No billed calls, no `--live`, no Astra or Dream-1 episodes. No hardware (no serial, CAN, /dev/video*,
  systemd). Only write inside `~/dreamtrial/viscous_sim` (new files only: `shackle_assets.py`,
  `assets/shackle/`, `runs/shackle_model/`, `shackle/`). Do not edit existing scenes, scripts or assets.
- Do not build the holder or the MuJoCo scene yet: those are the next tasks.
- If a reading cannot be reconciled with the photos, do not invent a number: use the table value, mark it
  `ESTIMATE` in the params, and put it on the list for Gavin.
