# Task: re-render Astra's successful flange run underwater on the BlueROV (render only, no new runs)

Work on gavin-linux in `~/dreamtrial/viscous_sim` (venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`,
`TMPDIR=$PWD/.cache/tmp`). Put new code under `astra_op/underwater/` and outputs under
`runs/astra_flange/astra_g3_20261025_006/underwater/`. No Astra, Dream-1 or ledger use; no hardware. Do not
edit the original flange/propeller sources, `viscous_arm.json`, or anything under
`~/Documents/makermodslab-viscous/scripts/viscous-dream1/cinematic/` (another session's work; read it freely).
The Lab repo on gavin-linux is `~/Documents/makermodslab-viscous`; do not commit there.

## Why (Gavin, 2026-10-25)
Astra (gpt-6-astra) bolted the flange fully autonomously in episode `astra_g3_20261025_006`: six attempts to
start the nut, three dropped nuts, no environment reset, nut finally run down snug. Gavin wants to post it, but
underwater: "remove the desk at the bottom, put the blue ROV there, make it look underwater". He chose to
**re-render this exact run** (not run Astra again), so the motion must be Astra's recorded motion and the video
will be labelled as a replay.

## Inputs
- `runs/astra_flange/astra_g3_20261025_006/episode.jsonl`: physics rows every 50 ms with `qactual` (both arms:
  6 joints + gripper), `nut_xyz`, `object_quat` (nut and bolt), `bolt_xyz`, `time`. Pipe and flanges are static
  (from the flange scene, `--profile astra` start).
- The existing social cut and its editor: `runs/astra_flange/astra_g3_20261025_006/astra_flange_social.mp4` and
  the script `/tmp/social_edit.py` (also in the Lab repo next to the video as `social_edit.py`). Copy it into
  `astra_op/underwater/` and adapt; keep its beats, speed ramp (16x before t=1138 s, 6x after), panel layout and
  text style.
- The BlueROV2 Heavy with its two native Viscous arms: `~/Documents/viscous-rov-model` (URDF
  `model/viscous_rov.urdf` and meshes), and the other session's verified notes on it in
  `~/Documents/makermodslab-viscous/scripts/viscous-dream1/cinematic/README.md` (native mounts 493.05 mm apart,
  vehicle must be level: zero pitch and roll, native mesh transforms unchanged).

## The scene
1. No table. A sandy seabed at the old table-top height, so the three nut drops land on the seabed where they
   landed on the table. The pipe run keeps its supports (pipeline supports on the seabed). Pipe, flanges, studs,
   bolt and nut keep their geometry and materials (maybe light marine fouling on the pipe, nothing on the flange
   faces, holes or fasteners).
2. The BlueROV2 Heavy, level, resting on (or hovering just above) the seabed in front of the flange, with the two
   Viscous arms on their native mounts. All 8 thrusters, frame, buoyancy foam, camera dome as in the model.
3. Underwater look: blue-green colour and depth fog (MuJoCo `visual/map` fog and `rgba/fog`, or equivalent), soft
   top-down light falloff, darker background, optionally sparse marine snow (static small particles). Must stay
   readable: nut, bolt tip, jaws and threads clearly visible in the close-up and wrist views.

## The motion (the key constraint)
The tabletop arm bases are not 493.05 mm apart, so recorded joint angles cannot simply be copied onto the ROV.
Keep Astra's **world-frame trajectory** exactly: the nut, bolt and both tool (hand) poses and gripper openings
per row, in the frame of the flange. Place the ROV (level) so both native mounts can reach the recorded hand
poses, and solve each arm's joints by IK on the native kinematics to track the recorded tool pose every row
(continuity from the previous row; same approach/jaw axes; gripper opening from the log). Report the max and 95th
percentile position/orientation tracking error, rows that fail, and how you handled them. If a fully level pose
cannot reach everything, say which rows and propose the least bad option (do not silently pitch the vehicle).
Check intersections (arms vs ROV, arms vs pipe/flange, ROV vs seabed and pipe) at the sampled frames and report.

## Rendering
Render only the source times the cut uses: [0, 1138) s sampled every 16/30 s, and [1138, 1406.6] s every 6/30 s
(interpolate between 50 ms rows). Three views, each 960x540 so the grid is 1920x1080:
- **wide**: an external shot showing the whole ROV on the seabed working at the flange (replaces "TOP"),
- **close-up**: the flange joint inspection view (same framing idea as before),
- **right wrist**: the right arm's wrist camera, rigid to the hand as before.
Label each pane small in the corner. Fourth pane: the commentary panel from `social_edit.py`, scaled to 960x540,
with these text changes: "on the table" becomes "on the seabed"; the footer becomes
"Simulation replay  |  rendered underwater  |  sped up". Output H.264 yuv420p 1920x1080 30 fps
`astra_flange_underwater.mp4`, no title or end cards, last frame held 2.5 s.

## Validate and report
A contact sheet of 8 frames across the video, the tracking-error numbers, the intersection check, and a short
`runs/astra_flange/astra_g3_20261025_006/underwater/README.md`: what is replayed exactly (object and hand
trajectories), what is re-solved (arm joints on the native mounts), and what is cosmetic (water look, seabed, ROV
not simulated hydrodynamically).
