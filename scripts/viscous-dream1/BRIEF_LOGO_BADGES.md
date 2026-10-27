# Task: put the real Viscous Robotics badges on the arms in the two video scenes (no video renders)

Work on gavin-linux. Blender work in `~/Documents/makermodslab-viscous/scripts/viscous-dream1/cinematic/`
(use the Blender/venv that `cinematic/README.md` and `render.py` already use); sim work in `~/dreamtrial/viscous_sim`
(venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`, `TMPDIR=$PWD/.cache/tmp`). No hardware, no Dream-1 or
Astra calls, no ledger. Do not commit; leave the git commit to the parent session.

## Why (Gavin, 2026-10-26)
The physical Viscous arms carry 3D-printed brand badges, but the arms in our videos don't. Update the CAD assets of
two videos so the badges are there:
1. **Ship propeller cleaning film** (Blender): `cinematic/results/final_scene.blend` (built from
   `cinematic/assets/native_rov.blend` by `build_scene.py`), the BlueROV with both Viscous arms.
2. **Underwater pipe-flange nut-on-bolt video** (MuJoCo): the `--scene rov` scene in `~/dreamtrial/viscous_sim`
   (profiles `astra_rov`, `astra_rov_backup`, `astra_rov_backup2`); the latest cut is
   `runs/astra_flange/astra_rov_backup2_g3_20261026_001/astra_rov_twohand_social.mp4`.

**Do not render either video.** Gavin will open the Blender scene by hand on this machine and pick the camera views
himself before rendering. So: **do not touch cameras, camera markers, the active camera, render settings, lights,
the motion/animation or the edit.** Only add geometry and materials. Low-resolution still images for checking the
placement are fine and expected (Eevee or few-sample Cycles, separate check cameras that you delete again, or
viewport/OpenGL renders; never written into `results/frames/` or over any existing deliverable image).

## The badges (source of truth: `~/Documents/viscous-arm-branding/`)
Per arm, four parts, one nameplate + one V on each side (`PosY_output` side and `NegY_bearing` side):
- Nameplates, "VISCOUS / ROBOTICS": `2026-10-06-print-files/parts/nameplate_{PosY_output,NegY_bearing}.step|.stl`
  (`_colored.step` carries the blue/white split). Mounted look: `2026-10-06-print-files/Mounted_*_Side.png`.
- V badges: the **Oct 9 "extended" variant** (Gavin: "the latest one"), which replaced the Oct 6 V on the far
  elbow joint: `2026-10-09-v-print-variants/parts/v_extended_{PosY_output,NegY_bearing}.step|.stl`, sitting on the
  circular aluminium end directly beside the nameplate, fixed by the two extra holes 32 mm apart.
  Mounted look: `2026-10-09-v-print-variants/Mounted_Extended.png`. **Do not use** `v_badge_*` (Oct 6) or
  `v_circle_*`.
- Part files are in **print frame** (flat back on Z=0). The mounted placement in the arm CAD frame is in the build
  scripts (`2026-10-06-print-files/build_badges.py`, `2026-10-09-v-print-variants/build_v_variants.py`,
  `render_variants.py`, `preview_source/badges.json`, `preview_source/arm.json`; source arm CAD
  `~/Documents/viscous-arm-model/source/2026-09-18/maker arm cnc装配.step`, URDF
  `~/Documents/viscous-arm-model/model/viscous_arm.urdf`). Recover the exact print-to-arm transforms from those
  and reproduce the `Mounted_*` renders before going further.
- Colours: the blue body is brand blue `#2D6BFF` in matte/satin PLA (not glossy, not metallic), raised glyphs white
  PLA. Blue through Z=4.6 mm, white 4.6-5.2 mm, i.e. split by height in print frame if you start from the STL.

## Placement on the video arms
The arm meshes in both scenes come from the ROV model (`~/Documents/viscous-rov-model`), not directly from the
arm CAD, and the right arm may be a mirrored instance. So:
1. Find the link that carries the nameplate and V (the bracket/plate the badge screws into), in each scene, for
   **both arms**. Register the arm-CAD frame to each scene's link frame by matching that link's own mesh (e.g.
   vertex/ICP fit of the plate the badges sit on, residual reported in mm), not by guessing offsets.
2. Parent each badge to that link so it moves with the arm through the whole animation/episode.
3. **Text must read correctly on every side of both arms.** If an arm instance is mirrored (negative scale or
   mirrored mesh), do not mirror the badge with it: use the nameplate/V of the matching handedness and make sure
   no badge ends up with a negative-determinant transform. "VISCOUS ROBOTICS" mirrored anywhere is a failure.
4. Badges sit flat on the aluminium with no gap and no visible z-fighting, and match the `Mounted_*` reference
   renders in position and orientation.

## 1. Blender (propeller cleaning)
- Write `cinematic/add_logo_badges.py`: idempotent (re-running replaces, never duplicates), imports the four parts
  per arm into a `LOGO_BADGES` collection, two materials `BADGE | blue PLA` and `BADGE | white PLA`, parented as
  above. Apply it to `cinematic/assets/native_rov.blend` and to `cinematic/results/final_scene.blend`, saving both
  in place (keep the previous `final_scene.blend` as `final_scene.prelogo.blend` next to it, outside git:
  add it to `cinematic/.gitignore`). If `build_scene.py` would drop the badges on a rebuild, add the call so a
  rebuild keeps them; do not rebuild the scene now.
- Packed data only (Gavin opens the file by hand): no external file paths the .blend needs at open time.
- Check stills: for each arm, close views of both sides at 2-3 animation frames (start, middle, end), plus one
  wide view at a few frames, from temporary check cameras. Put them in `cinematic/results/logo_check/` with a
  contact sheet. Then confirm the scene's camera objects, markers, active camera, frame range, render settings and
  every pre-existing object's transforms/animation are unchanged (diff against the backup and write it to
  `logo_check/unchanged.json`).
- Interference: at every film frame (or every 2nd), check the badges against all other arm links and the ROV and
  report frames where a badge intersects anything (the real V limits J4 rearward travel, so contact is possible).
  Report only; do not change the motion.

## 2. MuJoCo (flange underwater)
- Add the badges as **visual-only geoms** (`contype=0 conaffinity=0`, zero mass/density, `group` of the existing
  visual meshes) on the correct bodies of both arms in the `--scene rov` scene, for all `astra_rov*` profiles.
  Physics must stay bit-identical: same `nbody`, same collision geoms, same mass/inertia, and replaying the first
  2000 rows of `astra_rov_backup2_g3_20261026_001` commands, or stepping the scripted held G3 under
  `astra_rov_backup2`, gives identical joint states before and after. Two colours as above (blue body, white
  glyphs as separate geoms or a two-material split).
- Put new code in `astra_op/`; do not edit `viscous_arm.json` or the original flange/propeller sources. Check that
  tabletop `--profile astra` is unaffected.
- Check stills: the ROV camera view plus one external view per arm side at the episode's start pose and at the
  seated pose, in `runs/astra_flange/logo_check/` with a contact sheet. Do not re-render the episode video.

## Report
`cinematic/results/logo_check/README.md` and `runs/astra_flange/logo_check/README.md`: which files changed, the
transforms and registration residuals, the mirroring handling, the interference findings, the unchanged-scene
proof, and anything you could not do. Copy the MuJoCo check sheet and README into the Lab repo at
`scripts/viscous-dream1/runs/astra_flange/logo_check/` so it can be committed. Leave no Blender or sim process
running.
