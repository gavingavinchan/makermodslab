# Task: make the Viscous MuJoCo scene match the real bench, for a Dream-1 zero-shot test

You are working in `~/dreamtrial/viscous_sim` on gavin-linux. Python env: `~/dreamtrial/.venv/bin/python` (mujoco 3.x,
the `dream` client, numpy, PIL, opencv). Render offscreen with `MUJOCO_GL=egl` (already the script's default).

## Context
`viscous_dream_sim.py` drives a simulated Viscous arm (6 joints + gripper) with Dream Labs' Dream-1 model, through
their `dream` client and its client-side IK (arm file `viscous_arm.json`). Dream-1 is a two-arm model, so the Viscous
arm plays the LEFT arm; the right arm is virtual (held at rest, black `wrist_r` camera, answers dropped). Dream-1 sees
two real camera roles: `overhead` (the top-down camera) and `wrist_l` (the wrist camera). The goal is a first sim test
whose camera images look as much like the real bench as practical, because Dream-1 is a vision model trained on real
images. Read the script's docstring and code first.

The scene loads `~/Documents/viscous-arm-model/demo/pick_place_scene.xml` (CAD-derived Viscous model; arm base at the
world origin, x forward, y left, z up; `tool0` = midpoint of the finger pads, +Z toward the fingertips, +X jaw opening)
and adds cameras and colours through `mujoco.MjSpec` in `build_model()`.

## Real reference images (640x480, 4:3), in `real/`
- `real/topdown_cam_index0.jpg`: the real TOP-DOWN camera. Grey speckled desk, the gripper jaws enter from the left
  edge, a black square outline (the place target, roughly 4-5 cm) on the desk. The camera sits to the arm's right.
- `real/wrist_cam_index2.jpg`: the real WRIST camera, looking from behind the two jaws toward the fingertips (wide
  angle). Note: the file names reflect what each image SHOWS; the robot record's camera labels are swapped.
- `runs/preview/` holds the current sim renders (`--preview` regenerates them) for comparison.

## What to do
1. Re-pose the sim `overhead` camera (position, aim, fovy, image size/aspect = 640x480) so its view matches
   `real/topdown_cam_index0.jpg` as closely as you can: where the gripper appears, scale, viewing angle.
2. Re-mount the sim `wrist` camera on the gripper (tool0 or wrist_roll_link frame) so its view matches
   `real/wrist_cam_index2.jpg`: jaw placement in frame, apparent jaw size, field of view. 640x480.
3. Make the scene resemble the real task, "pick up the white cube and place into black square": grey speckled tabletop
   (a texture is fine), a ~2 cm WHITE cube, a thin BLACK square outline target on the table (no cup), reasonable
   lighting (not washed out). Keep the cube reachable and graspable by the sim gripper; keep the arm's rest pose.
   Update the default `--task` to "pick up the white cube and place into black square" and the success check to
   "cube resting inside the square".
4. Keep the third-person camera for humans (it is never sent to Dream).
5. Validate:
   - `python viscous_dream_sim.py --preview --out runs/astra_preview` renders cleanly.
   - Make side-by-side comparison images (real | sim) for both cameras, saved in `runs/astra_preview/`.
   - The DRY RUN still passes: `python viscous_dream_sim.py --out runs/astra_dry` (no `--live`). It must print
     "3 calls (none billed)" with 0 limit_rows and 0 clipped_rows.

## Hard rules
- NEVER pass `--live`, and never call Dream's network API in any other way. Live calls cost money; the user decides
  when to make them.
- Do not touch hardware: no serial ports, no CAN, no /dev/video*. The two real stills are all you get.
- Do not modify anything outside `~/dreamtrial/viscous_sim` (in particular `~/Documents/viscous-arm-model` is
  read-only reference). Do not change `viscous_arm.json` or the IK/control logic unless a change is required, and if
  so explain why.
- Keep edits in `viscous_dream_sim.py` (or a new scene helper module beside it); keep CLI flags working.

## Report (final message)
What you changed, the final camera parameters, how close each comparison is and what still differs, the dry-run
output lines, and file paths of the comparison images.
