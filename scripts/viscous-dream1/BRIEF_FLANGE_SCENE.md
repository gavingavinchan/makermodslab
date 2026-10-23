# Task: build a pipe-flange bolting scene for Dream-1 (scene only, NO billed calls)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`,
`TMPDIR=$PWD/.cache/tmp`. Read first: `SWEEP_README.md`, `BRIEF_PROPELLER.md`, `runs/propeller_report.md`, and
`propeller_scene.py`, `cross_scenes.py`, `sweep.py`: build this the same way the propeller task was built (a new
task in the same runner, same table, cameras, lights and `B0` arms), but do NOT run any live episode.

## Why (Gavin, 2026-10-23)
The propeller trial (Dream-1 carried a sponge to a propeller but never scrubbed) "looks bad". Gavin's next task:
"two pipes with flanges, and then the task: insert and screw in the nut". He wants to SEE the environment before
anything is run. This brief is only the scene, its physics and a scripted feasibility demo; Dream-1 comes later.

## The scene
- Two short steel pipe sections lying end to end along the table's left-right axis, centred between the two
  Viscous arms and in front of them, each on a simple stand/V-block so they sit at a comfortable working height.
  Their flanges meet face to face in the middle with the bolt holes aligned (welded-neck or slip-on flange shape,
  bolt circle with 4 holes). Fixed to the world, they must not move.
- A loose hex bolt and a hex nut on the table, within reach, in front of the flanges (free bodies).
- Sizes: real proportions (say which DN/PN and bolt size you modelled), but Dream-1 ignores objects as small as
  2 cm, so the nut must be at least ~4.5 cm across flats and the bolt head the same; scale the flange to suit.
  Everything the task needs (hole on the arm side, bolt, nut) must be graspable by the Viscous gripper (max jaw
  opening: check `viscous_arm.json` / the MJCF) and reachable at a sensible orientation: prove it with the native
  IK (tool point at the bolt, at the hole entrance with the bolt axis aligned, at the nut on the far-side bolt end).
  If one arm can't do insert-and-nut alone, place things so the left arm inserts the bolt and the right arm can
  reach the far end for the nut, and say so.
- Look: Gavin found the propeller scene ugly. Make this one look like real hardware: steel/galvanized pipe and
  flanges (painted flanges are fine, e.g. a typical industrial blue or grey), bright zinc bolt and nut, proper
  normals, smooth shading, hex faces visible, bolt holes visibly open. Keep the table, lights and cameras as in the
  existing scenes (the model was trained on that station look) unless something reads badly; if you change light
  or materials, say what and why.

## Threads (physics)
MuJoCo cannot simulate threads directly. Implement a simple, documented engagement model: when the nut's axis is
within ~10 deg of the bolt axis and its centre within ~3 mm of the axis near the bolt tip, constrain the nut to a
screw motion on the bolt (rotation coupled to advance at the real thread pitch, e.g. a hinge + slide with an
equality coupling, or an equivalent); release it if it is pulled back off the end. Same idea for the bolt in the
holes: the holes are real clearance holes (collision geometry), so insertion is real contact; no snapping.
Log per row: bolt inserted depth through both flanges, nut engaged (yes/no), nut turns and advance, plus the
grasp/contact signals the propeller task logs.

## Feasibility demo (scripted, no Dream-1)
Write a scripted controller (IK waypoints, no inference) that does the task with the Viscous arms in this scene:
pick the bolt, insert it through the aligned holes, pick the nut, start it on the bolt and turn it a few turns
(wrist roll regrasp cycles are fine). If the full thing is too hard in this pass, get as far as you can and say
exactly where it fails and why. It proves the scene is physically doable; it is not a benchmark.

## Deliverables (all under `runs/flange/`)
1. `overview_*.png`: 2-3 nice 1280x720 renders of the setup (front three-quarter, top, a close-up of the flange
   joint with the bolt and nut visible).
2. `contact_sheet.jpg`: the three call-1 policy images (top, wrist_l, wrist_r) at the resolution sent to Dream-1.
3. `scripted_demo.mp4`: H.264 yuv420p, top + a close-up camera side by side, of the scripted demo.
4. `flange_scene_report.md` (short): dimensions, materials, the thread model, the IK reach results, how far the
   scripted demo got, and what still looks wrong.
5. A dry run of one episode through the runner with the local stub (no calls) to show the task plugs in.

## Hard rules
- **No billed calls.** Do not use `--live`, do not touch the ledger. No hardware (no serial, CAN, /dev/video*,
  systemd). Only write inside `~/dreamtrial/viscous_sim`. Do not edit installed packages, the Lab repo,
  `viscous_arm.json` or `viscous_arm_fingertip.json`. Never print the API key. Do not touch `~/sonar-recon`.
- Existing tasks (`apple`, `box`, `propeller`) and modes must behave exactly as before: rerun the existing
  no-call checks and say they pass.
