# Task: does Dream-1 know how to scrub a ship propeller? (sim, zero-shot)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`,
`TMPDIR=$PWD/.cache/tmp`. Read first: `SWEEP_README.md`, `runs/viscous_sweep_report.md`,
`runs/viscous_onearm_isolate_report.md`, and the code in `sweep.py`, `sweep_scene.py`, `cross_scenes.py`,
`yam_baseline.py`. Reuse that runner (ledger, chunk fsync/resume, video, contact sheets); add to it, don't fork it.

## Why (Gavin, 2026-10-23)
The Viscous arm is for ROV operators; its showcase job is cleaning ship propellers underwater (see
`~/Documents/viscous-arm-model/cinematic/README.md`: a BlueROV2 Heavy with two Viscous arms polishing a bronze
blade). Gavin: "give it a ship propeller and ask it to clean it. Let's see whether it knows how to do that, or at
least ... do scrubbing motion on it. Let's see what happens." This is exploratory: we want to see behaviour and
videos, not a success rate to two decimals. A later task moves the arms onto the BlueROV2; not this one.

## Where we are
Zero-shot `dreamlabs/dream-1-beta`, vision mode (`other_embodiment`, client IK), Viscous arms with the Maker-style
wrist camera: configuration `B0` (two arms) puts the apple in the bowl 6/6; one arm + a `parked` second arm 6/6.
Dream-1 only acts on objects and prompts it knows, and objects must be about 5 cm or larger (a 2 cm cube was
ignored). The 5 cm box was grasped but never placed.

## Build first (no calls)
1. A new task `propeller` in the existing Dream table scene (same table, top camera, lighting, both Viscous arms,
   configuration `B0`; remove the apple/bowl/other clutter only if they get in the way, and say what you kept).
   - A small marine propeller, bronze/brass coloured, about 18-22 cm diameter, 3 or 4 twisted blades on a hub.
     Generate it procedurally (numpy/trimesh is fine; save the mesh under `assets/`), give it a collision model
     that follows the blades (convex pieces per blade, not one hull), and FIX it to the table (welded, it must not
     move). Mount it the way a person would put one on a bench for cleaning: hub on a short stand with the shaft
     axis horizontal, blades facing the arms, so the blade faces are reachable and visible from the top camera and
     the wrist cameras. Place it centrally in front of the arms, within reach of the left arm at least (check with
     the IK: the tool point must reach the blade faces at a scrubbing orientation).
   - A cleaning tool on the table beside it, graspable by the Viscous gripper: a yellow/green kitchen sponge about
     9 x 6 x 4 cm (light, free body). Optionally also a small scrub brush; say whether you added one.
   - Make it look plausible in the top and wrist images (the model reads only images): show the call-1 contact
     sheet of all three policy images and fix anything unreadable (e.g. propeller looks like a blob).
2. Measurements per executed row, logged like the existing tasks:
   - tool grasped (sponge centre >3 cm above its settled height while between the fingers),
   - tool-propeller contact and finger/gripper-propeller contact (any geom pair, counted once per row),
   - a scrub signal: while the tool (or, with no tool, the fingertips) is within 1 cm of the propeller, the
     number of direction reversals of its motion along the blade surface (project its velocity onto the local
     surface plane; count a reversal when travel > 1 cm in one direction then > 1 cm back), and total path length
     in contact,
   - the closest the requested hand ever came to the propeller and to the sponge.
   There is no "success" stop for this task: run a fixed number of calls per episode (30) unless the run is
   clearly finished (e.g. the arm rests unmoving for 5 calls), and say how many were used.
3. Dry-run one episode per prompt with the local stub (no calls) and check the logs and video are produced.

## Live runs (billed)
Prompts (Dream-1 is a language-conditioned model; try plain phrasing it may have seen):
- `P1` "pick up the sponge and scrub the propeller"
- `P2` "clean the propeller with the sponge"
- `P3` "wipe the propeller" (scene with the sponge still present)
- `P4` "scrub the propeller blade back and forth with the sponge"

Stage A: 2 episodes per prompt, `B0`, seeds paired across prompts (jitter the sponge ±2 cm, as the sweep does).
Stage B: pick the prompt with the most tool-propeller contact rows (ties: most reversals); run 3 more episodes of
it on the Viscous AND 3 episodes of the same prompt with YAM in the same scene (vision mode, as in `yam_baseline.py`
/ `cross_scenes.py`), so we know whether a failure is the task or our arm.

## Hard rules
- Billed: the ledger (`runs/sweep/ledger.json`) has cap 1312 with 1206 used. Gavin's new request authorizes 600
  more calls for this task: raise the cap once, under the ledger lock, to 1912, and record the reason in the ledger.
  Stage A needs at most 4 x 2 x 30 = 240 calls, Stage B at most 180. If anything would exceed the cap, run fewer
  episodes and say so. `pool_warming` (DA-2000) refusals are not billed and may be retried.
- No hardware (no serial, CAN, /dev/video*, systemd). Only write inside `~/dreamtrial/viscous_sim` (TMPDIR there).
  Do not edit installed packages, the Lab repo, `viscous_arm.json` or `viscous_arm_fingertip.json`. Never print
  the API key. Do not touch anything under `~/sonar-recon` or its processes.
- Existing configurations and tasks (`apple`, `box`, all previous modes) must behave exactly as before: rerun the
  existing no-call checks and say they pass.
- Disk: per episode only log.json, chunks.jsonl, run.mp4 (and a contact sheet for the first episode of each
  prompt). Also render, per episode, a run.mp4 that shows the top camera AND the left wrist camera side by side
  (or a second file `views.mp4`), since Gavin will watch these.

## Report (short, to `runs/propeller_report.md`)
- What the scene looks like (one call-1 contact sheet path) and what you kept/removed.
- One table: arm, prompt, episodes, sponge grasped (episodes), rows of tool-propeller contact, rows of
  gripper-propeller contact, reversals in contact, path in contact, closest requested hand to propeller / sponge,
  calls used.
- Per prompt in one or two sentences: what the arm actually did (watch the videos; say if it ignored the
  propeller, poked it, grasped the sponge and carried it somewhere else, rubbed, etc.).
- Whether YAM does it any better (the task vs our arm).
- The best and the worst video paths. Billed calls used. Files added or changed. Say plainly what is measured and
  what is a guess.
