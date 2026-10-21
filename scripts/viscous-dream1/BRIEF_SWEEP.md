# Task: tune Dream-1 on the simulated VISCOUS arm (no fine-tuning): measure success rates over a small sweep

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
Read first: `BRIEF_VISCOUS_MAKERWRIST.md` and your report `runs/codex_viscous_makerwrist_report.md`, plus
`BRIEF_BOX.md`, `BRIEF_IK_REACH.md`. You built `yam_baseline.py --rig viscous` with the Maker-style wrist camera,
`--task box`, chunk logging and analysis.

## Where we are
Viscous + Maker-style wrist camera, zero-shot `dreamlabs/dream-1-beta`, vision mode: the apple went into the bowl in
the one run we did (`runs/viscous_apple_makerwrist_live1`); the 5 x 5 x 3 cm box was targeted but dragged and tipped
(`runs/viscous_box_makerwrist_live1`); the Viscous fingers touched the table on ~40% of rows, because the arm file's
tool point (`viscous_arm.json` `tcp`) is the finger-pad midpoint, not the point between the closed fingertips that
Dream's convention requires. One run per condition says nothing about reliability. Gavin wants to know how to make
his arm work better in sim without fine-tuning, measured as success RATES.

## Build first (no calls)
1. **Episode runner** `sweep.py`: runs N episodes of one named configuration, each a fresh scene with a small random
   jitter of the target object's start position (uniform +-2 cm in x and y, seeded, seed logged), live calls until
   success or a per-episode call cap, then the next episode. **Stop an episode early on success**: the object resting
   at the target (your existing success check) on two consecutive calls. Per episode log: seed, calls used, success,
   call of first success, closest requested-hand-to-object, object-to-target min, grasped yes/no (object lifted
   > 3 cm), finger-table contact rows. Save run.mp4 per episode (no per-call PNGs, keep disk small) and one
   `summary.json` + `summary.md` per configuration. Resumable: re-running skips finished episodes, and the call
   counter persists across restarts. A global call ledger `runs/sweep/ledger.json` enforces the total cap below
   before every request.
2. **Configuration knobs** (all off by default = the current Maker-wrist setup, called `B0`):
   - `tcp=fingertip`: a COPY `viscous_arm_fingertip.json` whose `tcp` is the midpoint between the closed fingertips,
     computed from the sim CAD (finger geometry at grip 0), axes unchanged. Never edit `viscous_arm.json`. (In sim,
     deriving it from CAD is fine; on the real arm Gavin will set it with `dreamcli tcp`.) Show a picture of the old
     and new tool point on the gripper.
   - `arms=single`: the real Viscous setup has ONE arm. Remove the right arm from the scene entirely (not visible in
     any camera); send its state as the parked rest pose and its `wrist_r` frames as black frames; ignore its chunk.
     The working arm stays the left arm at YAM's left base.
   - `rows=24`: execute 24 rows of each 48-row chunk before the next request (instead of 48).
   - `region=table`: use the dream client's `ik` `region` (and/or `reach`) option, read its docs in
     `dream/client_ik.py`, to keep the left hand's targets inside a box over the reachable table area in front of the
     arm (from a few cm above the table to ~30 cm above it, covering the objects and targets). Explain the box.
3. Dry-run every configuration (no calls) and check each knob does what it says (fingertip tool point FK, single-arm
   images, rows executed, region clamp active).

## Sweep (live), in stages; each stage picks the better config by success count (ties: fewer calls to success)
Tasks: `apple` ("put apple into the bowl", per-episode cap 18 calls) and `box` ("pick up the black and white box and
put it on the black square", cap 24 calls). 6 episodes per configuration per task.
- Stage 1: `B0` vs `tcp=fingertip`, both tasks.
- Stage 2: the stage-1 winner with `arms=single`, both tasks (this is about the real setup; keep it even if it is worse,
  and carry the stage-1 winner forward, NOT single, unless single is at least as good).
- Stage 3: the carried config + `rows=24`, and the carried config + `region=table`: box task only.
- Stage 4 (only if budget remains): the best combination from stage 3, both tasks.

## Hard rules
- Billed: the ledger caps the WHOLE task at 900 live calls. Never exceed it; if a stage would, run fewer episodes and
  say so. `pool_warming` (DA-2000) refusals are not billed and may be retried. A crash must not lose the call count.
- No hardware (no serial, CAN, /dev/video*, systemd). Only write inside `~/dreamtrial/viscous_sim` (TMPDIR there).
  Do not edit installed packages, the Lab repo, or `viscous_arm.json`. Never print the API key. Keep existing modes
  working (dry-run them).
- Disk: per episode only log.json, chunks.jsonl, run.mp4 (and a contact sheet for the first episode of each config).

## Report (short)
A table per task: configuration, episodes, successes, grasps, median calls to success, finger-table contact rows.
Which knobs helped, which hurt, the recommended configuration, total billed calls, and the paths of one good and one
bad video per configuration. Files added/changed.
