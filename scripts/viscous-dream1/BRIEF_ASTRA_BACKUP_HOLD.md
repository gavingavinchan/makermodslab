# Task: make the bolt turn unless the left hand holds its head (ROV scene; no Astra runs)

Work on gavin-linux in `~/dreamtrial/viscous_sim`, same env and hard rules as `BRIEF_ASTRA_UNDERWATER_SCENE.md`.
Changes go behind a flag and into a new profile, so `astra_rov` and the tabletop profiles stay reproducible.

## Why (Gavin, 2026-10-26)
Astra seated the nut underwater on its first try (`astra_rov_g3_20261026_001`), with the left arm idle. Gavin:
"have the left arm also grab the bolt as well? Normally a human grabs the bolt and turns the nut on the other
side." In the sim the bolt is currently held rigidly by the fixture, so a left-hand hold would be decoration.
Make it physically necessary, as with a real loose bolt in clearance holes: the bolt can turn about its own axis,
so turning the nut spins the bolt unless the head is held (a backup wrench).

## Changes (`--bolt-free-spin`, new profile `astra_rov_backup` = `astra_rov` + this)
1. In G3 the bolt keeps its axial position (head side constrained as now) but gets a rotational degree of freedom
   about its own axis, with a small, documented rotational friction in the holes (lower than the thread drag, so
   that an unheld bolt co-rotates with a turning nut and the nut does not advance). State the values and why.
2. The thread model works on the **relative** rotation of nut and bolt (advance = pitch x relative turns). Check
   the seat stop and the `screw` snug detector still work on relative motion.
3. The left gripper can hold the bolt head across two opposite hex flats (the same contact-gated grasp weld as the
   nut), which stops the bolt turning. Check the left arm reaches the head with a comfortable pose, clear of the
   pipe, flange studs and the ROV, with the right arm working at the nut at the same time (no arm-arm contact).
4. Log bolt rotation (turns) in the hidden `episode.jsonl` next to nut turns, and whether the left hand holds the
   bolt.
5. `README_OPERATOR.md` for this profile: the bolt can turn in its clearance holes; like a technician with a
   backup wrench, hold the bolt head with the left gripper (across its flats) while turning the nut, otherwise the
   bolt may spin with the nut. Say on which side the head is, in words. No object coordinates.
6. Video: add the left wrist camera to `run.mp4`, four panes: ROV external, flange close-up (framed so both the
   head side and the nut side are visible if possible, otherwise the nut side), right wrist, left wrist.

## Validate
1. Unheld: from the validated start, catch the thread and run `screw right 720` with the left hand off: the bolt
   co-rotates and the nut barely advances (report both turn counts).
2. Held: left hand grips the bolt head, then the scripted CLI G3 (privileged targets only to place things, marked
   as a cheat) seats the nut with `screw`, bolt rotation under a few degrees, `stall` at seat, no arm/ROV or
   arm/arm intersections. Video `runs/astra_flange/validation/rov_backup_g3/run.mp4`.
3. `astra_rov` and tabletop profiles unchanged (their scripted G3 still pass).
4. Short section at the top of `runs/astra_flange/harness_report.md` with the launch command. No server left
   running.
