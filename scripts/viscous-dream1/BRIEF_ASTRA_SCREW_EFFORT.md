# Task: make the `screw` skill work with a real (imperfect) operator grasp (no Astra runs)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, same env and hard rules as `BRIEF_ASTRA_SCREW_SKILL.md`. **Astra
episode `astra_g3_20261025_006` may still be running with its own server: do not touch that server, socket,
workspace or output files. Read them only.**

## What happened (Astra episode 6)
Astra used the procedure: aligned the nut at the bolt tip, gripped, caught the thread with
`turn right 30 --feed-mm 0.291667` (engaged), then `screw right 1080`. Three times (commands 156, 374, 606) the
skill returned at once with `stop_reason: stall`, `degrees_turned` -0.6 / -2.2 / about 0, `regrasp_cycles: 0`. In
between, plain `turn` commands with feed did advance the nut (e.g. command 386: 0.08 to 0.21 turns). The skill's
0.15 Nm wrist-effort budget only worked for the perfectly centred validation grasp; with Astra's real grasp
(within the 3 mm / 10 deg entry tolerance, hand not exactly level) holding the wrist pose and the gripper's own
weight needs more than that, so the skill sees "no progress under budget" and calls it a stall.

## Fix
1. Do not use a fixed tiny effort budget as the snug detector. Use what a real nut-runner does: drive the roll with
   the same tracking control `turn` uses (bounded by the real actuator limits), and detect "snug" as rotation
   progress stopping while the wrist roll torque rises clearly above the free-running torque measured during the
   first part of that stroke (for example baseline + 1.5 Nm, sustained for 0.3 s). State the numbers.
2. A stroke that cannot start at all (no rotation in the first stroke while torque is at the arm limit) is still a
   `stall`, but report `stall_at_start: true` so the operator can tell "jammed / misaligned" from "snug after N
   turns".
3. Keep everything else (window, regrasp cycle, feed, stop reasons, 300 s limit, information boundary).

## Validate
1. Replay Astra episode 6's commands 1-155 against a fresh `--profile astra` server, then `screw right 1080`: the
   nut must advance about 3 turns (or report a truthful reason that is not a false stall). Same for 1-373 then
   `screw right 1080`, and 1-605 then `screw right 1080` (the replays may diverge a bit; report what happens).
2. Imperfect-grasp variants of the scripted G3 run: nut gripped 2 mm off centre, approach tilted 6 deg, hand rolled
   to each end of its window: `screw right 3600` runs the nut down and stops `stall` (not at start) when seated.
3. Earlier checks still pass: seated scripted run, released nut does not drift, pull-off releases, empty gripper
   gives `lost_grasp`.
4. Short section at the top of `runs/astra_flange/harness_report.md`; update `README_OPERATOR.md` for
   `stall_at_start`. Leave no server you started running.
