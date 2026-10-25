# Task: add a `screw` skill to the Astra robot interface (no Astra runs)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, same env and hard rules as `BRIEF_ASTRA_THREAD_GATE.md` (write only
in `astra_op/`, `runs/astra_flange/`, `~/dreamtrial/astra_ws`; no Dream calls, ledger, runner lock, hardware,
propeller files or original flange sources). Do not change the physics, the thread model or the start states.

## Why (2026-10-25)
Astra episodes 3 and 4 used all 600 commands. In episode 4 the thread fixes worked: Astra aligned the nut and caught
the thread several times (max 0.25 turns), but lost it during the many manual regrasp cycles the finite wrist roll
forces (turn, open, unwind, close, without disturbing the nut). The nut came off, fingers wedged, and commands ran
out. See `runs/astra_flange/astra_g3_20261025_004*` (episode.jsonl, `_astra.log`). A real manipulator controller
would offer that cycle as one skill; perception and decisions stay with the operator, so the episode is still fully
autonomous.

## The skill
`./robot screw <arm> <degrees> [--rate-deg-s R]` (positive = the operator README's tightening direction):
1. With the gripper closed on the part, roll the wrist in the requested direction with the matching axial feed
   (`degrees * 3.5 / 360` mm along approach, same convention as `turn --feed-mm`), as far as the wrist's roll range
   allows with a safety margin.
2. At the end of the usable roll: open the gripper just enough to release (check actual opening and zero bilateral
   load), roll the wrist back, close again on the part (contact-stopped, same preload as `grip close`), and continue.
   The hand position should follow the nut's axial advance (keep the tool point where the nut is, using the
   commanded feed), never by reading object state.
3. Stop when the requested angle is done, or on: a stall (the wrist cannot turn: torque saturation or no rotation
   progress, e.g. the nut is snug), lost grasp (close finds no part), or a regrasp that leaves a load on open. Return
   the degrees actually turned while gripped, the number of regrasp cycles, the stop reason, and the final pose.
4. Use only what a real controller has: joint angles, commanded motion, jaw opening and jaw loads, joint torques.
   No object poses, no thread state, no success detector. Log in the hidden `episode.jsonl` as usual.
5. One `screw` call counts as one command, with the 90 s wall limit raised for it to 300 s (progress lines every
   ~10 s as now).

## Also
- `--profile astra`: raise `--command-cap` to 1000 and `--wall-seconds` to 21600 (6 h).
- `README_OPERATOR.md`: document `screw` with an example, say that it does the regrasp cycles for you and needs the
  nut started on the thread and gripped first, and that its stop reason tells you what happened (a stall after
  several turns probably means the nut is snug).

## Validate
1. Scripted CLI operator (privileged targets allowed only to place the nut, marked as a validation cheat) from the
   astra G3 start: align at the tip, `grip close`, `turn` 30 with feed to catch, then a single `screw right 3600`:
   the nut runs down and the skill stops with a stall when seated. Report turns, regrasp cycles, stop reason,
   seated, and video `runs/astra_flange/validation/screw_skill/run.mp4`.
2. `screw` on an empty gripper or with the nut off the thread stops cleanly with the right reason.
3. Released-nut and pull-off checks from the earlier briefs still pass.
4. Short section at the top of `runs/astra_flange/harness_report.md`. Leave no server you started running.
