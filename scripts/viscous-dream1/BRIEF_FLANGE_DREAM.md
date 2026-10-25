# Task: get Dream-1 to bolt the flange (sim, zero-shot)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, same venv/env as before. Read first: `BRIEF_FLANGE_LOOK.md`,
`runs/flange_v2/flange_v2_report.md`, `runs/propeller_report.md`, `SWEEP_README.md`, and the flange code
(`flange_scene.py`, `flange_demo.py`, the `flange` task in `sweep.py`). Use the v2 scene as it is (Gavin approved
it: "good"); do not rotate the pipe run.

## Why (Gavin, 2026-10-24)
"Now get Dream-1 to do the task. If too difficult you can start the task with the [bolt] and nut already in the
grippers." The task: push the loose bolt through the empty lower hole on the arm side of the flange pair, and
screw the nut onto its far end. He will watch the videos, so make them good.

## Part 1: grasps that hold (no calls)
The v2 scripted pickup failed: the MJCF gripper is a +-10 N placeholder (never measured on the real arm), friction
capacity ~6.2 N vs the 650 g bolt's 6.4 N weight, and the lift saturated the arm. Fix holding in this order, and
say which you used:
1. A physically derived gripper force: estimate the real Viscous jaw force from the RS00 rating (5 Nm continuous)
   and the gripper's transmission in the CAD/MJCF (crank/rack geometry), and set the sim gripper's force limit to
   that, ONLY in the flange task's scene (other tasks keep the placeholder). Check `lerobot`/Lab limits do not
   matter here (sim only).
2. If held parts still slip or the arm still saturates: a documented grasp-attach model in the flange task only:
   when both jaws touch the part and the gripper command is closing, weld the part to the hand at its current
   relative pose; release the weld as soon as the gripper opens past a threshold. Log every attach/release.
Then add a start option `--start pregrasp`: the bolt held in the LEFT gripper (by its head end, shank pointing
forward along the approach, ready to insert), the nut held in the RIGHT gripper (bore along the approach, face
forward), both arms at a comfortable pose in front of the flange, clear of it (not already at the hole). The
default `--start` keeps the parts on the table as now.
Scripted feasibility (no calls), from `pregrasp`: left inserts the bolt through both rings, right brings the
nut to the far end, engages it and turns it at least 2 turns (wrist-roll cycles with regrasp are fine). Show
it in a video. If this cannot be made to work, stop before Part 2 and report why.

## Part 2: Dream-1 (billed)
Zero-shot `dreamlabs/dream-1-beta`, vision mode, Viscous `B0`, 30 calls per episode (stop early if the nut has
done >= 2 turns, confirmed at two consecutive call endpoints). Paired seeds (jitter loose parts / pregrasp hand
pose slightly as the sweep does).
Stage A (2 episodes each):
- `T` table start, prompt "put the bolt through the hole in the flange and screw the nut onto it"
- `G1` pregrasp start, same prompt
- `G2` pregrasp start, "insert the bolt into the flange"
- `G3` pregrasp start, "screw the nut onto the bolt"  (for G3 only, start with the bolt ALREADY inserted
  through both rings and held there by the fixture, and the nut in the right gripper: this tests threading alone)
Stage B: 2 more episodes of the best Stage-A condition (by insertion depth, then nut engagement/turns), and 2
episodes of YAM (vision mode, as in the propeller task) in that same condition, so we know task vs arm.

Log per row (already mostly there): bolt insertion depth and through-both, nut engaged, turns and advance, grasp
attach/release events, gripper/fixture contacts, closest requested hand to the hole and to the bolt tip.

## Hard rules
- Billed: ledger `runs/sweep/ledger.json`, cap 1912, 1626 used. Gavin's request authorizes 300 more: raise the cap
  once under the lock to 2212 with the reason recorded. This task may use at most 400 calls. Stage A needs <= 240,
  Stage B <= 120. If anything would exceed that, run fewer episodes and say so. `pool_warming` (DA-2000) refusals
  are not billed and may be retried.
- No hardware. Only write inside `~/dreamtrial/viscous_sim`. Do not edit installed packages, the Lab repo,
  `viscous_arm.json` or `viscous_arm_fingertip.json`. Never print the API key. Do not touch `~/sonar-recon`.
- Existing tasks and modes behave exactly as before (gripper force and attach are flange-task-only): rerun the
  no-call checks.
- Videos: H.264 yuv420p `run.mp4` per episode, three panes: top camera, the inspection close-up of the flange
  joint, and the left wrist. Keep per episode log.json, chunks.jsonl, run.mp4.

## Report (short, `runs/flange_dream_report.md`)
Which holding method and why (with the force number and its derivation). Whether the scripted pregrasp demo
succeeded (video path). One table: arm, condition, prompt, episodes, max insertion depth, through-both (eps),
nut engaged (eps), max turns, calls used. Per condition a sentence or two on what the arm actually did (watch the
videos). YAM vs Viscous. Best and worst video paths. Billed calls used. Files changed/added. Measured vs guessed.
