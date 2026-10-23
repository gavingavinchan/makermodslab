# Dream-1: bench propeller cleaning

Viscous sometimes picked up the sponge and touched the propeller, but made no qualifying back-and-forth reversal. YAM showed one sponge reversal and one bare-finger reversal; neither rig demonstrated sustained sponge scrubbing.

Zero-shot dreamlabs/dream-1-beta, vision / other_embodiment / client IK. Fixed 20.8 cm bronze three-blade propeller, horizontal shaft facing the arms on an 18.5 cm stand; six convex pieces/blade. Free yellow/green 9×6×4 cm sponge; no brush. Original table, lights, cameras and B0 bases retained; all fruit, bottle, bowl and plate removed.

[Call-1 policy images](sweep/B0/propeller/P1/ep_00/contact_sheet.jpg). All videos show top + left wrist. [Native IK verifies a collision-free 3 cm blade-face pass](propeller/validation/ik_reach.json); B0 starts unchanged.

Stage A: two episodes/prompt, paired seeds 20261023–24, sponge jitter ±2 cm. Selected **P3** by contact rows then reversals (P3: 455, P2: 79). Stage B: three fresh episodes/arm, paired seeds 20261025–27. All episodes: 30 calls × 48 rows, no success stop.

| Arm | Prompt | Episodes | Sponge grasped (ep) | Tool contact rows | Gripper contact rows | Reversals | Contact path (m) | Closest hand: prop / sponge (cm) | Calls |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Viscous B0 | P1 | 2 | 1 | 0 | 0 | 0 | 0.000 | 8.12 / 0.00 | 60 |
| Viscous B0 | P2 | 2 | 1 | 79 | 0 | 0 | 0.066 | 3.15 / 0.00 | 60 |
| Viscous B0 | P3 | 5 | 3 | 550 | 0 | 0 | 0.193 | 5.47 / 0.00 | 150 |
| Viscous B0 | P4 | 2 | 0 | 0 | 0 | 0 | 0.000 | 6.12 / 0.00 | 60 |
| YAM vision | P3 | 3 | 1 | 46 | 181 | 2 | 0.140 | 0.00 / 0.00 | 90 |

Measured: grasp = >3 cm lift while between distal fingers. Contact checks every physics substep, once/row. Path/reversals use sponge or bare fingertips within 1 cm, projected onto the local blade plane: >1 cm out then >1 cm back, resetting outside each bout. Reversals use that proximity band; the path column includes physical-contact rows only (band travel is also logged). Hand distances: raw requested TCP-to-surface, before execution, minimum across hands. No dirt removal is simulated.

**P1: “pick up the sponge and scrub the propeller”.** One trial lifted the sponge upright and slowly carried it toward the propeller; the other repeatedly tilted it low over the table. Neither reached contact.

**P2: “clean the propeller with the sponge”.** One trial contacted the lower blade area, then lowered/reoriented the sponge around the stand. The other stayed low; neither made repeated strokes.

**P3: “wipe the propeller”.** Two of five Viscous trials contacted the lower blade area: one long press followed by withdrawal, and one brief pass. Others stayed low or carried the sponge beside it; none reversed.

**P4: “scrub the propeller blade back and forth with the sponge”.** Both trials rotated/slid the sponge low over the table; one advanced toward the stand. Neither reached blade contact or scrubbed back and forth.

Fresh paired Stage B: YAM had 46 sponge-contact rows versus Viscous’s 95. YAM showed one sponge reversal at 34.15 s (1.4 mm gap, no physical contact at that instant) and one bare-left-finger reversal after dropping the sponge. These are brief scrub-like signals, not sustained cleaning. YAM’s stock wrists initially miss the propeller, limiting a task-versus-arm diagnosis. Its corrected distal-tip measurements were replayed without inference; chunks, sponge trajectories and contact counts matched exactly.

Best by sponge-reversal/contact/grasp evidence: [sweep/YAM/propeller/P3/ep_02/run.mp4](sweep/YAM/propeller/P3/ep_02/run.mp4).
Worst by sponge-reversal/contact/grasp evidence: [sweep/B0/propeller/P3/ep_02/run.mp4](sweep/B0/propeller/P3/ep_02/run.mp4).

Accounting: **420 billed calls**, 0 unbilled pool_warming refusals, 0 uncertain. Ledger 1206 → 1626 used, cap 1312 → 1912 once under lock with authorization reason; 286 remain. All 14 requested episodes completed.

All existing no-call checks pass: sweep/FK/clamp/budget/resume, 18 baseline modes, one-arm and isolation compatibility. Four full stub episodes/replay and all 14 video/row/accounting audits pass; protected JSON hashes unchanged. [Evidence](propeller/validation/). Behavior descriptions interpret sampled video frames; counts/trajectories are measured. No general success-rate claim.

Changed: sweep.py, cross_scenes.py, SWEEP_README.md, ledger.json; refreshed legacy validation. Added: propeller_scene.py, propeller_sweep.py, propeller_ik_check.py, propeller_check.py, propeller_report.py, assets/propeller/*.stl, runs/propeller/, runs/sweep/{B0,YAM}/propeller/, this report. Episode files: log.json, chunks.jsonl, run.mp4, plus first-episode contact_sheet.jpg.
