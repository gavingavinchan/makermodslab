# VISCOUS Dream-1 sweep

**872 billed calls; 0 uncertain; 28 unused.** 44 seeded episodes, no fine-tuning. Success requires two consecutive call-end rest checks; grasp means >3 cm lift. Median calls includes the confirming call.

## Apple

| Configuration | Episodes | Successes | Grasps | Median calls to success | Finger/table rows |
|---|---:|---:|---:|---:|---:|
| B0 | 6 | 6 (100%) | 6 | 13.5 | 5/3744 (0.1%) |
| Fingertip TCP | 6 | 4 (67%) | 6 | 11.5 | 0/3840 (0.0%) |
| B0 + single arm | 6 | 2 (33%) | 4 | 14.0 | 0/4800 (0.0%) |

## Box

| Configuration | Episodes | Successes | Grasps | Median calls to success | Finger/table rows |
|---|---:|---:|---:|---:|---:|
| B0 | 6 | 0 (0%) | 2 | — | 2359/6912 (34.1%) |
| Fingertip TCP | 6 | 1 (17%) | 1 | 14 | 2468/6432 (38.4%) |
| B0 + single arm | 6 | 0 (0%) | 1 | — | 425/6912 (6.1%) |
| B0 + 24 rows | 4 | 0 (0%) | 1 | — | 562/2304 (24.4%) |
| B0 + table region | 4 | 0 (0%) | 0 | — | 105/4608 (2.3%) |

## Interpretation

**Keep B0 for the two-arm apple simulation. No tested configuration makes box placement or the one-arm simulation reliable.** Fingertip TCP produced the only box success (1/6), but reduced apple success to 4/6; one box success is weak evidence of improvement. Single-arm reduced apple to 2/6 and box stayed 0/6. Twenty-four rows gave box 0/4. The region cut box table contact to 2.3% and also eliminated lifts in those trials; it did not improve placement.

Stage 1 selected B0 by combined successes (6 versus 5). Stage 2 retained its single-arm results but carried B0 (6 versus 2). Stage 3 used **four paired seeds per option**, because six could exceed the remaining budget; B0, 24 rows and region all scored 0/4 on those seeds. Stage 4 therefore selected the already-tested B0; **no new stage-4 episodes** were run. Its six episodes per task were reused. The remaining 28 calls cannot fund a fresh apple+box pair at full caps (42).

The fingertip copy moves only TCP, 3.433 cm forward to the closed CAD tip-face midpoint; axes remain unchanged. [Old/new tool-point picture](sweep/validation/tool_points.png). The table region is world x [−8,40], y [−32,16] cm, from 2.5 to 30 cm above the table; native client target clamp and region cost, with the default hard fence off.

## Representative videos

Good/bad means the automated success result. A missing outcome is marked none.

| Configuration/task | Good video | Bad video |
|---|---|---|
| B0/apple | [sweep/B0/apple/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/B0/apple/ep_00/run.mp4) | none |
| B0/box | none | [sweep/B0/box/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/B0/box/ep_00/run.mp4) |
| tip/apple | [sweep/tip/apple/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/tip/apple/ep_00/run.mp4) | [sweep/tip/apple/ep_01/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/tip/apple/ep_01/run.mp4) |
| tip/box | [sweep/tip/box/ep_05/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/tip/box/ep_05/run.mp4) | [sweep/tip/box/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/tip/box/ep_00/run.mp4) |
| single/apple | [sweep/single/apple/ep_03/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/single/apple/ep_03/run.mp4) | [sweep/single/apple/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/single/apple/ep_00/run.mp4) |
| single/box | none | [sweep/single/box/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/single/box/ep_00/run.mp4) |
| r24/box | none | [sweep/r24/box/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/r24/box/ep_00/run.mp4) |
| table/box | none | [sweep/table/box/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/table/box/ep_00/run.mp4) |

## Files and checks

Added `sweep.py`, `sweep_scene.py`, `sweep_check.py`, `sweep_runner_check.py`, `sweep_baseline_check.py`, `sweep_report.py`, `viscous_arm_fingertip.json`, `SWEEP_README.md`, this report, and `runs/sweep/` artifacts. Existing source and `viscous_arm.json` unchanged. All 18 previous modes pass dry; sweep geometry, native FK/clamp, early stop, ledger cap and resume checks pass. [Runner details](../SWEEP_README.md); [audit](sweep/validation/final_audit.json).
