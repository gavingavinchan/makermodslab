Implemented and verified the tooling. No real Astra operator episode or A/B episode was launched.

The [full report](/home/gavin/dreamtrial/viscous_sim/astra_learn/report.md) includes complete operator claims, validation evidence, lesson changes, and usage details. Instructions are in [astra_learn/README.md](/home/gavin/dreamtrial/viscous_sim/astra_learn/README.md).

All six historical episodes succeeded by the final relative-turn criterion. Four were seated. All ended through operator `done`.

| Episode | Seated | Gap mm | Nut turns | Bolt turns | Left hold fraction | Commands | Sim s | Wall s | Drops | Engagements |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| astra_g3_20261025_006 | Yes | 0.116 | 11.453 | — | — | 743 | 1410.15 | 9701.02 | 3 | 6 |
| astra_rov_g3_20261026_001 | Yes | 0.080 | 10.282 | — | — | 102 | 412.22 | 1236.33 | 0 | 3 |
| astra_rov_backup_g3_20261026_001 | Yes | 0.028 | 10.284 | −0.3948 | 0.9985 | 552 | 1326.72 | 8549.54 | 0 | 2 |
| astra_rov_backup_g3_20261026_002 | No | 2.032 | 9.702 | −0.0003 | 1.0000 | 174 | 601.85 | 1527.67 | 0 | 1 |
| astra_rov_backup_g3_20261026_003 | No | 2.213 | 9.647 | −0.0015 | 1.0000 | 88 | 486.15 | 734.37 | 0 | 1 |
| astra_rov_backup2_g3_20261026_001 | Yes | 0.130 | 10.246 | −0.0556 | 1.0000 | 103 | 585.51 | 1594.88 | 0 | 1 |

Nut turns are relative nut-minus-bolt turns. Missing telemetry is shown as —. Commands include `done`; hold fractions are time weighted during positive right-hand turn/screw commands.

Episode 6’s “23 minutes” was simulation time. Its recorded Codex session lasted approximately 162 wall-clock minutes. Historical wall times use session timestamps; new runs use the runner’s monotonic operator timer.

Detected claim disagreements:

- Backup 002 claimed seated; simulator measured a 2.03 mm gap.
- Backup 003 claimed seated and more than ten turns; simulator measured a 2.21 mm gap and 9.65 relative turns.
- Backup2 001 claimed more than eleven turns; simulator measured 10.25 relative turns.

The drop detector counts one fall per continuous interval without either gripper’s nut attachment or thread engagement, once world height falls at least 10 mm. Support returning rearms it. It detected episode 6’s three known drops at 188.65, 488.05, and 709.65 simulated seconds, and zero in every later episode.

The no-inference checks passed:

| Smoke run | Lessons | Result |
| --- | --- | --- |
| learn_smoke_N_20261027_02 | Absent | Status/done completed; absence asserted |
| learn_smoke_L_20261027_02 | v001 | Status/done completed; notes matched byte for byte |
| learn_smoke_cap_20261027_01 | Absent | Operator group terminated; recorded `wall_cap` |
| learn_smoke_crash_20261027_01 | Absent | Exit 7 recorded as `crash`; cleanup completed |

All four produced outcomes, timelines, audits, and videos. Every video decoded successfully. Six fixture tests passed, including memory-contamination refusal, export completion detection, and A/B continuation after a simulated crash. Server-present refusal and prompt byte equality also passed.

The five sequential consolidations completed on their first attempts. Every resulting version has 90 lines and passed coordinate and scene-label checks:

| Version | Parent | Learned from | Main revision |
| --- | --- | --- | --- |
| v002 | v001 | ROV 001 | Separates relative turns, success, and seating; updates stall interpretation |
| v003 | v002 | Backup 001 | Strengthens backup retention, common-rotation checks, and reset clearance |
| v004 | v003 | Backup 002 | Corrects the 2.03 mm seating overclaim and distinguishes `lost_grasp` from stall |
| v005 | v004 | Backup 003 | Corrects the 2.21 mm gap and turn count; rejects sustained stall as seating proof |
| v006 | v005 | Backup2 001 | Records confirmed seating at 10.246 relative turns and improves both-arm clearance guidance |

v001 remains byte-identical to the original episode-6 notes. Parentage, outcomes, diffs, model logs, checks, and audits are retained in [the lessons store](/home/gavin/dreamtrial/viscous_sim/astra_learn/lessons/CHANGELOG.md).

The no-lessons prompt differs only by this removal:

```diff
-This directory also has `LESSONS.md`: notes you wrote yourself after a previous attempt at the same task in a
-different (tabletop) scene. Read it before you start. Where it and `README.md` disagree about the robot, `README.md`
-is current.
-
```

After review, launch the prepared A/B from `/home/gavin/dreamtrial/viscous_sim`:

```bash
nohup setsid astra_learn/ab.sh > runs/astra_flange/ab_launcher.log 2>&1 < /dev/null &
```

It runs L1, N1, L2, N2, L3, N3 on `astra_rov_backup2`, G3, seed 20261024, with 240-minute operator caps. L always receives fixed v001; N receives no notes. Crashes are recorded and the chain continues. Per-episode results and final per-arm statistics go into `runs/astra_flange/ab/`.

No physics, profiles, `screw`, or operator-facing README changed. No server remains running, and `stage1_outputs` remains empty. The A/B is deliberately unlaunched, so no controlled lessons-effect claim is available yet. Claim parsing and coordinate lint remain documented heuristics.