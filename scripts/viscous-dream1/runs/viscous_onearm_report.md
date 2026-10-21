# One-arm Dream-1 apple trial

Zero-shot `dreamlabs/dream-1-beta`, vision mode; 18-call cap; paired seeds 20261020–20261025; ±2 cm independent x/y spawn jitter. Success is the existing resting-at-target check at two consecutive call endpoints; grasp means >3 cm lift. Median includes the confirming call.

| Configuration | Episodes | Successes | Grasps | Median calls to success | Finger/table contact rows |
|---|---:|---:|---:|---:|---:|
| B0 | 6 | 6 | 6 | 13.5 | 5/3744 (0.1%) |
| single | 6 | 2 | 4 | 14.0 | 0/4800 (0.0%) |
| single_copy | 6 | 2 | 2 | 15.0 | 1/4896 (0.0%) |
| parked | 6 | 6 | 6 | 9.0 | 99/2640 (3.8%) |

Paired outcomes against plain `single` (seed suffixes 20–25 correspond to episodes 0–5; W/L followed by calls used):

- **B0**: 20 W/11, 21 W/14, 22 W/13, 23 W/7, 24 W/15, 25 W/18. Loss→win: 20261020, 20261021, 20261022, 20261024; win→loss: none.
- **single_copy**: 20 L/18, 21 L/18, 22 L/18, 23 W/18, 24 W/12, 25 L/18. Loss→win: 20261024; win→loss: 20261025.
- **parked**: 20 W/11, 21 W/10, 22 W/8, 23 W/9, 24 W/9, 25 W/8. Loss→win: 20261020, 20261021, 20261022, 20261024; win→loss: none.

Measured: copying the left wrist produced 2/6 versus plain single’s 2/6 (+0 successes, +0.0 percentage points). Parked produced 6/6, +4 successes (+66.7 points) versus copying. Six episodes are a small sample; no real-bench result was measured.

**Use parked for the one-actuated-arm simulation.** Copying did not improve the measured success rate. Parked also increased finger/table contacts to 3.8% of rows; its success rate does not establish contact-free motion.

Real bench: `single_copy` needs no extra hardware; duplicate left wrist frames unchanged, keep right rest/open state, and discard right actions. `parked` needs a second held arm, or a comparable dummy with a camera at its wrist pose. A dummy’s effectiveness is a guess. Sim uses an ideal position hold, not measured real servo compliance. Black-frame distribution shift remains a hypothesis; these trials do not isolate its cause.

Stage 2 was skipped because at least one stage-1 configuration reached 5/6, as specified.

No-call checks passed: 32 exact legacy physics/policy comparisons, existing sweep checks, fake-client resume/budget checks, and all 18 legacy modes. Default image rendering is unchanged (repeat renders differ by at most 1 intensity level in 14 pixels). Both new modes ran 864 dry rows. Parked maximum measured deviation was **0.0 rad**, also zero during 37 initial finger contacts plus a sustained 20 N x/−20 N z force. Constraints plus projection hold each 1 ms step; the maximum pre-projection solver-coordinate residual was 0.000273 (hinges in rad, sliders in m). Copied JPEG windows match left exactly. [Checks and logs](sweep/onearm_validation/).

Call-1 policy images: [single_copy](sweep/dry/single_copy/apple/ep_00/contact_sheet.jpg), [parked](sweep/dry/parked/apple/ep_00/contact_sheet.jpg). Each sheet shows the latest encoded frame of all three call-1 camera windows.

Accounting: **157 additional billed calls**, 0 uncertain, 0 unbilled warming refusals. Ledger started at 872 used; cap changed once from 900 to **1312 = 872 + 440**. Final used 1029; 283 calls remain. Crash-safe reservations/fsync and global runner lock retained.

Representative videos (good/bad use the automated result; “none” means that outcome was absent):

- **B0**: good [runs/sweep/B0/apple/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/B0/apple/ep_00/run.mp4); bad none.
- **single**: good [runs/sweep/single/apple/ep_03/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/single/apple/ep_03/run.mp4); bad [runs/sweep/single/apple/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/single/apple/ep_00/run.mp4).
- **single_copy**: good [runs/sweep/single_copy/apple/ep_03/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/single_copy/apple/ep_03/run.mp4); bad [runs/sweep/single_copy/apple/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/single_copy/apple/ep_00/run.mp4).
- **parked**: good [runs/sweep/parked/apple/ep_00/run.mp4](/home/gavin/dreamtrial/viscous_sim/runs/sweep/parked/apple/ep_00/run.mp4); bad none.

Files changed: `sweep.py`, `sweep_scene.py`, `SWEEP_README.md`, `runs/sweep/ledger.json`. Added: `onearm_check.py`, `onearm_sweep.py`, `onearm_report.py`, this report, `runs/sweep/onearm_stages.json`, `runs/sweep/onearm_validation/`, new configuration episode/summary folders, and their dry folders. Protected arm JSON files unchanged. Validation reruns refreshed existing no-call artifacts.
