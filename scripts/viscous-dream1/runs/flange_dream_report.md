# Flange Dream-1 trial — 2026-10-24

**Stopped at the requested Part 1 gate. Scripted pregrasp assembly failed; no Dream-1 or YAM inference was run.** The approved v2 fixture, pipe direction, policy cameras, part masses, thread model and arm torque limits remain unchanged.

Holding uses **±160 N generalized jaw drive plus contact-gated physical MuJoCo welds**, flange only. CAD motor axis and bearing centres give crank radius **30 mm**, rod length **42 mm**, slider-line offset **16 mm**. With `z = r sin θ − sqrt(L² − (e − r cos θ)²)`, the maximum `|dz/dθ|` over the CAD-connected 0–24 mm opening branch is **30.57 mm/rad**. Virtual work gives `5 Nm / 0.03057 m = 163.56 N`; rounded down to 160 N, approximately **80 N/jaw** under symmetric contact. This estimates ideal force at the required 46 mm grasp width; transmission losses and the inherited full opening range are uncalibrated. [Derivation](flange_dream/validation/transmission.py).

Force alone failed the table pickup: **0.39 mm** maximum bolt lift despite **19.71 N** peak friction capacity; joint tracking error reached **14.74°**. The fallback weld activates only with both jaws touching while commanded closed, preserves current relative pose and payload load, and releases at an opening command **>0.65**. Every event is logged. A five-second pregrasp hold/release check passes: **1.81 mm** peak translational weld residual, no fixture contact, arm commands below 5 Nm. Other tasks retain ±10 N. Hardware/Lab/lerobot force limits do not participate: this runner sends normalized grip commands to local MuJoCo actuators; no hardware interface was used.

`flange_demo.py --start pregrasp` starts with head-held bolt in the left hand and bore-forward nut in the right, clear of the joint, with deterministic ±2 mm hand-position jitter. Table remains the default start. The [32.5-second scripted video](flange_dream/scripted/run.mp4) shows the left arm carrying the bolt toward the ring, pressing against its outside, and failing alignment/insertion; the right then lowers/turns the nut beside the joint without engagement. Both grasp welds remain attached. **Insertion 0 mm, engagement false, turns 0**; 566 bolt/fixture-contact rows, zero gripper/fixture or finger/table rows. Maximum joint tracking error is **33.35°**. H.264/yuv420p, top + enlarged inspection + left wrist; `log.json` and `chunks.jsonl` accompany it.

Clear forward-facing endpoint IK solutions exist, but the sampled collision-free solutions require at least **9.39 Nm** with the bolt and **7.84 Nm** with the nut. Their arm-only gravity terms already reach **7.00/7.15 Nm**, versus the unchanged **5 Nm** limit. The continuous attempt also encounters wrist/joint limits. These sampled searches do not prove every possible regrasp strategy impossible; they do establish that this scripted forward-held approach has not passed feasibility. [Pose/load search](flange_dream/validation/load_probe.json).

| Arm | Condition | Prompt | Episodes | Max insertion | Through-both eps | Nut engaged eps | Max turns | Calls |
|---|---|---|---:|---:|---:|---:|---:|---:|
| Viscous B0 | T / table | put the bolt through the hole in the flange and screw the nut onto it | 0 | — | — | — | — | 0 |
| Viscous B0 | G1 / pregrasp | put the bolt through the hole in the flange and screw the nut onto it | 0 | — | — | — | — | 0 |
| Viscous B0 | G2 / pregrasp | insert the bolt into the flange | 0 | — | — | — | — | 0 |
| Viscous B0 | G3 / inserted bolt | screw the nut onto the bolt | 0 | — | — | — | — | 0 |
| YAM vision | Stage B | No condition selected | 0 | — | — | — | — | 0 |

No policy behaviour was observed in any condition; **YAM versus Viscous is untested**. Stage A/B, G3 fixture-held initialization, live flange execution, and two-call success stopping were not implemented/run after the failed feasibility gate. Live flange execution remains blocked.

Best available video: [pregrasp feasibility](flange_dream/scripted/run.mp4). Worst available: [force-only table pickup](flange_dream/force_only/run.mp4), an exact command replay rendered into three panes. These are scripted diagnostics, not policy episodes.

**Accounting:** 0 billed calls, 0 refusals, 0 uncertain. Authorized cap raised **1912 → 2212 exactly once under the ledger lock**, with reason recorded; usage remains **1626**. No inference reservations were made.

All existing no-call suites pass: sweep/FK/clamp/budget, fake-client stop/resume, 18 baseline modes, one-arm/isolation, propeller, flange physics/native IK/clearance/stub replay. New holding and pregrasp replay checks pass. Protected arm JSON hashes, scene builder and thread implementation are unchanged. [Validation](flange_dream/validation/legacy_checks.json).

Changed: `cross_scenes.py`, `sweep_scene.py`, `sweep.py`, `flange_scene.py`, `flange_demo.py`, `SWEEP_README.md`, ledger. Added: `flange_holding.py`, `flange_start.py`, `flange_pregrasp_demo.py`, `flange_recording.py`, `flange_holding_check.py`, `flange_pregrasp_check.py`, this report and `runs/flange_dream/` evidence/source snapshots. Original v2 demo was regenerated with saved original source and 10 N physics after the first diagnostic overwrote it.

**Measured in simulation:** contacts, trajectories, torques, errors, weld residual/events, video decoding and accounting. **Estimated/unvalidated physically:** CAD linkage force, friction, masses/inertias, actuator ratings under load, calibration and ideal thread behaviour. Video descriptions are visual interpretation. No hardware or installed-package changes.
