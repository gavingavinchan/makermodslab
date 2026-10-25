# Astra flange control harness — 2026-10-25

**Astra-ready G3 profile validated — 2026-10-25.** `--profile astra` selects `--g3-seat-mm 30` (**33.0 mm protrusion** = 26 mm nut + two 3.5 mm pitches), `--thread-hand right`, `--g3-grasp flats`, `--command-cap 600`, and `--wall-seconds 14400` (4 h). The flats start keeps the trial-2 arm pose, puts the nut center **40 mm beyond the pad TCP**, bore along approach, and initializes **45.4 mm jaw opening** with light preload. Outside-flat contacts and **zero head/fixture contacts** verified. Explicit flags override profile values. `--profile trial2` (also the no-profile default) retains 0 mm extra insertion / 3 mm protrusion, stock grasp, legacy winding, 200 commands / 1800 s.

**Scripted CLI-only validation cheat passed:** **9.3690 right-hand turns**, **32.791 mm advance**, nut engaged, bolt through both flanges, **nut seated = true**, final face gap **0.150 mm** (0.5 mm criterion). Independent relative rotation confirms clockwise-from-tip winding. 306 commands; 59 clean releases, all with **zero bilateral jaw load**. [Video](validation/g3_astra_profile/run.mp4), [final frame](validation/g3_astra_profile/final_frame.png), [result](validation/g3_astra_profile/validation.json), [audit](validation/g3_astra_profile/audit.json). Privileged poses selected targets/certified outcome; all actuation used `./robot`. This is harness validation, not autonomous vision. No Astra or Dream calls, ledger/runner-lock access, hardware or protected-source edits.

[Flag/hand/geometry units](validation/profile_checks/checks.json), [outside-flat contact check](validation/profile_checks/flats_contacts.json), [CLI release and launch smoke](validation/astra_final_smoke/unit_checks.json), and [trial2 regression](validation/trial2_profile_regression/regression.json) pass. Trial-2 initial poses and state match the archive/original initializer **bit for bit**. Opening the Astra grasp reaches **104.832 mm**, zero load. Tip-side threading uses **positive turn / positive feed**, `degrees * 3.5 / 360` mm. Hand and seating/gap stay hidden. Astra reduces the in-memory IK planning margin from 2° to **0.25°** for reach; physical limits/forces/contacts stay unchanged. The right-hand subclass preserves entry/release gates and pitch.

Launch from `/home/gavin/dreamtrial/viscous_sim` (fresh names/empty destinations):

```bash
MUJOCO_GL=egl TMPDIR=$PWD/.cache/tmp PYTHONDONTWRITEBYTECODE=1 \
  /home/gavin/dreamtrial/.venv/bin/python astra_op/server.py \
  --profile astra --episode astra_g3_20261025_001 \
  --output runs/astra_flange/astra_g3_20261025_001
```

Wait for `ready`; give Astra only **`/home/gavin/dreamtrial/astra_ws/astra_g3_20261025_001`**. Start there with `./robot look` and `./robot status`. Smoke shutdown passed; **no server left running**.

---

Earlier trial-2 harness findings retained below (historical):

**Interface implemented; exact trial-2 G3 two-turn validation remains unmet.** Default G3 preserves the original start. The final controller's [exact-start run](validation/g3_wrist_servo/run.mp4) peaked at 1.8881 turns, then lost/restarted engagement. It is not a successful G3 trial.

The limiting geometry is concrete: the bolt protrudes 3 mm and the entry gate allows a 3 mm initial gap, while two turns require 7 mm advance. This leaves at least 1 mm nut/flange overlap. A privileged static [geometry probe](validation/g3_geometry_check.json) found about 1.01 mm penetration at two turns for all four tested nut phases. Repeated CLI attempts reached about 1.9 turns before the retained thread pull gate released. A separate [fixture-load probe](validation/fixture_force_probe.json) measured only 0.029 mm bolt displacement at 70 N. These probes diagnose the start/physics constraint; they are not operator runs. No physics changes bypass it.

**Separately labeled diagnostic: +5 mm initial bolt insertion passed.** Nut engaged for **2.1266 turns**, **7.443 mm advance**, bolt through both flanges; independently unwrapped physical relative rotation: 2.1257 turns. [Run video](validation/g3_seated5/run.mp4), [final frame](validation/g3_seated5/final_frame.png), [episode.jsonl](validation/g3_seated5/episode.jsonl), [CLI transcript](validation/g3_seated5/cli_trace.json), [artifact audit](validation/g3_seated5/audit.json). 66 CLI commands; 149.55 simulated seconds, seed 20261024. Explicit `--g3-seat-mm 5` changes the initial bolt/weld placement before the first physics tick. Default zero remains exact trial 2. This result does **not** satisfy the requested exact-start comparison. **Privileged poses selected targets and certified the result**; every actuation used only the CLI and real physics. No autonomous vision performance is certified. No Dream calls, Astra operator runs, ledger opens, runner-lock acquisition, hardware, installed-package edits, original task-source edits or propeller-file edits.

Code: [astra_op](../../astra_op/README.md), including the [operator template](../../astra_op/README_OPERATOR.md). Commands: `look`, `point`, `move` with optional approach axis, millimetre `nudge`, degree `turn`, `grip`, `status`, `done`. Optional `turn --feed-mm` synchronizes operator-chosen axial travel with rotation; default feed is zero. Conditions `G3`, `G1`, `T`, seed, command cap (200 default), and wall cap (1800 s default) are server flags. The workspace contains only `robot`, `README.md`, `obs/`. Pixels, calibrated depth picks, camera calibration, tool poses/axes, actual openings, encoders, jaw loads and IK residuals are available. Object transforms, insertion, thread state, part identity and success are unavailable through the protocol. This is an information boundary, not an OS sandbox.

Interface fixes required for the gate: precise local seeds for native client IK; bounded integral tracking expressed as torque bias (20 Nm maximum bias, with unchanged ±25 Nm actuator saturation); contact-stopped closing and opening integral control within unchanged ±160 N jaw-drive saturation; and a post-integration kinematic refresh before the unchanged observer. MuJoCo `mj_step` leaves body transforms from before integration; without that refresh the stock thread observer discarded newly integrated spin. No runtime object setters were added. The stock initial nut grasp can hook the bore: truthful force feedback detects the remaining load after an open request. The script engages the nut, withdraws once, and regrasps its outer rim so subsequent resets genuinely open the jaws. Measured right openings during the successful run: 104.832–104.832 mm, with zero bilateral load at release.

[Unit checks](validation/unit_wrist_servo/unit_checks.json): all command forms and invalid inputs checked; held nut −20° command produced 20.267° measured rotation, left-held bolt +10° produced 9.715°. Four visible-surface depth picks, including both wrists, agreed with independent geometry rays within **0.381 mm**. The hooked stock right grasp is explicitly reported as still loaded; left opening and subsequent G3 right openings are physically verified. [Unreachable IK](validation/additional_checks.json), [command/idle wall caps](validation/limit_checks.json), and final wrapper/guard smoke checks pass. Max joint reference speeds: 0.465 rad/s gripping and 2.946 rad/s overall. Video fully decodes as H.264/yuv420p, 1920×400, 10 fps: top, inspection close-up, right wrist. Hidden telemetry includes 1 ms torque sampling, 50 ms measurements, contact pairs and all attachment/thread events. Measured actuator peaks: 7.871 Nm arm, 72.000 N jaw drive.

[13 original guarded no-call checks](validation/legacy/results.json) pass, including all 18 baseline modes, sweep/FK/clamp/accounting-fixture/replay, one-arm/isolation, independent propeller IK, flange geometry/holding/native IK/clearance/pregrasp/runner and trial-2 stop/replay/torque isolation. Original synthetic accounting tests use disposable `state.json`/`state.lock`; all ledger-file and runner-lock opens are blocked. Full `propeller_check.py` is skipped because its authorization/migration test opens the prohibited lock and ledger. The real runner lock was observed free via inode plus `/proc/locks`; it was never opened or acquired. Initial guard/path failures and motion probes are retained separately from the final passing artifacts.

The requested physical thread specification is right-hand, 3.5 mm pitch. **The retained ideal model has the opposite nut-side winding sign** (`advance = pitch × positive rotation about the bolt’s head-to-tip axis`). It was left unchanged as required; the operator README gives the actual negative-turn/positive-feed convention for a tip-side grasp. The ideal thread projection and contact-weld surrogate do not establish physical thread torque or real-arm feasibility. No assembly from T/G1 or additional seeds is certified.

Launch a fresh episode from `/home/gavin/dreamtrial/viscous_sim`:

```bash
MUJOCO_GL=egl TMPDIR=$PWD/.cache/tmp PYTHONDONTWRITEBYTECODE=1 \
  /home/gavin/dreamtrial/.venv/bin/python astra_op/server.py \
  --episode astra_g3_001 --condition G3 --seed 20261024 \
  --output runs/astra_flange/astra_g3_001
```

Wait for `ready`, then give the operator only `/home/gavin/dreamtrial/astra_ws/astra_g3_001`. Start with `./robot look`. To reproduce the separately labeled diagnostic, use fresh episode/output names and explicitly add `--g3-seat-mm 5`. Full instructions are in [astra_op/README.md](../../astra_op/README.md). No live episode has been handed to Astra; no default start was changed.
