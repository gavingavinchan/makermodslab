# gc25: answers and offline fix

1. **`ff` is the final requested MIT torque**, after scale, startup ramp, region/support weights and the ±6 Nm clamp. The logged value is passed unchanged to `_mit_control`; it is not an unscaled model prediction. It is a software command snapshot, not a CAN capture or an independent measurement of delivered shaft torque. The RS00 driver quantizes it at 28/4095 = **0.00684 Nm/count**, with the same signed ±14 Nm range for command and feedback. No extra gearbox, radians or motor/URDF conversion is applied to torque there.

   At the settled compensated pose the archived model predicts **10.387 / 13.195 / 6.717 Nm** for shoulder_lift / elbow_flex / wrist_flex. At scale 0.25 and weight ≈0.933, these become **2.423 / 3.078 / 1.567 Nm**, matching the logged means **2.421 / 3.076 / 1.566 Nm**. The startup ramp is complete for essentially all of this settled interval; nothing approaches the clamp. The fit had **−6.377 / −7.821 / −5.544 Nm** offsets. Fitting used these offsets, but commanding intentionally omitted them as supposed sensor bias. Narrow data let the fit trade gravity against offsets, inflating the gravity-only command.

   Against the supplied approximate loads 4.5/5.5 Nm, ff is **53.8%/55.9%**; the model is predicting **2.31×/2.40×** those loads. However, the specified baseline archive actually measures **3.016/4.584 Nm** during settled hold, so the corresponding fractions are **80.3%/67.1%**. The 4.5/5.5 numbers are not this archive's settled shoulder/elbow means. Positive compensation substantially reduces shoulder/elbow error, but neither crosses its command: this run does not show literal shoulder/elbow overshoot.

2. **There is no evidence for a wrist_flex sign reversal in the command path.** Its commissioned angle sign is +1, like shoulder_lift/elbow_flex; only wrist_yaw has sign −1. `GravityModel.regressor` already transforms URDF effort back to motor effort by multiplying each row by that sign. Applying another sign at MIT output would duplicate that transformation. The RobStride encoder has no joint-dependent sign change: positive tau_ff encodes positive torque for every motor.

   With compensation, wrist_flex has **+1.363 Nm stiffness effort + 1.566 Nm ff = 2.928 Nm**, versus measured **2.886 Nm**: a **−0.043 Nm** residual. Treating the feedforward as negative instead leaves a **+3.089 Nm** mismatch. Shoulder/elbow likewise agree with **positive addition** within 0.091 Nm. Feedback is total motor torque, including feedforward; do not add ff to the feedback again.

   The wrist was already **3.050°** below its goal at the start of hold, with the ramp still rising; its error falls to **2.588°** as positive ff increases. Positive ff therefore helps within this ramp. The between-run comparison (1.071° → 2.588°) does not establish a reversed actuator sign. It comes with a change in measured wrist holding effort **0.522 → 2.886 Nm (+2.363 Nm)**; the command-effort change is **+2.367 Nm**. At constant load, +1.566 Nm with kp=30 would reduce error by **2.991°**, rather than increase it. The two runs are not equivalent static-load experiments, despite identical commanded poses. They have different achieved poses and approach histories, and even different gripper configurations (**−47.326° vs −5.880°** settled, low gripper motor effort in both), and the logs do not identify contact, preload, cable tension, stiction, a stop, or another changed load condition. None of those is asserted as the cause.

   **The bad fit term is established; a unique physical cause of the extra wrist load is not.** Even including its nuisance offset, the old fit predicts only **1.17 Nm** at the compensated wrist pose, versus the observed 2.89 Nm. The gravity-only value of 6.72 Nm is especially unsuitable for commanding. The current provisional fit is not physical validation of wrist compensation.

3. **Fix:** retain scale/sign semantics; constrain the nuisance terms and command only joints supported by the fit. The already-landed `c443ae34` pins offsets/friction to zero for the saved refit, rejects excessive offsets in the rehearsal, adds a nearest-measured-pose support gate, and replaces the nine Mac-path-dependent tests with committed arm/record fixtures. Concurrent commit `7340f21b` additionally refits the broader uncompensated logs and restricts `feedforward_joints` to shoulder_lift/elbow_flex; the wrist stays on its existing position controller. The same commit incorporates this task’s change moving the offset refusal into the shared command-model loader so Lab's environment installer cannot bypass it by using a nonprovisional flag. Telemetry/contact prediction can still explicitly load offsets. The report now shows signed `kp*error + ff` balance, and committed incident ticks reproduce the observed numbers and historical command path offline.

## Every joint's direction

For a fixed external load and positive stiffness, `error = (holding_effort − tau_ff)/kp`, so increasing tau_ff moves measured **motor q positive**. “Gravity-loaded side” below means the lower-q side seen in this baseline's positive standing error; for nearly unloaded axes that label is weak. These archives are coupled, multi-joint runs, not six isolated torque-step tests.

| Joint | motor→URDF sign | +tau_ff motor direction | Relative to baseline loaded side | Evidence in these runs |
|---|---:|---|---|---|
| shoulder_pan | +1 | +q | Nominally away; gravity load ≈0 | ff ≈0; no positive torque-step measurement, so physical response is untested here |
| shoulder_lift | +1 | +q | Away | ff +2.421 Nm; settled error 1.977° → 0.483°, q rises 1.492° |
| elbow_flex | +1 | +q | Away | ff +3.076 Nm; error 2.967° → 1.274°, q rises 1.694° |
| wrist_flex | +1 | +q | Away | Positive ff during hold ramp moves q toward goal; between-run extra load dominates |
| wrist_yaw | −1 | +motor q, −URDF q | Nominally away | Actual ff is −0.297 Nm; error grows 0.101° → 0.431°, consistent with negative motor support. No isolated +tau_ff test |
| wrist_roll | +1 | +q | Nominally away; gravity load ≈0 | Actual ff +0.305 Nm; q increases ≈0.261°, error goes 0.060° → −0.204°. No isolated +tau_ff test |

The motor-to-URDF signs constrain coordinate conversion; they are not empirical certificates of the firmware's torque response. Shoulder/elbow and the wrist-flex ramp support the expected direction; pan/yaw/roll cannot all be physically sign-certified from these logs. No hardware test was performed.

## Numeric evidence

Exactly `phase == "hold"`, excluding the first second, no trip rows: baseline **379** readings; gc25 **380**. Error is the median cmd−pos; torque/ff/stiffness/residual are arithmetic means so the effort equation is additive. kp is 60/90/90/30/30/30 Nm/rad. Settled damping is omitted; quantization, residual motion and telemetry bias remain.

| Joint | Baseline error ° | gc25 error ° | Baseline torque Nm | gc25 torque Nm | gc25 ff Nm | gc25 kp·error Nm | gc25 torque−(kp·error+ff) Nm |
|---|---:|---:|---:|---:|---:|---:|---:|
| shoulder_pan | 0.032 | 0.098 | −0.020 | 0.038 | ≈0 | 0.102 | −0.064 |
| shoulder_lift | 1.977 | 0.483 | 3.016 | 3.093 | 2.421 | 0.763 | −0.090 |
| elbow_flex | 2.967 | 1.274 | 4.584 | 5.000 | 3.076 | 2.010 | −0.086 |
| wrist_flex | 1.071 | 2.588 | 0.522 | 2.886 | 1.566 | 1.363 | −0.043 |
| wrist_yaw | 0.101 | 0.431 | 0.022 | −0.106 | −0.297 | 0.226 | −0.035 |
| wrist_roll | 0.060 | −0.204 | −0.004 | 0.165 | 0.305 | −0.104 | −0.036 |

Median vertical TCP sag (the tank FK and existing TCP): **26.168 → 15.458 mm**. Joint sag worsening at the wrist does not imply net TCP sag worsened.

The old model at the baseline measured pose predicts **9.592/12.251/6.695 Nm** for shoulder/elbow/wrist; adding its offsets gives **3.215/4.430/1.151 Nm** before its last-motion Coulomb terms. This directly shows why validating only the full fitted predictor could miss the gravity-only command error. Large negative offsets are far larger than the approximately 0.04–0.09 Nm settled effort residuals observed here.

## Reproducibility and provenance

Original runs (read only):

- `/home/gavin/dream-run-archive/mac-run-data/rehearse-live-20261117T024005Z/`
- `/home/gavin/dream-run-archive/mac-run-data/rehearse-live-20261117T090749Z/` (recorded source b9dccb60)

Portable fixtures under `tests/fixtures/viscous_gravity/gc25/` contain all approach/hold ticks' timestamps, phases and seven motors' pos/cmd/torque, plus gc25 ff/w. They preserve original numeric precision, omit temperature/stop data irrelevant to these regressions; the gripper configuration difference is retained, and include the exact archived fit. `sources.json` records original file SHA256s. No archive path is needed by tests.

The replay deliberately constructs the old model directly for diagnosis, then passes it through the real feedforward hook with a fake bus. After the startup ramp, every replayed torque matches the archived command to **0.02 Nm**; command/status equality is exact. The looser archive comparison allows different read/send times and within-tick feedback updates: ticks do not contain per-frame bus timestamps. Regression also checks the observed sag/effort means, the wrist ramp direction, original-fit rejection through both command entry points, and the actual installed driver's positive signed torque encoding on all six joints without constructing a bus connection.

Run from `/home/gavin/gravity-comp`:

```sh
PYTHONPATH=scripts/viscous-dream1/real .venv/bin/python -m pytest -p offline_test_guard \
  tests/test_arm_gravity.py tests/test_gravity_feedforward.py \
  tests/test_viscous_gravity_sweep.py tests/test_viscous_gravity_comp.py \
  tests/test_viscous_gravity_gc25.py tests/test_viscous_dream_real.py
.venv/bin/python scripts/viscous-dream1/real/gravity_comp_report.py \
  tests/fixtures/viscous_gravity/gc25/gc25.jsonl
```

Final gravity/runner validation: **198 tests passed + 31 subtests** (47.35 s). The incident regression alone passed **10 tests**. Ruff check, Ruff format check and `git diff --check` pass. Complete test output: `/home/gavin/gravity-comp-reports/gravity-tests.log`. The shared loader guard and shoulder/elbow-only command selection are in `7340f21b`; the final incident-report/regression commit is recorded in the accompanying completion message. No hardware, live runner, live Dream/API calls, or API keys were used. Tests use the existing offline audit guard; the checkout's ignored `.venv` symlink points to the existing installed Python environment without modifying that environment.
