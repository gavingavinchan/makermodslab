# Viscous contact floor: offline refit and friction evaluation, 2026-10-20

Goal A1: the pilot should feel what the follower touches, with the arm's own effort removed from the contact signal. Work was done locally on the Mac, branch `feat/viscous-arm`, using saved evidence only. No hardware, SSH, serial/CAN, Lab service routes, commits, pushes, or stash were used. `gravity-air-fit.json`, `gravity_feedforward.py`, `docs/viscous-haptics.md`, `frontend/dist/`, and `scripts/viscous-dream1/` were not edited by this work. Existing unrelated working-tree changes were left alone.


> **Amended after review (Claude, same day).** The wrist_yaw / wrist_roll "local
> slope" terms below (0.135 / 0.311 N m per deg/s) were identified only under
> 2 deg/s; applied at teleop speed (~20 deg/s) they would subtract 3-6 N m of
> invented friction and swamp the contact signal. They are no longer stored as
> terms (zero, slope kept under `friction_identification` as a diagnostic), and
> every fitted term now carries `max_speed_deg_s`: the estimator holds friction
> at that speed rather than extrapolating. Re-run of `viscous-contact-fit.py`
> after the change: shoulder_lift / elbow / wrist_flex numbers are unchanged;
> wrist_yaw / roll "v2 + friction" now equal "v2 gravity" (p99 0.43-0.46 / 0.39-0.40).

## Outcome and recommendation

The added gravity holds and velocity compensation lower the observed free-motion floor in both run-held-out tests. With acceleration and transition cuts applied equally to every variant, shoulder_lift p99 falls from **0.863 / 0.969 to 0.743 / 0.788 N m**, elbow_flex from **0.598 / 0.643 to 0.421 / 0.458**, and wrist_flex from **0.551 / 0.548 to 0.382 / 0.410** (evaluation run 181208 / 182410). With movement transients retained, shoulder p99 is still **0.899 / 0.897 N m**, down from 1.177 / 1.223; elbow is 0.551 / 0.556 and wrist flex 0.396 / 0.417. Those broader tables matter for contact during real moves; the lower filtered numbers alone would overstate the result.

**Keep v2 opt-in; do not replace the default yet.** It is a useful offline candidate, especially for elbow and wrist, but the shoulder still has a substantial motion tail and the wrist flex hold floor remains about 0.23 N m RMS. Pan friction is completely unobservable in these runs; yaw/roll are observed only near 1.25 / 0.45 deg/s, so their terms are local slopes with no validated high-speed extrapolation. The two complete runs repeat nearly identical paths. Gravity validation holds out each file's whole pose; friction validation holds out the other run's motion, but uses the fixed gravity model fit to all four sources' holds. This is the requested friction run cross-validation, not a fully independent validation of gravity plus friction on unseen poses or loads. No powered validation was performed, and no default setting was changed.

Push amplitudes are preserved: v2 gravity changes the baseline-relative push p01/p99 by at most about 0.004 N m. Friction changes those amplitude quantiles by less than 0.001 N m at printed precision. The maximum single-tick friction change inside a push window is about 0.046 N m on the elbow, because the central command derivative sees the following return move at the **last push tick**. Excluding window boundary ticks, the maximum friction correction is below 4e-13 N m. The command stays still while the hand displaces the measured arm, so this compensation does not subtract the hand push.

## Files and behavior

| File | Change and reason |
|---|---|
| `scripts/viscous-gravity-fit.py` | Accept collector `samples` and contact-check `holds`; validate the referenced fit's URDF hash/condition; group by resolved file + pose across all approach/scale repetitions. `--compare` scores the current fit on the same holds and prints its original CV separately. Above/below pairs now also preserve scale. Telemetry is already total motor torque; stiffness mode reconstructs total with the collection-time gravity feedforward and no offsets. |
| `configs/viscous/viscous_01/gravity-air-fit-v2.json` | New fit, same base schema, 51 holds from four repo-relative sources, same regularization 1.0 as current. Adds `friction` and identification provenance/diagnostics. Original fit intact. |
| `makermodslab/contact_torque.py` | Loads optional friction. Subtracts it only if commanded velocities are supplied to `estimate(..., velocities_deg_s=...)` or `from_observation(..., velocities_deg_s=...)`. Missing joint velocities leave those joints uncompensated. Invalid supplied velocities produce NaN on that joint. Old fits/static calls retain their behavior. |
| `scripts/viscous_contact_analysis.py` | Shared offline arrays, motion selection, smooth compensation and nonnegative least-squares identification, using NumPy only. |
| `scripts/viscous-contact-fit.py` | Reproducible offline CLI: train friction on one complete run and evaluate the other, both ways; print free motion, still holds, wrist biases and pushes; fit final coefficients on the two complete runs plus usable motion from the aborted run. |
| `scripts/viscous-contact-check.py` | `--analyze --fit ...` now recomputes hold contact rather than printing contact cached under the collection-time fit. Motion tables cover all six joints with common fold/acceleration cuts; a fit carrying friction prints both uncompensated and compensated motion tables. Push statistics use the compensated signal when available. |
| `tests/test_contact_torque.py` | Tests rest/oddness/smoothness, opt-in estimation and observation API, invalid velocities, old-fit compatibility, relocation with repo-relative sources, loader grouping/hash checks, selection and exclusion of all non-fit torque data, partial-friction analysis parity, and recomputation of saved hold contact. |
| This report | Rationale, limits, reproduction commands, validation, and verbatim requested output. |

Feedforward still commands gravity alone: neither offsets nor friction enter its model or commands. A relocated-clone test copies the fit, its URDF and every recorded source to a new root, changes cwd, and loads it there.

## Identification choices and limits

Use `v = np.gradient(<joint>.cmd, t)` in deg/s and the residual `reported - gravity(v2) - offsets`. Friction is `C * tanh(v / v_s) + B * v`, zero at rest and odd/smooth in velocity. Both coefficients are constrained nonnegative; the optimum is allowed to put either coefficient at zero. Thus the shoulder data selects a viscous term and the elbow/wrist data selects Coulomb terms rather than forcing both to be positive.

Only `move:*` / `approach:*` ticks with **measured shoulder_lift < -15 and elbow_flex > 15 deg** enter the fit. Drop ticks within 0.15 s of any joint's command acceleration above 50 deg/s², plus 0.15 s around phase boundaries. Motion below 0.1 deg/s on a particular joint cannot identify its coefficients. Push, push-settle, ramps, settling and holds never enter friction identification. The cut values are fixed analysis choices, not tuned on the held-out run. There are 2,175 / 2,310 selected motion ticks in the two complete runs, and 896 in the aborted run. All variants use identical evaluation masks. The report also includes the wider motion cohort with transitions retained.

For excited joints, choose `v_s` from 1, 2, 3, 4, 6 deg/s using leave-one-contiguous-motion-leg-out MSE within the training data only. Final fitting uses all three motion runs; the outer run validation coefficients are printed separately. If Coulomb is zero, `v_s` is irrelevant and stored as 3. For yaw/roll, max commanded speed is below 2 deg/s: the Coulomb/viscous split is not identifiable at a few-deg/s smoothing width. Store Coulomb=0 and the nonnegative local velocity slope, with `v_s=3` (irrelevant), and flag weak excitation in the JSON. Pan never moves and has both coefficients zero, explicitly flagged unidentifiable. These zeros do **not** demonstrate zero physical friction. Even for elbow, most clean motion is near 6 or 15 deg/s, so a chosen 1 deg/s knee is not a precise measurement of near-zero friction behavior.

Final coefficients (not held-out estimates):

| Joint | Coulomb N m | Viscous N m/(deg/s) | v_s deg/s | Evidence |
|---|---:|---:|---:|---|
| shoulder_pan | 0.000000 | 0.000000 | 3.0 | unidentifiable: no commanded motion |
| shoulder_lift | 0.000000 | 0.020707 | 3.0 | fitted |
| elbow_flex | 0.177691 | 0.000000 | 1.0 | fitted |
| wrist_flex | 0.230180 | 0.000000 | 2.0 | fitted |
| wrist_yaw | 0.000000 | 0.134874 | 3.0 | weak excitation: local slope only; Coulomb unidentified |
| wrist_roll | 0.000000 | 0.310790 | 3.0 | weak excitation: local slope only; Coulomb unidentified |

Wrist hold biases (current -> v2, N m; friction is zero at rest): yaw **-0.179 -> -0.090** and roll **-0.146 -> -0.084** in 181208; yaw **-0.170 -> -0.081**, roll **-0.133 -> -0.072** in 182410. The remaining bias is real residual error in these logs; it was not hidden with a motion-only intercept or a push-trained offset.

## Data issues and interpretation

- There is no pan excitation, and yaw/roll have essentially one slow speed in each direction. A full six-joint Coulomb/viscous identification is impossible from these logs. The JSON preserves six joint keys but distinguishes fitted, weak, and unidentifiable terms. High yaw/roll coefficients are local slopes; extrapolating them to 15 deg/s would be unjustified.
- Shoulder gravity improves modestly with the added holds, not by the entire roughly 10% sag discrepancy. On all 51 holds, current -> v2 RMS is 0.184 -> 0.165 N m; v2 held-out RMS is 0.180. Elbow all-hold RMS gets slightly worse (0.099 -> 0.106), and wrist flex stays near 0.18. Offset/gravity/stiction separation is limited by these postures. Unconstrained gravity identification also produces effective inertial parameters; they are not independently measured body properties.
- Older gravity holds reached 60–63 °C at the hottest motor, while contact-check holds reached 34–37 °C. Temperature, posture and approach history differ; the observed wrist offsets should not be assumed to be a universal sensor zero. A velocity-only model cannot remove static hysteresis. The existing wrist-flex push baseline around -0.4 N m and its stiction step remain visible in the analysis.
- Raw push min/max contain brief spikes (run 182410 shoulder reaches about -2.42/+2.68 N m with the old fit). Baseline-relative p01/p99 is reported alongside raw extrema to distinguish sustained hand-push size from those spikes.
- Central differentiation mixes phases at boundary ticks. This is excluded during identification but intentionally visible in the whole push-window delta check; the last push tick sees the start of the return. Interior push commands are zero-velocity to floating-point precision.
- The shoulder's motion tail remains the limiting channel. The learned viscous coefficient can absorb control damping, timing and residual load effects as well as mechanical friction. Repeating a path on a second run tests reproducibility on that path, not broad dynamic-model validity.
- Existing NumPy matrix operations in `arm_gravity.py:213–214` emitted divide/overflow/invalid warnings in this Mac environment, including in pre-existing synthetic gravity tests. No fit values or predictions were non-finite. An independent elementwise product-and-sum check agreed with the matrix product to 1.78e-15 N m and with saved RMS to 5.56e-17 N m. The existing gravity module was not changed to suppress these warnings. Friction fitting and all four `--analyze` runs completed without stderr.

## Reproduction and gravity output (verbatim stdout)

Run from the repository root with `.venv/bin/python`. The gravity command is:

```sh
.venv/bin/python scripts/viscous-gravity-fit.py \
  configs/viscous/viscous_01/evidence/gravity-air-20261020-022551.json \
  configs/viscous/viscous_01/evidence/gravity-air-20261020-023356.json \
  configs/viscous/viscous_01/evidence/contact-check-20261020-181208.json \
  configs/viscous/viscous_01/evidence/contact-check-20261020-182410.json \
  --regularization 1 \
  --compare configs/viscous/viscous_01/gravity-air-fit.json \
  --out configs/viscous/viscous_01/gravity-air-fit-v2.json
```

The frozen current-fit column is scored on all 51 holds. Its original CV column comes from its 27-hold fit file and is **not the same evaluation population** as v2's leave-file-pose-out CV. CAD is a fixed predictor, so there is no training/holdout distinction for CAD on a fixed cohort. For reference, the original current-fit numbers on its original 27 holds are:

| Joint | CAD RMS | Current in-sample RMS | Current held-out RMS |
|---|---:|---:|---:|
| shoulder_pan | 0.064 | 0.025 | 0.027 |
| shoulder_lift | 0.329 | 0.201 | 0.231 |
| elbow_flex | 0.149 | 0.114 | 0.134 |
| wrist_flex | 0.132 | 0.121 | 0.127 |
| wrist_yaw | 0.193 | 0.156 | 0.166 |
| wrist_roll | 0.223 | 0.183 | 0.201 |

```text
51 holds from 4 run(s), condition air

Torque sensing check uses collector samples (or reconstructed total with --source stiffness).
Torque sensing check (slope ~1 and r ~1 mean they agree):
joint             telemetry vs kp*err     telemetry vs CAD      kp*err vs CAD
shoulder_lift  slope  1.00 r  1.00     slope  1.05 r  1.00   slope  1.05 r  1.00
elbow_flex     slope  1.00 r  1.00     slope  0.99 r  1.00   slope  0.99 r  1.00
wrist_flex     slope  0.99 r  1.00     slope  0.98 r  0.99   slope  0.99 r  0.99

Prediction error, RMS N m (fit on telemetry):
joint              CAD     fit  fit held-out   offset
shoulder_pan     0.050   0.025         0.026   -0.040
shoulder_lift    0.287   0.165         0.180   +0.014
elbow_flex       0.147   0.106         0.117   -0.014
wrist_flex       0.207   0.181         0.188   -0.053
wrist_yaw        0.150   0.146         0.154   +0.043
wrist_roll       0.169   0.152         0.163   +0.009

Existing fit comparison (current scored on ALL input holds; original CV from its file):
joint              CAD  current      v2  v2 held-out  current orig CV
shoulder_pan     0.050    0.027   0.025        0.026            0.027
shoulder_lift    0.287    0.184   0.165        0.180            0.231
elbow_flex       0.147    0.099   0.106        0.117            0.134
wrist_flex       0.207    0.186   0.181        0.188            0.127
wrist_yaw        0.150    0.165   0.146        0.154            0.166
wrist_roll       0.169    0.165   0.152        0.163            0.201

Friction (half the above/below difference), mean N m:
shoulder_lift   0.068
elbow_flex      0.031
wrist_flex      0.116

Bodies (mass kg; centre of mass in joint frame, mm):
shoulder_pan   fit  0.450 [   -0.1    -1.2    45.8]   CAD  0.450 [   -0.1    -1.2    45.8]
shoulder_lift  fit  0.487 [ -135.7    -0.6     0.1]   CAD  0.487 [ -129.7    -0.6    -0.0]
elbow_flex     fit  0.570 [   91.4     0.1    46.7]   CAD  0.523 [  103.4     0.1    52.7]
wrist_flex     fit  0.419 [   77.4     0.0    34.4]   CAD  0.405 [   73.7     0.0    50.7]
wrist_yaw      fit  0.412 [   -3.7    -0.6   -53.7]   CAD  0.398 [   -0.8     0.0   -41.8]
wrist_roll     fit  0.683 [   39.7    -7.7   -14.2]   CAD  0.644 [   44.5    -0.0    -2.8]

Wrote configs/viscous/viscous_01/gravity-air-fit-v2.json
```

## Task 4: run cross-validation output (verbatim)

```sh
.venv/bin/python scripts/viscous-contact-fit.py \
  --fit configs/viscous/viscous_01/gravity-air-fit-v2.json \
  --out configs/viscous/viscous_01/gravity-air-fit-v2.json \
  configs/viscous/viscous_01/evidence/contact-check-20261020-181208.json \
  configs/viscous/viscous_01/evidence/contact-check-20261020-182410.json \
  configs/viscous/viscous_01/evidence/contact-check-20261020-180543.json
```

`v2 + friction` below uses coefficients trained on the other complete run. The aborted run is excluded from validation training and included only in the final fit. Still-hold statistics here use `hold:*` ticks only. The final fit is not presented as a cross-validation score.

```text
Friction validation by run; fixed v2 gravity uses all four sources' holds.
Smooth speed is chosen on TRAINING data by leave-one-contiguous-leg-out CV.
Joints below 2 deg/s use only a local viscous slope; no motion => zero terms.
No push/hold ticks used for friction fitting. Coefficients constrained nonnegative.
Selection: move/approach, measured lift < -15, measured elbow > 15 deg;
exclude |command acceleration| > 50 deg/s^2 plus +/-0.15 s, and +/-0.15 s at phase edges.

TRAIN 20261020-181208 -> TEST 20261020-182410
         joint  Coulomb Nm  viscous Nm/(deg/s)  v_s deg/s             n (+/-)  status
  shoulder_pan    0.000000            0.000000        3.0     0 (   0/   0)  unidentifiable: no commanded motion
 shoulder_lift    0.000000            0.022438        3.0  2175 (1084/1091)  fitted
    elbow_flex    0.179574            0.000000        1.0  2175 (1087/1088)  fitted
    wrist_flex    0.232121            0.000000        2.0  2175 ( 917/1258)  fitted
     wrist_yaw    0.000000            0.136905        3.0   893 ( 440/ 453)  weak excitation: local slope only; Coulomb unidentified
    wrist_roll    0.000000            0.322557        3.0   893 ( 440/ 453)  weak excitation: local slope only; Coulomb unidentified

=== Evaluate 20261020-182410 (2310 selected free-motion ticks) ===
Free motion, N m: each triple = rms / p95 / p99 (absolute percentiles)
         joint                current             v2 gravity          v2 + friction
  shoulder_pan   0.029/ 0.055/ 0.076   0.025/ 0.046/ 0.064   0.025/ 0.046/ 0.064
 shoulder_lift   0.307/ 0.608/ 0.969   0.280/ 0.545/ 0.918   0.235/ 0.468/ 0.788
    elbow_flex   0.235/ 0.433/ 0.643   0.235/ 0.435/ 0.609   0.153/ 0.289/ 0.458
    wrist_flex   0.262/ 0.450/ 0.548   0.261/ 0.445/ 0.523   0.169/ 0.310/ 0.410
     wrist_yaw   0.213/ 0.441/ 0.531   0.159/ 0.363/ 0.459   0.114/ 0.234/ 0.312
    wrist_roll   0.176/ 0.396/ 0.444   0.144/ 0.343/ 0.391   0.106/ 0.215/ 0.256

Motion selection audit: 2917 move/approach ticks, 2777 away from fold, 2310 after transient cuts.
All away-from-fold motion INCLUDING transients (N m; rms / p95 / p99):
  shoulder_pan   0.030/ 0.055/ 0.076   0.025/ 0.050/ 0.064   0.025/ 0.050/ 0.064
 shoulder_lift   0.365/ 0.771/ 1.223   0.339/ 0.733/ 1.183   0.269/ 0.568/ 0.897
    elbow_flex   0.266/ 0.540/ 0.743   0.266/ 0.538/ 0.730   0.181/ 0.387/ 0.556
    wrist_flex   0.272/ 0.467/ 0.588   0.271/ 0.463/ 0.571   0.177/ 0.323/ 0.417
     wrist_yaw   0.206/ 0.422/ 0.525   0.149/ 0.349/ 0.448   0.110/ 0.228/ 0.301
    wrist_roll   0.170/ 0.374/ 0.444   0.135/ 0.328/ 0.388   0.102/ 0.207/ 0.254

Still-hold contact, N m: each pair = mean / rms (hold:* only)
         joint           current        v2 gravity     v2 + friction
  shoulder_pan   +0.024/  0.032   +0.012/  0.024   +0.012/  0.024
 shoulder_lift   -0.080/  0.162   -0.030/  0.112   -0.030/  0.112
    elbow_flex   -0.005/  0.084   +0.004/  0.081   +0.005/  0.083
    wrist_flex   -0.032/  0.241   -0.019/  0.229   -0.018/  0.228
     wrist_yaw   -0.170/  0.171   -0.081/  0.084   -0.081/  0.084
    wrist_roll   -0.133/  0.137   -0.072/  0.080   -0.072/  0.080

Wrist rest bias, N m (hold:* only, current -> v2):
      wrist_yaw  -0.170 ->  -0.081
     wrist_roll  -0.133 ->  -0.072

Push amplitude, N m: min / max; amplitude = baseline-relative p01 / p99 (first 2 s baseline mean)
         joint           current range; amplitude        v2 gravity range; amplitude     v2 + friction range; amplitude  max friction delta
  shoulder_pan      -2.338/+1.689; -2.269/+1.642      -2.350/+1.677; -2.269/+1.642      -2.350/+1.677; -2.269/+1.642            0.000e+00
 shoulder_lift      -2.423/+2.677; -1.629/+1.603      -2.429/+2.666; -1.627/+1.601      -2.429/+2.666; -1.627/+1.601            2.329e-03
    elbow_flex      -1.799/+2.148; -1.379/+2.127      -1.835/+2.103; -1.375/+2.124      -1.835/+2.103; -1.375/+2.124            4.559e-02
    wrist_flex      -0.794/+0.761; -0.324/+1.094      -0.824/+0.730; -0.323/+1.094      -0.824/+0.730; -0.323/+1.094            2.995e-02
     wrist_yaw      -0.424/+1.070; -0.056/+1.225      -0.340/+1.153; -0.056/+1.224      -0.340/+1.153; -0.056/+1.224            0.000e+00
    wrist_roll      -0.275/-0.063; -0.068/+0.041      -0.221/-0.009; -0.068/+0.041      -0.221/-0.009; -0.068/+0.041            0.000e+00
Maximum absolute commanded speed in push: 2.595e-01 deg/s
Maximum friction delta excluding push-window boundary ticks: 1.634e-13 N m

TRAIN 20261020-182410 -> TEST 20261020-181208
         joint  Coulomb Nm  viscous Nm/(deg/s)  v_s deg/s             n (+/-)  status
  shoulder_pan    0.000000            0.000000        3.0     0 (   0/   0)  unidentifiable: no commanded motion
 shoulder_lift    0.000000            0.020737        3.0  2310 (1137/1173)  fitted
    elbow_flex    0.179141            0.000000        1.0  2310 (1167/1143)  fitted
    wrist_flex    0.230088            0.000000        2.0  2310 ( 997/1313)  fitted
     wrist_yaw    0.000000            0.144852        3.0   893 ( 440/ 453)  weak excitation: local slope only; Coulomb unidentified
    wrist_roll    0.000000            0.365245        3.0   893 ( 440/ 453)  weak excitation: local slope only; Coulomb unidentified

=== Evaluate 20261020-181208 (2175 selected free-motion ticks) ===
Free motion, N m: each triple = rms / p95 / p99 (absolute percentiles)
         joint                current             v2 gravity          v2 + friction
  shoulder_pan   0.042/ 0.076/ 0.103   0.033/ 0.064/ 0.091   0.033/ 0.064/ 0.091
 shoulder_lift   0.304/ 0.589/ 0.863   0.277/ 0.517/ 0.831   0.222/ 0.443/ 0.743
    elbow_flex   0.231/ 0.433/ 0.598   0.232/ 0.433/ 0.594   0.148/ 0.278/ 0.421
    wrist_flex   0.259/ 0.442/ 0.551   0.257/ 0.443/ 0.533   0.164/ 0.302/ 0.382
     wrist_yaw   0.213/ 0.429/ 0.514   0.158/ 0.349/ 0.428   0.114/ 0.226/ 0.297
    wrist_roll   0.185/ 0.408/ 0.458   0.149/ 0.351/ 0.403   0.120/ 0.237/ 0.277

Motion selection audit: 2917 move/approach ticks, 2777 away from fold, 2175 after transient cuts.
All away-from-fold motion INCLUDING transients (N m; rms / p95 / p99):
  shoulder_pan   0.042/ 0.076/ 0.103   0.033/ 0.064/ 0.091   0.033/ 0.064/ 0.091
 shoulder_lift   0.363/ 0.754/ 1.177   0.337/ 0.723/ 1.142   0.260/ 0.558/ 0.899
    elbow_flex   0.261/ 0.525/ 0.737   0.261/ 0.517/ 0.724   0.175/ 0.364/ 0.551
    wrist_flex   0.265/ 0.461/ 0.580   0.264/ 0.461/ 0.564   0.170/ 0.312/ 0.396
     wrist_yaw   0.206/ 0.394/ 0.502   0.146/ 0.332/ 0.418   0.110/ 0.216/ 0.288
    wrist_roll   0.178/ 0.381/ 0.451   0.138/ 0.325/ 0.398   0.114/ 0.229/ 0.269

Still-hold contact, N m: each pair = mean / rms (hold:* only)
         joint           current        v2 gravity     v2 + friction
  shoulder_pan   +0.031/  0.040   +0.019/  0.032   +0.019/  0.032
 shoulder_lift   -0.083/  0.165   -0.033/  0.115   -0.032/  0.115
    elbow_flex   -0.011/  0.075   -0.002/  0.076   -0.001/  0.077
    wrist_flex   -0.035/  0.239   -0.022/  0.226   -0.022/  0.226
     wrist_yaw   -0.179/  0.180   -0.090/  0.092   -0.090/  0.092
    wrist_roll   -0.146/  0.149   -0.084/  0.091   -0.084/  0.091

Wrist rest bias, N m (hold:* only, current -> v2):
      wrist_yaw  -0.179 ->  -0.090
     wrist_roll  -0.146 ->  -0.084

Push amplitude, N m: min / max; amplitude = baseline-relative p01 / p99 (first 2 s baseline mean)
         joint           current range; amplitude        v2 gravity range; amplitude     v2 + friction range; amplitude  max friction delta
  shoulder_pan      -0.068/+0.274; -0.056/+0.176      -0.080/+0.262; -0.056/+0.176      -0.080/+0.262; -0.056/+0.176            0.000e+00
 shoulder_lift      -2.216/+2.231; -2.019/+2.246      -2.222/+2.221; -2.017/+2.245      -2.222/+2.221; -2.017/+2.245            2.152e-03
    elbow_flex      -1.379/+1.573; -1.256/+1.528      -1.416/+1.529; -1.252/+1.525      -1.416/+1.529; -1.252/+1.525            4.545e-02
    wrist_flex      -0.582/+0.636; -0.116/+1.019      -0.611/+0.606; -0.115/+1.019      -0.611/+0.606; -0.115/+1.019            2.967e-02
     wrist_yaw      -0.273/-0.116; -0.047/+0.042      -0.188/-0.035; -0.048/+0.041      -0.188/-0.035; -0.048/+0.041            0.000e+00
    wrist_roll      -0.255/-0.090; -0.060/+0.035      -0.201/-0.036; -0.060/+0.035      -0.201/-0.036; -0.060/+0.035            0.000e+00
Maximum absolute commanded speed in push: 2.594e-01 deg/s
Maximum friction delta excluding push-window boundary ticks: 3.666e-13 N m

=== Final friction fit (all supplied free-motion runs; not a validation score) ===
         joint  Coulomb Nm  viscous Nm/(deg/s)  v_s deg/s             n (+/-)  status
  shoulder_pan    0.000000            0.000000        3.0     0 (   0/   0)  unidentifiable: no commanded motion
 shoulder_lift    0.000000            0.020707        3.0  5381 (2674/2707)  fitted
    elbow_flex    0.177691            0.000000        1.0  5381 (2697/2684)  fitted
    wrist_flex    0.230180            0.000000        2.0  5381 (2357/3024)  fitted
     wrist_yaw    0.000000            0.134874        3.0  2682 (1323/1359)  weak excitation: local slope only; Coulomb unidentified
    wrist_roll    0.000000            0.310790        3.0  2682 (1323/1359)  weak excitation: local slope only; Coulomb unidentified

Wrote configs/viscous/viscous_01/gravity-air-fit-v2.json
```

## Task 5: four --analyze outputs (verbatim)

These use each complete run with the old and final v2 fits. Unlike the run-held-out tables above, the v2 friction table here uses the final three-run coefficients, so it is descriptive rather than held-out evaluation. The printed `still rms` here preserves the existing script's `hold:*` + `settle:*` selection; task 4 reports holds alone. The uncompensated direction columns require |v| > 1 deg/s, so NaN for a stationary pan or a 0.45 deg/s roll means no qualifying direction samples. The saved sag/feedforward measurements and telemetry slope checks remain unchanged; only fit-dependent predictions/contact are recomputed.

### 181208, old

```sh
.venv/bin/python scripts/viscous-contact-check.py --analyze configs/viscous/viscous_01/evidence/contact-check-20261020-181208.json --fit configs/viscous/viscous_01/gravity-air-fit.json
```

```text

=== viscous_01 contact check 20261020-181208: complete ===

Feedforward A/B (mean of above/below; friction band = half their difference):
    pose         joint  fitted  sag@0%   pred sag@100%  band contact@0   @100
   reach shoulder_lift   +1.74   +3.50  +3.32    -0.14  0.07     +0.00  -0.14
   reach    elbow_flex   -0.73   -1.17  -1.40    -0.08  0.13     +0.07  -0.08
   reach    wrist_flex   +0.05   +0.32  +0.10    +0.21  0.09     +0.10  +0.10
 forward shoulder_lift   +2.40   +5.04  +4.58    -0.01  0.02     +0.15  -0.08
 forward    elbow_flex   +2.72   +5.17  +5.19    +0.19  0.05     -0.05  +0.08
 forward    wrist_flex   +1.47   +2.77  +2.80    -0.00  0.53     -0.04  -0.02
    back shoulder_lift   -2.12   -4.54  -4.05    -0.09  0.01     -0.31  -0.12
    back    elbow_flex   -2.86   -5.47  -5.47    -0.03  0.01     -0.03  -0.05
    back    wrist_flex   -1.54   -3.40  -2.93    -0.16  0.29     -0.25  -0.09

Telemetry vs kp*error + feedforward at static holds, ff 0 % (slope, r):
  shoulder_lift slope  0.99 r  1.00
     elbow_flex slope  1.00 r  1.00
     wrist_flex slope  1.00 r  1.00

Telemetry vs kp*error + feedforward at static holds, ff 100 % (slope, r):
  shoulder_lift slope  0.99 r  1.00
     elbow_flex slope  1.00 r  1.00
     wrist_flex slope  1.00 r  1.00

Free-motion contact torque (N m): the floor a real contact must clear
        joint     n    rms    p95    p99    max    v>0    v<0  still rms
 shoulder_pan  2175   0.04   0.08   0.10   0.14   +nan   +nan       0.04
shoulder_lift  2175   0.30   0.59   0.86   1.25  +0.12  -0.20       0.17
   elbow_flex  2175   0.23   0.43   0.60   0.87  +0.15  -0.20       0.08
   wrist_flex  2175   0.26   0.44   0.55   0.68  +0.16  -0.23       0.24
    wrist_yaw  2175   0.21   0.43   0.51   0.59  +0.04  -0.29       0.18
   wrist_roll  2175   0.19   0.41   0.46   0.52   +nan   +nan       0.15
  Selection: move/approach, lift < -15 deg, elbow > 15 deg; acceleration <= 50 deg/s^2,
             excluding 0.15 s around high acceleration and phase boundaries.

Push test (contact torque, N m): baseline = first 2 s of the window
   shoulder_pan baseline rms  0.04  range -0.07..+0.27  above free-motion p99    8 % of the window
  shoulder_lift baseline rms  0.12  range -2.22..+2.23  above free-motion p99   12 % of the window
     elbow_flex baseline rms  0.06  range -1.38..+1.57  above free-motion p99   12 % of the window
     wrist_flex baseline rms  0.40  range -0.58..+0.64  above free-motion p99    6 % of the window
      wrist_yaw baseline rms  0.19  range -0.27..-0.12  above free-motion p99    0 % of the window
     wrist_roll baseline rms  0.15  range -0.25..-0.09  above free-motion p99    0 % of the window
```

### 181208, v2

```sh
.venv/bin/python scripts/viscous-contact-check.py --analyze configs/viscous/viscous_01/evidence/contact-check-20261020-181208.json --fit configs/viscous/viscous_01/gravity-air-fit-v2.json
```

```text

=== viscous_01 contact check 20261020-181208: complete ===

Feedforward A/B (mean of above/below; friction band = half their difference):
    pose         joint  fitted  sag@0%   pred sag@100%  band contact@0   @100
   reach shoulder_lift   +1.71   +3.50  +3.27    -0.14  0.07     +0.05  -0.09
   reach    elbow_flex   -0.73   -1.17  -1.39    -0.08  0.13     +0.07  -0.08
   reach    wrist_flex   +0.08   +0.32  +0.16    +0.21  0.09     +0.09  +0.10
 forward shoulder_lift   +2.43   +5.04  +4.64    -0.01  0.02     +0.14  -0.08
 forward    elbow_flex   +2.77   +5.17  +5.29    +0.19  0.05     -0.10  +0.03
 forward    wrist_flex   +1.53   +2.77  +2.92    -0.00  0.53     -0.07  -0.05
    back shoulder_lift   -2.22   -4.54  -4.24    -0.09  0.01     -0.19  -0.01
    back    elbow_flex   -2.93   -5.47  -5.60    -0.03  0.01     +0.05  +0.01
    back    wrist_flex   -1.58   -3.40  -3.01    -0.16  0.29     -0.18  -0.03

Telemetry vs kp*error + feedforward at static holds, ff 0 % (slope, r):
  shoulder_lift slope  0.99 r  1.00
     elbow_flex slope  1.00 r  1.00
     wrist_flex slope  1.00 r  1.00

Telemetry vs kp*error + feedforward at static holds, ff 100 % (slope, r):
  shoulder_lift slope  0.99 r  1.00
     elbow_flex slope  1.00 r  1.00
     wrist_flex slope  1.00 r  1.00

Free-motion contact torque (N m): the floor a real contact must clear
        joint     n    rms    p95    p99    max    v>0    v<0  still rms
 shoulder_pan  2175   0.03   0.06   0.09   0.13   +nan   +nan       0.03
shoulder_lift  2175   0.28   0.52   0.83   1.26  +0.13  -0.19       0.12
   elbow_flex  2175   0.23   0.43   0.59   0.83  +0.14  -0.22       0.08
   wrist_flex  2175   0.26   0.44   0.53   0.70  +0.15  -0.23       0.23
    wrist_yaw  2175   0.16   0.35   0.43   0.50  +0.13  -0.21       0.09
   wrist_roll  2175   0.15   0.35   0.40   0.47   +nan   +nan       0.09
  Selection: move/approach, lift < -15 deg, elbow > 15 deg; acceleration <= 50 deg/s^2,
             excluding 0.15 s around high acceleration and phase boundaries.

Free-motion contact torque WITH friction compensation (N m):
        joint     n    rms    p95    p99    max  still rms
 shoulder_pan  2175   0.03   0.06   0.09   0.13       0.03
shoulder_lift  2175   0.22   0.44   0.74   1.02       0.12
   elbow_flex  2175   0.15   0.28   0.42   0.65       0.08
   wrist_flex  2175   0.16   0.30   0.38   0.51       0.23
    wrist_yaw  2175   0.11   0.23   0.30   0.38       0.09
   wrist_roll  2175   0.12   0.24   0.27   0.34       0.09

Push test (contact torque, N m): baseline = first 2 s of the window
   shoulder_pan baseline rms  0.04  range -0.08..+0.26  above free-motion p99    8 % of the window
  shoulder_lift baseline rms  0.13  range -2.22..+2.22  above free-motion p99   13 % of the window
     elbow_flex baseline rms  0.10  range -1.42..+1.53  above free-motion p99   13 % of the window
     wrist_flex baseline rms  0.43  range -0.61..+0.61  above free-motion p99   66 % of the window
      wrist_yaw baseline rms  0.10  range -0.19..-0.03  above free-motion p99    0 % of the window
     wrist_roll baseline rms  0.09  range -0.20..-0.04  above free-motion p99    0 % of the window
```

### 182410, old

```sh
.venv/bin/python scripts/viscous-contact-check.py --analyze configs/viscous/viscous_01/evidence/contact-check-20261020-182410.json --fit configs/viscous/viscous_01/gravity-air-fit.json
```

```text

=== viscous_01 contact check 20261020-182410: complete ===

Feedforward A/B (mean of above/below; friction band = half their difference):
    pose         joint  fitted  sag@0%   pred sag@100%  band contact@0   @100
   reach shoulder_lift   +1.77   +3.53  +3.37    -0.10  0.04     +0.01  -0.12
   reach    elbow_flex   -0.71   -1.05  -1.35    -0.13  0.02     +0.12  -0.10
   reach    wrist_flex   +0.06   +0.33  +0.12    +0.26  0.05     +0.09  +0.12
 forward shoulder_lift   +2.40   +5.04  +4.58    -0.03  0.03     +0.15  -0.08
 forward    elbow_flex   +2.72   +5.17  +5.19    +0.20  0.03     -0.05  +0.09
 forward    wrist_flex   +1.47   +2.79  +2.80    -0.02  0.55     -0.03  -0.02
    back shoulder_lift   -2.12   -4.52  -4.05    -0.10  0.01     -0.31  -0.12
    back    elbow_flex   -2.86   -5.51  -5.47    -0.04  0.02     -0.04  -0.05
    back    wrist_flex   -1.54   -3.41  -2.93    -0.17  0.29     -0.26  -0.10

Telemetry vs kp*error + feedforward at static holds, ff 0 % (slope, r):
  shoulder_lift slope  1.00 r  1.00
     elbow_flex slope  1.00 r  1.00
     wrist_flex slope  1.00 r  1.00

Telemetry vs kp*error + feedforward at static holds, ff 100 % (slope, r):
  shoulder_lift slope  1.00 r  1.00
     elbow_flex slope  1.00 r  1.00
     wrist_flex slope  1.00 r  1.00

Free-motion contact torque (N m): the floor a real contact must clear
        joint     n    rms    p95    p99    max    v>0    v<0  still rms
 shoulder_pan  2310   0.03   0.06   0.08   0.12   +nan   +nan       0.03
shoulder_lift  2310   0.31   0.61   0.97   1.43  +0.13  -0.18       0.17
   elbow_flex  2310   0.24   0.43   0.64   1.23  +0.16  -0.20       0.09
   wrist_flex  2310   0.26   0.45   0.55   0.73  +0.16  -0.22       0.24
    wrist_yaw  2310   0.21   0.44   0.53   0.59  +0.05  -0.30       0.17
   wrist_roll  2310   0.18   0.40   0.44   0.47   +nan   +nan       0.14
  Selection: move/approach, lift < -15 deg, elbow > 15 deg; acceleration <= 50 deg/s^2,
             excluding 0.15 s around high acceleration and phase boundaries.

Push test (contact torque, N m): baseline = first 2 s of the window
   shoulder_pan baseline rms  0.03  range -2.34..+1.69  above free-motion p99   18 % of the window
  shoulder_lift baseline rms  0.12  range -2.42..+2.68  above free-motion p99   11 % of the window
     elbow_flex baseline rms  0.11  range -1.80..+2.15  above free-motion p99   11 % of the window
     wrist_flex baseline rms  0.41  range -0.79..+0.76  above free-motion p99   11 % of the window
      wrist_yaw baseline rms  0.18  range -0.42..+1.07  above free-motion p99    5 % of the window
     wrist_roll baseline rms  0.15  range -0.28..-0.06  above free-motion p99    0 % of the window
```

### 182410, v2

```sh
.venv/bin/python scripts/viscous-contact-check.py --analyze configs/viscous/viscous_01/evidence/contact-check-20261020-182410.json --fit configs/viscous/viscous_01/gravity-air-fit-v2.json
```

```text

=== viscous_01 contact check 20261020-182410: complete ===

Feedforward A/B (mean of above/below; friction band = half their difference):
    pose         joint  fitted  sag@0%   pred sag@100%  band contact@0   @100
   reach shoulder_lift   +1.74   +3.53  +3.32    -0.10  0.04     +0.05  -0.07
   reach    elbow_flex   -0.70   -1.05  -1.34    -0.13  0.02     +0.12  -0.10
   reach    wrist_flex   +0.09   +0.33  +0.18    +0.26  0.05     +0.09  +0.12
 forward shoulder_lift   +2.43   +5.04  +4.64    -0.03  0.03     +0.14  -0.09
 forward    elbow_flex   +2.77   +5.17  +5.29    +0.20  0.03     -0.09  +0.04
 forward    wrist_flex   +1.53   +2.79  +2.92    -0.02  0.55     -0.06  -0.05
    back shoulder_lift   -2.22   -4.52  -4.24    -0.10  0.01     -0.19  -0.02
    back    elbow_flex   -2.93   -5.51  -5.60    -0.04  0.02     +0.03  +0.01
    back    wrist_flex   -1.58   -3.41  -3.02    -0.17  0.29     -0.18  -0.03

Telemetry vs kp*error + feedforward at static holds, ff 0 % (slope, r):
  shoulder_lift slope  1.00 r  1.00
     elbow_flex slope  1.00 r  1.00
     wrist_flex slope  1.00 r  1.00

Telemetry vs kp*error + feedforward at static holds, ff 100 % (slope, r):
  shoulder_lift slope  1.00 r  1.00
     elbow_flex slope  1.00 r  1.00
     wrist_flex slope  1.00 r  1.00

Free-motion contact torque (N m): the floor a real contact must clear
        joint     n    rms    p95    p99    max    v>0    v<0  still rms
 shoulder_pan  2310   0.02   0.05   0.06   0.10   +nan   +nan       0.02
shoulder_lift  2310   0.28   0.55   0.92   1.31  +0.14  -0.16       0.12
   elbow_flex  2310   0.24   0.43   0.61   1.17  +0.15  -0.21       0.09
   wrist_flex  2310   0.26   0.45   0.52   0.73  +0.15  -0.23       0.23
    wrist_yaw  2310   0.16   0.36   0.46   0.50  +0.14  -0.22       0.08
   wrist_roll  2310   0.14   0.34   0.39   0.42   +nan   +nan       0.08
  Selection: move/approach, lift < -15 deg, elbow > 15 deg; acceleration <= 50 deg/s^2,
             excluding 0.15 s around high acceleration and phase boundaries.

Free-motion contact torque WITH friction compensation (N m):
        joint     n    rms    p95    p99    max  still rms
 shoulder_pan  2310   0.02   0.05   0.06   0.10       0.02
shoulder_lift  2310   0.23   0.47   0.78   1.25       0.12
   elbow_flex  2310   0.15   0.29   0.46   0.99       0.09
   wrist_flex  2310   0.17   0.31   0.41   0.56       0.23
    wrist_yaw  2310   0.11   0.23   0.31   0.37       0.08
   wrist_roll  2310   0.11   0.22   0.26   0.29       0.08

Push test (contact torque, N m): baseline = first 2 s of the window
   shoulder_pan baseline rms  0.02  range -2.35..+1.68  above free-motion p99   18 % of the window
  shoulder_lift baseline rms  0.13  range -2.43..+2.67  above free-motion p99   11 % of the window
     elbow_flex baseline rms  0.15  range -1.84..+2.10  above free-motion p99   12 % of the window
     wrist_flex baseline rms  0.44  range -0.82..+0.73  above free-motion p99   39 % of the window
      wrist_yaw baseline rms  0.10  range -0.34..+1.15  above free-motion p99    5 % of the window
     wrist_roll baseline rms  0.09  range -0.22..-0.01  above free-motion p99    0 % of the window
```

## Validation

Commands used `.venv/bin/python`, `.venv/bin/pytest`, and `uvx ruff` with an explicit touched-file list only. Neither `frontend/dist/` nor `scripts/viscous-dream1/` was linted or formatted.

```sh
.venv/bin/pytest tests/test_contact_torque.py tests/test_arm_gravity.py tests/test_gravity_feedforward.py -q
```

Final focused result (verbatim summary):

```text
======================= 53 passed, 23 warnings in 3.53s ========================
```

```sh
.venv/bin/pytest
```

Final full-suite result (verbatim summary):

```text
FAILED tests/test_arm_registry.py::test_single_device_configs_carry_the_port_and_id[viscous] - FileNotFoundError: Missing commissioned Viscous hardware profile: /var/folders/2g/08qkq2wj087grfg6w31m979c0000gn/T/makermodslab-home-9kxgrpkc/viscous_profiles/cal-f.json
====== 1 failed, 4311 passed, 19 skipped, 34 warnings in 86.03s (0:01:26) ======
```

This is the one known pre-existing failure specified in the brief. Other warnings include existing FastAPI/httpx deprecations, the matrix warnings discussed above, and an unrelated accept-thread `ConnectionAbortedError` warning from `test_wait_for_port_probes_the_host_it_is_given`; no additional test failed.

```sh
uvx ruff check scripts/viscous-gravity-fit.py scripts/viscous-contact-fit.py scripts/viscous_contact_analysis.py scripts/viscous-contact-check.py makermodslab/contact_torque.py tests/test_contact_torque.py
uvx ruff format --check scripts/viscous-gravity-fit.py scripts/viscous-contact-fit.py scripts/viscous_contact_analysis.py scripts/viscous-contact-check.py makermodslab/contact_torque.py tests/test_contact_torque.py
git diff --check
```

Ruff: `All checks passed!`; format: `6 files already formatted`; diff whitespace check: clean. The original fit and commanded feedforward module have no diff. All edits remain uncommitted. The report is stored beside the fit/evidence so it is visible to Git without changing the repository's rule that ignores new scratch files in `docs/`.

