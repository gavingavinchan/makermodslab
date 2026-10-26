<!-- astra-rov-scene-20261026 -->
## Native underwater BlueROV scene — validated 2026-10-26; no Astra runs

Launch a **fresh** G3 operator episode from `/home/gavin/dreamtrial/viscous_sim`:

```bash
cd /home/gavin/dreamtrial/viscous_sim
MUJOCO_GL=egl PYTHONDONTWRITEBYTECODE=1 TMPDIR="$PWD/astra_op/.cache/tmp" \
  /home/gavin/dreamtrial/.venv/bin/python astra_op/server.py \
  --scene rov --profile astra_rov --condition G3 --seed 20261024 \
  --episode astra_rov_g3_20261026_001 \
  --output runs/astra_flange/astra_rov_g3_20261026_001
```

Wait for `ready`; the operator workspace is `~/dreamtrial/astra_ws/astra_rov_g3_20261026_001`. No server remains running from these checks.

The level, fixed BlueROV2 Heavy retains native meshes, mounts, kinematics, limits and inertials, with solid convex vehicle contacts. Both arms pass native IK/clearance for start, head-side help, bolt tip and a shared hand-off zone. The seabed catches dropped nuts; low sand footings ground the unchanged supports. The front camera is **inside the native dome**, with the CAD optical window omitted only from its internal optical view; all collision hulls remain solid. Wrist mounts are unchanged.

All six centered / 2 mm offset / 6° tilted G3 checks pass across ROV and tabletop: seated nut, `stall`, and `stall_at_start: false`. The clean ROV run reached **9.4350 turns**, **0.136 mm** face gap and the adaptive snug detector in **146.5 s**. Native triangle audit: **3937 poses, zero arm/ROV intersections**. Body-contact stall, sand recovery reach, released-nut stability, pull-off and empty-gripper checks pass. Six camera depth picks have maximum ray error **0.496 mm**.

The thread has a finite compliant face stop and seating constraint; `screw` still detects seating from encoder motion and wrist torque (baseline + 1.5 Nm for 0.3 s below 1°/s), with ordinary saturation guards retained. Releasing resets the learned arm load bias so it cannot kick the unloaded hand into the pipe. Gains, 25 Nm / 160 N actuator caps, jaw geometry and grasp welds remain unchanged. `astra_rov` uses the existing 180°/s screw rate by default; tabletop remains 90°/s. Video exports on shutdown from exact 10 Hz state captures, outside the command watchdog; wait for export completion.

[Video](validation/rov_g3/run.mp4) · [three-camera start/tip sheet](validation/rov_cameras_final/camera_contact_sheet.png) · [native reach / clearance](validation/rov_preflight/reach_clearance.json) · [all certificates and measurements](validation/rov_final_summary.json). Scripted targets use privileged poses only as an explicitly marked **validation cheat**; no Astra/Dream inference, hardware, ledger or runner lock was used. Native mesh audit uses 50 ms telemetry, not every 1 ms integration tick.

<!-- /astra-rov-scene-20261026 -->

---

### Earlier harness reports (historical)

**Adaptive `screw` effort fix — 2026-10-25; requested seated-stall validation remains unmet (no Astra runs).** Removed the 0.15 Nm clamp and its special anti-windup/saturation handling. Gripped strokes now use `turn` tracking at the unchanged **25 Nm actuator limits**. The snug detector requires **<1°/s roll**, **absolute wrist torque ≥ moving baseline + 1.5 Nm**, sustained **0.3 simulated seconds**, using **100 ms** windows. Baseline: median torque while moving >3°/s through **3–15° of each stroke**; an immediately blocked stroke retains the previous moving baseline. `stall_at_start` is true for a stall before **1° forward progress**. [Patch](validation/adaptive_controller.patch), [sensor/response boundary tests](validation/ad_detector_checks.json), [operator README](../../astra_op/README_OPERATOR.md). Working window, release/reset/reclose, feed, ordinary guards, stop reasons, command counting and 300 s deadline remain unchanged.

Fresh Astra-profile CLI results; hidden poses are **validation cheats for placement/certification only**. Replay 1–155 matches the recorded arm joints **bit for bit**. All replay inputs are exact, including earlier `screw` calls under the updated controller, so later trajectories diverge. All three replays satisfy the allowed truthful-stop alternative, not the three-turn target.

| Case | Hand degrees / regrasp cycles | Stop | Final nut turns / seated |
| --- | ---: | --- | --- |
| [Episode 6, 1–155 + 1080°](validation/ad_r155a/adaptive_audit.json) | 16.70 / 0 | `stall`, **real 25 Nm saturation for 0.5 s**, not at start | 0.10816 / no; +0.07229 turn |
| [Episode 6, 1–373 + 1080°](validation/ad_r373a/adaptive_audit.json) | 0 / 0 | `lost_grasp`; empty hand, nut off thread | −0.10021 / no |
| [Episode 6, 1–605 + 1080°](validation/ad_r605a/adaptive_audit.json) | 0 / 0 | `lost_grasp`; empty hand, nut off thread | −0.10021 / no |
| [Centered](validation/ad_seat1/adaptive_audit.json) | 3599.96 / 69 | `completed` | 10.08805 / yes |
| [2 mm off center (measured 1.85 mm)](validation/ad_offset2/adaptive_audit.json) | 3600.00 / 70 | `completed` | 10.08473 / yes |
| [6° tilt (measured 6.07°)](validation/ad_tilt4/adaptive_audit.json) | 3600.15 / 67 | `completed` | 10.05979 / yes |
| [Lower / upper 60° window endpoints](validation/ad_rolllo1/adaptive_audit.json) | 125.51 / 3; 3599.95 / 60 | `loaded_on_open`; [`completed`](validation/ad_rollhi1/adaptive_audit.json) | 0.63344 / no; 10.08887 / yes |

**Seating does not produce the requested physical snug signal in the unchanged model.** Centered seated samples still rotate **11.03–85.33°/s** with wrist torque **≤0.245 Nm**; the face gap ends at **0.131 mm**, yet rotation continues for another ~0.8 turn after entering the seating tolerance. The loose-thread/pull-offset surrogate permits this continued rotation; this is not evidence of a torque stop. Consequently the seated `stall` requirement fails for centered, offset, tilted and upper-window runs. The lower-window run stops honestly with **3.15 N remaining jaw load on open**. No controller stop has been fabricated from hidden seating/thread state. Earlier unreachable/one-pad placement attempts are retained as `ad_offset1` and `ad_tilt1`–`ad_tilt3`.

**Regressions pass:** [release](validation/ad_release/thread_checks.json): zero turn/advance drift over **10 s** stationary plus **12.25 s** noncontact nudging; [10 mm pull-off](validation/ad_pull/gate_checks.json): releases at **3.507 mm**; [empty](validation/ad_empty/screw_checks.json): `lost_grasp`; [signed motion/parser/last](validation/ad_interface/interface_checks.json), [timeout](validation/ad_timeout/interface_checks.json), [loaded-open](validation/ad_loaded/interface_checks.json). Videos fully decode. [Protected original-source hashes match](validation/ad_source_checks.json); physics, thread model and starts were not edited. No Astra/Dream calls, ledger/runner-lock access or hardware; episode 006's server/socket/workspace/output files were only read, never controlled or written.

**All 18 validation servers started here are stopped**, with removed sockets and intact workspace boundaries: [shutdown audit](validation/ad_shutdown.json). The requested robust seated-stall behavior is **not certified**; it requires resolving the missing physical seating response and the lower-window release failure within a separately agreed scope.

---

**Historical 0.15 Nm `screw` validation — superseded by the adaptive-controller results above.** `./robot screw right 3600 [--rate-deg-s R]` performs roll/feed and sensed release/reset/contact-stopped regrasp cycles as **one command**, with a **300 s** deadline and ~10 s progress messages. Default roll rate is 90°/s. Astra defaults are **1000 commands / 21600 s (6 h)**. [Operator README](../../astra_op/README_OPERATOR.md), [controller](../../astra_op/server.py).

The controller preserves the entry approach, follows commanded 3.5 mm/revolution feed, and uses a **60° local working window**, clipped **10° inside physical wrist limits**. The extra window avoids pipe contact during a full-range sweep. A **0.15 Nm controller wrist-effort budget** provides a genuine no-progress stop: full-effort testing seated the nut but continued rotating. Physical 25 Nm limits, physics, thread model and starts remain unchanged. Control uses only encoders/FK, commanded motion, jaw openings/contact loads and torques. Close reacquires the existing 1 mm preload; open verifies actual opening and unloaded jaws.

- **G3 CLI validation cheat passed:** privileged targets only place/center the nut at the tip; then `grip close`, a fed 30° catch, and one **`screw right 3600`**. **3355.84° while gripped (9.3218 hand turns), 9.414184 nut turns, 64 regrasp cycles, `stall`, seated = true, 0.119 mm gap; 201.26 s wall**. Final second: 0.001377° progress, nut/flange contact and no arm/fixture contact. All 64 releases had zero load. [Result](validation/screw_skill/screw_checks.json), [rotation/video/source audit](validation/screw_skill/audit.json), **[video](validation/screw_skill/run.mp4)**.
- **Stops pass:** [empty](validation/screw_empty_final/screw_checks.json) / [off-thread](validation/screw_offthread_verified/screw_checks.json) → `lost_grasp`; [injected loaded-open](validation/screw_loaded_final/interface_checks.json) → `loaded_on_open`; [accelerated timeout](validation/screw_timeout_final/interface_checks.json) → `timed_out`, measured-joint hold and `last`/`status` recovery. Signed-angle/rate/parser/cap checks pass for [right](validation/screw_interface_final/interface_checks.json) and [legacy](validation/screw_interface_legacy/interface_checks.json) settings.
- **Regressions pass:** released-nut tests have zero drift for [right](validation/screw_release/thread_checks.json) and [legacy](validation/screw_release_legacy/thread_checks.json); [10 mm pull-off](validation/screw_pull/gate_checks.json) releases. Profile/bitwise trial-2 initialization and motion guards pass. Video fully decodes; protected original-source hashes match.

No Astra/Dream calls, ledger/runner-lock access, hardware, propeller or original-source edits. **All servers started here are stopped** ([shutdown](validation/screw_shutdown.json)).

---

# Astra flange control harness — 2026-10-25

**Thread-entry chatter fixed; exact episode-3 replay target remains unmet — 2026-10-25.** The old gate summed discarded axial error, so stationary elastic mismatch repeatedly reached its 3 mm release threshold. The harness now retains physical outward pull travel, filters updates below **1 N excess axial load**, releases strictly **one pitch (3.5 mm) behind the engagement position**, and requires an inward return past that position before re-entry. This adds bounded axial pull-off play; physical position includes that offset. Entry gates (3 mm radial/axial, 10°), screw pitch/hand, screw friction law/values, −0.10-turn reverse release and deep-travel safety gate are unchanged. [Exact patch](validation/thread_gate_change.patch), [checks](validation/gate_summary.json).

- **Controlled tip catch passed:** face **2.674 mm beyond the tip**; 60° with matching feed produced **0.166762 turns / 0.583667 mm physical advance**, one engage and zero releases. [Result](validation/gate_tipd/gate_audit.json).
- **Release/pull passed:** both hands show **zero turn/advance drift**, physical axial span below **5e-16 m**, over 10 s waiting plus 12–12.25 s noncontact nudging. A **10 mm CLI withdrawal without turning** releases at **3.507 mm pull**, stays released. [Right](validation/gate_released/thread_checks.json), [legacy](validation/gate_release_legacyd/thread_checks.json), [pull](validation/gate_pulld/gate_checks.json).
- **G3 passed, seated:** **9.33143 turns**, **32.660 mm advance**, **0.299 mm gap**; held gain **0.99930 nut°/commanded°**, released gain zero, no stalls/timeouts. [Video](validation/g3_gated/run.mp4), [audit](validation/g3_gated/audit.json).
- **Exact fresh-server replays 1–366 and 1–517 failed the requested catch criterion.** Changed release at 187 alters the later nut trajectory; after opening at 209, the recorded regrasp at 221 misses. By 360 the hand holds the **bolt**; at 513/515 it is **empty**. Nut turn increments are zero and the nut is away from the tip. Both target windows have zero thread events, so these do not certify the requested replay catch. [366 audit](validation/ep3_gate366d/gate_audit.json), [517 audit](validation/ep3_gate517d/gate_audit.json).

These are privileged scripted validations, not autonomous results. Protected-source hashes match. No Astra/Dream calls; episode 003's server/socket/workspace/outputs were never written or controlled. **All validation servers stopped**, [shutdown check](validation/gate_shutdown.json).

---

**Released-thread friction fixed and validated — 2026-10-25.** The cause was **missing screw-coordinate resistance in both right-hand and legacy models**. Episode 2's opening contacts kicked the nut at 178.55–178.75 s; after contact ended it coasted at **0.621 turns/s**. Isolated old-model tests show no gravity-driven motion from rest, but both hands retain approximately 4 rad/s and accumulate **1.27324 turns in 2 s** after an initial spin: conserved residual spin, rather than a reversed helix sign or growing energy. [Diagnosis](validation/thread_diagnosis/diagnosis.json).

The harness-only [thread model](../../astra_op/thread_model.py) now applies **0.020 Nm breakaway, 0.015 Nm sliding friction, and 0.002 Nm·s/rad viscous drag**. This assumes light dry-thread drag: 1 N tangential breakaway at 20 mm mean radius (e.g. friction coefficient 0.1, 10 N flank preload), equivalent to approximately 35.9 N axial breakaway. These are surrogate values, not calibrated tightening friction. Axial/angular momentum is projected onto the helix; a passive implicit friction impulse and corrected-angle integration prevent static drift. No hand/grasp/command state is consulted. Winding signs, entry gates, 3.5 mm pitch, pull/reverse release gates, initialization and controller remain unchanged. [Physics checks](validation/thread_units/checks.json) pass for breakaway, torque/50 N axial drive, passivity and release gates; 4 rad/s stops within **0.00443 turn**. [Profile/bitwise-initial-state checks](validation/profile_checks/checks.json) pass.

- **Release:** CLI engagement/open, then 100 `status` calls: **10.000 simulated seconds, 0 turn / 0 mm drift**. Nearby open-hand nudges/status: **12.25 s, zero drift/contact**. [Right result](validation/release_friction1/thread_checks.json). Legacy: **10.000 s stationary + 12.000 s nudging, zero drift/contact**; a held 15° command tracks at 0.99930 nut°/commanded°. [Legacy result](validation/release_legacy_friction3/thread_checks.json).
- **Episode 2 replay:** original commands **1–109**, fresh `--profile astra`: **0.02473 turns at 81, 0.15267 at 83, 0.17667 at 100, 0.18786 at 109** (previously 7.58324). Clear-hand moves/observations produce zero drift; subsequent closing/opening contacts produce small additional motion. [Replay](validation/ep2_friction1/thread_checks.json), [video](validation/ep2_friction1/run.mp4).
- **Scripted G3, `--profile astra`: passed, seated**, **9.33143 turns**, **32.660 mm advance**, **0.299 mm face gap**, through both flanges; 285 CLI commands including `done`, zero stalls/timeouts. Held: **9.32680 turns / 3360° = 0.00277583 turn/degree = 0.99930 nut°/commanded°**, normal 25°/s bound. Released: **0 turns / 3255.78°**, 55 clean releases. Peak arm torque **9.469 Nm overall**, **7.751 Nm during threading**, threading wrist-roll **0.358 Nm**, below 25 Nm. [Result](validation/g3_friction1/validation.json), [gain/torques](validation/g3_friction1/thread_gain.json), [video](validation/g3_friction1/run.mp4), [video/relative-rotation audit](validation/g3_friction1/audit.json).

Episode 2 remains invalid as an autonomous result; earlier frictionless results below are historical. These scripts/replays are **no-inference validation cheats**, using privileged target selection/certification and CLI-only actuation. Auxiliary legacy fixture attempts hitting wrist/reach limits remain archived; the clean legacy result uses 15° within wrist travel. Protected-source hashes match the prior audit. No Astra/Dream calls, ledger/runner-lock access, hardware, propeller or original-source edits. **No server remains running.**

---

**Stall and interrupted-client recovery validated — 2026-10-25.** Client/socket faults and malformed input are logged without ending the episode. `./robot last` recovers the saved command/result without stepping physics or consuming a command. Moves, nudges and turns stop on 0.5 simulated seconds of continuous torque saturation, or under 1 mm target progress in 1 simulated second while tracking lags (rotation counts at 50 mm/rad). Stops hold measured joint references and report pose/error, saturated joints, `stalled` / `timed_out`, and clear IK reachability wording. Every command has a 90 s wall timer, including planning, with progress about every 10 s. [Operator instructions](../../astra_op/README_OPERATOR.md) updated.

**Recovery checks passed:** the fresh Astra-profile replay of all 29 recorded inputs returned command 29 **stalled in 0.948 s**, with working `last`/`look` afterward. Earlier stalled moves now stop too. SIGINT mid-move and malformed/disconnected clients survived ([replay/fault results](validation/stall_replay2/resilience.json)). Injected blocked IK stopped at **90.001 s** (90.015 s client latency), printed eight progress lines, held measured joints and recovered ([90 s test](validation/timeout90_1/resilience.json)). Disconnect during progress also passed ([accelerated fault test](validation/progress_drop1/resilience.json)); [guard boundaries](validation/motion_guard_checks.json) and [bounded-memory trajectory sampling](validation/trajectory_iterator_checks.json) passed.

**Scripted G3 regression passed:** **9.3690 turns**, **32.791 mm advance**, engaged, through both flanges, **seated with 0.150 mm gap**. 306 CLI commands including `done`, zero stalls/timeouts, longest request 11.392 s; 59 clean releases. Privileged target selection/certification remains a validation cheat. [Video](validation/g3_resilience1/run.mp4), [result](validation/g3_resilience1/validation.json), [full video/physics audit](validation/g3_resilience1/audit.json). Protected-source hashes match the prior audit. No Astra/Dream calls, ledger/runner-lock access, hardware or protected-source edits. **No harness server remains running.**

---

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
