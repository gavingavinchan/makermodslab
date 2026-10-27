**The recurring problem is fragmented calibration ownership.** Unit 1 relies on driver defaults, waterproof profiles differ between machines, and posture control, haptics and simulation apply separate interpretations.

**The evidence does not establish 30° of missing physical gripper closure.** Both September and October sources report unit 1’s closed stop near −33°, contradicting its −2.8° configured limit. Do not remove the −33° offset until the actual jaw endpoints are measured.

This report uses the current worktree and supplied snapshots. I did not modify files, access hardware or SSH; the running Linux driver version remains unverified.

**1. Inventory and runtime precedence**

Angles below are degrees. Ordinary teleop computes `follower = m × unwrapped_leader + b`, then clamps. Follower CAN IDs are 1–7; corresponding Star IDs are 0–6.

| Joint | Unit 1 follower limits | Unit 1 `m`; `b` | Waterproof follower limits, Linux/bundle | Waterproof `m`; `b` |
|---|---:|---:|---:|---:|
| shoulder_pan | −185.4…127.6 | +1.439742; 0 | −126.852856…187.529086 | +1.446099; 0 |
| shoulder_lift | −176.4…−2.4 | −0.868697; 0 | −199.446608…−2.807493 | −0.981723; 0 |
| elbow_flex | 2.9…251.1 | −0.915867; 0 | 2.864789…250.038782 | −0.912081; 0 |
| wrist_flex | −120.8…92 | −1.082952; 0 | −122.269193…82.104852 | −1.040071; 0 |
| wrist_yaw | −89.2…86.2 | −1.260963; 0 | −86.058261…87.318768 | −1.246420; 0 |
| wrist_roll | −151.3…153.6 | +0.945426; 0 | −154.985084…150.000351 | +0.945691; 0 |
| gripper | −120…−2.8 | −1.778452; −33 | −115.107221…−2.922085 | −1.702354; 0 |

Sources: [dry follower:46](.venv/lib/python3.12/site-packages/lerobot/robots/viscous_follower/config_viscous_follower.py:46), [dry leader:61](.venv/lib/python3.12/site-packages/lerobot/teleoperators/rebot_102_leader/config_rebot_102_leader_viscous.py:61), [waterproof profile:2](configs/viscous/viscous_waterproof_01/viscous_profiles/viscous_waterproof_01.json:2). Values shown rounded where necessary.

| Joint | Archived Star raw travel¹ | Dry calibration ranges: follower / leader | Waterproof calibration ranges² | Follower CAD URDF limits, radians |
|---|---:|---:|---:|---:|
| pan | −107.1…110.3 | −185…127 / same | −126…187 | −3.235…2.227 |
| lift | −0.9…199.4 | −176…−2 / −176…−3 | −199…−3 | −3.078…0 |
| elbow | −270.5…0.5 | 2…251 / 3…251 | 3…250 | 0…4.383 |
| flex | −101.2…95.3 | −120…92 / same | −122…82 | −2.109…1.606 |
| yaw | −71.4…67.7 | −89…86 / same | −86…87 | −1.556…1.505 |
| roll | −159.2…163.3 | −151…153 / same | −154…150 | −2.641…2.680 |
| gripper | −1.3…64.6 | −120…−2 / −120…−3 | −115…−3 | Each sliding jaw: 0…0.0524125 metres |

¹ Historical capture reused by both units, not fresh measurements of both leaders. ² Same range values for leader and follower, on both machines—even where profiles differ. All calibration `homing_offset` and `drive_mode` fields are zero. Sources: [Star capture:3](configs/viscous/viscous_01/evidence/commissioning/star-ranges-2026-08-29.json:3), [dry follower JSON:2](configs/viscous/viscous_01/calibration/robots/viscous_follower/viscous_01.json:2), [dry leader JSON:2](configs/viscous/viscous_01/calibration/teleoperators/rebot_102_leader/star_viscous.json:2), [waterproof JSON:2](configs/viscous/viscous_waterproof_01/calibration/robots/viscous_follower/viscous_waterproof_01.json:2), [URDF:1393](configs/viscous/model/viscous_arm.urdf:1393).

| Definition location | Unit 1 | Waterproof / disagreement |
|---|---|---|
| Lab robot records | Select `viscous_01` + `star_viscous`; Linux wrist/front cameras | Select waterproof IDs. Records select configurations; they contain no joint coefficients. [Linux dry:10](/tmp/viscous-state/gavin-linux/.makermods/makermodslab-viscous/robots/viscous_01.json:10), [Mac waterproof:10](/tmp/viscous-state/mac/.makermods/makermodslab-viscous/robots/viscous_waterproof_01.json:10) |
| `<Lab home>/viscous_profiles/` | **Absent on both machines**, also absent from recovery bundle | Two identical profile copies per machine, named for follower/leader IDs. **Mac gripper limit is −115.107221…−0.034377; leader clamp −115…0.** Linux/bundle retain the table above. [Mac profile:28](/tmp/viscous-state/mac/.makermods/makermodslab-viscous/viscous_profiles/viscous_waterproof_01.json:28) |
| Installed LeRobot presets | Runtime fallback; leader clamps use inward-rounded follower limits | Overridden by commissioned profiles. Unit 1’s fallback is explicitly documented. [bundle README:16](configs/viscous/viscous_01/README.md:16) |
| Calibration JSONs | Bundled and present in both machine caches | Same arrangement. They are metadata, not measured Star raw travel. Hardware zeros live in actuator flash. [calibration builder:354](makermodslab/arms/can_common.py:354) |
| Commissioning evidence | Raw ROM, J5 recapture, zero-frame JSON, final YAML plus `.bak`/`.prezero`; YAML uses radians, direction +1, offsets 0 | Separate waterproof captures/YAML/limits and J1 re-zero evidence. Historical evidence is preserved, but is not loaded into ordinary teleop. [dry contents:84](configs/viscous/viscous_01/README.md:84) |
| Lab posture mapper | Refused for dry unit | Six signed unit gains `(+ − − − − +)`, offsets 0; unwrap centers `(0,99.25,−135,−2.95,−1.85,2.05)`. J1 limits become **−32.372856…282.009086 in RAM only**. Gripper unchanged. [mapper:28](makermodslab/joint_posture_mapping.py:28), [eligibility:110](makermodslab/joint_posture_mapping.py:110) |
| Haptics/actuation paths | POC constructs presets directly, uses **session-relative** mapping and its own −33 closed clamp. `star_follow` mixes posture inverse with scaled gripper inverse; derives leader travel from follower limits | These paths do not consistently consume commissioned profiles. [POC:112](scripts/gripper-haptics-poc.py:112), [relative target:256](scripts/gripper-haptics-poc.py:256), [inverse:59](makermodslab/star_follow.py:59) |
| Sim mapping profiles | **Draft**: signed unit gains, zero reference anchors, same unwrap centers; gripper leader 0→65.9, follower −33→−120 | Also draft; corrected J1 limits, gripper follower −2.922085→−115.107221. Assumed leader **0→65.9** incorrectly treats archived travel width as endpoints. [dry draft:6](viscous-arm-model/leader/mapping/profiles/viscous_01.json:6), [wet draft:76](viscous-arm-model/leader/mapping/profiles/viscous_waterproof_01.json:76) |
| URDF / frontend / gravity | Lab and sibling follower URDFs are byte-identical. Gravity passes motor degrees directly to radians. **No Viscous frontend model or URDF telemetry mapping exists.** Maker’s separate preview conversion is not used | Same. Star CAD limits are nominal preview ranges, not encoder calibration. Visual meshes are missing from Lab’s vendored model. [gravity:120](makermodslab/arm_gravity.py:120), [frontend:107](frontend/src/lib/urdfConfigs.ts:107), [model README:22](configs/viscous/model/README.md:22), [Star caveats:80](viscous-arm-model/leader/README.md:80) |

Runtime precedence is:

`record IDs → calibration staging → family builder → commissioned profile, otherwise dry defaults → Star unwrap/scale/offset/clamp → optional waterproof posture override → follower clamp/startup handling → CAN`.

See [factory:133](makermodslab/utils/robot_factory.py:133), [profile loader:49](makermodslab/arms/viscous.py:49), [leader mapping:364](.venv/lib/python3.12/site-packages/lerobot/teleoperators/rebot_102_leader/rebot_102_leader.py:364), [teleop loop:1075](makermodslab/teleoperate.py:1075).

The pairing check compares configured bounds—not whether measured leader endpoints actually reach them. Lab also uses `connect(calibrate=False)`, skipping the Star driver’s configured 30° start check. [pair check:110](makermodslab/arms/viscous.py:110), [connection:777](makermodslab/teleoperate.py:777), [driver gate:179](.venv/lib/python3.12/site-packages/lerobot/teleoperators/rebot_102_leader/rebot_102_leader.py:179).

**2. Why it recurs**

Three different concepts share ambiguous fields: physical encoder frames, desired control mapping, and display geometry. `joint_directions` combines sign and scale; `joint_ranges` means follower output bounds but also determines leader unwrapping. Haptics then incorrectly inverts those bounds into “measured” leader travel.

Travel-width scaling alone does not align endpoints. For example, archived dry pan travel maps to −154.2…158.8°, versus follower limits −185.4…127.6°: one end remains unreachable while the other clips.

History shows successive local fixes: commissioned profiles (`add5762e`), preserved bundles (`4745784d`, `9eac0f58`), fractional gripper endpoints (`e38826b8`), then a separate posture override (`f825cdca`). None unified ownership.

Waterproof J1 illustrates the consequence: its motor zero changed by 94.48°, while saved limits deliberately remained unchanged; ordinary teleop consequently permits targets beyond the real negative stop. [re-zero evidence:9](configs/viscous/viscous_waterproof_01/evidence/j1-rezero-2026-10-11.json:9).

**3. Proposal that stays fixed**

Use one canonical file per pair: `configs/viscous/<unit>/viscous_profiles/<unit>.json`. Extend the existing mechanism; make leader/follower IDs aliases to that file. Installed profiles become checksummed copies, never independent authorities. Replace the dry fallback and its “do not create profiles” documentation.

Schema:

| Group | Required contents |
|---|---|
| Identity | Schema/revision, unit ID, paired calibration IDs, hardware identities, verification status |
| Encoder frames | Joint IDs, units, zero-pose descriptions, frame revisions, independent leader unwrap centers and measured raw bounds |
| Six arm joints | Direction ±1, measured corresponding leader/follower reference angles, follower soft limits; explicitly selected posture or scaled mode |
| Gripper | Measured leader/follower closed/open endpoints; separate command margins and jaw-gap calibration |
| Model/evidence | Motor-to-URDF signs/zeros, model hash, measurement provenance and repo-relative evidence paths |

Reuse the sim mapper’s explicit anchors and endpoint interpolation, but verify its drafts first. [existing implementation:44](viscous-arm-model/leader/joint_mapping.py:44).

One pure mapping library should serve teleop, recording, haptic forward/inverse calculations and simulation. Generate driver coefficients and calibration metadata from it. Pass explicit follower limits to inference too: today its CLI supplies type/port/ID without commissioned limits. [inference:2292](makermodslab/rollout.py:2292).

Keep CAD geometry separate; derive unit-specific limits/transforms for visualization and sim. Vendor required visual meshes. Include profile/frame hashes in datasets and fits.

**Lab-only:** loader, bundle/restore checks, shared mapping adapter, scripts, visualization and inference arguments. **Fork changes:** explicit independent leader unwrap data and float endpoints for native driver/CLI parity; decouple startup validation from calibration. Correct the false “Star has no motors” comments. [can_common:22](makermodslab/arms/can_common.py:22).

Add offline tests for endpoint equality, matched-pose anchors, direction, ±360 invariance, inverse consistency, clipping and generated-file hashes. Before connection, reject missing/draft/mismatched profiles; before enabling motion, require fresh angles and validate the first mapped target. Preserve speed limiting and explicit engagement.

**4. Gripper mapping and measurements**

Use independently unwrapped Star angle `L`:

```text
u = clamp((L − Lclosed) / (Lopen − Lclosed), 0, 1)
F = Fclosed + u × (Fopen − Fclosed)
```

Use the same endpoints for the inverse haptic mapping; clamp inverse targets to **measured Star travel**.

Current evidence favors `Fclosed ≈ −32.4…−33`, `Fopen ≈ −120`. The preset already calls −33 the physical closed stop; October reports stop loading there. The “zero = gripper open” comment conflicts with this evidence. Neither endpoint should be treated as newly verified. [preset:79](.venv/lib/python3.12/site-packages/lerobot/teleoperators/rebot_102_leader/config_rebot_102_leader_viscous.py:79), [October findings:1222](docs/viscous-haptics.md:1222).

The offset is not itself a −33 command cap: raw −4 maps to −25.9°. Removing it could increase pressure against the stop.

Proposed measurement session, not performed:

1. Support the powered arms; establish torque-off beforehand. Use a reviewed telemetry-only path without normal connection handshakes, origin writes or position commands.
2. Human gently moves each empty gripper through closure/opening three times. Record fresh raw angles, visible jaw contact, mechanical stops and jaw gaps; distinguish first closure from squeezing a compliant stop.
3. Record follower raw and whole-turn-corrected angles. `+327.5 → −32.5` is a session branch correction, not a new zero. [driver:170](.venv/lib/python3.12/site-packages/lerobot/robots/maker_follower/maker_follower.py:170).
4. Measure intermediate gaps and corresponding arm reference poses/signs. Select usable endpoints and backoffs explicitly. Do not drive toward the derived −16.9° Star bound: it already contradicted measurement. [actuation finding:1140](docs/viscous-haptics.md:1140).

**5. Powered-motion risks**

Changing signs, anchors, scales or turn branches changes the first target; profile activation must require a stopped session. Increasing gripper closure may create sustained stop pressure. A 1:1 mapping sacrifices follower reach where Star travel is narrower; show that limitation instead of silently rescaling.

Changing encoder/model frames also invalidates gravity assumptions and existing policy data. Preserve recorded revisions and revalidate fits.

Camera orientation is separate: unit 1 lacks rotation metadata, and Lab currently drops rotation during session construction. Confirm rotation versus mirroring before fixing it. [camera filtering:1233](makermodslab/utils/config.py:1233), [camera builder:497](makermodslab/record.py:497).