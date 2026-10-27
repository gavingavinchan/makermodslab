<!-- Claude subagent, read-only, 2026-10-27, brief: investigation-brief-2026-10-27.md. It received the 10-27 gripper range measurement mid-run. Paths: LR/ = .venv/lib/python3.12/site-packages/lerobot/ (fork pin 2a5ea4e58, source ~/Documents/lerobot-viscous-lab). -->

# Unit 1 (`viscous_01`) joint mapping: why it keeps going wrong, and a fix that stays fixed

## Correction to the brief

The gripper DOES close. In unit 1's frame **more-positive = closed, more-negative = open** (fork `docs/source/viscous.mdx:124` "zeroed folded, gripper CLOSED"; `examples/viscous/EXPERIENCE.md:55`; and the 10-27 run: closing on an object stalled the follower at -57, moving toward -33). -32.65 is the real closed stop and "closed Star -> -33" is right. What is lost is the **open** side: open stop -155.9 vs soft limit -120, so ~36 deg (~29 %) of jaw opening is unreachable and the Star saturates at +48.9 raw (its last ~12 deg do nothing). Secondary: a closed Star reads -0.7..-4, which the preset maps to -31.8..-25.9, i.e. 1-7 deg past the stop: a steady 0.3-2.4 N m squeeze into it at kp 20 (the 10-27 POC measured 0.45 N m at 1.4 deg).

## 1. Inventory

| # | Location | What | Wins at runtime? |
|---|---|---|---|
| 1 | `LR/robots/viscous_follower/config_viscous_follower.py:46-56` | follower `joint_limits` (gripper -120, -2.8); comment `:39-40` "gripper open" (wrong) | **Yes** (follower clamp) |
| 2 | `LR/teleoperators/rebot_102_leader/config_rebot_102_leader_viscous.py:61-71,77,85` | leader directions, ranges = derived from #1, `joint_offsets={"gripper": -33.0}` (added 09-10, fork `058abeb9d`) | **Yes** (leader map) |
| 3 | Star servo origin (`set_origin_point`, `rebot_102_leader.py:219`) + RobStride zero in actuator flash | the actual zeros | **Yes**, hardware state, in no file |
| 4 | calibration JSONs (`~/.cache/.../viscous_follower/viscous_01.json`, `.../rebot_102_leader/star_viscous.json`, `configs/viscous/viscous_01/calibration/`, fork `examples/calibration/`, `~/Documents/viscous-arm-calib/`) | `range_min/max` copies of #1/#2 | **No**: RobStride ignores them (`LR/motors/robstride/robstride.py:957-968`), the leader clamps with `config.joint_ranges` (`rebot_102_leader.py:366`). Decoys |
| 5 | `configs/viscous/viscous_01/evidence/commissioning/` (`joint-limits-zeroed-frame.json`, `viscous_arm_v1.yaml:13,26`) | 08-28 capture; YAML also says "zero = fully open" (wrong) | No |
| 6 | `makermodslab/arms/viscous.py:59-64` | no profile file -> fork presets #1/#2 | **Yes** (unit 1's path) |
| 7 | `makermodslab/arms/viscous.py:110-123` | leader/follower pairing check | Yes (check only) |
| 8 | `makermodslab/joint_posture_mapping.py:31` (`LEADER_CENTERS`), `:117` (refuses all but waterproof) | 1:1 posture | Never for unit 1 |
| 9 | `makermodslab/star_follow.py:41,72` | reuses `LEADER_CENTERS` | Draft |
| 10 | `scripts/gripper-haptics-poc.py:98` | own `--follower-closed-deg -33.0` | Scripts |
| 11 | `configs/viscous/model/viscous_arm.urdf:1398..1446` | unit 1 follower limits in rad (copy of #1); gripper prismatic 0..0.0524 m | Gravity uses angles only, limits unused |
| 12 | `~/Documents/viscous-arm-model/leader/mapping/profiles/viscous_01.json` | draft: gripper closed -33 / open -120 (open wrong), leader 0/65.9 | Sim only |
| 13 | `~/Documents/viscous-arm-model/leader/mapping/inputs.json:4-12` | provenance as absolute `/home/gavin/...` paths (breaks the self-contained rule) | No |
| 14 | `makermodslab/arms/urdf.py:23`, `makermodslab/arms/can_common.py:129` | Viscous = degrees readout; no URDF view | Frontend not involved |

| Joint | Unit 1 follower limits (preset) | Unit 1 leader dir / offset | Star travel 08-29 | Waterproof limits (profile) | WP dir / offset |
|---|---|---|---|---|---|
| shoulder_pan | -185.4, 127.6 | 1.4397 / 0 | -107.1..110.3 | -126.9, 187.5 (stale frame) | 1.4461 / 0 |
| shoulder_lift | -176.4, -2.4 | -0.8687 / 0 | -0.9..199.4 | -199.4, -2.8 | -0.9817 / 0 |
| elbow_flex | 2.9, 251.1 | -0.9159 / 0 | -270.5..0.5 | 2.9, 250.0 | -0.9121 / 0 |
| wrist_flex | -120.8, 92.0 | -1.0830 / 0 | -101.2..95.3 | -122.3, 82.1 | -1.0401 / 0 |
| wrist_yaw | -89.2, 86.2 | -1.2610 / 0 | -71.4..67.7 | -86.1, 87.3 | -1.2464 / 0 |
| wrist_roll | -151.3, 153.6 | 0.9454 / 0 | -159.2..163.3 | -155.0, 150.0 | 0.9457 / 0 |
| **gripper** | **-120, -2.8** (measured stops **-155.9 / -32.65**) | **-1.7785 / -33** | -1.3..64.6 (10-27: **-3.1..60.6**) | -115.1, -2.9 (Mac copy: -0.03) | -1.7024 / **0** |

**Unit 1 vs waterproof.** Unit 1 has no profile file at all (fork class defaults). The waterproof unit has `viscous_profiles/` in the repo bundle and on each machine. Its gripper frame is intact (reads -0.41 folded, `configs/viscous/viscous_waterproof_01/evidence/j1-rezero-2026-10-11.json`), hence no offset. But its J1 is a stale frame (re-zeroed 10-11, profile deliberately left in the old frame; only the posture trial shifts it +94.48 in RAM, `joint_posture_mapping.py:121-129`), so ordinary teleop allows ~94 deg past J1's real negative stop. Its profiles have already drifted between machines: the Mac profile (sha `cfc98...`) differs from the repo/Linux copies (`d4404...`) on the gripper's upper bound. Both bundles carry the identical `star-ranges-2026-08-29.json` (same sha): the waterproof directions were computed from unit 1's Star travel (unsure whether it is the same physical Star).

**Runtime precedence (Lab teleop, unit 1):** `teleoperate.py:730-733` -> `robot_factory.build_single_configs` (`:124-138`, stages the inert calibration JSONs) -> `ViscousFamily.build_single_configs` (`viscous.py:105`) -> `_commissioned_config`: no `~/.makermods/makermodslab-viscous/viscous_profiles/viscous_01.json`, so the bare fork config (`viscous.py:59-64`) -> leader `raw(Star EEPROM origin) x direction + offset`, clamped to `joint_ranges` (`rebot_102_leader.py:362-377`) -> follower clamp to `joint_limits` (`LR/robots/maker_follower/maker_follower.py:546-552`, debug-level log only) -> speed cap -> MIT. **The fork presets (#1, #2) plus the hardware zeros win.**

**Unit-1 identity hazard:** on gavin-linux the `viscous_waterproof_01` record points at unit 1's CANable (`207E37A55445`, record line 11). Selecting that record drives unit 1 with the waterproof limits/mapping (e.g. J1 -126..187 against unit 1's -185..127).

## 2. Root cause

**Physically:** unit 1's gripper frame moved rigidly by ~-33 deg between 08-28 and 09-10 (closed stop ~+0.06 -> -32.65; open stop -122.96 from the commissioning YAML -> -155.9). The ~35 deg figure in the coordinator's note compares the shifted stops against the backed-off soft limits; the stop-to-stop shift is ~33 deg. The frame has been steady since (the gripper read -32.63 in every 10-20 run). The fork's troubleshooting doc (`viscous_teleop.mdx:385-390`) names "a slipped gripper coupling" as the usual cause.

**Why it keeps recurring:**
1. **No per-unit source of truth.** Unit 1's numbers are class defaults in another, SHA-pinned repo, and the unit-1 README says not to create a profile (`configs/viscous/viscous_01/README.md:24`). Correcting a physical change properly costs a fork commit plus a pin bump, so every fix lands as a local patch in whichever consumer is being worked on: the 09-10 leader offset of -33 (instead of moving the follower frame; the open-side limit was left stale), the haptics script's own `-33`, the waterproof J1 re-zero patched in RAM only and only in the posture trial.
2. **The mapping is stored as derived numbers, not measured endpoints.** Scale is a travel ratio, offset is separate, ranges are derived from the follower limits. One changed physical fact needs three derived numbers changed together, and the derivation hides the error: the 10-21 Star closed bound of -16.9 is just (-2.8 + 33) / -1.778, a follower limit past the stop.
3. **Decoy copies.** Inert calibration-JSON ranges, the sim draft profile, URDF limits, machine-local profiles that already disagree, the fork's `examples/calibration/`. People edit or consult the wrong one.
4. **No drift detection.** The only frame check is the stale-zero test (>20 deg outside limits and not a whole turn, `maker_follower.py:61,170-202`). A 33 deg shift that leaves the arm resting inside the old limits passes silently. Clamping is logged at debug level only, so a steady squeeze into the closed stop is invisible.
5. **Contradictory documentation.** "Gripper open" in `config_viscous_follower.py:39-40`, the SDK YAML line 13 and `urdf.py:23`; "closed" in `viscous.mdx:124`. `can_common.py:22-23,226,460` says the Star has "no motors", which the 10-21 actuation runs disprove.
6. **Nothing binds a record or profile to a physical arm** (the identity hazard above).

## 3. Proposal: one file per physical unit

**`configs/viscous/<unit_id>/unit.json`**, tracked, in the existing bundle directory. One file per commissioned pair. Its schema extends the sim's `joint_mapping.py` schema, which already uses gripper endpoints (`GripperRule`):

```json
{ "schema_version": 2, "unit_id": "viscous_01", "leader_id": "star_viscous",
  "identity": {"can_adapter_serial": "207E37A55445"},
  "follower": {"zero_pose": "folded, gripper closed",
    "joints": {"gripper": {"stops_deg": [-155.9, -32.65], "limits_deg": [-153.0, -32.6],
                           "rest_deg": -32.6, "rest_tol_deg": 3}, "...": {}}},
  "leader": {"joints": {"gripper": {"travel_deg": [-3.1, 60.6], "unwrap_center_deg": 29}, "...": {}}},
  "mapping": {"mode": "scaled",
    "arm_joints": {"shoulder_pan": {"direction": 1.4397, "leader_reference_deg": 0, "follower_reference_deg": 0}, "...": {}},
    "gripper": {"leader_closed_deg": -0.7, "leader_open_deg": 58, "follower_closed_deg": -32.65, "follower_open_deg": -153.0}},
  "cameras": {"wrist": {"rotation": null}},
  "verification": {"status": "draft|physically_verified", "evidence": [{"path": "evidence/...", "sha256": "..."}]} }
```

The wrist camera's rotation is a fact about the unit; its device index stays per machine in the robot record.

**How each consumer uses it:**
- **Lab teleop, record, replay, inference:** a new `makermodslab/viscous_unit.py` loader compiles the file into the existing lerobot fields (`joint_limits`; `joint_directions`/`joint_offsets` with gripper direction = (f_open - f_closed)/(l_open - l_closed), offset = f_closed - direction x l_closed; `joint_ranges`). `_commissioned_config` uses it and **drops the preset fallback** (`viscous.py:59-64`): a missing or invalid file refuses to start. Retire `~/.makermods/.../viscous_profiles/`; if a copy is present it must be byte-identical to the repo file or the session refuses (this catches the current Mac drift). The restore script then writes only the calibration JSONs and the record.
- **Posture trial:** reads `mapping.arm_joints`. Remove the id gate (`:117`), `LEADER_CENTERS` (`:31`) and the RAM J1 shift (`:121-129`), and put the waterproof J1 new-frame limits in its `unit.json` in the same change.
- **Scripts** (`gripper-haptics-poc.py`, `gripper-range-read.py`, `star_follow.py`, contact/gravity scripts): `load_unit(robot)`; no literal defaults like `--follower-closed-deg`.
- **URDF/visualisation:** nothing today (Viscous uses the degrees readout). Note in `configs/viscous/model/README.md` that URDF limits are model limits, not unit limits. A future URDF view takes the gripper's metres from the endpoints.
- **Sim repo:** replace `leader/mapping/profiles/*.json` with a vendored copy of the Lab file plus source commit/sha (URDF vendoring in reverse); a sim test fails on sha mismatch; remove the absolute paths in `inputs.json`.
- **lerobot fork:** no functional change needed; everything above is Lab-only. Optional: fix the "gripper open" comment; label the Viscous presets "unit 1 as of 08-28, superseded by the Lab unit file"; rate-limited info-level clamp log; account for `joint_offsets` in the leader's unwrap window (`rebot_102_leader.py:372-374`, harmless today because the window is +-180).

**Drift checks:**
1. **pytest (no hardware):** every `unit.json` validates against the schema; gripper endpoints inside limits, limits inside measured stops; the compiled config equals what `_commissioned_config` returns and passes the pairing check; evidence paths repo-relative with matching sha; each record template's follower id maps to a unit file; a sweep (modelled on `tests/test_arm_registry.py`) fails on unit literals (`-33.0`, `1.778452`, `LEADER_CENTERS`, hardcoded unit ids) in `makermodslab/` or `scripts/`.
2. **Startup check (read-only, after connect, before the first `send_action`):** refuse unless the record's adapter serial matches `identity`; refuse a joint more than ~5 deg outside its measured stops after whole-turn correction (tighter than the 20 deg grace); with the leader folded (already enforced by `max_start_offset_deg`), refuse when a joint that rests on a hard stop reads more than `rest_tol_deg` from `rest_deg` (shoulder_lift and elbow_flex are clean candidates).
3. **Runtime warning (never a stop):** a joint held clamped, or pressed into a stop with steady torque, for more than N seconds. This would have caught 09-10 on day one.
4. **Recommissioning script:** a generalised `gripper-range-read.py` that writes evidence plus a *proposed* `unit.json` diff, never the file itself.

## 4. The gripper

Map closed Star -> follower closed stop, open Star -> follower open limit, with leader endpoints chosen so every "closed" squeeze saturates at closed and full open is reachable. Example from the 10-27 data: follower limits (-153.0, -32.6) (open backed off 0.05 rad, closed at the stop), leader closed -0.7, open 58. That compiles to direction ~ -2.05, offset ~ -34.1; the leader range becomes [-153, -32], which the outward-rounding rule at `viscous.py:116-119` accepts; the startup window becomes -173..-12.6, so a gripper powered up fully open (-155.9) is no longer a stale zero. (The coordinator's -1.936 x star - 38.6 is the line through the extremes; endpoints inside the Star's travel are better, because a light closed Star (-0.7) should still mean fully closed.)

**To measure on hardware:**
1. Follower closed and open stops, 3x and again after a power cycle (jaws closed at power-up: the turn correction uses the current, stale limits).
2. Star closed at a light grip and at a hard squeeze, and fully open, in a window where only the Star moves. The 10-27 file's Star-phase window shows no motion ([-1.8, -1.7]); its -3.1..60.6 came from the follower phase.
3. Whether jaw contact coincides with the motor's closed stop (decides whether the closed limit sits at the stop or ~1 deg short).
4. A paint mark across the gripper coupling, so a future slip is visible.
5. Unsure, worth checking: unit 1 reads wrist_yaw -13..-15 and wrist_roll -4..-6 "folded" (10-20 `start` blocks): either the folded pose isn't exact on those joints or they shifted too. Read each joint at its stops the same way.

**Read-only procedure:** arm powered, all Lab sessions stopped, torque off; the `scripts/gripper-range-read.py` pattern (CAN opened without handshake, poll frame CLEAR_FAULT `robstride.py:944` which clears faults but sends no enable or MIT command, Star read with MONITOR only, disconnect with `disable_torque=False`). Follower: close by hand to the stop, hold 2 s; open fully, hold 2 s; x3. Star: light close, hard close, full open, each held; x3. Power-cycle the follower; with the jaws closed, re-read closed (should match within 0.5 deg). Save the evidence and draft the `unit.json` diff for Gavin to review.

## 5. Powered-motion risks

- A new mapping changes the first target at connect, and the follower slews to it at startup. Before the first powered run, do a read-only dry run that prints the mapped target against the present reading for every joint.
- Widening the open limit to -153 lets the jaws drive to ~3 deg from the open stop: fine at kp 20, but if the coupling slips again the new frame drives into a stop.
- The waterproof J1 frame must move in one change: moving it into the file while the RAM shift remains double-shifts J1. `prepare_test` refuses an unrecognised frame (`joint_posture_mapping.py:126-129`), but ordinary teleop would not. Remove the RAM shift in the same commit.
- Posture mode on unit 1 changes every joint's gain to +-1; that matters only away from the folded pose (the leader start check keeps the first target near zero).
- Haptics scripts take the Star clamp from `leader.travel_deg`: a wrong value drives the Star into its stop (power-capped, no hardware watchdog).
- The new startup refusals happen before torque, so they cannot move anything, but they can block Gavin: word them clearly.
- The Linux `viscous_waterproof_01` record -> unit 1 adapter mismatch is live today. Fix it, or add the identity check, before anything else.
