# Unit 1 on the left ROV mount — offline preparation, 2026-11-05

**Status: J1 RE-ZEROED on the Mac 2026-11-05 18:30 UTC (see "Re-zero done" below); new-frame profiles STAGED, NOT installed in Lab, NOT deployed.** Subtask A5 (one mapping per
unit), A3 (haptics), and B3/B4 (ROV scenes). Gavin: “the dry one on the left
side of the ROV … mirror some things across … clockwise 90 degrees I think …
use a URDF.” The uncertainty in “I think” matters: no motor-zero action is
justified by this CAD alone.

## Re-zero done (2026-11-05, Mac, Gavin at the arm)

Unit 1 and its Star were plugged into the MacBook (CANable `207E37A55445` on
`/dev/cu.usbmodem207E37A554451`), mounted on the ROV's left side, Mac Lab not
running. Every step below was torque off, motor 1 only, bus opened without the
energizing handshake.

- *First read, arm folded:* J1 **+118.4°**, 28° outside the +90 ±10 prediction,
  which would have left only ~9° of inward (CW) travel. Not zeroed.
- *Gavin:* "base is not straight forward but its already mounted on the rov, so
  that is fixed. the arm is pointed forwards, aka the grippers is pointed
  forwards". Then he stretched the arm out: "now its actually pointed forwards
  relative to the rov". Second read: **+82.54°**, inside the prediction.
  **Lesson: judge J1's forward neutral with the arm stretched out.** In the folded
  pose the gripper's heading misread J1 by ~36°.
- *Zero:* Gavin said "go". `scripts/viscous_left_mount.py`'s own `operate()` ran
  through a throwaway Mac wrapper. The wrapper swaps the by-id port for the Mac
  path and the systemd check for "nothing listening on :8000". Expected
  +82.54 ±3, re-read after the confirmation (no movement), SET_ZERO 0xFE.
  Read **−0.01°** after, and again −0.01° on a fresh connection. Torque disabled
  at the end. Shift `s = +82.54`: old frame = new frame + 82.54.
- *Staged, not installed:* `configs/viscous/viscous_01/evidence/left-mount-2026-11-05/operator-run/`
  holds the receipt `j1-rezero.json`, `viscous_profiles/viscous_01.json` +
  `star_viscous.json` and `viscous_01-calibration-new-frame.json`. New J1
  follower soft limits **−267.94…+45.06°** (≈45° inward/CW, ≈268° outward/CCW);
  Star J1 clamp **−154…+45**. Install both profiles together with Lab stopped,
  per step 6 below (`RELEASE=.../operator-run`). Until then **do not start Lab
  teleop on unit 1**: the fallback preset is in the old frame and is now 82.5°
  off on J1.
- The tables below that assume `s = +90` are superseded by `s = +82.54`.
  Approximate hard stops are now −270.8…+47.9.

## Answer and sign derivation

ROV coordinates are +X forward toward the grippers, +Y vehicle left, +Z up.
Positive world yaw is CCW when looking down from +Z. Numbers below are degrees
unless stated otherwise.

| Quantity | Left CAD | Right CAD |
|---|---:|---:|
| Mount translation, metres | (0.165721504063, +0.246525, −0.226) | (0.165721504063, −0.246525, −0.226) |
| Mount rotation | Identity (roundoff ~10⁻¹⁵ rad) | Identity (roundoff ~10⁻¹⁵ rad) |
| Shoulder-pan origin above base | 0.0574 m | 0.0574 m |
| Shoulder-pan axis in ROV coordinates | (0,0,−1) | (0,0,+1) |
| Increasing native CAD J1 | CW | CCW |

Lab does **not** currently have a Viscous viewer conversion in
`makermodslab/arms/urdf.py` (only Maker and Metal). Its operative reference
mapping is `GravityModel.q()` in `arm_gravity.py`: `q = radians(motor_degrees)`,
no sign or zero offset. The vendored single-arm J1 axis is **−Z**. Thus for a
physically identical arm mounted upright, `world_yaw_change = −motor_change`.
The two copies of the single-arm URDF supplied in the brief have the same
SHA-256, `c1e6c387…f7`.

**Right waterproof:** recorded old-frame reading −94.48 (raw 265.52 was its
whole-turn alias), new reading −0.01. This means a **94.48° CCW** physical J1
rotation from its old table neutral, assuming the existing direct motor/URDF
mapping and upright mount. It agrees with “right = CCW about 90”, with 4.48°
of departure from an exact quarter turn. It does not establish the cause of
that departure. `old = new − 94.48`. Driving the mirrored right CAD yaw with
the *same-handed physical motor* requires the opposite yaw sign; naively
feeding −94.48 to native `right_shoulder_pan` would show CW, the wrong answer.

**Left unit 1:** Gavin's proposed **90° CW** move corresponds to **+90° in
unit 1's current motor frame**, not −90 and not +94.48 borrowed from the wet
unit. Its own calibration has homing_offset=0, drive_mode=0, J1 metadata
−185…127, and its measured backed-off range is −185.4…127.6. For an actual
measured re-zero shift `s`, `old = new + s`.

**This is a conditional prediction, not a CAD-established installation angle.**
Both CAD mount rotations are identity; the recovered reference-arm J1 pose is
also zero. The ROV CAD contains no ±90° table-to-motor mounting transform.
To reconcile +90 old motor degrees with forward CAD yaw zero for an identical
left arm, its physical fixed-base indexing would have to contribute +90°
CCW, cancelled by the moving J1's 90° CW. For the right recorded neutral the
corresponding inferred indexing is −94.48° world yaw. These fixed-base phases
are hypotheses about installation, absent from the model. If Gavin poses the
left neutral and J1 reads near 0, −90, or a different whole-turn branch, **abort;
do not force it to +90 to agree with this drawing**. Settle the physical base
index and desired forward posture first.

### Is the mount really the same?

The CAD explicitly models mirrors: left source base plate **1264**, right
**1338** (right source part name contains `镜像`, “mirror”); chassis brackets
are **1250/1321**. The full plate mesh is an exact local-Y reflection. A ±90°
rotation fails to align it (maximum bidirectional vertex-to-triangle distance
~3.625 mm); a 180° rotation leaves ~0.211 mm on the tessellated surface.
Its convex hull alone coincides after 180° to ~0.000013 mm, hiding details.
Therefore the author used mirrored plates/assemblies; **the evidence does not
certify the same manufactured part can simply be rotated**, nor does it imply
the motor must turn ±90. Manufacturing equivalence and the actual installed
bolt index need inspection. Do not infer handedness from the symmetric hull.

Unit 1 is an **identical physical arm, not a mirrored mechanism**. The
single-arm URDF, and the ROV's **left reference chain**, represent its
handedness. The native right arm has mirrored meshes, axes and frames. If the
waterproof arm is also the same mechanism, that right CAD arm is a visual
proxy, not its calibrated hardware model. For hardware-linked simulation,
instance the single-arm mechanism on each measured rigid mount; do not mirror
the real arm's motors just because the CAD author mirrored the assembly.

### Folded CAD zero is not all-zero motor posture

Recovered reference-model coordinates for native ROV `q=0` are:

| Joint | Single-arm reference degrees at folded ROV CAD pose |
|---|---:|
| shoulder_pan | 0 |
| shoulder_lift | −144.999999876 |
| elbow_flex | +119.999998218 |
| wrist_flex | −25.748097295 |
| wrist_yaw, wrist_roll | approximately 0 |

For the unmodified **left reference CAD**, `q_native = radians(m_old) − q_ref`.
After re-zero, reference-model input becomes `m_old_J1 = m_new_J1 + s`.
An explicitly indexed **candidate mounted physical model** cancels that J1
phase with its measured base rotation, so its forward neutral has native
left J1=0 when m_new_J1=0; other joints still subtract the table above. Do not
add both the base phase and a CAD yaw-zero correction twice. Gripper CAD zero
is the supplied jaw shape, not motor zero; its motor-to-gap calibration is
still unresolved and is not supplied by these six rotary offsets.

100 random-pose comparisons match the single-arm/left chain within 9.7×10⁻⁹
in matrix entries; the right chain at equal native q is its exact Y reflection.
These are numerical consistency checks, not physical encoder calibration.

### Travel around the proposed left neutral

Unit 1 commissioning captured endpoints with a **0.05 rad = 2.864789° backoff**.
With `s=+90`:

| J1 quantity | Old frame | New candidate frame |
|---|---:|---:|
| Measured backed-off soft endpoints | −185.4…+127.6 | **−275.4…+37.6** |
| Approximate hard stops (restore the backoff) | −188.265…+130.465 | **−278.265…+40.465** |
| Prepared leader output clamp (inward, intersect Star reach) | −185…+127 | **−154…+37** |

The new neutral is inside the captured range, with **37.6° CW** and **275.4°
CCW** backed-off travel. That is asymmetric but mechanically plausible;
neutral need not be travel centre. Hull clearance may reduce it much further.
The hard-stop estimates inherit rounded archived zero-frame endpoints and
are not a new stop measurement. Use `endpoint − s` with the **actual** reading,
not a hardcoded 90, for the new profiles. Unit 1's gripper endpoints are known
stale; this task does not qualify the other six archived ranges for hull use.

## Prepared changes and what still needs work

| Item | Prepared disposition / location |
|---|---|
| Follower soft limits | `scripts/viscous_left_mount.py` builds a full per-unit profile, translating only J1 by measured `s`. No preset or waterproof profile edits. Its successful future run stages `viscous_profiles/viscous_01.json`; no live file exists from this task. |
| Leader J1 | Keep unit-1 gain **+1.439742**, offset **0** so raw leader neutral=0 maps to new follower neutral=0. Intersect new-frame follower limits with measured Star reach, rounding inward; do **not** subtract `s` from the mapped command, which would move new neutral away from 0. This remaps the desired posture; it is not preserving old physical targets. |
| Leader raw unwrap / range | Native leader unwrap centre is midpoint(output range)/gain. Naively using −275…37 gives centre −82.66° raw: values above ~+97.34° wrap to the negative branch, producing a large target jump. The prepared clamp is therefore **−154…37**, with centre −40.63° and raw unwrap window −220.63…139.37°. This contains the entire archived raw J1 capture −107.1…110.3. Its mapped reach is about −154.20…158.80 before clamps: positive motion clips near raw +25.70°, negative follower travel below −154 stays unreachable. Fake tests exercise 501 raw samples and ±2-turn aliases at shifts +80/+90/+100. Keep current scale; no claim of full-range 1:1 motion. Re-measure matched leader poses later under A5. |
| Other leader joints / haptics | Preserve gains, ranges, offsets including gripper −33. J1 physical mechanism and leader response sign do not reverse. No bilateral/haptic sign flip. Gripper-only haptics are independent of J1; its unresolved range problem remains. The uncommitted POC was not touched and bypasses the profile loader: do not use it for arm-joint commands after re-zero. |
| Calibration library metadata | Future tool stages a copy of unit 1's follower calibration with only J1 range_min/max translated/rounded inward. Motor zero is actuator flash, not homing_offset. Preserve original archive and annotate datasets by frame. Leader calibration itself stays unchanged. |
| URDF / viewer | No Viscous viewer is wired today. `table_degrees()` prepares the explicit new→old reference conversion. `scene-assignment.pending.json` records mount identity and conditional yaw phase. Do not change shared URDF, its hash, or waterproof's posture trial. Physical visual verification of J2–J6 and jaw mapping is still required. |
| Sim / ROV assignment | Pending assignment is **left=viscous_01**, **right=viscous_waterproof_01**. Existing `--scene rov` in `scripts/viscous-dream1/astra_op/rov_scene.py` has geometric left/right and canonical IK signs `[−,+,+,+,−,−]` on the right; it has no physical unit assignment. Leave historical runs and cinematic untouched. Its CAD-mirrored right chain is not a hardware-ready wet-arm model. New hardware-linked scene must use identical mechanisms and measured base phases, then recheck reach/collision. |
| Gravity FF / contact subtraction | J1 rotates about vertical −Z on the assumed level base. The fitted air model is invariant to +90° J1: max difference **1.8×10⁻¹⁵ Nm** over 100 poses including telemetry biases. No fit-sign reversal or refit is needed solely for upright yaw/re-zero. The stored coordinate semantics remain old-frame; use explicit conversion in future pose-aware consumers. A tilted ROV, bracket roll/pitch, new payload or immersion changes gravity and needs a model update/wet fit. Initial acceptance must have FF and gripper haptics off; do not inherit the Linux FF service drop-in without reviewing it. No service edits here. |
| Envelope | New left mounting invalidates any right-arm hull envelope and any old-frame samples. Paint a fresh unit-1 envelope mounted in place, record zero receipt/base transform/hash/handedness, then enforce. Joint limits alone are not hull protection. See updated envelope plan. |
| Data / actions / rest | Pre-re-zero datasets, policies, recorded rest poses and hardcoded targets are in old degrees. Convert J1 with `new=old−s` and version them; never replay them as new-frame poses. Existing return-to-session-start remains relative to the current session. Thermal limits, gains, motor IDs and adapter identity do not change. Wrong waterproof robot record on Linux remains an open hazard; no robot records edited. |

## Operator procedure — READY BUT NOT RUN

**Everything in this section except the dry run is a future, hands-on action
by Gavin.** Do not remotely execute it during offline review. No physical
configuration has been established by these illustrations.

1. Review both PNGs and confirm the actual mount bolt index, intended forward
   neutral and physical handedness. Check the new 37.6° CW travel is acceptable.
   Support the arm throughout: it has no brakes. Keep the Star at its existing
   J1 neutral; do not recalibrate or re-zero the Star. Clear the hull and all
   other joints. No arm is commanded to move by this procedure.
2. Stop any session normally, then Gavin stops Lab and confirms no other bus
   clients own the adapter. All joints must already be torque off. Do not rely
   on the tool to release the other six motors; it addresses **motor 1 only**.
   With the arm supported and operator at its power switch:

   ```sh
   cd ~/Documents/makermodslab-viscous
   systemctl --user stop makermodslab-viscous
   systemctl --user is-active makermodslab-viscous  # must print inactive
   .venv/bin/python scripts/viscous_left_mount.py --dry-run
   ```

   Dry-run imports no motor/serial/CAN code, opens no port, and writes nothing.
   It is also the default if no mode is given. The two hardware modes below
   refuse unless the service reports `inactive`, and require the enrolled
   unit-1 CANable by-id path containing **207E37A55445**.
3. Pose J1 by hand at the intended **left ROV neutral**, other joints supported
   clear of the vehicle. Use the unit-1 adapter path archived in
   `evidence/gripper-haptics-20261027T050418Z.json`; Gavin must confirm the
   adapter still connects to the dry arm (USB serial identifies the adapter,
   not the motors). Never substitute an arbitrary `/dev/ttyACM*`:

   ```sh
   UNIT1_CAN='/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207E37A55445-if00'
   .venv/bin/python scripts/viscous_left_mount.py --inspect \
     --port "$UNIT1_CAN" \
     --output configs/viscous/viscous_01/evidence/left-mount-2026-11-05/inspection-actual
   ```

   `--inspect` **does access hardware**: connects without handshake, disables
   J1, reads five fresh positions via the pinned driver's status/CLEAR_FAULT
   query, disables J1 again and disconnects. It never enables a motor, sets
   gains, changes operation mode or writes zero. Reads are not passive CAN
   listening. Expected J1 is **+90°**, acceptable **+80…+100°**, stable within
   1°. **Abort for −90, −270, +450, near 0, or any other branch**, even if
   adding/subtracting 360 could make it look correct. Inspect actual pose and
   reference frame; do not silently unwrap it or change the threshold.
4. Only when the physical posture and reading agree, keep J1 in place and run:

   ```sh
   .venv/bin/python scripts/viscous_left_mount.py --execute \
     --port "$UNIT1_CAN" \
     --output configs/viscous/viscous_01/evidence/left-mount-2026-11-05/rezero-actual
   ```

   The tool rechecks readings, prints the actual shift/new bounds, then asks
   Gavin to type `LEFT J1 NEUTRAL VERIFIED`. It re-reads after the pause and
   rejects pose drift. It journals that a zero write may occur **before**
   sending SET_ZERO. Only motor 1 receives `set_zero_position([shoulder_pan])`,
   the pinned implementation's seven 0xFF bytes + **0xFE**, standard CAN ID 1.
   Five fresh post-write readings must all lie within ±1°. Disable J1 after
   zero and on every exit, including SIGINT/SIGTERM; no torque-enabled context
   manager is used. Communication failure can prevent the release command;
   Gavin's power switch remains the hard stop.
5. Inspect `rezero-actual/j1-rezero.json`. Successful run has
   `activation_ready: true`, actual before/after arrays and
   `final_torque_disabled: true`, alongside **two** generated hardware profiles
   and `viscous_01-calibration-new-frame.json`. New ranges use measured `s`.
   Other joints are not read or written, so the receipt makes no invented
   “unchanged” numerical claim about them. Persistent zero survives power
   cycles. If the write or verification fails, **leave Lab stopped**: an
   attempted write may already have changed zero. Do not retry SET_ZERO or
   try to undo it automatically. Preserve the journal and resolve the actual
   frame with Gavin.
6. **Activation gate:** the tool only stages files inside the new evidence
   directory, never `~/.makermods/`. No current code path consumes the pending
   JSON. Only after the successful receipt, Gavin backs up the two existing
   live profile paths (including recording that a path was absent) and the
   follower calibration; he installs **both profiles together while Lab stays
   stopped**. The existing `_commissioned_config` loader then overrides the
   unit-1 fallback without changing other units. Example future commands:

   ```sh
   RELEASE="$PWD/configs/viscous/viscous_01/evidence/left-mount-2026-11-05/rezero-actual"
   LAB_HOME="$HOME/.makermods/makermodslab-viscous"
   # First back up the existing destination files/absence, outside this example.
   mkdir -p "$LAB_HOME/viscous_profiles"
   cp "$RELEASE/viscous_profiles/viscous_01.json" "$LAB_HOME/viscous_profiles/viscous_01.json"
   cp "$RELEASE/viscous_profiles/star_viscous.json" "$LAB_HOME/viscous_profiles/star_viscous.json"
   cp "$RELEASE/viscous_01-calibration-new-frame.json" \
     "$HOME/.cache/huggingface/lerobot/calibration/robots/viscous_follower/viscous_01.json"
   ```

   Do not run the old bundle restore over the new frame. Never delete these
   new profiles while leaving the motor re-zeroed: that restores the unsafe
   old fallback. To recover, preserve zero and reinstall the matching release;
   moving the physical zero back requires a separately reviewed procedure.
7. Before **any** powered start: review/disable FF in the service configuration
   and leave the haptics checkbox off; verify the selected record is unit 1,
   both loaded profiles have new-frame bounds, and no stale action source is
   active. With torque off, verify the new J1 reading remains near 0 after a
   power cycle and compare the physical posture to the correctly indexed
   single-arm model. Only then plan a separate, announced, small-motion
   acceptance with Gavin at the arm; no hull-near teleop until envelope work.
   Commit the actual receipt, generated release/calibration and updated
   bundle/manifest as one frame migration, then push. Synchronise that release
   to the Mac before using unit 1 there; do not overwrite its wet-arm profile.

### Provenance and reproduction

The 2026-10-11 tracked evidence records the waterproof method. Commit
`586e162d` added the evidence but **no reusable zeroing script**. Searches of
tracked script history for SET_ZERO/set_zero_position found none. The new
tool reconstructs the method using the installed pinned
`lerobot/motors/robstride/robstride.py`: `connect(handshake=False)`, selected
`disable_torque`, selected `set_zero_position`, selected fresh reads and final
release. It deliberately avoids the follower's normal connect/handshake and
the all-joint GUI calibration path.

Evidence directory: `configs/viscous/viscous_01/evidence/left-mount-2026-11-05/`.
`rov-kinematics.urdf` is the original ROV joints with mesh/inertia-free links;
`source.json` pins the full source hashes and recovered reference angles.
`mount-hulls.json` and compressed `mount-plates.npz` retain small source
geometry for reviewing the mirror claim. `analysis.json` contains the
reproducible frame/gravity/hull checks; `full-plate-check.json` preserves the
full-plate surface comparison and source hashes. `dry-run.json` is explicitly
candidate-only, not an installable active profile. `scene-assignment.pending.json`
is an unconsumed, physically unverified assignment proposal.

```sh
.venv/bin/python scripts/analyze_viscous_left_mount.py  # NumPy + arm-dynamics extra
# Optional: same script with --plates-only in any Python with NumPy + trimesh.
.venv/bin/python scripts/viscous_left_mount.py --dry-run
.venv/bin/python -m pytest -q tests/test_viscous_left_mount.py tests/test_viscous_arm.py
~/Applications/blender-4.4.3-linux-x64/blender -b --python-exit-code 1 \
  --python scripts/render_viscous_left_mount.py
```

The renderer uses the **already committed** native ROV Blender asset under
`scripts/viscous-dream1/cinematic/assets/`, without modifying it, so it needs no
external CAD checkout. Drawings show folded CAD articulation with unit labels,
a conditional +90° left move and the recorded −94.48° right move; the right
arm is marked as a mirrored CAD proxy. They illustrate the proposed yaw
neutral, not a measured all-joint pose or a hull-safety result.

## Mac Lab install as a third arm (2026-11-05)

Gavin: "put this into the gui? as a 3rd arm?" On the Mac (Lab home
`~/.makermods/makermodslab-viscous`, launch agent `com.gavin.makermods-viscous`):

- New robot record **`viscous_01_rov_left`** (follower config `viscous_01_rov_left`,
  leader config `star_viscous_01_rov_left`), cloned from `viscous_01`.
- Profiles `viscous_profiles/viscous_01_rov_left.json` and
  `star_viscous_01_rov_left.json` = the staged `operator-run/` profile (s=+82.54).
  Follower calibration `viscous_follower/viscous_01_rov_left.json` = the staged
  new-frame calibration; Star calibration `star_viscous_01_rov_left.json` = an
  unchanged copy of `star_viscous.json`.
- `configs/viscous/viscous_01/manifest.json` enrolls `viscous_01_rov_left` under
  adapter `207E37A55445`, so plug-and-play recognises it without the warning.
- The old `viscous_01` record was left in place but would now let J1 travel ~82 deg
  past the real inward stop, so the same new-frame profiles were also installed as
  `viscous_profiles/viscous_01.json` / `star_viscous.json` on the Mac. Its cache
  calibration file is untouched. Backups: `~/.makermods/makermodslab-viscous/backups/left-mount-20261105/`
  (before this, the Mac had no unit-1 profile files, only the waterproof's).
- **gavin-linux still has NO new-frame profiles**: its `viscous_01` record would run
  the old-frame preset. Install the same files there before unit 1 runs on Linux.
- The waterproof's 1:1 posture trial is gated to `viscous_waterproof_01` in both
  frontend and backend, so it does not apply to unit 1.

## First powered teleop on the left mount (2026-11-05, Mac) — shoulder overheat, safety bug fixed

- First start was refused (no motion): Lab's pairing check requires the Star
  ranges to equal the follower limits rounded inward, and the staged pair
  translated follower J1 to −267.9 while the Star stayed at −154. Fixed by cutting
  follower J1 to the Star-reachable −154…+45.06 (installed profiles, staged files,
  `profiles()`, and a test that runs `build_single_configs`).
- Then teleop worked (12:36). With the arm held stretched out horizontally on
  the ROV, **shoulder_lift went 27 → 85 C in ~2 min**, three sessions running
  (12:39, 12:42, 13:06), peaking at **101 C (MOS) at 13:07**. Two all-motors-silent
  dropouts (12:36:48, 13:15:38) look like the supply tripping under that load.
- **Safety bug:** the follower's thermal stop and all-silent stop are raised from
  `get_observation()`, which Lab's teleop loop called only inside the joint
  broadcast `try`, so both were logged and swallowed and torque stayed on. Fixed:
  `teleoperate.read_can_observation` re-raises exactly those two, and the loop
  reads it outside the broadcast `try`, so they end the session through the
  error exit (immediate torque release, no return-to-rest).
  `tests/test_can_follower_safety_stop.py`.
- Gavin reported joint 4 (wrist_flex) "stopped working" at a later start. A
  read-only probe right after (torque off) shows all seven motors answering, J4 at
  63.4°, 31 C. The log shows J4 stalling 25–39° short of its goal at the 13:14 and
  13:16 starts. Not yet explained; a supply dip resetting a motor into its
  disabled state is a guess. Next: a single-joint powered J4 wiggle
  (`scripts/viscous-wiggle-joints.py`) with Gavin's OK.
- **Hardware constraint, Gavin's call:** on the ROV this arm's shoulder cannot hold
  the stretched-out horizontal pose for more than ~1–2 min. Keep sessions near
  folded, or give J2 gravity help, before longer teleop.
