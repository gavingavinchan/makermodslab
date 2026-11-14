# Viscous haptic teleoperation: objective, reasoning, progress

A running record of **why** this work exists and how decisions were made, so the
history can be reconstructed later. Newest results at the bottom of each section.

Everything here is a subtask of a larger goal. Each one is recorded with its
objective, **Gavin's intent** (why it matters, in his framing), what it unblocks
and where it stands. Quotes are Gavin's words from the working sessions. When a
subtask changes direction, add a dated line rather than rewriting the old one.

**Resuming? Read the To-do list first:** "Handoff: state and how to resume" →
"To-do". The newest physical change (2026-11-05) is unit 1 on the LEFT ROV
mount: J1 was re-zeroed for it on 2026-11-05 (s=+82.54) and the new-frame
profiles are staged but NOT installed; read the left-mount doc before any powered motion. The
plug-and-play work remains open.

## Goal tree

**Mission: make the Viscous arm something ROV operators want.** Two branches:
pilot teleoperation that feels right, and autonomy through learned policies.
The 2026-10-19 session opened with autonomy ("I'm trying to make the viscous arm
autonomous") and asked for the Star leader to follow the follower for DAgger.
Gavin then reprioritised.

### A. Pilot teleoperation with force feedback (current priority)

- **Objective:** a pilot driving the Viscous with the Star leader feels what the
  follower touches.
- **Intent:** "a lot of the ROV pilots, people in the industry, kind of want force
  feedback or some haptic feeling" (2026-10-19). DAgger "is not my most important
  thing right now; my most important thing right now is teleoperation."
- **Guiding rule:** the pilot should feel contact, not the arm's own effort.
  "Lifting itself up because of its own weight ... is not something that is
  meaningful for the pilot to feel. If we hit something, yes, we should feel that."

**A1. A clean contact signal.** contact torque = measured motor torque − predicted
gravity torque.
- *Intent:* the haptic effort must exclude self-weight (the guiding rule above).
- *Unblocks:* A3. *Status:* validated live on unit 1 dry, 2026-10-20 (see
  "Contact-signal bench check"): a hand push reads ~2 N m against ~0.1 N m
  still noise. Free-motion floor ~0.6-1.2 N m p99 still to bring down.

**A1a. Gravity model, measured on the arm.**
- *Intent:* not what gravity compensation was for the Metal arm. "Before,
  gravity compensation for Ethan was mainly so that the leader arm would not
  feel so heavy. That's not really the case for us because our leader arm is
  very light." Here it exists to separate contact from load.
- *Method intent:* "Dry arm first, and once we've perfected this method, then we
  can do it for the wet one." The dry bench proves the method; the aquarium fit
  (buoyancy) is the one that matters in use.
- *Status:* dry fit done 2026-10-20 (results below). Wet fit pending.

**A1b. Gravity feedforward on the follower.**
- *Intent:* a side benefit, not the goal. Removing sag makes position error mean
  contact, and the follower goes where it is told.
- *Status:* A/B done 2026-10-20: sag 3.5-5.5 deg at 0 % falls to under 0.25 deg
  at 100 % on every gravity joint.

**A2. A position-drivable leader.** The Star's servos must be commanded toward
the follower with capped effort (`star_follow.py` draft).
- *Intent:* serves A3 and, later, DAgger (B1): one piece of hardware work for both.
- *Status:* draft code; Star actuation not yet characterised on hardware.
- *Status (2026-10-21):* first powered runs, gripper only (see "Star actuation
  check"): the power cap on command 0x08 limits stall effort, unloaded moves
  track at 100 mW, and first motion comes 20-40 ms after the write. Hold
  stiffness and damping feel still need a hand on the leader. `star_follow.py`
  still uses `set_angle` (full power) and must move onto `fashionstar_raw.py`.
- *Protocol research (2026-10-20, [star-servo-protocol.md](star-servo-protocol.md)):*
  FashionStar position commands carry a per-command power limit in mW, but
  motorbridge 0.0.4's `set_angle` always sends power 0, which means the servo's
  protection CEILING, not zero effort. So `star_follow.py` as drafted drives
  the leader at full power: do not run it against a hand. A capped move needs
  the raw command (0x0D multi-turn angle + power) or a motorbridge patch;
  hold (0x11) cannot move its hold point. A zero effort gate must be an
  explicit release (0x10), never power 0. No documented bus-loss watchdog: the
  last target persists if the host dies, and stop writes are not acknowledged.
  Monitor current/power are unsigned magnitudes (30 mA / ~365 mW at rest,
  released), so they are not a signed torque reading.

**A3. Input-gated bilateral loop** (approach section below).
- *Intent:* the "spring between leader and follower" Gavin described, made to feel
  light in free motion and firm on contact. Chosen after checking how others do it.
- *Status:* not started; needs A1 and A2.
- *Status (2026-10-27):* one-joint proof of concept on the gripper works on
  hardware (see "Gripper haptics proof of concept"): free motion stays free,
  and closing the follower on an object drives the Star back to where the jaws
  stopped. Gavin asked for it: "can we do this quickly and do a proof of
  concept with the gripper?"

**A5. One measured mapping per unit (2026-10-27).** Every Viscous unit gets a
single committed, measured leader-to-follower description that teleop, haptics,
URDF and sim all use, with drift caught before an arm moves.
- *Intent:* "this is a recurring problem" ... "solve this once and for all so
  that our workflow is working properly" (Gavin, 2026-10-27).
- *Status:* problem written up in [viscous-mapping/README.md](viscous-mapping/README.md)
  with two independent read-only investigations; nothing changed yet.

**A5a. Unit 1's LEFT ROV mount and zero-frame migration (2026-11-05).**
- *Intent:* Gavin: “the dry one on the left side of the ROV … mirror some
  things across … clockwise 90 degrees I think … use a URDF.” Establish the
  sign before moving anything, then keep limits, leader mapping, visual/sim
  and envelope in the same frame. This supports A3 and B3/B4 as well.
- *Status:* offline audit, drawings and gated operator tool prepared; no
  hardware access, zeroing, live-state edits or service changes. The right
  −94.48° motor shift means 94.48° CCW under the existing −Z motor/URDF
  mapping. Left 90° CW predicts **+90° motor**, conditional on physical base
  indexing; CAD has identity mount rotations and does not prove that index.
  See [the complete audit/procedure](viscous-left-mount-2026-11-05.md).

**A4. 1:1 posture mapping.** A horizontal leader gives a horizontal follower.
- *Intent:* pilots expect the follower to mirror their pose. It is also needed to
  judge the rest by eye: "When I hold [the wrist] horizontal, the follower arm is
  not horizontal, so then I can't really tell whether the gravity compensation
  is working."
- *Status:* waterproof unit trial "worked very well" (committed 2026-10-20).
  Unit 1 has a draft profile in viscous-arm-simulation, not yet in Lab.

### B. Autonomy (deferred, not abandoned)

**B1. DAgger handover.** The leader tracks a policy-driven follower so the
operator can take over without a jump. Needs A2, plus CAN-safe go-limp and
restore paths in the DAgger runner.

**B2. Policy data and rollouts** (ACT/SmolVLA, `scripts/record-rollouts.sh`).

**B3. Dream-1 (DreamLabs' hosted world action model) on the Viscous arm.**
- *Intent:* the Viscous arm is very close to the Maker Arm, which Dream-1
  officially supports, so Gavin wants Dream-1 to drive it, possibly through "a
  translation layer ... so that ... the Dream One model just pretends that it is
  ... using the maker arm but then in reality it's using my arm" (2026-10-20).
- *Status (2026-11-07):* first step toward the real arm. Gavin: "can we now
  build a bounding box to run dream1?" A keep-out box that gates every joint
  target against the measured tank is built and tested offline; the real-run
  executor is planned, not built. See [dream1-tank.md](dream1-tank.md).
- *Status (2026-10-20):* zero-shot MuJoCo trial done; the result is "completely
  stupid ... hopelessly bad, no resemblance of any meaningful actions". Gavin wants
  to find out why. See "Dream-1 trial" below.
- *Status (2026-10-20, later):* cause found. Dream-1 never sees a non-YAM arm's
  state, and from images alone it does not aim at the object when the arm is the
  Viscous (it does on YAM). The Viscous IK and frames are fine. The task (2 cm white
  cube) also fails even on YAM. Next: a Maker-tuned checkpoint or a Viscous fine-tune,
  after Gavin sets the tool point with `dreamcli tcp`. Details under "Findings".
- *Status (2026-10-21):* resolved in sim. With a Maker-style wrist camera the
  Viscous puts the apple in the bowl 6/6, and one arm plus a parked second arm
  also 6/6. Details under "Findings".
- *Status (2026-11-07):* Grace (World Intelligence) blamed the URDF; it checks out.
  The box failures come mainly from the sim's 5 Nm shoulder cap (the RS00's
  continuous rating): at 14 Nm the box goes 7/12 vs 2/12. See "Grace's URDF claim"
  under "Findings".
- *Status (2026-11-07, later): real-arm runner built, offline only.*
  `scripts/viscous-dream1/real/dream_real.py` (README beside it) lets Dream-1 drive
  the waterproof arm through Dream's own client and IK and the Lab's CAN machinery
  (`remote_can`, `maker_rest_pose`). `--dry-run` runs the loop on a fake arm with
  Dream's real client offline and fires every trip once; `--preview` (Gavin) opens
  only the cameras and prints the inputs id for `python -m dream.input_check`;
  `--live` refuses without the `dreamcli tcp` record, the recorded inputs id, an
  approved tank model, acknowledged J5/J6 signs, the dry run's plan id, the tank
  guard and an idle Lab. A tank IK model (`real/tank_model/`, root turned 180 deg,
  IK pan = motor - 180, the unit's profile limits) is generated with provenance and
  **pending Gavin's approval**; the tool point must be set on that model, since the
  digest covers the URDF. 29 offline tests. No hardware, camera or billed call.
  Pending: approval, `dreamcli tcp` on the tank model, an overhead camera, the
  inputs check, J5/J6 hand tests, merging the tank guard, credentials.
- *Status (2026-11-08): J5/J6 signs hand-tested, both confirmed.* Why it was
  needed: teleop is motor-to-motor and the Lab viewer uses `maker_joint_positions`,
  so neither ever exercises `motor_to_urdf.json`; only Dream's client IK does, and a
  wrong wrist sign there would turn the wrist the wrong way under Dream while teleop
  looked fine. Prediction from FK on the tank URDF (`dream_base`, x forward, hand
  level along +x): URDF +yaw moves the tcp to -y (right), URDF +roll turns the hand
  top left (counter-clockwise seen from behind). So "fingertips right" must take the
  `wrist_yaw` motor DOWN (sign -1) and "roll CCW" must take `wrist_roll` UP (sign +1).
  Gavin chose to do it **leader only** (follower unplugged): a read-only logger read
  the Star leader's `get_action()`, which is exactly the follower motor target teleop
  sends, while he moved the leader's hand step by step. Fingertips right: `wrist_yaw`
  -3.2 -> -54.2 deg and back. Roll CCW: `wrist_roll` +7.1 -> +51.0 and back, yaw
  within 0.3 deg. Both match; `motor_to_urdf.json` now says "hand-tested 2026-11-08"
  for both, so `--live` no longer needs `--ack-unverified-sign`, and `make_tank_arm.py`
  lists only genuinely unverified joints in the provenance. Caveat: this measures the
  follower COMMAND for a leader motion; it relies on the follower turning the same
  physical way as the leader on J5/J6, which teleop use shows and this test did not
  re-measure. Evidence with a 5 Hz trace:
  `configs/viscous/viscous_waterproof_01/evidence/j5-j6-sign-test-20261108.json`.
- *Status (2026-11-08, later): tank start pose recorded and applied.* Why: the tank
  model's `rest` was Dream's sim pose, a placeholder, and the rest steers Dream's
  client IK, so it had to be a pose the real arm actually starts from in the tank.
  Gavin teleoped the follower (session started from chat at his request, lease
  heartbeated, stopped through the normal return-to-rest + torque-off) and said
  "record now"; the pose was taken from the follower broadcast with
  `record_start_pose.py`'s own snapshot/record (median of 0.5 s, spread < 0.3 deg):
  motor pan 176.59, lift -122.9, elbow 99.34, flex 8.56, yaw -9.0, roll -3.99, tcp
  462 mm out, centred, 143 mm up, no box violation
  (`scripts/viscous-dream1/real/runs/start_pose/20261108T183949Z.json`). Gavin: "i like
  this pose", and later: "its the follower arm pose that i like, the leader arm does
  not map nicely to it". The recorded numbers are the follower's MEASURED joints (the
  broadcast's `joints_deg`), so they are the right ones; never reproduce this pose by
  matching the leader to it, or take a start pose from the leader's reading. The
  leader-to-follower shape mismatch at this pose is unexplained (scale/offset in the
  leader profile?). The J5/J6 test only needed the direction of each wrist joint
  through that mapping, not its shape. `make_tank_arm.py --rest-motor-deg ...` rewrote only `rest` (arm file,
  provenance); the URDF bytes and its normalized hash 8975766e...cef7f are unchanged,
  so the station arm file `viscous-waterproof-tank.json` from `dreamcli tcp` stays
  valid (untouched). The dry run with that arm file and `--overhead head` puts the new
  rest in its run-local arm copy. Remaining live blockers: tank-model approval, the
  `wrist_l` slot, the inputs check, the tank guard merge. Noticed: the dry run's plan
  id (01054fd58f) was the same before and after the rest change, so the plan id does
  not cover the rest.
- *Status (2026-11-08, later): tank guard reviewed, not yet merged.* Gavin: "how
  about you check the tank guard, i have no idea how good or bad it is". Branch
  `feat/viscous-dream1-tank-box` (aa7075a8, 5 commits): the code is
  `makermodslab/tank_guard.py` (324 lines), its keep-out config and 15 tests; the
  other ~92k lines are validation evidence. Verdict: **the geometry is sound**. It
  checks every collision-mesh vertex of every moving link against six planes and
  *proves* the straight joint-space path clear (a per-joint reach bound sets the
  sampling so nothing moves more than 2 mm unchecked; vertex-only checks are exact
  for half-spaces). An independent checker re-did 562,508 poses at <= 0.05 deg and
  found no unsafe grant. It merges into `feat/viscous-arm` with no conflicts; on the
  trial merge all 44 tests pass (15 guard + 29 runner) and the dry run is guarded.
  From today's start pose: start gate passes (tightest: upper-arm heel vs the near
  wall, 12.8 mm), the dry run's 481 substeps are all ok, median 1.4 ms, max 73 ms.
  **What it cannot fix are its inputs:** (1) the near (back) wall is a 40 mm
  estimate, never traced, and it is the binding face everywhere around the start pose
  (upper-arm heel at 7-13 mm slack, on a link that has 0 mm margin precisely because
  the wall is unknown); (2) the base slides about 30 mm under a push, which no margin
  covers; (3) cable padding is an unmeasured 40 mm. Integration findings in
  `dream_real.py`: (a) `plan_chunk` runs inline in the 20 Hz loop, so a chunk that
  runs into a wall is clamped substep by substep at up to ~257 ms each (the report's
  wall-clamp benchmark), stalling feedback and trip checks for seconds while the arm
  holds; proposed: stop on a keep-out clamp (or cap planning time) for first runs.
  (b) Nothing moves the arm from its torque-off rest (folded, J1 ~189-191) to the
  start pose before Dream's first call, and a Lab teleop stop returns the arm to its
  session start, so a live run would start Dream from the folded pose, not from the
  recorded start pose. (c) Minor: the runner's gripper window ends at -1.03 deg, the
  guard's at -2.92, so the guard clips the closed end; and from the folded pose its
  limit clips make a 3.5 deg pan step the 30 deg/s bus backstop may slew-limit,
  slightly bending the proven path.
- *Status (2026-11-08, later): tank guard merged* into `feat/viscous-arm` (5fecff6e),
  Gavin: "we measured the tank once before. so can you just merge it?". 44 tests pass;
  the dry run is guarded and `--live` no longer lists the guard. The near wall is still
  the keep-out file's 40 mm estimate (the 2026-11-08 trace covered the side walls and
  floor), and runner gaps (a) and (b) above are still open.
- *Status (2026-11-08, later): near wall set to 80 mm.* Gavin asked whether all of
  that was needed "to just a simple tank boundary limits": no, the boundary is done;
  the runner gaps are not boundary issues and wait until a first run shows they
  matter. Gavin: "lets say the back wall is 8cm from the center of j1". J1's axis is
  the base_link origin, so the keep-out's `near` plane went 40 -> 80 mm (status
  "stated", not traced) and `far` 694 -> 654 behind it. The upper-arm heel's near
  rule went from 0 to 40 mm margin: same heel limit as before (<= 40 mm behind J1),
  now meaning >= 40 mm from the glass; with 0 margin the moved wall would have let it
  touch. At the start pose the forearm/elbow now has 74.6 mm spare (was 34.6), the
  heel 12.8; start pose and folded rest both pass the start gate. `tank.json` and the
  sim scene keep the old 40 mm estimate (their tests pin it). Also fixed
  `tests/test_viscous_aquarium_env.py`, which still pinned J5's old "hand test
  pending" status. Running the full pytest suite on the Mac froze it (all RAM; Gavin
  had to restart): big jobs run on gavin-linux only.
- *Status (2026-11-08, later): tank model approved (provisional), cameras, rehearsal.*
  Gavin: "lets approve the tank model for now, i have not really checked it but as
  long as we go slow, must keep the 15deg/s limits". The sidecar now says approved by
  Gavin with that note, hash 8975766e...cef7f, which matches the station arm file's
  URDF copy. Hence a hard 15 deg/s on every live motion (chunks, approach, return),
  being added with the stop work. Base: "it is clamped good enough fro now". Cameras:
  "the 'head' camera is the wrist camera, but grace from dreamlabs wants to try it",
  so the first runs feed `head` as Dream's wrist view with black overhead/right-wrist
  slots (the runner needs an explicit setting for that). Stop/start (Gavin): "one big
  red button to stop. when stop, use the gradually return to the current rest
  position, (lying flat on the bottom), but you need to immediately release torque on
  the grippers. for start position, you probably need to lift the elbow up first";
  and "we should do one rehearsal": a powered run with no Dream calls (rest -> start
  pose -> hold -> red button -> back to rest -> torque off) before the first billed
  call, because the runner's hardware path has never moved the real arm.
- *Status (2026-11-08, later): start/stop built (two subagents), offline only.*
  `dream_real.py`: the live run starts from the arm lying flat (start gate on the
  rest = limits + guard; the workspace box applies from the Dream start pose on),
  then an approach `core.approach_waypoints`: leg 1 lifts shoulder_lift 40 deg and
  folds the elbow 45 deg with pan and wrist held, so the hand peels off the floor;
  leg 2 goes in the air to the start pose (114 substeps, 5.7 s). Every stop: the
  gripper motor alone goes limp at once (zero-gain MIT frame, then
  `RobstrideMotorsBus.disable_torque("gripper")`), then the six arm joints retrace the
  route to the measured rest leg by leg (each leg guard-checked; not clear = hold
  until the second press), then all torque off. Hard 15 deg/s on every arm motion,
  bus backstop 15 too (was 30). `--rehearse [--live]`: the same hardware path with
  no Dream, holds at the start pose until the button or `--hold-s`. New
  `stop_button.py`: one big red STOP (SIGTERM via `--pid-file`), then RELEASE ALL
  (second SIGTERM, only by a click >= 1 s later). Proven with a real nohup'd process
  and two real signals. Guard change found on review: a joint resting a few degrees
  outside a soft limit used to SNAP to the limit on the first target (pan 1.5-5 deg
  in 50 ms, 30-100 deg/s, hand on the floor); it now eases back in at the leg's pace,
  and a return may end exactly at the run's start rest. 113 targeted tests pass.
  Only a powered run shows: the gripper really limp while the six hold, its feedback
  still answering (the same clear-fault query reads torque-off motors today).
- *Status (2026-11-08, later): first powered rehearsals, stop path works.* Two
  `--rehearse --live` runs on the real arm, no Dream. Run 1 (19:47:01Z): the approach
  completed, then the hold tripped `torque_sustained` (shoulder_lift ~3.85 Nm for 2 s
  vs the 3.5 Nm cap) and the stop path ran: gripper off ok, both return legs arrived,
  back at rest, all torque off, no release problems. Run 2 (19:47:24Z): Gavin pressed
  STOP mid-approach; the same clean return. Peak temp 25 C; peak lag 8.4 deg (the
  shoulder sags ~7 deg under load in MIT position control, near the 10 deg lag trip).
  Holding the start pose needs ~3.9 Nm on the shoulder, so Gavin: "increase the torque
  cap, just make sure the speed is limited, cuz the torque alone cant break glass, its
  impact with the momentum that can break glass". Sustained cap 3.5 -> 4.5 Nm; the
  5 Nm instant trip and the hard 15 deg/s stay. Logs:
  `scripts/viscous-dream1/real/runs/rehearse-live-20261108T1947{01,24}Z/`.
- *Status (2026-11-08, later): independent anti-impact layers.* Gavin: "make sure not
  impact with momentum onto glass in any circumstances, like independent/dont rely on
  the boundary limits, should be separate layers of safety". A third rehearsal
  (19:52:11Z) reached the hold at 3.98 Nm and stopped cleanly on the button. Added
  (subagent, reviewed): torque tiers 12 Nm instant / 5 Nm for 5 s (Gavin: visitors
  will push, "10nm for at least 5sec"); temperature trip stays 80 C (Gavin); a
  measured-speed trip, 30 deg/s for 2 readings on the arm joints, in the loop and on
  every return leg (rehearsal peaks 18-20 deg/s); a no-spring-back re-anchor before
  every return (`can_speed_limit.rebase`: goal = fresh measured pose + the standing
  sag, ~7.4 deg on the shoulder, without slewing); and **no mid-air release**: a leg
  short by <= the 10 deg lag trip counts as arrived and the return goes on; any other
  failure (path not clear, overspeed, no reading, a leg far off) re-anchors and HOLDS
  until the second press. Review found a real bug fixed here: the 6 deg arrival rule
  would have released the arm at start-pose height after a Dream stop. Lag trip stays
  10 deg (Gavin chose A), so visitors feel ~5 Nm before a stop (the arm deflects
  ~19 deg at 10 Nm). Cameras: the `head` camera's unique id changed after the restart
  (0x1300000c45636b -> 0x1130000c45636b, a different USB port), record updated with a
  backup; `--overhead black` is now a deliberate wrist-only setup.
- *Status (2026-11-08, later): first live Dream calls.* Gavin raised the arm's base;
  the rest then read 15 mm into the old floor, so floor -6 -> -31 mm (measured lying
  on the glass) and rim 427 -> 411, keep-out and workspace box both. Live 205156Z:
  approach tripped lag at 10.2 deg on the shoulder (sag ~9 deg at ~5 Nm) -> Gavin
  "make it 20deg" for shoulder_lift only. Live 205544Z and 205933Z (1 billed call
  each): the start pose was reached and Dream's first chunk asked for tcp z 16 then
  -5 mm (toward the golf ball on the floor), refused by the box; each stop returned
  to rest cleanly. Grace (DreamLabs), via Gavin: "the control loop should overwrite
  what dream is outputting so that its remains in the aquarium limits" -> a box or
  keep-out violation now cuts the chunk at the last allowed substep and holds there
  until the next call instead of ending the run (jumps, bad rows, other faults still
  stop). `--pause-s` holds at the start pose before the first call so Gavin can place
  the ball. Refused chunks are now logged.
- *Status (2026-11-08, later): first complete Dream-1 run on the real arm*
  (`runs/live-20261108T210555Z`, prompt "pick up the golf ball", wrist camera only,
  3 billed calls). Approach, 20 s pause (Gavin placed the ball), call 1's chunk ran in
  full, call 2 was overridden at row 10 (tcp z -8 < box -5), call 3 at row 0; call cap
  -> re-anchor, gripper limp, 3 return legs, released at the rest, no problems. Dream
  kept the gripper closed (0.01-0.04 of 1) and kept reaching down. Findings: the
  shoulder's sag puts the real hand ~22 mm below its command (measured tcp z -27 mm,
  4 mm above the floor bound, while commands stayed >= -5), so the box's z limit does
  not bound the real hand; floor contact is allowed, walls are less affected (the sag
  is mostly vertical). Peaks: shoulder 5.5 Nm, lag 11.4 deg, speed 20.8 deg/s,
  shoulder 25 -> 39 C in under a minute (watch heat on longer runs).
- *Status (2026-11-08, later): slide along the limits.* Gavin: "it should be able to
  slide along the limits of the aquarium" (the override cut the chunk and froze the
  arm, wasting two of three calls). Dream's left hand targets are now clipped into the
  tcp box in Dream's reply, before Dream's own IK (`SlideTransport`,
  `core.clamp_eef_rows`), so the hand rides the limit while the rest of the motion
  (sideways, wrist, gripper) continues; the box and the tank guard still check every
  substep after the IK, and cut only as a backstop. Slide margins: 2 mm off the floor
  and rim limits, 40 mm off far/near, 100 mm off the side limits, because the
  wrist_roll_link mesh reaches ~90 mm past the tcp toward a side wall and the guard
  keeps it 70 mm off the glass. Dry run, Dream pushing 25 cm into the right wall:
  37-47 rows per call slid, no cuts, 153 substeps executed. A pick needs ~10-20 calls
  (each call = one ~2.4 s chunk from the current image).
- *Status (2026-11-08, later): first sliding run* (`runs/live-20261108T211253Z`,
  --max-calls 20, stopped by me at call 8 on stale monitor notifications: I thought
  the elbow cuts were repeating, but calls 5-8 were already sliding cleanly; 13 billed
  calls today). Calls 1-3 cut by the workspace box's elbow x <= -30 limit (set when the
  near wall was the 40 mm estimate; the wall is now 80 and the mesh guard keeps 70 mm).
  Calls 4-8: hand slid along the floor limit (48/48 targets clipped), creeping from
  37 to 34 cm out, ~1.5 cm right. Dream never opened the gripper (0.00-0.04 of 1 in
  every call) and kept asking below the floor limit. Next: raise the ball 3-4 cm,
  loosen the elbow's back limit, ask Grace about grip behaviour with wrist-only input.
- *Status (2026-11-08, later): sag, gains, elbow limit.* Gavin saw the arm "go back
  to rest" mid-run: it was (a) Dream's call 1 pulling the hand up and back (46 -> 23
  cm out, 17 cm up) and (b) my STOP. Cause of (a): sag, much larger than I first said.
  At the start pose, in water (holding torque still ~4.9 Nm), the soft 30/2 gains left
  the shoulder 9.5 deg and elbow 5.4 deg low, the hand ~12 cm under its command, so
  Dream saw a hand near the floor and lifted it. Runner-only gains 60/3 -> hand 7 cm
  low (rehearsal 211903Z); 90/4 -> shoulder 3.2, elbow 2.0 deg, hand ~5.4 cm low
  (212220Z), Gavin's choices. The plan id now also covers the gains and the keep-out.
  Live 212338Z (90/4, 20-call cap, stopped by me at call 17 on a fresh log check): 13
  of 17 calls executed nothing, cut by the workspace box's elbow x <= -30 limit while
  Dream's hand was 32-39 cm out; Dream again never opened the gripper (convention
  checked: Dream and the runner both use 0 closed, 1 open). Fixed: elbow x max
  -30 -> +10 and wrist_pitch -20 -> +20 (the wall is 80 mm, not the 40 estimate), and
  the run stops after 3 calls in a row that execute nothing. 31 billed calls today.
- *Status (2026-11-08, later): Dream reaches the ball area, never opens.* Floor
  limit lowered 3 cm (tcp/wrist z min -5 -> -35, slide margin 0 at the floor; the
  tank guard's 10 mm press stays the real floor limit), Gavin's call. Live 213138Z
  (90/4 gains, stopped by Gavin after call 10, 41 billed calls today): call 1 cut by
  the guard at row 14, calls 2-4 one substep each, then calls 5-10 full chunks: a
  steady descent of the hand target 98 -> -19 mm at 31-41 cm out, 4-5 cm right; the
  real hand reached -26 mm (floor -31). Dream kept the gripper at 0.00-0.03 in every
  call. Open questions: overhead camera (wrist-only is the condition Grace wanted
  tested; Dream approaches but never opens), and the physical gripper direction
  (README: closed -2.9, open -115.1, never confirmed on the arm).
- *Status (2026-11-08, later): jaws on the glass; floor -28; orientation-aware slide.*
  Gavin pressed STOP in 213138Z because the jaws hit the glass: the tip's lowest mesh
  point plateaued at -27.3..-28.3 mm while commands kept going lower, so the floor is
  -28 (keep-out updated, Gavin approved). Gavin then asked that the arm follow Dream down
  to the limit and keep going forward/sideways rather than give up. A subagent built
  `core.FloorSlide` (pre-IK, in `SlideTransport`): each row's tcp z is raised just
  enough that the hand's lowest mesh point (fingers at the row's grip, wrist_roll,
  wrist_yaw at worst roll) stays at floor + 10 mm + 224 mm/m^2 x reach^2 (a sag fit on
  1520 ticks at 90/4, sd 2 mm, max 8.9; re-fit if the gains change); x, y, rot6 and grip
  untouched. Dream's eef_rot6 = flange rotation x the arm file's tool axes (z approach,
  x jaw), residual median 0.6 deg on well-tracked rows. Corrections to what I had told
  Gavin: none of 213138Z's cuts were the floor (call 1: forearm vs the back wall; calls
  2-4: wrist_pitch vs the rim, both from the elbow folding up and back when the hand
  comes close to the base), the tcp is AT the fingertips (not ~3 cm above), and the
  sag at the hand is 27-53 mm off the glass, not 5-7. Offline replay of 213138Z through
  Dream's real IK: no floor cuts; calls 9-10 (reach ~310 mm) still cut by the forearm
  vs the back wall. 154 targeted tests pass.
- *Status (2026-11-08, later): stuck folded at the back wall; a hold that would not
  release.* Live 215535Z (20 calls, 61 billed today): Dream's call 1 folded the arm up
  and back (as at the start of every run) until the forearm sat ~4 mm off the guard's
  back-wall limit; calls 2-20 were then cut at row 0 one after another (each sent one
  substep, so the stuck check, which counted only empty calls, never fired). At the
  call cap the return's first leg was not clear of the back wall, so the arm HELD
  mid-air as designed. Bug: Gavin supported the arm and pressed STOP once, but the stop
  had come from the call cap, not a signal, so that press only counted as "stop" and
  the hold kept waiting; I sent the release after he said he was holding it. Fixed:
  entering a hold arms the release, so any one press releases (test proves it fails
  without the fix). Still to do: let the guard move a pose that sits on a limit away
  from it (it currently freezes), report the commanded rather than the sagged pose to
  Dream (its first move "corrects" the ~5 cm sag by lifting and folding back), and
  count near-empty calls as stuck.
- *Status (2026-11-08, later): three fixes for the stuck runs (subagent, reviewed).*
  (1) Dream is now told the COMMANDED arm pose (gripper still measured), so the ~5 cm
  sag no longer looks to Dream like the hand dropping (`--dream-state`, default
  commanded); every safety check still uses the measured pose. (2) The tank guard walks
  each segment with the motion bound (a sample with worst slack m proves +-m/B around
  it) instead of demanding the 2 mm sweep clearance at sample 0, so a pose clamped onto
  a limit can move away from it but never closer; replay of 215535Z: 12 of 19 later
  chunks now get past row 0 (the other 7 put the forearm past the back wall in Dream's
  own row 0), and all three stop legs from the final held pose are clear (the old guard
  held the arm). (3) Calls that execute <= 2 substeps count as stuck (3 in a row stop the
  run). Also: floor buffer 0 (Gavin: "sagging a bit to slide on the bottom ... is ok"),
  and the loop no longer catches up after a slow planning tick (it used to send queued
  substeps back to back, faster than the 15 deg/s plan). 162 targeted tests pass.
- *Status (2026-11-08, later): first run with the three fixes* (`live-20261108T222518Z`,
  11 calls, 72 billed today; full suite on gavin-linux clean apart from the 3 failures
  that predate today). Dream no longer retreats: told the commanded pose, it kept the
  hand ~45 cm out and 12-15 cm up and drifted slowly in (49 -> 41 cm) and right
  (-18 -> -54 mm) over calls 1-7, all executed in full (~20 s of uncut Dream control).
  It never descended (gripper tip never below +62 mm) and never opened the gripper.
  Call 8 was cut at row 43 and calls 9-11 at row 0 with the forearm on the back-wall
  limit (0.1 mm slack) even at 41 cm reach, so the new stuck rule ended the run after 3
  calls; the return reached the rest. The back-wall limit (Gavin's 80 mm + 70 mm forearm
  clearance) binds whenever the IK folds the elbow up.
- *Status (2026-11-08, later): two cameras.* A rehearsal hold showed the wrist camera
  does see the golf ball from the start pose (upper left, near the far glass, likely
  beyond 42 cm). Gavin then mounted an overhead camera: the record's cameras are now
  `left` = the wrist camera (0x1130000c45636b, rotation 180, formerly `head`) and `head`
  = the new overhead (0x1120000c45636b, rotation 0, pending Gavin's upright check);
  backup `backups/viscous_waterproof_01.before-two-cameras-20261108.json`. Runs use
  `--overhead head --wrist-l left --wrist-r black`. The inputs id stays 9430f986ab69
  (Dream's id covers slot names and frame sizes, which did not change). Both ids are
  USB-port based: swapping the cameras' ports swaps them.
- *Status (2026-11-08, later): first two-camera run* (`live-20261108T223649Z`, overhead
  `head` + wrist `left`, commanded state, 15 calls, 87 billed today). 11 calls executed
  in full (~35 s), then calls 12-15 were cut near the back wall and the stuck rule ended
  it; return to rest clean, peak 43 C. Dream's own hand targets (before any clamp) hovered
  12-15 cm up and crept in 49 -> 42 cm and ~3 cm right; it never descended and never
  opened the gripper, same as the wrist-only commanded-state run. The only run where
  Dream descended to the floor (213138Z: hand target 21 cm -> -19 mm) was wrist-only
  with the MEASURED (sagged) state, so the state change may have removed the descent
  too; untested. Next: ask Grace with this data before spending the last calls.
- *Status (2026-11-08, later): skip unreachable rows instead of dropping the chunk.*
  Live 223929Z (two cameras, MEASURED state, 4 calls): call 1 retreated again (48 -> 36
  cm, up), parking the forearm on the back-wall limit; calls 2-3 then asked to reach back
  out to ~48 cm and down to ~4 cm (toward the ball) but were cut at row 0 and the stop
  held mid-air (Gavin supported it; one press released it, the earlier fix works).
  Diagnosis: not a guard bug. Dream's chunk opened by folding further back (row 10: the
  forearm 29 mm past the back-wall line) before swinging out, and the planner dropped
  the whole chunk at the first refused row. Now an unreachable row is skipped and the
  arm heads for Dream's later rows (every substep still guard-proven and <= 15 deg/s;
  IK flips judged row to row on Dream's own chunk). Replaying call 2 from the stuck pose:
  17 rows skipped, 118 substeps (5.9 s), hand 41 -> 48 cm out and 119 -> 39 mm down;
  this replay is now a test. Gavin raised today's call limit to 200 (ledger cap edited).
- *Status (2026-11-08, later): Dream opens the gripper and works the ball area.* The
  approach first tripped the 5 Nm / 5 s tier (live 224555Z, 0 calls: lifting off the floor
  holds 5.3-6.2 Nm for 5-6 s); Gavin approved 6.5 Nm. Live 224827Z (both cameras,
  measured state, skip-ahead, 11 calls, 102 billed of today's 200, stopped by Gavin):
  ~48 s of Dream control, 764 substeps. Call 1 the usual pull-in (skip-ahead kept 83
  substeps); calls 2-3 down to the floor at ~40 cm, centred, gripper opened to 0.72 /
  0.51; calls 5-11 at ~16-18 cm out, at floor height, side to side (y -54..+40 mm),
  opening and closing (measured jaw down to -83 deg of -115). First full reach/open/close
  behaviour on the real arm. Gripper tip lowest 6.5 mm under the -28 floor line (the
  guard allows 10). Return: three legs settled, released at the rest. Peaks 6.1 Nm,
  41 C, 23 deg/s.
- *Status (2026-11-08, later): camera bandwidth, and a stop that must not need cameras.*
  Gavin stopped 224827Z because Dream kept folding back and grasping short of the ball,
  which was further forward. Likely cause: our wrist camera (0C45:636B, narrow field of
  view) is not the Maker's KD-USB fisheye (2C7F:4A60, ~110 deg, mounted above/behind the
  jaws looking along the fingers); the 2026-10-20 sim found wrist-camera match decisive
  (6/6 only with the Maker camera). Live 225249Z then tripped 3 s into the pause: the
  overhead camera fell to 640x360. Both cameras share one USB 2.0 hub with MJPG refused
  (uncompressed 640x480 at 30 fps each, ~300 Mbit/s), and neither accepts 15 fps or
  320x240 through OpenCV on macOS. Worse, the return read the robot's full observation
  (motors + cameras), so the dead camera made every reading fail and the arm HELD
  mid-air (Gavin supported it; one press released it). Fixed: every stop now closes the
  cameras first and returns on motor readings only (test: a camera dying mid-run still
  reaches the rest). Gavin replugged `head` (now 0x1300000c45636b, on the outer hub port); a 60 s dual-camera soak
  then read 640x480 from both with no failures (both still share the upstream hub).
- *Status (2026-11-13): keep-out moved to the new base mount, offline, pending
  Gavin's approval of the diff.* Why: the arm now sits on an upright L-bracket (CAD
  from Gavin's 3MF): it bolts to a floor plate with J1 at the bolt pattern's centre,
  and a 12 mm truss post 80 mm wide stands flush on the back glass (Gavin confirmed
  flush). J1 is 106 mm from the post's inner face and 118 mm from the glass, 50 mm
  from the post's +y edge and 30 mm from its -y edge. The clamp is weak, so the base
  can slide or rotate if pushed; Gavin resets the post to the middle of the back glass
  on marker lines each time, and the numbers assume it centred (177 mm gaps each side).
  Keep-out: `near` 80 -> 106 (the post face; the 12 mm to the glass is slack for J1
  off the bolt centre), `far` -654 -> -616 (from the glass, 118 - 734; near - 734
  would place it 12 mm too far), `left` 210 -> 227, `right` -224 -> -207 (17 mm
  closer), statuses "cad", yaw "assumed". The upper-arm heel keeps its absolute limit
  (40 mm behind J1, margin 40 -> 66, padding 0) because of a corner fin of the mount
  42-50 mm to +y of J1, from about 16 mm behind J1 back to the post, which is not
  modelled; the shoulder came within 11-17 mm of it in logged poses. Workspace box:
  the wall-derived limits moved with their walls (elbow x max +10 -> +36, wrist_pitch
  +20 -> +46, every y range [-154, 140] -> [-137, 157]); the J1-relative tcp/wrist
  limits stay. Replay of the five live runs of 2026-11-08 (planner and guard
  parity-checked against the runner's own code): 2828 of 2928 rows reach their
  target, vs 2167 with the 80 mm wall; every remaining cut is the upper-arm heel at
  the near plane, none at a side wall or the far wall. The start pose, the tank rest
  and the five runs' measured rests and Dream starts all pass the start gate. The far
  glass is now close at the rest: the fingers lying down the tank keep 10-28 mm of
  slack beyond the 30 mm margin (was about 55). Targeted tests pass after two pinned
  numbers were updated (the heel's 66 mm rule; the trace route's right/far slack).
  The plan id changes, so the next live run needs a fresh dry run. `tank.json` and the
  sim scene still say 40 mm. The base can still slide or rotate under a push on the
  glass and neither the box nor the guard can see it, so before every powered run:
  post between the marks and flush on the back glass.
- *Status (2026-11-13, later): the mount's fin is not modelled; the post is the back wall.*
  Gavin: "i dont see how the fin is an obstacle as the tank will limit things before the
  fin is hit. the back post is still a thing as j3 can hit it". A Codex round did model
  the fin and post from the mount 3MF as convex obstacles (5 mm margin, local branch
  `feat/viscous-tank-mount-fin-2` at 61299e9e on the Mac, not pushed) and agrees: in
  the replay no row was ever cut by the fin (2838/2928 with the heel loosened, vs 2828
  here), and the shoulder housing never comes within 11.3 mm of it at any J1. It cost
  about 5x per pose check (0.5 -> 2.7 ms), so it was parked. The post stays covered by
  the near plane at its face (106 mm), applied across the full width, so J3 and every
  link beyond the elbow keep 70 mm off it; the heel keeps its 40 mm limit.
  Gavin, same night: "no need to leave the 5mm gap between fin and the arm, they dont
  hit each other, ive played with it" (moved by hand through its range, no contact), so
  the fin needs no keep-out at all.
- *Status (2026-11-14): limp-motor trip, offline, on branch `fix/viscous-motor-limp-trip`
  (not merged).* Why: in the powered rehearsal `rehearse-live-20261114T082911Z` the
  shoulder (RS00) held the start pose underwater at 5.1 Nm, 48 C, and 115 s in its
  torque read 0.00 Nm in one tick while it kept answering with fresh positions; the arm
  fell. The runner saw it only through the overspeed trip, a tick later, then returned
  and "held" against a dead motor, and the tick log stopped at the trip. Every read
  sends the RobStride clear-fault frame and the driver decodes no mode/fault bits from
  the MIT reply, so a cut-out motor is invisible except as torque collapse. Now: a `limp` trip (arm joint
  < 0.25 Nm while > 3 deg behind its goal, one reading; zero hits over all 23 logged
  runs, fires on the drop tick), no return and a loud "support the arm" when a joint
  is limp, the dead motor never re-enabled, every stop/hold reading in `ticks.jsonl`,
  and every joint's torque/temperature at the trip in the summary (`at_trip`). Details
  in the runner README (safety layer 6a). The plan id changes: fresh dry run first.
- *B3a. Propeller scrubbing, zero-shot (Gavin, 2026-10-23).* "Give it a ship
  propeller and ask it to clean it. Let's see whether it knows how to do that, or
  at least ... do scrubbing motion on it. Let's see what happens." This is the
  Viscous arm's showcase job (the ROV propeller-cleaning film in
  viscous-arm-simulation `cinematic/`), here on the sim table: a small bronze
  propeller fixed on a stand, a sponge beside it, four prompts, YAM as a control.
  Brief `scripts/viscous-dream1/BRIEF_PROPELLER.md`. Exploratory: Gavin wants to see
  what it does, not a precise success rate.
  *Result (2026-10-23):* it knows to take the sponge to the propeller but not to
  scrub. See "Propeller scrubbing" under the Dream-1 trial.
  *Revisit (2026-10-24, Gavin):* the earlier result "wasn't done very nicely".
  First find real cleaning videos and set the environment up correctly. Keep a
  desk for Dream-1; use **four blades pointing up/down/left/right**, select the
  horizontal **left blade**, **left arm gripping its outer edge**, **right arm
  scrubbing it**. Codex Astra may help if Dream-1 cannot do it. Commercial
  polishing footage found and representative frames inspected; references and
  the environment-first brief are in
  `scripts/viscous-dream1/BRIEF_PROPELLER_BIMANUAL.md`. Build and show the scene
  and simultaneous working poses before any further billed policy trial.
  *Environment result (2026-10-24):* Codex Astra built the isolated
  `propeller_cardinal` scene: 320 mm pitched bronze propeller, cardinal blades,
  fixed horizontal shaft, original desk and three policy cameras. Bases moved
  to 440 mm spacing so both arms reach the screen-left blade. The left jaws
  pinch its outer 135–160 mm radius; the right jaws hold a free 52 mm circular
  pad. A 15 s scripted physics clip, from an explicitly initialized pregrasp,
  maintains both grasps throughout the 12 s scrub and actual pad contact for
  99.46%, with 26.14 mm radial travel, two continuous-contact reversals, no
  unintended collisions and no welds. Opening the right jaws drops the tool.
  The left arm stays at its 5 Nm cap; pickup/approach, torque margin, abrasion
  and Dream performance remain unproven. No new Dream calls or hardware access.
  Show `scripts/viscous-dream1/runs/propeller_cardinal/ready_detail.png` and
  `scripted_feasibility.mp4`; report and reproduction instructions are linked
  from `scripts/viscous-dream1/PROPELLER_CARDINAL_README.md`. Code, procedural
  blade assets and required arm CAD are repo-local; old tasks are unchanged.
  *Scale/presentation refinement (2026-10-24, Gavin):* make the blade "at least
  double or triple the size". The tabletop is temporary scaffolding for Dream-1;
  establish motion first, then remove the desk and make the task look underwater
  and beautiful in a video. An underwater scene preview is requested now too.
  Target 3× linear propeller size (320 → 960 mm diameter), same-size arms/tool,
  same left outer-edge hold/right scrub. Recheck reach and clearance rather than
  enlarging only the camera view. Keep a table motion view and a distinct
  underwater presentation view; neither is a new Dream result. Brief:
  `scripts/viscous-dream1/BRIEF_PROPELLER_LARGE.md`.
  *Larger scene result (2026-10-24):* `propeller_large` achieves **960 mm**
  diameter in both visual and collision geometry, with unchanged arms and
  52 mm pad. Temporary 320 mm arm risers and a 70 mm inward move of the right
  base keep the outer left blade reachable and the bottom blade 50 mm above
  the desk. The final 12 s scripted scrub maintains both jaw holds and actual
  pad contact at every 1 ms substep (100%), with 34.14 mm radial travel, four
  strict reversals and no unintended penetration. Left contacts are at
  453.59–473.70 mm of the 480 mm radius. The initialized pregrasp and left-arm
  5 Nm saturation remain limitations; no new Dream calls or hardware runs.
  `runs/propeller_large/underwater/underwater_preview.mp4` replays that exact
  recorded motion with the desk removed, a submerged service frame, procedural
  vessel stern/shaft, blue-green water/fog and broad lights. It is a visual
  preview, not floating-ROV or wet dynamics. Both 15 s videos fully decode;
  geometry/contact checks pass. See `PROPELLER_LARGE_README.md` and the report
  in `scripts/viscous-dream1/runs/propeller_large/`; the 320 mm baseline remains.
  *Correction (2026-10-24, Gavin):* the blades are "wonky". Correct the marine
  propeller geometry and motion, changing only the propeller and its interaction
  targets in the current bench scene, then show him before changing the
  environment. The underwater service-frame/generic-hull preview is rejected.
  The later scene must mount both arms on the actual BlueROV2 Heavy, working
  beside a correctly referenced ship underside/stern/propeller arrangement in
  open water, without a fabricated seabed. Manufacturer propeller images and
  primary shipyard references recorded in
  `scripts/viscous-dream1/BRIEF_PROPELLER_CORRECT.md`; no scenery rebuild now.
  *Corrected propeller result (2026-10-24):* `propeller_corrected` replaces the
  leaf-strip blades with smooth cylindrical helical sections, rounded tips and
  coherent pitch, using Solé/France Hélices/Palumbo photographs as visual
  references. Diameter stays 960 mm; four blades are exact 90° copies. The
  visible surface uses the collision cells' exterior triangles. Non-blade XML
  and compiled table/risers/stand/bases/cameras/lights/tool/force limits are
  verified unchanged. The 12 s scripted scrub maintains both holds and pad
  contact at every substep, with 31.506 mm radial travel and three strict
  reversals. Left contacts lie at 453.601–478.827 mm radius; unintended
  penetration is zero and maximum intended penetration is 0.772 mm. Release
  drops the free tool 561.829 mm. Mesh/contact checks and all 150 video frames
  pass. Initialized pregrasp, fixed shaft and left-arm 5 Nm saturation remain;
  this is not a policy/autonomy or hardware result. No Dream calls or scenery
  changes. Show the head-on/oblique/side renders and 15 s clip under
  `scripts/viscous-dream1/runs/propeller_corrected/`; reproduction and full
  evidence are in `scripts/viscous-dream1/PROPELLER_CORRECTED_README.md`.
  Gavin reviews the geometry/motion before the later BlueROV/ship scene.
  *Larger-motion request (2026-10-24, Gavin):* "i want larger motions, the side
  way motions is not enough." The corrected scene's 31.506 mm radial stroke
  reads as a small wiggle. Keep the corrected propeller and all scene/model
  settings unchanged; expand real pad sweeps across the same blade, targeting
  100–150 mm actual in-contact travel with a slower/longer clip if needed.
  Preserve the left outer-edge hold and measure stroke spans and contact,
  rather than substituting accumulated distance or camera movement. Brief:
  `scripts/viscous-dream1/BRIEF_PROPELLER_WIDE.md`. Still no scenery rebuild.
  *Wide-motion result (2026-10-24):* `propeller_wide` shifts the scrub patch
  inward and commands wider, slower sweeps on the unchanged corrected scene.
  Actual radial span is 131.641 mm (4.178× baseline); three complete strokes
  measure 128.354 / 122.144 / 125.432 mm, each at 100% simultaneous contact.
  The full 24 s scrub is 99.9667% in contact, with one initial 8 ms pad gap;
  both grips stay at 100%. No unwanted collisions, maximum intended penetration
  0.769 mm, release drops the free tool 578.599 mm. Model/XML and 3,563 frozen
  baseline-file checks pass. The right hand rolls its grip 90° and follows
  the surface normal; no geometry, fixture, camera, friction or force-limit
  changes. Both 27 s real-time clips fully decode. Show
  `scripts/viscous-dream1/runs/propeller_wide/contact_detail.mp4`; diagnostic
  views and evidence are linked in `PROPELLER_WIDE_README.md`. Initialized
  pregrasp, fixed shaft and continuous left-arm 5 Nm saturation remain;
  this is scripted simulation, with no Dream calls or hardware activity.
  *Presentation phase authorized (2026-10-24, Gavin):* extend the sweep toward
  the edge if possible and explain the limit if difficult. Now make it realistic:
  improve the drill/polisher 3D model, clean different overlapping areas, and
  reveal copper/bronze beneath green fouling only where the pad has passed.
  Bring in the actual BlueROV with both arms mounted on it, proper ship-underside
  context and no seabed. "Do a lot ... in Blender"; his follow-up explicitly
  says find the CAD in MakerMods Brain and do Blender/most hard work on
  **gavin-linux**, not the Mac. Brain's `Viscous Bimanual ROV Model` note and
  source JSON locate the native CAD-derived BlueROV2 Heavy rig at Linux
  `Documents/viscous-rov-model/blender/viscous_rov.blend`; its original SHA-256
  matches on disk, with native arm mounts 493.05 mm apart. This is separate from
  the newer concept vehicle. Physical edge coverage, the tool asset, and native
  ROV Blender integration are separate coordinated subtasks; no policy/autonomy
  or validated wet-dynamics claim. Brief and source/interface details:
  `scripts/viscous-dream1/BRIEF_PROPELLER_CINEMATIC.md`.
  *Zigzag refinement (2026-10-24, Gavin):* explicitly use a zigzag/serpentine
  trajectory: long sweep, shift across the blade, reverse on the next lane.
  The video should visibly cover new dirty area throughout instead of cleaning
  the same strip repeatedly. Overlap adjacent lanes to avoid gaps, measure
  unique swept area, and drive the persistent green-to-copper reveal from the
  actual pad footprint. Forwarded to both ongoing coverage and Blender workers.
  *Polisher asset milestone (2026-10-24):* `cinematic/tool/polisher.blend`
  replaces the primitive visual tool with a sealed marine-blue housing,
  rubber grasp sleeve, machined spindle/fittings, layered backing and grainy
  abrasive pad. Original procedural Blender geometry/materials; 32 mm grasp
  diameter, 52 mm pad and 52×52×145.12 mm overall envelope. Native append,
  deterministic rebuild, clear jaw band, local anchors and packed resources
  pass on Linux Blender 4.5.3; three CPU-rendered previews reviewed. Source,
  manifest and focused verifier are beside the asset. Integration into the
  native ROV is in progress; the visual shell has no validated mass/collision
  model, so old free-tool physics does not validate this new casing.
  *Edge/zigzag milestone (2026-10-24):* the unchanged corrected bench now runs
  four continuous lanes (`u=.13,.17,.21,.25`), shifting at every reversal.
  Actual radial span 143.10 mm; all 25,500 scrub substeps retain pad contact
  and both bilateral holds, with no unintended contacts. Conservative pad-rim
  clearance is 1.90 mm and left-arm clearance 18.66 mm. Exact rim contact is
  intentionally avoided. Geometric pad/surface overlap union is 57.40 cm²
  versus 24.22 cm² for the instrumented single-lane rerun (2.37×); nominal
  whole-disk footprint is 137.09 cm². Actual overlap covers only 4.59% of the
  blade face, with no abrasion/material-removal claim. Parent reran the Linux
  validator: 28,500 samples, 3,596 frozen baseline files and decoded videos
  pass. Show `runs/propeller_edge/contact_detail.mp4`; source, coverage masks,
  rejected trials and reproduction are in `PROPELLER_EDGE_README.md` under
  `scripts/viscous-dream1/`. Frozen `coverage_plan.json` SHA-256 is
  `b18d8c4e36ad3d784e0d4e498b34cf3c4bfb0410bec9e2fc9d1c628dc98df7db`.
  Initialized pregrasp, fixed shaft and continuous left-arm 5 Nm saturation
  remain. Those bench results do not validate native-vehicle dynamics or the
  new visual tool's physics.
  *Native-ROV cinematic complete (2026-10-25):* the 28 s, 720p/24 fps film is
  `scripts/viscous-dream1/cinematic/results/cleaning_preview.mp4`; editable,
  packed `final_scene.blend` and reproducible source/assets are alongside it.
  Built and rendered on gavin-linux in Blender 4.5.3/Cycles/OptiX. The original
  2026-09-18 CAD-derived BlueROV2 Heavy retains all 100 native mesh instances,
  their parents/local geometry, 493.05 mm arm mounts and eight thrusters. The
  approved 960 mm four-blade STLs sit under a reference-informed single-screw
  hull aperture with shaft boss and aft rudder, in open water with no seabed.
  Parent reviewed final establishing/contact views, matched green-to-copper
  frames and the finished frame sequence; all 65 manifest file hashes match.
  The full 672-frame movie decodes without errors.
  Exact IK alone had hidden arm/chassis crossings: evaluated Blender/native
  FK assertions and visible-mesh BVH checks drove the final placement and
  continuous elbow branch. All 1,343 film/half-frame poses pass 8,399 geometry
  pairs with no unintended intersections; explicit native bearing and intended
  grip/pad interfaces are excluded. This is an intersection audit, not a
  signed-distance clearance certificate. Tool v1.1 narrows only its rear
  fittings to clear native fingers; a +20 mm mounting offset puts its contact
  anchor 101 mm from tool0. The actual sleeve/jaw fit was checked.
  Native retargeting keeps the 143.10 mm radial span and four alternating lanes
  but narrows chord offsets by 40%, shifts 1.185 mm inboard, aligns the pad to
  the face normal and retimes 25.5 s scrub to 22 s. Maximum source-path change
  is 10.56 mm. The vehicle pitches up 62.4°; the limiting shoulder has only
  0.005 rad joint margin. These are explicit kinematic-presentation limits,
  with no stationkeeping, force/torque, wet-dynamics, autonomy or hardware
  feasibility claim. The persistent copper reveal covers 117.56 cm² of nominal
  pad footprint, clipped to the frozen source nominal union; it is distinct
  from the bench's 57.40 cm² actual ellipsoid overlap and models no abrasion.
  The isolated package rebuild reproduces motion/mask bytes and nine-keyframe
  scene geometry/transforms. Manifest, audits and all limitations are in
  `cinematic/README.md` and `cinematic/results/`; no Dream or hardware calls.
  *Horizontal vehicle correction (2026-10-25, Gavin):* "have the bluerov be
  horizontal"; the previous 62.4° presentation tilt is rejected. Keep native
  mounts/CAD and solve whole-vehicle yaw/translation plus arm branches with
  pitch and roll fixed at zero. Gavin also requests better ship references or
  an adapted ship CAD model. Additional SMH Marine dry-dock stern photographs
  now inform a closed, fuller transom/hull run, shaft boss, rudder and anodes;
  this replaces the earlier crude illustrative loft. The level solution now
  follows all 672 frames with original t/u and the full 143.10 mm radial span,
  removing the earlier chord narrowing and inboard shift. The same outer-edge
  point is held with a revised tangent-plane grip approach; allowing circular
  pad-axis roll in IK avoids the earlier pitched-vehicle workaround. Both grips
  check against the actual CAD. Minimum joint margin is now 0.02541 rad; peak
  joint speed 0.4745 rad/s. The final saved scene passes all 1,343 film/half-frame
  poses over 9,069 intersection pairs, with no unintended intersections. The
  isolated rebuild reproduces motion/mask bytes and exact scene geometry,
  transforms and camera lenses. Nominal reveal is 135.41 cm²; it
  remains distinct from the bench's actual geometric contact area. The revised
  first full render's image-variation check caught an obstructed ending: the
  camera pullback crossed the vehicle and produced nearly uniform frames.
  It is replaced by a cut at frame 625, after holding the clear contact shot.
  Final review also caught the opening camera's equivalent +π/−π Euler keys
  interpolating through a turn. Compatible Euler keys fix it, with aim/up
  checks at every film frame and a stronger raster-variation gate. The final
  28 s movie is complete: all 672 frames decode, no frame fails the image
  variation check, and maximum camera aim/up errors are 0.0211°/0.0156°.
  The final scene hash is
  `0f97ba1c44acbb2122908d9c3eba8d3eac9c2a85761437aabab05958da90d93c`;
  the movie hash is
  `072b41d2322ca0bdd7c905d978cf30c051923d3239c926130a80dea76c48a2d5`.
  The new frame cache is bound to scene/motion hashes. Parent reviewed the
  corrected opening, ending, full contact sheet and eye-level profile; all
  72 manifest files match across machines. The final isolated rebuild matches
  motion/mask bytes and exact geometry/transforms/lenses at twelve keyframes,
  including the former opening-wrap frame and the final camera cut. Use
  `cinematic/results/cleaning_preview.mp4`, `final_scene.blend` and
  `horizontal_profile.png`; reproduction and source photographs are linked in
  `cinematic/README.md`. This is still a kinematic presentation, not validated
  wet dynamics. Keep the horizontal constraint in later refinements.
  *Continuous stripes and imported ship CAD (2026-10-25, Gavin):* the hull still
  needs improvement, so use an existing CAD model; at each stripe reversal,
  "once it stops then immediately start" the next motion. Gavin also requests
  existing Blender cinematic skills. The film remains entirely Blender/Cycles
  on gavin-linux. Applied the external Blender Cinematic Modeling workflow
  (source and selected guidance in `cinematic/assets/production_workflow.json`).
  The original illustrative stern is replaced with the official KRISO KCS hull
  and horn rudder IGES files from SIMMAN2020, uniformly adapted to the existing
  960 mm rotor. Raw CAD, tessellated surfaces, analytic normals and provenance
  are self-contained under `cinematic/assets/ship/`; `convert_ship_cad.py` can
  regenerate the meshes. The approved four-blade rotor replaces the KCS's
  original five-blade rotor; this is an adapted presentation, not a dimensional
  reconstruction of a real installation. The vehicle remains at 0° pitch/roll,
  with its original CAD, mounts and all eight thrusters unchanged.
  `continuous_path.py` smooths observed surface-coordinate geometry and uses
  positive arc-length timing through rounded lane changes, eliminating the
  source run's long reversal slowdowns. The 22 s active scrub preserves
  143.08 mm radial span and all three alternating reversals. Independent saved
  Blender-rig checks at 96 Hz (2,113 samples) measure zero dwell intervals below
  1 mm/s, 6.48 mm/s minimum and 25.97 mm/s median pad speed. Peak joint speed
  is 0.5802 rad/s; minimum limit margin stays 0.02541 rad. Maximum deviation
  from the corresponding source arc-parameter sample is 2.87 mm. This new
  authored path/timing has not been rerun in bench dynamics; original 100%
  physics-contact figures apply only to the frozen observed bench run.
  Nominal copper reveal is 135.20 cm², clipped to the frozen permitted nominal
  union, distinct from the original 57.40 cm² actual bench overlap. The close
  shot adds restrained f/16 depth of field while preserving readable contact.
  Final saved-scene audit passes all 1,343 film/half-frame samples over 7,863
  collision pairs, with zero unintended intersections. The 96 Hz contact-path
  check stays within 5.71 µm of the interpolated analytic surface. The isolated
  rebuild reproduces motion/mask bytes and exact geometry/transforms/cameras
  at twelve keyframes. All 672 frames of the 28 s, 720p/24 fps movie decode;
  every frame passes the black/uniform-image gate. Parent reviewed the revised
  stern, level profile, matched contact stills and final contact sheet. The final
  scene SHA256 is
  `830c46c197062821a53de54a6f1582414c5a0370286ea994a8708e5755e90e64`;
  movie SHA256 is
  `77bc7f064f45bea52b8d72df51450d25f76952a5ed0a90b30a3edd0f7459a700`.
  The compact 83-file manifest covers source CAD, scripts, assets, reports and
  media. Use the same `cleaning_preview.mp4` and editable `final_scene.blend`
  paths under `cinematic/results/`. No Dream calls or hardware access.
  *Camera readability correction (2026-10-25, Gavin):* the rear ROV view
  blocks the moving arms, and the tight under-ROV shot shows hands without
  enough context to understand the task. Change cameras only; retain the approved
  level vehicle, existing ship CAD, tool, continuous stripes and reveal. Tested
  side/front compositions in Blender on gavin-linux. The selected film uses one
  consistent oblique view: a gentle three-second opening move, then a fixed
  working view through all scrubbing and the result. Both mounting bases, hands,
  whole working blade and the blue vehicle remain framed. No underbody cut or
  change of viewing side; deep focus keeps the task context sharp. A tested
  ending pullback reduced tool visibility, so it was removed.
  Every film frame passes sampled surface-visibility checks: minimum visible
  camera-facing sample fractions are 31.25% left hand, 25% right hand, 37.5% pad
  and 12.5% complete polisher. Both hand groups and tool samples remain entirely
  framed; the native body bounding boxes retain at least 9.7% screen margin.
  These finite samples support visibility, not simultaneous visibility of every
  surface point. Exact geometry and non-camera transforms match the approved
  7f51c28e scene at all 1,343 prior audited frame/half-frame poses; lights and
  packed contact clock match too. The prior BVH intersection result is carried
  forward only after that equivalence proof, recorded explicitly in
  `cinematic/results/camera_only_verification.json`. Motion and cleaning mask
  remain byte-identical; no geometry or physical trajectory was changed.
  The isolated rebuild matches motion/mask bytes and geometry/camera transforms
  at twelve keyframes. Final 28 s, 720p/24 fps movie: all 672 PNGs validate, all
  frames pass the raster-variation gate, ffprobe counts 672 frames and full
  ffmpeg decoding reports zero errors. Parent reviewed the opening, all three
  reversals, end framing and final contact sheet. All 88 manifest files match
  independently recalculated hashes on both machines. Final scene SHA256:
  `0f5a3abffc2784a18f44dff2a967bae93fae8e671fc62272a682be8befc97f1f`;
  final movie SHA256:
  `635a8793857ec1147f83b882c9b960d0240a94e5a6de11e8bf4cc012e4b359c9`.
  The existing `cinematic/results/cleaning_preview.mp4` and `final_scene.blend`
  paths now contain this consistent-view revision. Kinematic scope remains;
  no Dream calls or hardware access.
  *Camera direction and black canisters (2026-10-26, Gavin):* the side view is
  better but still unsatisfactory; find a camera/cinematography-specific Blender
  skill. Acrylic canisters flicker, so replace them with black anodized aluminium.
  Read and applied the dedicated `camera-cinematography` skill from arjun988's
  Blender skill library (entry/reference links retained in
  `cinematic/assets/production_workflow.json`). New shot design: three-second
  36 mm context view, then a same-side 48 mm task medium, fixed f/16 focus and
  no camera movement through scrubbing. Blade and hands take more frame space;
  both mounted arm bases remain visible while the vehicle's rear is cropped.
  `native_appearance.py` changes only material assignments on the two canister
  meshes: original transparent CAD_9 (alpha .2) and matching CAD_41 barrel become
  satin black metal with alpha 1, transmission 0, metallic .95, roughness .38.
  Native source blend, all mesh geometry, arm mounts and motion remain intact.
  The presentation-equivalence check allows exactly these two material changes
  and verifies all 1,343 prior audited poses remain identical. Both canister
  shader graphs are constant, with no transparency/refraction animation.
  An eight-second motion pilot covers the first reversal. Ray-probed canister
  interior ROIs pass the localized temporal check over its 120 working frames:
  maximum frame-mean luminance step .030 levels on the 0–255 scale, worst 95th
  percentile pixel step 1 level. This is a sampled stability check, not a claim
  about every reflected highlight or pixel. Full film is complete: all 672
  frames validate and decode, and all 600 fixed-working-camera frames pass the
  canister-region check. Final worst frame-mean step is .0549/255; worst 95th
  percentile pixel step remains 1 level. The isolated rebuild reproduces both
  canister shader parameters, motion/mask bytes, geometry and camera transforms.
  Parent reviewed the shot studies, eight-second pilot evidence and final film
  contact sheet. The 93-file package is synchronized and independently checked.
  Final scene SHA256:
  `597cf16f823f2039140c814498aff0724a90b641d5a017335cb0c98072c00b97`;
  final movie SHA256:
  `6c3774a3ce9416dde71cf428a04e7c187aaa404c4a813810afa7a836b1b2741e`.
  Canonical film and editable scene remain under `cinematic/results/` as
  `cleaning_preview.mp4` and `final_scene.blend`. No hardware or Dream activity.
- *B3c. Pipe-flange bolting (Gavin, 2026-10-23).* The propeller result "looks bad";
  instead "two pipes with flanges, and then the task: insert and screw in the nut".
  Read as: two pipe sections end to end with their flanges meeting, a loose bolt to
  push through the aligned holes and a nut to thread onto it. Gavin wants to see the
  environment before any billed run ("I want to see the environment setup first").
  Scene, thread model and a scripted feasibility demo first, no calls:
  `scripts/viscous-dream1/BRIEF_FLANGE_SCENE.md`.
  2026-10-23: v2 look (0.6 m coated pipes, weld-neck flanges, 7 coated studs, one
  empty hole) approved ("good"); the sim gripper's placeholder 10 N cannot lift the
  650 g bolt. 2026-10-24, Gavin: "now get Dream-1 to do the task. If too difficult you
  can start the task with the [bolt] and nut already in the grippers."
  `BRIEF_FLANGE_DREAM.md`: grip force derived from the RS00 rating (or a logged
  grasp-attach), a `pregrasp` start, then Dream-1 on table and pregrasp starts.
  Result: holding works (160 N jaw drive derived from the RS00 rating and the CAD
  crank, plus contact-gated welds), but the scripted insertion needs 7.8-9.4 Nm
  (7 Nm for the arm alone) against the 5 Nm limit, so no calls were made
  (`runs/flange_dream_report.md`). Gavin: "don't worry about the torque limits":
  the flange task alone lifts them to 25 Nm and logs time above 5 and 14 Nm
  (`BRIEF_FLANGE_DREAM2.md`). The real dry arm dropped at ~5.6 Nm on the shoulder,
  so these poses need a lower, closer fixture or buoyancy before hardware.
  *Result (2026-10-24):* 0/12. 360 billed calls (ledger 1986/2212). With the parts
  held, Dream-1 brings the nut toward the bolt in free air (G1), or drops the bolt
  and re-grasps it (G2). With the bolt already through (G3) it carries the nut to
  the flange and holds it near the bolt tip, at best 10 mm off axis against the 3 mm
  engagement gate, and never turns it. YAM does no better, so like scrubbing, this is
  a skill zero-shot Dream-1 lacks. The scripted demo also fails, on IK branch and
  joint limits at the hole rather than torque, so the fixture placement itself is
  marginal for the Viscous. Report `runs/flange_dream2_report.md`; logs gzipped,
  six videos kept, chunks on gavin-linux.
  *Next (Gavin, 2026-10-24): "try with Astra then".* Codex model `gpt-6-astra` as the
  robot's brain instead of Dream-1: it looks through the same cameras, can query a
  calibrated depth point per pixel, and commands hand moves, wrist turns and grips
  through a `robot` CLI on a live sim (no object poses, no Dream calls). Interface
  first (`BRIEF_ASTRA_HARNESS.md`, validated by a scripted operator on G3), then
  Astra on G3, G1 and T.
  *Goal reframed (Gavin, 2026-10-24):* "I don't really care how it's done ... go
  very slow, whatever ... as long as I can post this online and say that this is
  fully autonomous." So the deliverable is one clean, successful, fully autonomous
  sim video. Rules that keep the claim true: one continuous episode with no human
  input, resets or edits; the agent decides from cameras and depth only (no
  privileged object poses); real-time or labelled speed-up; logs kept. Post it as
  simulation, with the arm torque limit lifted beyond the real motors.
  *Result (2026-10-25): done in sim, episode 6.* Astra ran G3 alone, in one
  continuous run (743 commands, 23 min of robot time, ~2.7 h wall clock). It
  threaded the nut and ran it down snug against the flange (0.12 mm face gap;
  about 10 physical turns, 11.45 by the thread model, which counts axial play).
  Its Codex session log shows only `cat README.md`, `./robot` commands and 352
  views of its own camera images. Postable video:
  `runs/astra_flange/astra_g3_20261025_006/astra_flange_autonomous_10x.mp4`
  (16:9, 10x, caption panel). Evidence: Astra log, its final claim and the audit
  are there too; the full session (122 MB) and `episode.jsonl` stay on gavin-linux.
  What it took (each a harness bug, fixed by Sol before the next episode):
  - The trial-2 G3 bolt stuck out 3 mm (2 turns need 7): `--profile astra` seats
    it 30 mm deeper (33 mm protrusion), uses a real right-hand thread, and starts
    with a flats grasp.
  - Ep 1: a stalled move hung and the server died on the client's Ctrl-C. Moves
    now stop on stall or after 90 s, and the server survives.
  - Ep 2: a released nut freewheeled 7.6 turns by itself. The thread now has
    friction.
  - Ep 3: engage and release chattered at the bolt tip. The gates now have
    hysteresis.
  - Ep 4: manual regrasps lost the thread. Added `./robot screw` (turn, release,
    rewind and regrip as one skill, driven only by joint and jaw sensing).
  - Ep 5: Codex hit its usage limit (Gavin reset it). The prompt now gives the
    general steps (align, short fed turn, then `screw`).
  - Ep 6: `screw` falsely stalled on imperfect grasps (0.15 Nm wrist budget), but
    Astra regripped until it ran.
  Caveats for a post: simulation; arm torque limit lifted (25 Nm peaks against the
  RS00's 14 Nm); ideal thread model; the bolt starts inserted with the nut in the
  gripper; regrasping is a built-in skill. Open item: Sol's later adaptive-torque
  `screw` change (`BRIEF_ASTRA_SCREW_EFFORT.md`) did not pass its seated-stall
  check. Ep 6 ran the earlier version.
  *Social cut (2026-10-25):* `astra_g3_20261025_006/astra_flange_social.mp4`
  (1:56, captioned beats, no title cards). Gavin posts it as the funny "six
  attempts, no environment reset" video. A replay of ep 6 on the BlueROV failed:
  the native mounts cannot track the tabletop hand poses without clipping through
  the vehicle. Gavin: "Let's do it properly", so the next run was a real one.
  *Underwater, first try (2026-10-26).* New harness scene `--scene rov --profile
  astra_rov` (`BRIEF_ASTRA_UNDERWATER_SCENE.md`):
  - The BlueROV2 Heavy, level, with the arms on their native mounts and a solid
    vehicle body.
  - A sandy seabed, a water look, and the ROV dome camera as Astra's overview.
  - A thread seat stop, so `screw` detects snug.
  Astra wrote its own `LESSONS.md` from its ep 6 transcript (`ASTRA_LESSONS_PROMPT.md`,
  `runs/astra_flange/lessons_from_006/`). It then ran episode
  `astra_rov_g3_20261026_001` with those lessons:
  - success on the first try in 102 commands (412 s sim, about 20 min wall);
  - no nut drops and no vehicle contact;
  - nut seated after 10.28 turns, 0.08 mm face gap.
  Its session shows only `cat README.md LESSONS.md`, `./robot` and 61 own-camera
  images. Cut: `astra_rov_g3_20261026_001/astra_rov_social.mp4` (1:43, 4x).
  Same caveats as above (sim, torque limit lifted, ideal thread, bolt
  pre-inserted, `screw` skill). The ROV is fixed, with no hydrodynamics.
  *Two-handed, in progress (2026-10-26, overnight).* Gavin: "have the left arm
  also grab the bolt", as a technician uses a backup wrench. Sol is building
  `--profile astra_rov_backup` (`BRIEF_ASTRA_BACKUP_HOLD.md`):
  - The bolt turns in its holes unless the left gripper holds the head.
  - The thread works on nut-relative-to-bolt rotation.
  - The video adds the left wrist camera.
  The first Sol run hit a model-capacity error mid-way (partial probes: head held,
  nut 5.66 turns with the bolt still; unheld, the bolt co-rotated). It was
  relaunched with a resume note.
  **Resume here:** on gavin-linux, `astra_op/overnight_backup.sh` (a copy is in
  `scripts/viscous-dream1/overnight_astra_backup.sh`) runs unattended:
  - It waits for Sol, checks for its report, the profile and the held-validation
    video, then runs Astra episode `astra_rov_backup_g3_20261026_001` with
    `ASTRA_OPERATOR_PROMPT_BACKUP.md` and `LESSONS.md`.
  - It then exports the video, the timeline and the audit.
  - Progress log: `runs/astra_flange/overnight_20261026.log`.
  Next: review the log, Sol's report and the held-validation video. Audit the
  Astra session (`audit.txt`), make the social cut (`social_edit_rov.py`
  pattern, four panes incl. left wrist), commit, and show Gavin.
  *Two-handed results (2026-10-26).* All audits are clean (README, LESSONS,
  `./robot` and own-camera images only).
  - Backup ep 1: seated, but the nut left the bolt sideways 8 turns deep. That was
    a harness bug, fixed by a captive thread (`BRIEF_ASTRA_CAPTIVE_NUT.md`).
  - Ep 2: one try, no drops, the bolt held throughout (0.0003 turns), nut 9.70
    turns, stopped 2.0 mm short of the flange. The right upper arm pressed into
    the ROV base. Cut: `astra_rov_backup_g3_20261026_002/astra_rov_twohand_social.mp4`
    (1:43, 6x, 3x2 grid including the left wrist).
  - Ep 3: 88 commands, 9.65 turns, 2.2 mm short.
  Astra's final claims said "seated" both times; the sim says 2 mm short.
  Next: `BRIEF_ASTRA_ROV_ROOM.md` re-parks the ROV (`astra_rov_backup2`), with
  Astra episode `astra_rov_backup2_g3_20261026_001` chained after it on gavin-linux
  (log `runs/astra_flange/overnight_astra_rov_backup2_g3_20261026_001.log`).
  *Two-handed, seated (2026-10-26).* `astra_rov_backup2` moves the whole level ROV
  20 mm toward ROV-left. Sol found ep 3's stop was the right wrist housing hitting
  the pipe, and ep 2's was upper-arm contact with the chassis. Astra episode
  `astra_rov_backup2_g3_20261026_001`:
  - One try in 103 commands (586 s sim, about 37 min wall), one thread engagement,
    no drops.
  - The left hand held the bolt head throughout (0.002 turns during tightening).
  - Nut seated: 10.25 turns, 0.13 mm gap. Astra claimed "more than eleven turns".
  - Audit clean: 58 own-camera images.
  Cut: `astra_rov_backup2_g3_20261026_001/astra_rov_twohand_social.mp4` (1:38, 6x).
  *Logo badges on the video arms (2026-10-26, Gavin).* The real arms carry the
  printed badges from `~/Documents/viscous-arm-branding/` on gavin-linux (blue
  VISCOUS ROBOTICS nameplate and V badge on each side of the forearm). The video
  arms did not. Gavin: "update the CAD assets ... don't render out the video yet":
  he will set the Blender camera views by hand on gavin-linux before rendering.
  Added the Oct 6 handed nameplates and the Oct 9 **extended** V (the latest
  design) to both arms of the propeller film (`cinematic/assets/native_rov.blend`,
  `cinematic/results/final_scene.blend`, rebuilt by `build_scene.py` via
  `add_logo_badges.py`) and, as visual-only geoms, to the MuJoCo `astra_rov*`
  scenes behind the flange video (`astra_op/logo_badges.py`). Placement is
  registered on the forearm bracket mesh (~0.01 mm RMS); the right arm's mesh is
  mirrored, so each side gets its own handed part and the text reads correctly
  everywhere. Cameras, lights, motion and render settings are unchanged (all
  pre-existing objects identical over 672 film frames); sim physics is unchanged
  (2,000-row replay bit-identical). One visual clash: the left arm's inner
  nameplate crosses the ROV at 27.0-29.4 s of `astra_rov_backup2_g3_20261026_001`.
  No videos re-rendered yet. Evidence: `cinematic/results/logo_check/` and
  `runs/astra_flange/logo_check/`.
  *Saving Astra's learning across sessions (Gavin, 2026-10-27).* Gavin noticed
  ep 6 started "completely stupid" and later controlled the arm well. He asked
  whether that is in-context learning and whether it can be saved "so that we
  don't have to redo in-context learning from scratch every single time" in a
  new session. Answer: yes, it is in-context learning. Ep 6 was one session with
  six approaches, and each was planned with the earlier failures in view
  (episodes 1-5 failed mostly on harness bugs, so the cross-episode gain was not
  all Astra). It was already saved once, in `LESSONS.md`, which ep 6's Astra
  wrote from its own transcript. Every later episode read that file (ROV ep 1:
  first try, 102 commands vs 743). There are two gaps. The file never grows: all
  five later episodes used ep 6's version. And there is no controlled
  comparison. Gavin said "sure" to both fixes, briefed to Sol in
  `scripts/viscous-dream1/BRIEF_ASTRA_LESSONS_AB.md` (tooling in
  `astra_learn/` on gavin-linux):
  - **Running lessons:** a versioned store (`lessons/v001` = ep 6's file). After
    each episode Astra revises the lessons from its transcript plus the sim's
    measured outcome, which corrects its overclaims ("seated" when 2 mm short).
    The catch-up on the five later episodes gives v002..v006.
  - **A/B on `astra_rov_backup2`:** three runs with lessons v001 against three
    without, alternated, with a 4 h wall cap each. v001 stays fixed so the test
    isolates "notes from the 23-minute session in the video". Codex's own
    cross-session memory store on gavin-linux is empty and is asserted empty
    before each run, so the lessons file is the only carry-over.
  This feeds B4 ([shackle-sim2real.md](shackle-sim2real.md)), which asks whether
  the lessons carry to the real arm.
  *Tooling and catch-up done (2026-10-27).* Sol's report and code are in
  `scripts/viscous-dream1/astra_learn/` (the lessons store is under `lessons/`;
  `report.md`). Scored from the sim log, all six past episodes reached two or
  more turns and four seated. Backup ep 1, although seated, took 552 commands
  and 2.4 h. The drop detector finds ep 6's three drops and none elsewhere.
  Three final claims disagree with the sim: two "seated" calls at about 2 mm
  short, and "more than eleven turns" against 10.25. v002..v006 each passed the
  length and no-coordinates checks. A caution on v006: revising made it more
  hedged and general. It dropped v001's concrete recipe (start with
  `turn right 20 --feed-mm 0.194`, then `screw right 1080`) in favour of "not a
  fixed recipe". If the A/B shows v001 helps, test v006 against v001 before
  adopting the running file. *A/B launched 2026-10-27 23:39 CDT* on gavin-linux
  (`astra_learn/ab.sh`; progress in `runs/astra_flange/ab/ab.log`, results in
  `runs/astra_flange/ab/summary.md`).
  *A/B result (2026-10-28).* Same scene and start for all six runs; all
  audits are clean. With notes (v001):
  - L1: 6.6 turns, 9.8 mm short, 19 min, 157 commands. Claimed "at the
    flange", and the scorer missed that overclaim.
  - L2: seated, 40 min, 180 commands.
  - L3: seated, 27 min, 127 commands.
  - No drops.

  Without notes:
  - N1: failed, 0 turns at the 4 h cap, 846 commands, 3 drops.
  - N2: seated, 2.5 h, 429 commands, 1 drop.
  - N3: seated, 21 min, 94 commands, 0 drops (the best run of the six).

  Reading: the notes did not make the best case faster (N3 beat every L run).
  They removed the bad tail: no failure, no drops, every run under 40 min,
  against a no-notes spread from 21 min to failure. Three runs per arm is
  enough to see that pattern, but not enough to quote rates. Copied here:
  `scripts/viscous-dream1/runs/astra_flange/ab/` (summary, per-run
  `outcome.json` and audit). The videos stay on gavin-linux. Gavin's follow-up
  question, still open: how should notes be written (short or detailed), does
  Astra get confused as notes pile up, and can it pick the relevant ones from a
  large library of "robot skills" (turn a nut, insert a bolt, grab a cube)?
  The proposed tests are short vs detailed notes, the one useful note among
  20-30 unrelated ones, and a skill library against one page, on the shackle
  pin (B4) as the second task.
- *B3b. Next, after B3a (Gavin, 2026-10-23, "in case I forget").* Move the sim off
  the tabletop and onto the BlueROV2 ("the blue RV"): the two Viscous arms mounted
  on the BlueROV2 Heavy as in `~/Documents/viscous-rov-model` (URDF
  `model/viscous_rov.urdf`, two arms, eight thrusters), and make it "look like it's a
  real" ROV, i.e. the real vehicle's look rather than a bare table rig. Questions to
  settle with Gavin before building: what the arms work on (the propeller from B3a?),
  underwater look or dry, and where the "top" camera goes, since Dream-1 expects a
  station-like overhead view and an ROV has a forward camera.

### P. How the work is done (process intent)

- **Nothing lost.** Commit and push across both machines. The repo must be
  self-contained: "If I have a brand new machine, I can just get everything from
  GitHub and then it's all there."
- **Record intent, not just objectives.** "Record down the intention of the
  objective ... so that when we go back in history ... we can see the thought
  process." This document is that record.
- **Hardware runs:** say plainly which joints move. Prove the stop works before
  energizing. Gavin keeps the power switch as the hard stop. Gavin accepts
  motion that is surprising but not dangerous: "it actually was not doing
  anything dangerous, just surprised me." He set the thermal ceiling to 85 °C,
  the driver's own stop, so scripts abort just below it.
- **Demo reliability comes first (2026-10-27).** Gavin travels to show the arm:
  "the most important thing is when I deploy it, it works ... Last thing I want
  is to go there, press teleop and it doesn't work for whatever stupid reason"
  and have to start an agent debugging in front of a guest. A fix counts only
  if it holds on the demo machine at the visit. Background and the three
  reviews: `docs/viscous-mapping/workflow-review-*.md` and
  `fable-workflow-review-2026-10-27.md`.

## Objective (haptics detail)

ROV pilots teleoperating the Viscous arm with the purple Star Arm 102-HD leader
should **feel** what the follower touches (contact, collisions, grip) through
the leader, the way industry pilots ask for "force feedback / haptic feel".

What they should *not* feel: the follower holding up its own weight, or water
drag that carries no information. That distinction drives most of the work below.

Priority set on 2026-10-19: teleoperation and haptics first. DAgger handover
(the leader following a policy-driven follower) is deferred; it needs the same
leader drive, so that part is shared groundwork.

## Approach and why

**Bilateral control family.** Options considered (2026-10-19 literature check):

| Scheme | Needs | Feel |
|---|---|---|
| Position–position (virtual spring between arms) | position-controlled actuators | stable, but heavy/sticky: you feel the follower's inertia, friction, sag |
| Force reflection | torque-controllable leader + force estimate | good |
| 4-channel (Bi-ACT, ALPHA-α, Yamane et al.) | torque motors at ~1 kHz + dynamics model | best |
| **Input-gated bilateral** (Kanai et al., arXiv 2509.08226) | position servos only | light in free motion, firm on contact |

Chosen: **input-gated bilateral**. The Star's FashionStar servos accept position
commands (`set_angle`) and stop modes (0x10 free, 0x11 hold, 0x12 damping) but
no torque mode, which rules out force reflection and 4-channel on this leader.
Input-gating drives the leader toward the follower's position but caps the
leader's effort by the follower's *contact* effort, which fixes position–position's
heaviness. It was shown on Dynamixel position servos at 100 Hz with default gains.
(A gravity-compensated, torque-capable leader such as the Metal leader would
allow full force reflection later.)

Correction recorded: `arms/can_common.py` says the Star leader has "encoders and
no motors". That is wrong for the 102-HD: its lock button holds the pose with
the servos, and `set_angle` positions them.

On unit 1's Star (gavin-linux) the lock button is dead (Gavin, 2026-10-20: "the
button is long dead"): id 7 does not answer a ping, so leave
`button.enabled` at its default `false`. Servos 0-6 answer at 1 Mbaud. Holding
or driving the leader has to be commanded over the bus (stop modes,
`set_angle`), never through the button.

**Why a gravity model first.** The follower's effort signal is RS00 motor
torque, which contains gravity, friction, drag and contact. The pilot should feel
only contact:

    contact torque = measured motor torque − predicted gravity torque(q)

So gravity compensation here is *not* what it was for the Metal arm (making a
heavy leader feel light; Ethan's Pinocchio model, Isaac's gravity-compensated
Metal leader). Our Star leader is already light. For us the model (1) cleans
the haptic signal and (2) as feedforward, removes the follower's sag so position
error means contact.

**Pinocchio, not hand-rolled numpy.** Proven URDF parsing (fixed and mimic
joints, frame conventions), the joint-torque regressor needed for
identification, and Coriolis/inertia terms for later. Its cost is no Windows
wheels, so it is the optional `arm-dynamics` extra, imported lazily.

**Measure, don't trust CAD.** The CAD URDF's masses are estimates without
cables, screws or potting, and the Metal arm's model ended up with hand-copied
1.1–1.2× factors. Gravity torque is linear in each body's mass and first moment,
so a least-squares fit to torques measured at static poses, pulled toward CAD
only where the data can't decide, gives numbers measured on this arm. Each pose
is approached from above and below so friction cancels. Air first (unit 1, dry
bench) to prove the method; then the aquarium, where the same fit absorbs
buoyancy.

## Results (2026-10-20, unit 1 `viscous_01`, dry bench)

- **RS00 torque telemetry is trustworthy**: it agrees with stiffness × error at
  slope 0.98–1.00, r = 1.00. (The Metal arm's Damiao telemetry was not usable.)
- **CAD is already close**: measured/CAD 1.05 shoulder, 0.98 elbow, 0.95 wrist.
  With 27 holds the fit beats CAD on held-out poses (shoulder 0.33 → 0.23 Nm
  RMS); elbow and wrist are about equal to CAD.
- **Static friction**: ~0.11 Nm shoulder, 0.04 elbow, 0.07 wrist. This and the
  ~0.2 Nm model error set the floor for contact detection.
- **Feedforward works**: at 50% in GUI teleop, the elbow's stiffness torque at
  the fold fell to under half its gravity load, matching the model. Sign is right.
- **The dry shoulder is overloaded**: 33 → 65 °C in under 2 min at 5–6 Nm; and
  moving between two heavy poses (~5.6 Nm predicted, after it had held 6.5 Nm)
  it lost its hold and the arm dropped until Gavin caught it. No fault logged;
  probably thermal derating or a protective cutout. The RS00 is rated ~5 Nm
  continuous and **14 Nm peak** (Gavin, 2026-10-20: "max torque is 14nm"), so
  the drop was not the motor's torque limit; the dry-arm 3.5 Nm planning cap is
  a continuous-load and heat caution, not a ceiling the motor imposes. The arm
  is all-RS00 because it is designed to be near-neutral in water.

## Safety lessons (apply to every powered run)

- State every joint that will move before energizing (J5/J6 moving surprised
  Gavin when only "the vertical plane" was described).
- Prove the stop works before energizing. A process started with `nohup … &`
  ignores SIGINT unless it installs its own handler; the first "stop" did nothing.
  Check liveness with `kill -0 <pid>`, never `pgrep -f` inside the same ssh command.
- Dry arm: cap planned load ≈3.5 Nm, trip at ~10° lag (25° was far too late),
  recover toward the low-load upright pose, cool before heavy poses, log per-tick
  torque and motor status.
- Releasing torque at the fold tips the dry arm forward ~25° onto its gripper
  (every stop, including Lab teleop's). Not yet fixed. Cause (2026-10-20, from
  the fit): the elbow is fine, gravity pushes the forearm INTO its fold stop
  (~4.1 N m the stop carries). `shoulder_lift` sits just past balance at the
  fold and gravity pulls it AWAY from its stop (0.2 N m, 0.6 N m by -25 deg),
  so the arm rolls forward until the gripper lands. Gravity never holds the
  shoulder in the fold; the fix is to lower the arm under control onto where it
  ends up (gripper on the bench) and release there.

## How to use what exists

- Gravity feedforward in teleop (Linux service drop-in or environment):
  `MAKERMODSLAB_GRAVITY_FF=configs/viscous/viscous_01/gravity-air-fit.json`,
  optional `MAKERMODSLAB_GRAVITY_FF_SCALE=0.5`. Off when unset.
- Identify another arm/condition: `scripts/viscous-gravity-collect.py --robot NAME
  [--condition water] --dry-run`, then without `--dry-run`, then
  `scripts/viscous-gravity-fit.py <runs…>`.
- Code: `makermodslab/arm_gravity.py` (model, fit, planner, collision checker),
  `makermodslab/gravity_feedforward.py`, `makermodslab/star_follow.py` (draft
  leader drive), model in `configs/viscous/model/`, data in `configs/viscous/viscous_01/`.

## Next steps

1. ~~Contact-torque signal live~~ and ~~feedforward A/B~~: done 2026-10-20
   (see "Contact-signal bench check"). Friction term + refit done offline
   (`gravity-air-fit-v2.json`) and confirmed live the same day; v2 is the
   contact check's default. Remaining floor: shoulder model error at load.
2. Feedforward at 100 % in GUI teleop on unit 1 (the drop-in is still at 50 %).
3. Characterize the Star as an actuator: power-capped moves (does the protocol's
   power field work through `set_angle`?), damping levels, latency, feel.
   2026-10-21: power cap and latency measured on the gripper (raw 0x08, not
   `set_angle`); hold stiffness and damping feel still need Gavin's hand.
4. Input-gated leader drive built on `star_follow.py`.
5. 1:1 posture mapping for unit 1 (draft profile in viscous-arm-simulation
   `leader/mapping/profiles/viscous_01.json`).
6. Water fit in the aquarium (waterproof unit).
7. Stop path: avoid the tip-forward on release at the fold.

## Dream-1 trial (2026-10-20, MuJoCo only, goal B3)

**Where it ran:** gavin-linux, scratch venv `~/dreamtrial/.venv` (Python 3.12,
`dreamlabs==1.0.1`, `dreamlabs_client==1.0.3`), API key in
`~/.dreamlabs/credentials` (0600, `DREAM_API_KEY=`; never commit it). The key was
pasted into a chat during the trial: revoke and replace it. The trial's code,
briefs, reports and the two live runs' `log.json` + `run.mp4` are committed in
[`scripts/viscous-dream1/`](../scripts/viscous-dream1/). The script reads the
scene from `~/Documents/viscous-arm-model/demo/pick_place_scene.xml` and
`viscous_arm.json` points at that repo's URDF by a relative path; run it from
`~/dreamtrial/viscous_sim` or fix those paths. Modes: `--preview` (no client),
default dry run (client path with a local stub, no network), `--live` (billed,
capped by `--max-calls`); `--rig bench|yam`, `--start neutral|work`, `--model beta|flash`.

**How Dream-1 handles robots (read from the installed client, not their website):**
- The client's `robot` setting takes only `"yam"` or `"other_embodiment"`
  (`dream/client.py:68-72`). The checkpoints the key may run are only
  `dreamlabs/dream-1-beta` and `-beta-flash`, both `embodiment: yam`, cameras
  `overhead`, `wrist_l`, `wrist_r`.
- On `"other_embodiment"` the model gets zeros in its joint slots. It reads the
  cameras, the prompt and each arm's hand pose (`state.<side>_eef_xyz`, `_eef_rot6`,
  grip), and answers hand poses in 48-row chunks at 20 Hz. The client's own IK
  (`dream_ik`) solves them onto the arm.
- "Maker support" is client-side: `ik: {"arm": "makers"}` selects
  `dream_ik/specs.py` `MAKERS_ARM` (`maker_arm_ik.urdf`, tool point 0.1696 m along
  `link_007` -z relabelled into YAM axes, rest pose from 94 Maker episodes, camera
  mount "not measured", no joint mapping to YAM). `"metal"` (`METAL_ARM`) is the
  same plus a YAM joint-sign map and "our Metal policy" tuning. With the public
  YAM checkpoints, Maker and Metal are `other_embodiment` exactly like Viscous, so
  disguising Viscous as a Maker changes nothing the model sees. It only matters
  with a Maker- or Metal-fine-tuned checkpoint ("Metal and Makers checkpoints read
  the URDF's joints"), which this key cannot run. Ask Ryan/DreamLabs for one.
- Today's runs used `ik.arm` = `viscous_arm.json` (tool point `tool0`, the
  finger-pad midpoint, never confirmed with `dreamcli tcp`), head `eef`.

**Results (both on beta, 79 billed calls, about 10¢):**
1. `runs/live_beta_1`, bench rig (overhead camera beside the arm, ~0.28 m up;
   `wrist_r` black): 40 calls. Cube never moved; the arm swung away then rose to
   z 0.57 m; up to 13 of 30 executed rows per chunk clipped by the 2 rad/s cap.
2. `runs/live_yamrig_1`, Viscous placed where YAM's left arm sits in Dream's own
   station, YAM's calibrated overhead camera (`yamrig_report.md`): 39 calls. The
   gripper opened and hovered 13-23 cm above the table; cube never moved.

**Why it might be failing, to check in this order (2026-10-21):**
1. *Baseline:* run Dream's own `dream_examples/yam_mujoco` (installed in the
   venv) on a similar pick-and-place. If YAM also flails, the fault is in how we
   call the API, not the Viscous arm.
2. *Start state out of distribution:* both live runs used `--start neutral`
   (all joints 0, gripper closed); YAM resets with its gripper open, and
   `--start work` (the arm file's rest, gripper open) was never run live.
   Compare the hand pose we send at step 0 with what `yam_mujoco` sends at its reset.
3. *Hand-pose frame:* base-local EEF with no world transform, and the
   tool-point axes (`approach`, `jaw` in `viscous_arm.json`) never checked
   against YAM's convention (z = approach, x = jaw travel). A wrong frame makes the
   model see a nonsense hand pose and its answers solve to nonsense.
4. *Images:* Viscous meshes look nothing like YAM; the cube is ~5 px wide in the
   320x176 overhead; wrist camera mount differs from YAM's.
5. *Execution:* 30 of 48 rows run per chunk, speed clipping, IK fallbacks.
6. Only after 1-5: zero-shot on an unseen arm may simply not work. Then the
   options are a Maker/Metal fine-tuned checkpoint, or ~50 Viscous demos and
   Dream's fine-tune service (LeRobot v2.1 only; we record v3.0, so convert).

**Findings (2026-10-20, second session).** Gavin chose to work B3 in its own
session, in parallel with a haptics session.
- *The model is blind to our arm's state, by design.* For any arm but YAM the
  client sends `robot: other_embodiment`, and the serve then "withholds the whole
  state, joints, hand pose and grip alike: the model reads none of it, and the hand
  pose sent is only the anchor its answer is composed on" (`dream/client_ik.py`
  docstring; `dream/__init__.py` on `robot`). On the Viscous, Dream-1 acts from
  the three images and the prompt alone. Dream's own `yam_mujoco` example never
  uses this mode: for beta it sends the YAM station keys and the model reads the
  joints. So the earlier comparison with "YAM works in Dream's sim" was not like
  for like.
- *Hypothesis 3 (hand frame) ruled out* with dream's own FK
  (`scripts/viscous-dream1/frame_check.py`): with the hand straight forward,
  our arm file gives approach = +x and jaw = +y (left) in base, the same as YAM
  and Makers and as Dream's convention requires. `--start neutral` puts the hand at
  (0.22, 0, 0.13) m pointing forward 9° down, near YAM's rest (0.26, 0, 0.21) m.
- *Hypothesis 1 done properly, and the vision-only mode is NOT the problem.*
  `scripts/viscous-dream1/yam_baseline.py` runs Dream's YAM scene ("put apple
  into the bowl") in two modes: `station` (the model reads the state) and
  `vision` (`other_embodiment`, client IK: the Viscous path). 20 billed calls
  each, beta. Both succeed: the left arm grasps, lifts and drops the apple in the
  bowl (closest 0.008 m station, 0.004 m vision; vision did it by call 6-7, station
  by 8-9). Evidence in `scripts/viscous-dream1/runs/yam_{station,vision}_live1/`.
  So Dream-1 can act from images alone, and something specific to the Viscous
  trial broke it: the arm's appearance, the task/scene (white cube into a black
  square), or our executor (30 of 48 rows, 2 rad/s per-row clip, gripper
  closed at start).
- *The 2x2: both new cells fail, so there are two separate problems.*

  | | apple -> bowl (Dream's scene) | white cube -> black square |
  |---|---|---|
  | YAM | works (`yam_vision_live1`) | fails (`yam_cube_vision_live1`, 20 calls) |
  | Viscous | fails (`viscous_apple_vision_live1`, 10 calls) | fails (earlier runs) |

  (1) *The task:* even YAM never goes for a 2 cm white cube; it wanders around
  the bottle and orange. The cube is a few pixels in the 320x176 top image.
  Dream-1 zero-shot needs objects and prompts it knows. (2) *The arm:* the
  Viscous, in Dream's scene with Dream's prompt and the baseline's executor (48
  rows, no extra speed clip, grippers open), rises near its base and never touches
  the apple. A scripted Viscous pinch does lift the apple 7.8 cm, so the gripper
  can do it. Call 10 answered a grip of -0.089, which the example's check rejects
  (clip it instead next time). Scene builder: `cross_scenes.py`; report
  `runs/2x2_report.md`.
- *Next: why the arm fails.* Either the model is confused by how the Viscous looks
  (it aims wrong), or it aims right and the Viscous cannot reach the hand poses
  it asks for (the IK compromises, pulled toward `rest`). The runs logged where the
  hand went, not the poses the model asked for, so log the reply's hand poses and the
  IK's tracking error, then replay YAM's successful hand path through the Viscous IK
  offline (free).
- *Verdict: the model aims wrong; the Viscous reaches fine* (`runs/ik_reach_report.md`,
  15 billed calls per arm, checked against the raw `chunks.jsonl`). On YAM the
  requested left-hand path goes straight to the apple by call 3-4 (within 7 mm),
  closes, lifts, carries and opens over the bowl. On the Viscous, with the same scene,
  prompt and executor, the requests dither around the start for five calls, then
  drift up and away, never closer than 19 cm. The Viscous IK follows those
  requests to 6.7 mm / 1.1° median, as close as YAM's own (4.9 mm / 0.7°), with no
  joint at a limit when it matters. So with the state withheld, what the model sees
  of the Viscous (its look in the top camera, and/or its wrist camera views) makes
  it act without purpose. Nothing in our kinematics, frames or executor causes it.
- *A second, smaller problem: the tool point.* Replaying YAM's successful hand path
  on the Viscous (no calls), the commanded joints track it to ~5 mm, but the Viscous
  fingers hit the table on the way down. The real hand ends ~8 cm off and only
  nudges the apple. The arm file's `tcp` is the finger-pad midpoint; Dream defines it
  as "the point between the closed fingertips", and says a person sets it on the real
  gripper with `dreamcli tcp`, never an agent. Gavin to do that before any model that
  does aim right (a fine-tune, a Maker checkpoint) is run.
- *What this means for B3 (2026-10-20):* zero-shot Dream-1 will not drive the
  Viscous; disguising it as a Maker arm changes nothing with the public YAM
  checkpoints, because the model reads images, not the arm's identity. The realistic
  routes are (1) a checkpoint that has seen a Maker-like arm (ask Ryan/DreamLabs for
  the Maker fine-tune; Gavin's translation-layer idea fits that checkpoint) or (2)
  ~50 Viscous demos through Dream's fine-tune service (LeRobot v2.1; we record v3.0).
  Either way, use objects and prompts the model knows. Optional cheap diagnostic
  (~30 calls): black out the wrist cameras on both arms to see whether the wrist
  view or the top view of the Viscous causes the confusion. Total billed today: 100
  calls (40 baseline, 30 2x2, 30 reach), about the same per call as 2026-10-20's ~10¢
  for 79.
- *Object size, confirmed (2026-10-20, Gavin's idea).* Gavin guessed Dream-1 was
  not trained on objects as small as the 2 cm cube and put the real Feetech servo
  box (about 5 x 5 x 3 cm, white top, black sides; `real/topdown_box_20261020.jpg`)
  on the bench beside it. The same box in Dream's scene, YAM, vision-only mode,
  "pick up the black and white box and put it on the black square": the left arm
  grasps it at about call 6, lifts it 17.6 cm, carries it and sets it on the square
  by call 13 (centre about 2 cm from the square's centre; the 5 cm box covers the
  4.5 cm square). Later calls nudge it a little. 20 billed calls,
  `runs/yam_box_vision_live1`, `runs/box_report.md`. So on a real bench, use
  objects of about this size or larger for any Dream-1 trial; 120 billed calls today.
- *Maker and Metal in sim also fail (2026-10-20).* Gavin: Dream supports the Maker
  arm ("works 60% of the time") and the Metal, so follow their documented setup in
  sim, Maker first, then Metal. Dream's docs are behind the platform login; the
  client is the documentation we have. This key runs only the two public YAM base
  models, so "Maker support" for us is `dream-1-beta` + `ik: {"arm": "makers"}`
  (`other_embodiment`). Two Maker arms at YAM's bases in Dream's scene, built from
  our `frontend/public/maker-urdf` and checked against `dream_ik`'s own Maker FK to
  0.0003 mm / 0.00003 deg; scripted pinches lift the apple and the box. Live, beta:
  Maker apple 20 calls, Maker box 20, Metal apple 20. No object contact in any run;
  the requested hands never came within ~23 cm of an object (Metal 30 cm). Report
  `runs/maker_metal_report.md`, scripts `supported_arms.py`,
  `supported_arm_check.py`, `analyse_supported_runs.py`. 180 billed calls today.
  Two open questions decide what this means: (1) the wrist cameras. Dream calls the
  Maker mount "not measured", and our improvised one is clearly wrong: it looks at the
  back of the gripper with the table upside-down (`runs/maker_metal_cameras/`), where
  YAM's shows the scene with fingertips in the corners. (2) Whether Dream's "60%"
  is this base model zero-shot or a Maker fine-tune. The Maker spec's rest pose comes
  from 94 recorded Maker-station episodes, and the client speaks of "Metal and Makers
  checkpoints", which suggests fine-tunes this key cannot run.
- *It was the wrist camera: the Maker works once its camera matches the real one
  (2026-10-20).* Gavin pointed at the MakerMods brain: every Maker and Metal box ships
  the KD-USB Sonix UVC camera (`2C7F:4A60`, 1080p, fisheye; undistorted HFOV 109.6°
  / VFOV 77.2°, best focus ~0.2 m; mount never measured). Real Maker wrist frames
  from two Maker datasets in gavin-linux's Hub cache (`real/makerwrist_*.jpg`) show
  the camera above the gripper, behind the jaws, looking forward along the fingers,
  which rise from the bottom centre. Ryan also confirmed (2026-10-19) that Maker data
  trained Dream-1. With the sim camera on the CAD camera assembly on `link_007`, 20°
  down, VFOV 77.2°, 16:9 (`runs/maker_wrist_match/compare.png`), beta + `ik makers`
  zero-shot: Maker apple succeeds (apple in the bowl at call 12 and stays there);
  Maker box grasped, lifted 12.9 cm and carried to 1.8 cm of the square but never
  released. 40 billed calls, `runs/maker_{apple,box}_vision_live2`,
  `runs/maker_wrist_report.md`. So the earlier Maker failure was our improvised
  camera, and wrist-camera placement is decisive. The real Viscous wrist camera is
  mounted the other way round (jaws enter from the TOP of the image), which may
  explain much of the Viscous failure. Next: the Viscous with a Maker-style wrist
  camera in sim (`BRIEF_VISCOUS_MAKERWRIST.md`). 220 billed calls today.
- *The Viscous works too, with a Maker-style wrist camera (2026-10-20).* Same rig as
  `viscous_apple_vision_live*`, only the wrist cameras moved to the Maker-relative
  pose (above the gripper behind the jaws, ~20° down, VFOV 77.2°, 16:9;
  `runs/viscous_wrist_match/compare.png`; the old mount is `--viscous-wrist legacy`).
  Zero-shot, beta, `viscous_arm.json` unchanged: **apple success**. The left arm
  reaches it by calls 5-7, lifts it 11.6 cm at call 9 and drops it in the bowl at
  call 10, where it rests (9 mm from centre) to call 20. The box was targeted (requested hand
  within 2 cm) but dragged and tipped, not placed. The Viscous fingers touched the table on
  418 (apple) and 292 (box) rows: the tool-point problem above. 40 billed calls,
  `runs/viscous_{apple,box}_makerwrist_live1`, `runs/viscous_makerwrist_report.md`.
  **So the main cause of the Viscous failure was its wrist camera view.** For the
  real arm: (1) mount the Viscous wrist camera like the Maker's, ideally the same
  KD-USB camera, looking forward along the fingers from above, fingers rising from the
  bottom of the image; (2) Gavin sets the tool point with `dreamcli tcp`; (3) a centre
  top camera placed like Dream's station's; (4) objects of about 5 cm or more; (5)
  a Lab runner for Dream-1 on the real follower with the safety limits. 260 billed
  calls today.
- *Tuning sweep without fine-tuning, success rates (2026-10-20).* Gavin asked whether his
  arm can do better in sim without fine-tuning, or else "play gacha". `sweep.py` runs
  episodes (object start jittered +-2 cm, early stop on success, a crash-safe 900-call
  ledger). 44 episodes, 872 billed calls (`runs/viscous_sweep_report.md`):

  | Setup (Viscous, Maker-style wrist) | Apple into bowl | 5 cm box onto square |
  |---|---|---|
  | B0, as in `viscous_apple_makerwrist_live1` | **6/6** (median 13.5 calls) | 0/6 (2 grasps) |
  | tool point at the fingertips (`viscous_arm_fingertip.json`, +3.4 cm) | 4/6 | 1/6 |
  | B0, one arm (right arm removed, `wrist_r` black) | 2/6 | 0/6 |
  | B0, 24 of 48 rows per chunk | not run | 0/4 |
  | B0, `ik` region over the table | not run | 0/4 (table contact 34% -> 2%, no lifts) |

  So: the apple is reliable on the two-arm sim; the box is not solved by any knob
  tried; the fingertip tool point removes the apple's table contact but did not raise
  the success rate here. **One arm hurts badly (apple 6/6 -> 2/6)**, and the real bench
  has one arm, so that is the next thing to solve before the real world. Ideas: a
  second (parked, maybe static) arm or a mirrored image in the right slot, the
  object on the working arm's side, or the right wrist slot fed the left view.
  1132 billed calls today (about $1.45).
- *One arm, what restores it (2026-10-20/21).* Apple task, same seeds as the sweep, 6
  episodes each (`BRIEF_ONEARM.md`, `BRIEF_ONEARM_ISOLATE.md`,
  `runs/viscous_onearm_report.md`, `runs/viscous_onearm_isolate_report.md`):

  | One working arm, plus... | Apple into bowl |
  |---|---|
  | nothing (`single`, right wrist image black) | 2/6 |
  | left wrist image copied into the right slot (`single_copy`) | 2/6 |
  | a fixed camera where the parked right wrist camera sits (`single_fixedcam`) | 1/6 |
  | a second arm held still at rest, its wrist image black (`parked_blind`) | 4/6 |
  | a second arm held still at rest, with its wrist camera (`parked`) | **6/6**, median 9 calls |

  The parked arm never touched anything (contact replay), so it helps only through what
  the cameras see. Both parts count: a second arm in the top view (2 -> 4) and its wrist
  view (4 -> 6). The fixed camera matches the wrist camera's pose but not its picture:
  it lacks the parked gripper's fingers and the arm's shadow
  (`runs/sweep/isolate_validation/parked_fixedcam_wrist_r.png`). *Guess, not measured:*
  Dream-1 expects every wrist image to show its own fingers, which is why the bare
  fixed view did not help. Six episodes each, so 4/6 against 2/6 is suggestive only.
  For the real bench: a second, unpowered arm parked at rest with its own wrist camera
  is the measured fix; a dummy (camera plus a gripper in its view) is untested. 1206
  billed calls so far, 106 left under the ledger cap.

- *Propeller scrubbing, zero-shot (2026-10-23, goal B3a).* Scene: the Dream table
  with the clutter removed, a fixed 20.8 cm bronze three-blade propeller on an 18.5 cm
  stand (shaft horizontal, blades facing the arms), a free 9 x 6 x 4 cm sponge. Viscous
  `B0` (two arms, Maker-style wrists), 30 calls per episode, no success stop. 420
  billed calls (ledger cap raised 1312 -> 1912 for this; 1626 used).

  | Arm | Prompt | Ep | Sponge grasped | Sponge-propeller contact rows | Scrub reversals |
  |---|---|---|---|---|---|
  | Viscous | "pick up the sponge and scrub the propeller" | 2 | 1 | 0 | 0 |
  | Viscous | "clean the propeller with the sponge" | 2 | 1 | 79 | 0 |
  | Viscous | "wipe the propeller" | 5 | 3 | 550 | 0 |
  | Viscous | "scrub the propeller blade back and forth with the sponge" | 2 | 0 | 0 | 0 |
  | YAM | "wipe the propeller" | 3 | 1 | 46 | 2 (brief) |

  So zero-shot Dream-1 gets half the idea: it picks up the sponge, carries it to the
  propeller and presses it on the lower blade, then withdraws (checked by eye in
  `runs/sweep/B0/propeller/P3/ep_00/run.mp4`). It never strokes back and forth. YAM
  does no better (two momentary reversals, one with the sponge dropped), so this is
  the task, not our arm: scrubbing is not in Dream-1's repertoire, and the more
  explicit prompts ("scrub ... back and forth") did worse than the vague "wipe". The
  sim has no dirt, so "clean" cannot be judged. A real scrub needs a fine-tune on
  scrubbing demos or a scripted stroke once the sponge is on the blade. Report
  `runs/propeller_report.md`, code `propeller_*.py`, mesh `assets/propeller/`; only
  `log.json` and three videos are committed (chunks stay on gavin-linux).
- *Grace's URDF claim and the box: it is shoulder torque, not the URDF (2026-11-07).*
  Grace (World Intelligence / DreamLabs) said the URDF was why Dream-1 fails. Checked
  three ways, no URDF fault found: (1) against Dream's `maker_arm_ik.urdf`, frames,
  q=0 pose and joint senses match except wrist_yaw (opposite to Dream's Maker
  convention; irrelevant in sim, where IK and MuJoCo share the URDF and joint slots
  are zero); (2) IK forensics on the logged runs: tracking matches Maker and YAM
  (~5 mm median), observation equals sim FK, grip mapping identical to Maker's;
  (3) Grace ran our `viscous-sim-kit` on her Mac (dreamlabs 1.0.37 / client 1.1.3)
  and B0 apple succeeded at call 15; our rerun on gavin-linux (1.0.1 / 1.0.3, same
  seed) succeeded at call 10 (`runs/apple_confirm_1107/`). Grace's agent then found
  the box's real limit: the MJCF caps every arm actuator at 5 Nm, while holding the
  hand over the square needs 5.7-7 Nm on shoulder_lift, so the hand sags 14-30 mm
  (`box_torque_hold.py`) against a ~2 cm placement tolerance. The RS00 is ~5 Nm
  continuous / 14 Nm peak (see Results), so 5 Nm is the honest continuous number.
  Sweep options added: `t14` (arm actuators at the 14 Nm peak) and `buoy` (5 Nm with
  `gravcomp=0.9` on every arm body, a stand-in for near-neutral in water; 0.9 is an
  assumption, not measured; the gravity feedforward subtracts `qfrc_gravcomp`, which
  is zero otherwise, so B0 is unchanged). Every box episode now logs left-arm torque.
  12 box episodes each, seeds 20271020+i, beta, 762 billed calls on a fresh ledger in
  `~/dreamtrial/confirm_1107` (main sweep ledger untouched); logs `runs/box_torque/`:

  | Box, 12 episodes | Placed | Grasped | Shoulder at limit (median) | Shoulder peak | Finger-table rows (median) |
  |---|---|---|---|---|---|
  | B0, 5 Nm | 2/12 | 9/12 | 42% of sim time | 5.0 Nm | 180 |
  | t14, 14 Nm | **7/12** | 12/12 | 0% (above 5 Nm 26%) | 7.2 median, 8.3 max | 0 |
  | buoy, 5 Nm + 0.9 gravcomp | 4/12 | 12/12 | 0% | 4.0 median | 0 |

  Fisher two-sided: B0 vs t14 p=0.089 alone; pooled with Grace's 6-episode runs (5 Nm
  1/6, 25 Nm 4/6), 3/18 vs 11/18, p=0.015. So the 5 Nm cap is a real cause of the box
  failures. buoy grasps every time but 6 of its 8 failures end within 2.25 cm of the
  square; the last calls show the box still ~1 cm above flat rest, likely not released
  (the Maker did the same in October). Unconfirmed: needs a frame-by-frame look.
  Grace's YAM control on the same box got 0/6 (her env, not our `yam_baseline.py`).
  *Meaning for the real arm:* t14 needs 7-8 Nm on the shoulder for about a quarter of
  the episode, which the dry RS00 can give only briefly (it heats from 33 to 65 C in
  under 2 min at 5-6 Nm). Dry, keep work closer to the base or lower; wet, buoyancy
  removes most of the load. Next: measure the waterproof arm's net weight in water to
  replace the 0.9 guess, and check buoy's unreleased boxes.

## Contact-signal bench check (2026-10-20, goals A1 and A1b, next steps 1-2)

*Intent:* prove the haptic signal on the arm before building the leader loop
(A3) on it: contact torque near zero in free motion, a push standing clear of
that floor, and the feedforward removing sag. "If we hit something, yes, we
should feel that."

- Code: `makermodslab/contact_torque.py` (`ContactEstimator`: reported torque
  minus fitted gravity minus the fit's telemetry bias; the feedforward never
  commands that bias). `gravity_feedforward.install` reads `status["scale"]`
  every frame so one session can A/B it. `scripts/viscous-contact-check.py`
  runs it; `scripts/viscous_bench.py` holds the helpers it shares with the
  gravity collector.
- Run: three poses x feedforward 0/100 % x approach above/below, 50 Hz log of
  every move, then a 60 s push hold at `forward`. `--analyze run.json` prints
  sag, telemetry-vs-(kp*err + ff) slope, the free-motion floor (p99) and how
  much of the push window clears it.
- Pose choice: the planner caps fitted load at the 0 % sag of both approaches,
  not just the nominal pose. With the 3.5 N m dry cap that limits the shoulder
  to ~1.5 N m, so the first-draft poses (up to 2.8 N m) were refused and
  replaced. The legs out of and back into the fold are exempt from the cap
  (collision-checked only): leaving the fold carries ~4 N m on any route, as
  every Lab teleop unfold does.
- First powered run (`evidence/contact-check-20261020-180543.json`) aborted
  1.3 s into the unfold on a 10 deg elbow lag with nothing wrong: at 0 %
  feedforward the elbow carries ~4 N m leaving the fold (~8 deg sag at kp 30)
  and the driver's startup sync stalled lifting the forearm off its stop. Fixed
  by tripping on lag beyond the expected sag and ramping feedforward on at the
  fold. Even that run showed the signal working: with the forearm resting on its
  stop, elbow contact read about -3.6 N m, the stop carrying the load.

**Results, unit 1 dry, `evidence/contact-check-20261020-181208.json`:**

- *Feedforward:* sag at 0 % 3.5-5.5 deg (the model predicts it within ~10 %;
  it under-predicts the shoulder at the two heavy poses), at 100 % at most
  0.21 deg on every gravity joint. Gravity feedforward works at full scale.
- *Telemetry includes the feedforward:* reported torque = kp x error + tau_ff
  at slope 0.99-1.00, r = 1.00, at both scales. So contact = reported - gravity
  is valid with feedforward on, which is how teleop will run.
- *Static holds:* contact within +/-0.31 N m at all 12 holds (friction band
  0.01-0.13 deg, wrist up to 0.5 deg).
- *Free motion (15 deg/s legs, 2917 ticks):* p99 1.17 shoulder, 0.79 elbow,
  0.61 wrist; rms 0.27-0.36 N m. Not the fold: excluding it changes nothing.
  Coulomb friction ~0.2 N m shows as the v>0 / v<0 split (+0.16 / -0.24);
  subtracting a fitted friction term brings elbow and wrist p99 to 0.54 / 0.39.
  The shoulder's remainder (0.74) looks like model error at its larger loads.
- *Push test (at `forward`, feedforward on):* two clean pushes, about 4 s each,
  reading +2.1 / -2.2 N m shoulder and +1.5 / -1.3 N m elbow (3-4 deg of give),
  against a still baseline of 0.06-0.12 N m rms (wrist 0.4). That is about 20x
  the still noise but only ~2x the free-motion floor, so A3 needs the floor
  brought down (friction compensation, a shoulder refit) before it can gate on
  contact during motion. The wrist's still offset stepped -0.4 to +0.2 N m
  across the first push: stiction, the floor for a held pose. Gavin pushed
  only up and down that run, so nothing showed on pan, yaw or roll.
- *Sideways (rerun `evidence/contact-check-20261020-182410.json`, down / up /
  left / right):* down and up repeat run 2 (lift +1.5 / -1.7, elbow +2.0 /
  -1.5 N m). Left and right land on `shoulder_pan`, +1.65 / -2.25 N m against
  a still baseline of about +/-0.03 N m: pan carries no gravity, so its
  contact is raw torque and it is the cleanest channel (50-70x its noise). The
  right push also showed +1.0 N m on `wrist_yaw`. Yaw and roll sit at a steady
  -0.15 to -0.2 N m at rest, a bias the fit's offsets do not remove.
- *Lowering the free-motion floor (offline, 2026-10-20):* `gravity-air-fit-v2.json`
  refits gravity on all 51 holds (the two collection runs plus both complete
  contact checks) and adds a per-joint friction term (`scripts/viscous-contact-fit.py`,
  Coulomb x tanh(v / v_s) + viscous x v on COMMANDED velocity, zero at rest).
  Trained on one complete run and scored on the other, both ways, moving-arm p99
  falls shoulder 0.86-0.97 -> 0.74-0.79, elbow 0.60-0.64 -> 0.42-0.46, wrist_flex
  0.55 -> 0.38-0.41 N m; pushes unchanged; yaw/roll rest bias halves. The
  estimator subtracts friction only when given velocities, and holds each term
  at the fastest speed it was identified at (16 deg/s): never extrapolate, since
  a wrong friction torque reads as contact. Pan never moved and yaw/roll only
  under 2 deg/s, so they carry no friction term. v2 stays opt-in (`--fit`) until
  a live run confirms it; the shoulder's tail (model error at load) is still
  the limiting channel. Details: `configs/viscous/viscous_01/contact-floor-report-20261020.md`.
- *Live confirmation (`evidence/contact-check-20261020-192434.json`, a run
  neither fit saw):* moving-arm p99, old fit -> v2 + friction: shoulder
  0.87 -> 0.77, elbow 0.63 -> 0.45, wrist_flex 0.56 -> 0.41, wrist_yaw
  0.53 -> 0.45, roll 0.48 -> 0.43 N m; still rms falls on every joint. Pushes
  read the same (shoulder +/-2.3, elbow +/-1.8, pan +/-2.2 N m; the right push
  also +1.2 / -1.5 on wrist_yaw). Feedforward holds at <= 0.27 deg with v2. A
  2 N m push now clears the moving floor ~4-5x on elbow and wrist, ~2.5x on the
  shoulder. v2 is now the contact check's default fit. The teleop feedforward
  drop-in on gavin-linux still names the original fit (gravity only; both
  hold sag under 0.3 deg). Run with `--max-torque 3.55`: v2 puts the elbow at
  3.50 N m on the back approach at 0 % (Gavin approved 3.55 for this run).
- Known unrelated test failure: `test_single_device_configs_carry_the_port_and_id[viscous]`
  (missing commissioned `cal-f` profile), present at `HEAD` too.

## Star actuation check (2026-10-21, goal A2, next step 3)

*Intent:* find out whether the Star leader can be driven gently enough to push
back on a pilot's hand, which the input-gated loop (A3) depends on. Gavin moved
the first test from wrist_flex to the gripper: "safest to move is the gripper if
you only want to test something". He then asked for safe tests like this one to
be run without waiting for him: "I told you it's safe already, right, so why are
you still waiting for me?" So both runs below were detached on gavin-linux
(`--yes`, stdin closed) with nobody at the arm, so nothing pushed on it.

Only the gripper (id 6) was energized. Every other servo was released before
and after each phase, and every release readback showed all seven back at the
30 mA rest current. Servo temperature stayed at 29-31 °C.

**Run 1, 1 s ramps** (`evidence/star-actuation-20261021T051835.266732Z.json.gz`,
powers 100/200/400/800 mW):
- *Free motion:* each ±10 deg step tracks its 1 s ramp at every power, ending
  within 0.2-0.4 deg, at the 30 mA rest current. Moving the bare gripper costs
  no measurable effort, so 100 mW is enough for unloaded tracking.
- *The power field works:* driven into the closed stop, the current rose to
  77 / 79 / 92 / 136 mA at 100 / 200 / 400 / 800 mW (30 mA at rest). The cap
  sets the stall effort. 100 and 200 barely differ, so there may be a floor
  near 200 mW; finding where pushback starts needs a hand on the gripper.
- *Closed stop at about -2.2 deg:* every step toward -11.5 deg stopped at
  -2.0 to -2.5 deg, and the gripper rests released at -1.5 deg, i.e. against
  that stop. The derived travel's lower bound (-16.9 deg) cannot be reached on
  unit 1. The calibration (or the frame the script derives travel in) is off on
  the closing side, so check it before driving the gripper from the follower.
- *Hold with nobody pushing:* 0.0 deg deflection at 30 mA. Holding costs
  nothing when unloaded. No stiffness data without a push.

**Run 2, 50 ms steps** (`evidence/star-actuation-20261021T052114.957006Z.json`,
`--move-s 0.05 --phase-s 1 --powers 100,800`):
- *Latency:* motion of at least 0.3 deg shows in the first 50 Hz sample after
  the write (about 20 ms). The script's figure is 42 ms. Both are upper bounds
  that include the serial poll, so a 25-50 Hz leader loop is not limited by the
  servo.
- *Speed:* in a 50 ms leg it covers about 4-5 deg (80-100 deg/s) at both 100
  and 800 mW. The cap limits effort, not unloaded speed.
- Shutting fast into the closed stop overshoots to -3.1 deg at 181 mA, so the
  stop gives a little.

**Still unknown, and why it matters for A3:** how stiff a 100-800 mW hold
feels, and what damping mode 0x12 feels like at each power. Those set the
range the loop can map contact torque onto, and only a hand on the leader can
measure them. A one-minute run with Gavin at the arm covers it:
`.venv/bin/python scripts/star-actuation-check.py --joint gripper --powers 100,200,400,800`
(it waits for Enter between phases). Until then, the next build step (next
step 4) can go ahead: put `star_follow.py` on `fashionstar_raw.py` with an
explicit power on every command and an explicit release for zero effort.

## Gripper haptics proof of concept (2026-10-27, goal A3 on one joint)

*Intent:* Gavin, at the gavin-linux bench: "can we do this quickly and do a
proof of concept with the gripper?" The gripper is the joint already judged
safe to drive on the Star, and a grasp is the contact a pilot most wants to
feel. One joint proves the loop before the arm joints need A1's full model.

`scripts/gripper-haptics-poc.py` energizes exactly two actuators: the follower
gripper (a CAN bus carrying only that motor, opened without the handshake, at
its own MIT gains kp 20 / kd 0.5) and the Star gripper (id 6, through
`fashionstar_raw.py` with an explicit power on every command). The other six
joints on each arm are never commanded, and all seven Star servos are released
at start and on every exit. Loop at 50 Hz:
- The Star drives the follower relative to where both started, through the
  stock preset's scale (-1.778 follower deg per Star deg). The follower target
  may lead its actual position by at most 8 deg, so the squeeze is capped at
  about kp x 8 deg = 2.8 N m.
- Contact = |follower torque - free-motion median| while the jaws move slower
  than 40 deg/s. The gripper carries no gravity load to subtract.
- Gate off: Star released (0x10). Gate on (threshold, off at half of it):
  the Star is commanded toward where the follower's jaws actually are, at
  150 mW at the threshold rising to 800 mW at 1.5 N m above it, clamped to the
  Star travel seen during calibration.

**Run `evidence/gripper-haptics-20261027T050813Z`** (Gavin's hand on the Star,
a firm object in the follower's jaws):
- Free open/close over the full range (Star -2..+59 deg, follower -32..-119):
  the gate never fired, so the Star stayed limp.
- Closed on the object, the follower stalled at -57 deg with contact up to
  2.3 N m (the lead cap). The Star was driven toward +13 deg, the jaw
  position mapped back, at 150-738 mW (median 625) while Gavin held it 5-11 deg
  past that. Each time he eased off, it returned to about +13 deg.
- Ticks: 20.0 ms median, 20.3 ms p99.
- Gavin's feel (2026-10-27): "its very bad. at first, without the cube, it was
  sluggish. then with the cube, you can see it sometimes oscillates, even
  though the cube is static, so its a control loop induced oscillation (just
  like pilot induced oscillations)."
- *Why sluggish:* the 8 deg lead cap applied in free motion too. With kp 20
  and the gripper's friction it held the follower near 100 deg/s, so any Star
  motion faster than ~56 deg/s (scale 1.78) left it behind, ~19 deg at
  47.0-47.4 s. The cap is only needed in contact.
- *Why it oscillated:* a relay limit cycle at ~3.5 Hz (gate on ~160 ms, off
  ~120 ms, 54.2-58.0 s) on top of 0.4 Hz hand-vs-Star swings of 0-13 deg.
  The torque gate is an on/off switch: pushing the Star back to the wall
  removes the follower's squeeze, contact falls below the off level, the Star
  is released, the hand closes in again. And each 20 ms move at up to 700 mW
  shoves the finger out at near full servo speed instead of resisting
  penetration, with no damping anywhere.
- *Planned v2:* render the cube as a wall (power continuous in how far the
  Star is past the jaw position, no torque on/off level), longer Star move
  times and a power slew limit as damping, lower starting stiffness, the lead
  cap only while the jaws are nearly still, baseline only while moving, and
  the follower clamped at its real closed stop.

**v2, Run A (`evidence/gripper-haptics-v2-20261027T051940Z-follower`, the Star
never energized):** with the lead cap only while the jaws are still, the
follower trails the Star by 60 ms (v1: 260 ms), peaks at 437 deg/s (v1:
116, capped) against the hand's 425, and the median tracking error while
moving falls from 21.8 to 10.1 deg. Free-motion acceleration alone reaches
3.1 N m, which is why contact only counts while the jaws are still.

**v2, Run B (`evidence/gripper-haptics-v2-20261027T054317Z`, the cube):**
the oscillation is gone. Each cube contact (3.8, 5.1 and 1.2 s) switched the
Star's push on once and off once, against 21 switches in 14 s in v1. Gavin:
"I don't see any oscillations that we had before. And then when I squeeze on
the cube, it feels good ... acceptable ... not disastrous." The wall is soft:
he pushed up to 11 deg past it at the 600 mW cap while the Star drew only
60-72 mA (v1: 660 mA), probably because the 150 ms move time keeps the servo
following a slow path instead of loading up. Still to fix:
- false contacts at the closed stop (4, 0.3-0.8 s each) when closing fully on
  nothing: the jaws hitting their own stop. Ignore contact near the closed stop
  and require the squeeze to persist ~60 ms.
- impact spikes of 6-7.4 N m when closing fast onto the cube, before the
  squeeze cap engages (no lead cap while moving). Keep a moderate cap while
  moving too.
- Gavin: the gripper "never fully closes ... and then also never fully opens".
  Open is the known soft-limit bug (-118.5 reached, -156 measured). Closed
  reached -32.6, the same as the hand-measured stop, so either that
  measurement was not at the real stop or the jaws keep a gap there: to check.
  Owned by the mapping work ([viscous-mapping/README.md](viscous-mapping/README.md));
  the haptics script will read its committed mapping rather than patch numbers
  locally.

**Gripper range is wrong on unit 1 (2026-10-27).** Gavin: "the follower arm
did not move the full range". Measured by hand, torque off
(`scripts/gripper-range-read.py`, `evidence/gripper-range-20261027T002654.json`):

| | measured | configured |
|---|---|---|
| follower closed stop | -32.6 | soft limit -2.8 |
| follower open stop | -156 | soft limit -120 |
| follower width | 123 | 117 + 2 x 2.9 back-off = 123 |
| Star travel | -3.1..+60.6 | preset assumes -16.9..+48.9 |

The soft-limit window has the right width but sits ~35 deg off, so the
follower's gripper zero has moved since the limits were captured (2026-08-28).
Closing is reachable (the preset's -33 offset matches the closed stop); the
last 36 deg of OPENING is cut off by the -120 soft limit. The preset scale
(-1.778) also maps the Star's 64 deg onto 113 deg, not 123 (fit: -1.936 x Star
- 38.6). A follower powered up fully open would sit 36 deg outside its limits,
past MakerFollower's 20 deg wrap grace, and be refused as a stale zero.
Gavin called this a recurring problem and asked for a fix that stays fixed; a
Claude subagent and Codex (gpt-6.1-sol) are each investigating independently,
read-only, to be discussed before any change.

**Findings:**
- *Turn reset at power-up:* the follower gripper read +327.5 deg after power
  on; the script applies the follower driver's own +-360 deg session
  correction (`MakerFollower._detect_full_turn_offsets`) to get -32.5.
- *The follower's closed stop is about -32.4 deg*, not the -2.8 soft limit. The
  Star's closed position maps about 1.4 deg past it, so a closed, idle gripper
  pushes a steady +0.45 N m into its own stop. In this run the operator only
  started moving 30 s in, so the calibration window was mostly that hold. The
  baseline came out 0.45 N m, the threshold 0.98 N m rather than the ~0.25 the
  0.027 N m still noise allows. Fix: baseline only while the jaws move, and
  clamp the follower at its real closed stop.
- *Chatter:* 21 gate-on events in about 14 s of contact. Some is the squeeze
  and release; some is the loop: the Star reaches the wall, contact drops below
  the off level, the Star is released, the hand squeezes again. Ramp power
  down instead of cutting to release.
- *The power field is not a current bound when back-driven:* the Star drew up
  to 660 mA while gated (136 mA stalled into its own stop at 800 mW on
  10-21). mW is a setting, not a calibrated force.
- Runs 050418Z and 050446Z are aborts: nobody moved the Star during
  calibration, the script refused to drive a Star whose travel it had not
  seen, and released everything. They double as stop-path proof on the real
  launch method (foreground ssh).

### Gripper haptics in Lab teleop (2026-10-28, offline, not yet powered)

*Intent:* Gavin: "can we add a checkbox button next to the teleop button in
the gui to turn on and off the haptic feedback ... there should be one for
only the gripper, i played with it 2 days back". What he played with was
the PoC script; this moves its v2 loop into Lab teleop behind a checkbox.

- **UI:** a "Gripper haptics" checkbox beside Teleop, off by default, shown
  for a single arm whose family has `supports_gripper_haptics` (Viscous only).
  It sends `gripper_haptics: true` in the teleoperation session options. The
  teleop dialog shows the phase: connecting, calibrating (open and close the
  Star freely), on, contact at N mW, or off with the reason.
- **Code:** `makermodslab/gripper_haptics.py`. Same constants as v2 Run B, plus
  two items from Run B's to-fix list: contact must persist 60 ms, and no
  contact counts within 4 deg of the closed stop (so an object thinner than
  that does not render). Not yet done: the moderate lead cap while moving
  (impact spikes).
- **Transport, and why:** motorbridge cannot send a capped-power move, so a
  haptic session opens the Star through `fashionstar_raw` (`HapticStarLeader`
  = the stock leader on `RawStarBus`), never both on the port. With the box
  off, teleop is unchanged and still uses motorbridge. The raw bus reads the
  seven servos one monitor request at a time; **the loop rate this gives is
  unmeasured** (the Star was unpowered on 10-28). Measure it on the first run.
- **Mapping:** the session's own leader mapping (Lab, absolute) replaces the
  PoC's relative one. The closed stop is the leader's gripper offset (-33 on
  the built-in unit 1 preset), and the follower never gets a target past it.
- **Safety:** only the Star gripper is ever energized (100-600 mW, toward the
  wall, released at or outside it). Any haptics fault (stale reading, Star at
  55 C or more, follower gripper at 82 C or more, a serial error) releases it and
  turns haptics off while teleop carries on. The loop releases it before the
  return to rest; disconnect releases all seven twice. `RawBus.close()` is
  not used, because its signal handlers only work in the main thread and teleop runs
  in a worker, where it would close the port without releasing anything.
- **Evidence:** each session writes
  `$MAKERMODSLAB_HOME/gripper_haptics/gripper-haptics-<UTC>.csv.gz`, with the
  PoC's columns.
- Tests: `tests/test_gripper_haptics.py` (fakes and an injected clock).

**First powered runs (2026-10-28, unit 1, Gavin at the arm, demo prep).**
Logs: `~/.makermods/makermodslab-viscous/gripper_haptics/` on gavin-linux.
- Loop rate with the raw Star transport: 21.4 ms median tick, 22-24 ms p95
  (about 47 Hz), so the one-at-a-time servo reads are fine.
- Run 1 (`223401Z`): Gavin, "did not work". The follower started 27 deg from
  the Star's pose (Lab maps absolutely, the PoC relatively), the squeeze cap
  stalled it on friction for 4 s, and that 2.75 N m became the baseline
  (threshold 4.4 N m). Fix: no cap while calibrating, and the baseline counts
  only ticks within 3 deg of their command.
- Run 3 (`223652Z`): three ticks of a squeeze being released (of 59) set
  p99, giving a 5.7 N m threshold while 60 squeezed ticks sat above 1 N m. Fix: p95,
  threshold clamped to 0.5..1.5 N m.
- Run 4 (`225901Z`): works. Baseline 0.25 N m, threshold 0.5 N m. Three grasps
  on the cube at -53..-56 deg (squeeze 2.9-3.2 N m), each latched once and
  released once when the Star opened past the wall. The Star pushed back for
  1227 ticks, up to the 600 mW cap at 15 deg past the wall. Gavin's feel:
  not yet recorded.

### Independent fast-close controller attempt (2026-10-28, goal A3, offline)

Gavin needs to demo gripper feedback ASAP: fast closes feel like no wall,
and the shorter/reduced-resend moves feel only slightly better. Independent
implementation and full diagnosis/constants/handoff:
[gripper-haptics-independent-2026-10-28.md](gripper-haptics-independent-2026-10-28.md).

Newest 234727Z corrects the earlier current interpretation: at >=590 mW,
82 mA median / 279 mA maximum, versus 46.5 / 82 in 234213Z. Still, fast
contact latches 12-14 Star degrees through the wall and spends another
~107 ms reaching 600 mW. The replacement uses 200-400 mW damping when a fast
hand outruns the follower, then a fixed contact wall with a 1 ms requested
setpoint and depth/velocity effort, **800 mW maximum**
(previously exercised at 136 mA stalled, 29-31 C on 10-21; not a force bound).
An impact path detects high squeeze plus sharp deceleration before the jaw
stops. Closing follower lead is bounded at 8 deg still / 12 deg moving.
The full newest trace replays through fake buses without fault; for one
grasp braking starts 106 ms before observed impact and full wall effort is
requested at impact, 129 ms before the old command reaches 600 mW.
This predicts command timing only, not hardware feel. All 27 focused tests
pass. No hardware access, deployment, restart or push; implementation is on
the independent worktree branch. Next: Claude deploys for Gavin to judge
earlier resistance, fast empty-close drag, opening shove and chatter.
Thermal/fault/worker release paths remain; partial writes and failed-release
cleanup are hardened. Read the linked handoff before the first live trial.

## Handoff: state and how to resume (any agent, any tool)

This section is the source of truth for picking the work back up. Nothing about
this project should live only in one assistant's private memory: update this
file instead.

### To-do (open items, in priority order; tick and date them, don't delete)

- [ ] **Dream-1 on the real arm in the tank** (Gavin, 2026-11-07: "can we now
  build a bounding box to run dream1?"). The keep-out box is built and tested
  offline (`makermodslab/tank_guard.py`, branch `feat/viscous-dream1-tank-box`);
  no executor and no powered run yet. The plan and Gavin's open questions are in
  [dream1-tank.md](dream1-tank.md). Before any powered step: base clamped,
  near wall traced, cameras mounted, and Gavin's go-ahead on the plan.

- [ ] **Aquarium glass protection** (Gavin, 2026-11-07: "do we need anything more
  for protection so the arm does not break the glass of the aquarium?"). Agreed
  and in progress: the torque-off "Measure tank" touch-probe in the Lab GUI,
  branch `feat/viscous-tank-measure`. Proposed, awaiting Gavin's pick: the follower
  start gate and J1 wrap guard (top priority: today's start targets drive the arm
  through or across the glass), force cap, tank limits, clamp off the glass, tank
  support, keep-out check. See "Aquarium glass protection and the tank
  measurement (2026-11-07)".

- [x] **2026-11-05 offline left-mount preparation (A5a):** URDF/plate audit,
  top/three-quarter PNGs, pending unit assignment, candidate limits, and
  motor-1-only operator tool with hardware-free default/dry-run and fake-bus
  tests. [Findings and exact future commands](viscous-left-mount-2026-11-05.md).
- [x] **2026-11-05, done on the Mac:** forward neutral confirmed by Gavin with the arm
  stretched out; J1 read +82.54 (folded it read +118.4, misleading), zeroed to −0.01.
  See "Re-zero done" in the left-mount doc.
- [x] ~~**Before any next powered run:**~~ Gavin confirms unit 1's actual left
  mount index/handedness and forward neutral. Candidate reading +90° ±10°
  in unit 1's existing branch; wrong sign/whole-turn branch aborts. CAD alone
  cannot settle the physical index. New soft range at exactly +90 would be
  −275.4…+37.6°, with limited CW travel; judge suitability at the hull.
- [ ] **Remaining from here (re-zero DONE 2026-11-05, s=+82.54; profiles staged in
  `operator-run/`, NOT installed):** install BOTH staged profiles + calibration with Lab
  stopped on whichever machine runs unit 1, before ANY Lab teleop on it.
  Original item: ~~Gavin runs the prepared motor-1
  re-zero procedure, preserves actual evidence, stages and installs BOTH
  measured-shift profiles with Lab stopped, updates calibration metadata and
  commits the release.~~ Do not start the old fallback
  after zeroing. Keep FF/haptics off for initial acceptance; preserve the wet
  arm's separate frame/profile. Repaint the left-unit hull envelope before
  hull-near teleop, and synchronise the verified unit-1 release to the Mac.
- [ ] **Live Viscous 3D view in the Lab** (Gavin, 2026-11-07: "can we make the live
  viscous arm 3d view just like how makermods labs does it?" then "build that
  later"). Same shape as Maker/Metal: the URDF under `frontend/public/`, an entry
  in `frontend/src/lib/urdfConfigs.ts`, the family's `urdf_joint_positions`
  (`arms/urdf.py`) and `telemetry_kind`. Blocked on verified motor-to-URDF signs:
  on 2026-11-07 the waterproof arm's read pose rendered with J5 (wrist_yaw) tilted
  the wrong way against Gavin's photo, and no joint's sign has ever been checked
  on hardware (teleop is motor-to-motor, so it never exercised the model). Do the
  hand-push sign test per joint first and use the sign map from the aquarium
  environment work (`configs/viscous/model/motor_to_urdf.json`).
- [ ] **Update the fork from makermods-robotics** (Gavin, 2026-11-07: "about time
  to update our fork ... without breaking the stuff for our viscous arm", then
  "we can do the updates later"). State on 2026-11-07: `feat/viscous-arm` is 121
  commits behind `upstream/staging` (merge base `a031174e9`, 2026-09-14) and 182
  ahead; 17 files changed on both sides (`teleoperate.py`, `sessions.py`,
  `server.py`, `arms/base.py`, `arms/can_common.py`, `arms/maker.py`,
  `remote_can.py`, `maker_rest_pose.py`, `record.py`, `api_errors.py`,
  `schemas/sessions.py`, `sessionApi.ts`, the two contract/registry tests,
  `openapi.json`, `CLAUDE.md`, `frontend.yml`). The lerobot side needs no merge:
  our pin `gavingavinchan/lerobot-MakerMods@2a5ea4e` already contains upstream's
  `5ce2fe41a` plus 16 Viscous commits. Do it in its own worktree, keep the
  Viscous tests green, and do a powered acceptance run before it replaces the
  running Lab on either machine.

**Parallel track (Gavin, 2026-10-27): B4, shackle pin insertion, sim to real.**

2026-11-06, B4 holder: Gavin requested 2–3 zip-tie routes and clarified that the
filled tank cannot take tape/glue. New freestanding 150 × 130 mm base, two
top-access tie channels and curved saddles keep the bow at floor height.
75° is a CAD example pending the environment thread's final layout; do not
treat it as the approved print angle. Files, mesh/fit/gravity checks and the
handoff to use the actual holder in sim are in
[the holder report](../scripts/viscous-dream1/runs/shackle_tied_holder/README.md).
This supersedes the old glued mounting assumption; no powered run or print.

2026-11-06, later B4 direction: Gavin rejected the initial freestanding holder's
appearance and explicitly asked Astra to redesign it, with Prusa MK3-style
guided zip-tie channels and a cylindrical pin cup about 3 cm. Task sequence is
now pickup **from the cup**, insert, then tighten. Astra implementation is
running; first exports remain review material. Cup height is provisionally
30 mm, pin-size bore; final tilt/placement awaits the environment session.

2026-11-06, Astra round 1 reviewed: curved cradle/elliptical foot and enclosed
Prusa-style routes replace the blocky first form. New environment finding:
upright pin pickup violates the underwater wrist constraint. Astra round 2 is
building a tilted cup on a raised pedestal; 30 mm is its axial cup length,
with the pedestal height determined by pickup feasibility. Final dimensions
and layout remain with the orientation session; no print has been selected.

2026-11-07, B4 fixture CAD ready for the pre-print gate: Astra's curved holder
and raised tilted cup are generated at the selected 85° / 80 mm tip-height
pose, with local and placed scene meshes in `runs/shackle_tied_holder/tilt_85deg`.
Parent independently rebuilt the prints and checked the hand envelope. Required
states clear with 0.5 mm padding, provided cup extraction stays at roll 0° and
late threading uses ≤26 mm jaw opening per side. Full-splay/in-cup-roll stress
overlaps remain in the evidence. No print, full-motion acceptance or hardware
trial yet; see the holder handoff and task 7's pre-print gate. The Mac remains
unreachable over SSH; the Linux changes are the current handoff.

2026-11-07, B4 pre-print gate passed kinematically (`3393c787`): the parts
need no revision. The old cup radius500/bore575 placement failed extraction
and water checks; use **cup radius565 at +15°, tip80 mm up, bore600 mm**.
Updated generator defaults, placed meshes and layout manifest reproduce that
passing placement; all print files/local fixture meshes remain byte-identical
to `0de8facd`. The gate uses head TCP120.15 mm from the tip, 90° strokes and
18 mm-per-side resets to clear the collar/ear. Force, friction, torque, drag,
slicing and physical fit/grasp trials remain untested; see the gate report.

Model a U-shaped part and its bolt, have Astra insert and tighten it in sim,
then test whether its in-context lessons carry to the real arm. Plan and task
list in [shackle-sim2real.md](shackle-sim2real.md). 2026-10-28: model,
threads, printable holder, sim scene done; Dream-1 0/3; Astra seated the pin in
sim on attempt 2 using its own notes from attempt 1 (n=1). Open: Gavin's
measurements, repeat runs, real-arm torque.

**Top priority: "fixes stay fixed" / plug-and-play teleop (Gavin, 2026-10-27).**
"If we don't fix this we're gonna encounter issues again." Gavin travels to
demo the arm and must never debug in front of a guest: plug into any USB
socket, press Teleop, it works, with no rehearsal. Background, the three
reviews (Claude, Codex, Fable), Codex's design and the stage 1 report are in
[viscous-mapping/](viscous-mapping/) (start with
`plug-and-play-design-2026-10-27.md` and `plug-and-play-stage1-report-2026-10-27.md`).

- [x] Stage 1 built and merged (2026-10-27): USB rediscovery on every Teleop
  press, adapter enrollment with a one-start "Continue anyway" warning,
  bounded wait for arm power, plain error sentences, fake-hardware tests, CI
  triggers for this branch. Viscous only; Maker unchanged. Offline only.
- [ ] **Go live** (ask Gavin first, no session running): rebuild
  `frontend/dist` locally on each machine (never commit it), then
  `systemctl --user restart makermodslab-viscous` on gavin-linux; the Mac Lab
  just needs a normal start.
- [ ] **First powered try, Gavin at the arm (~10 min):** swap USB sockets;
  press Teleop with the arm off, then power it on (should connect in the same
  click); Stop during the wait; on gavin-linux the `viscous_waterproof_01`
  record must show the wrong-arm warning. Agents run it and talk through each
  phase (Working rules).
- [ ] **GitHub Actions on the fork:** `gavingavinchan/makermodslab` has no
  workflows registered (Actions never enabled on the fork), so CI does not run.
  Gavin enables it in the fork's Actions tab (his account; don't do it for him).
- [ ] **Stage 2, unit 1's measured profile:** ask Gavin re-zero vs keep the
  zero and re-measure the limits (Codex recommends keep). Measure all seven
  joints' stops and matched leader poses with him at the arm, commit
  `configs/viscous/viscous_01/viscous_profiles/viscous_01.json`, remove the
  preset fallback in `arms/viscous.py`, route `scripts/gripper-haptics-poc.py`
  through the same loader. Fixes the gripper not opening fully.
  **2026-11-05 update:** the LEFT mounting now has a prepared J1-only migration
  under A5a above. Its operator-generated profile translates J1 only; all-joint
  stop/leader/gripper measurement remains outstanding. Do not install nominal
  +90 candidate data as a measured profile.
- [ ] **Reconcile the Mac's waterproof profile:** the live copy (Oct 13) is
  newer than the committed bundle (Oct 9); commit the live one before anything
  enforces the committed copy.
- [ ] **Linux `viscous_waterproof_01` record** points at unit 1's ports: stage 1
  now warns, but ask Gavin whether to delete it or blank its ports.
- [ ] Later stages (design doc §8): motor-ID arm identity, pre-torque
  stale-zero check, record/replay/inference using the same checks, a small
  deploy script + running-version label, a one-line-per-bug log.

**Machines** (Gavin's):
- **MacBook Air** (this repo at `~/Documents/makermodslab-viscous`). The
  waterproof arm `viscous_waterproof_01` is connected here: CANable serial
  `205038953846` (`/dev/cu.usbmodem2050389538461`), Star leader on
  `/dev/cu.wchusbserial*`. Lab state lives in `~/.makermods/makermodslab-viscous`.
- **gavin-linux** (Tailscale `100.64.91.4`, `ssh gavin-linux`; same repo path).
  Unit 1 `viscous_01` (dry, non-waterproof) is connected here: CANable
  `/dev/serial/by-id/usb-Openlight_Labs_CANable2_..._207E37A55445-if00`, Star on
  `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`. Lab runs as the user
  service `makermodslab-viscous` (`http://100.64.91.4:8000`).
  **2026-11-05 physical update:** Gavin says dry `viscous_01` is now on the
  LEFT ROV mount. Prior right-mounted unit was `viscous_waterproof_01`. This
  reports physical placement, not a change to USB ownership or robot records.
- The Mac repo has a `linux` remote (`gavin-linux:Documents/makermodslab-viscous`)
  for moving unpushed work between machines. Git on Linux authenticates through
  `gh` (`gh auth setup-git` is done); its GitHub fetches occasionally time out, so retry.

**State at the end of 2026-10-20:**
- Everything is committed and pushed on both machines (`feat/viscous-arm`).
- On gavin-linux the service has a drop-in,
  `~/.config/systemd/user/makermodslab-viscous.service.d/gravity-ff.conf`, that turns
  on gravity feedforward for `viscous_01` teleop at 50%
  (`MAKERMODSLAB_GRAVITY_FF`, `MAKERMODSLAB_GRAVITY_FF_SCALE=0.5`). Delete it and
  `systemctl --user restart makermodslab-viscous` to turn it off.
- Unit 1 was left torque-off, tipped forward ~25° onto its gripper (the release
  tip described under Safety lessons). Refold it before any powered run; the
  collection planner refuses to start from the tipped pose.
- Pinocchio is installed in both venvs. On a new machine: `uv sync --extra test
  --extra arm-dynamics`.

**Resume left mounting here (2026-11-05):**
Read [viscous-left-mount-2026-11-05.md](viscous-left-mount-2026-11-05.md) and
show `configs/viscous/viscous_01/evidence/left-mount-2026-11-05/top-down.png`
and `three-quarter.png`. Both CAD mount rotations are identity. Left J1 axis
−Z, mirrored right +Z; Lab's reference model uses direct motor radians about
−Z. The CAD folded reference pose is [0,−145,+120,−25.748,0,0]°, not the Lab
zero. Full plate geometry is exactly Y-reflected, not evidence of a simple
±90° rotated same part. Physical manufacturing equivalence remains unverified.
Unit 1 must use the reference/single-arm handedness, not a reflected motor map.

The proposed left +90° old-frame motor reading would move 90° CW, with new
backed-off J1 range −275.4…+37.6° (estimated physical stops −278.265…+40.465°).
This is **conditional**: no existing CAD/calibration file records the actual
base bolt index. The right recorded −94.48° shift supports about 90° CCW;
its 4.48° deviation is not a left-unit offset. Future operator tool
`scripts/viscous_left_mount.py` defaults to a completely offline dry run;
`--inspect`/`--execute` are explicitly future hardware operations. It rejects
wrong sign/branch, journals the persistent write, verifies zero and final
release, then stages paired profiles using the actual shift. It never edits
live Lab state. The shared presets/URDF, wet profiles, existing sim scenes,
uncommitted gripper POC and `frontend/dist` are unchanged by this work.

The prepared leader J1 clamp is narrower, −154…37° at the nominal shift,
because shifting the whole follower range into the Star's output range would
move its raw unwrap boundary inside the leader's measured positive travel and
cause a large negative target jump. Gain stays +1.439742 and offset 0;
501-point travel/whole-turn-alias tests cover shifts +80/+90/+100.

Verification: 31 offline tests pass (new operator-tool fakes plus existing
Viscous family tests); 100-pose FK/mirror checks pass, fitted gravity is
invariant to upright J1 yaw to 1.8e−15 Nm, source snapshots/hash evidence and
both reviewed Blender PNGs are committed. The air fit needs no sign flip for
upright yaw alone; ROV tilt/immersion/payload and hull clearance remain outside
that result. No arm/Star port opened, no service restart/reconfiguration, no
robot record edits. Only this Linux checkout's task changes are committed and
pushed; no cross-machine deployment or unrelated-repository commits are part
of this offline task. Next action is physical inspection with Gavin, not an
automatic motor command.

**Resume Dream-1 here (2026-10-23):** B3a (propeller scrubbing in sim) is done:
zero-shot Dream-1 carries the sponge to the propeller but never scrubs. B3c (flange
bolting scene) is being built by Codex on gavin-linux (`flange_codex.log`, output
`runs/flange/`); show Gavin the scene before any live run. Then B3b,
the BlueROV2 scene (see B3); its three open questions are for Gavin first.

**Resume here (2026-10-21):** Gavin's first item is goal B3, finding why
Dream-1 drives the Viscous sim so badly: work the checklist under "Dream-1 trial"
in order. Haptics: steps 1-2 done 2026-10-20, friction term and shoulder
refit done and confirmed live (v2 fit). Before energizing, show Gavin the
dry-run plan and say exactly which joints will move.

**Mapping problem, open (2026-10-27):** unit 1's joint mapping (today: the
gripper's soft limits sit ~35 deg off its real stops) keeps going wrong; Gavin
wants it solved once, in a fresh session. Start from
[viscous-mapping/README.md](viscous-mapping/README.md). Gripper haptics Run B
waits on it.

**Plug-and-play stage 1 (2026-10-27, goal P / A3):** offline implementation
on `feat/viscous-plug-and-play`: live USB descriptor rebinding, committed adapter
enrollment with a one-start warning override, and a leased/cancellable RobStride
and Star power-up wait. Gavin's intent: “If I go anywhere, I plug in, it just works.”
See [the stage 1 report](viscous-mapping/plug-and-play-stage1-report-2026-10-27.md)
for scope, fork safety evidence, baseline failures and powered acceptance still
needed. No powered test, live-state change, deployment or push was performed;
this brief explicitly limited work to the dedicated worktree and forbade pushes.
Unit 1's gripper profile and motor-UID identity remain deferred.

*Review fixes (2026-10-27):* stage 1 opts in Viscous only; Maker retains its
existing slcan ports, bimanual support and synchronous connect. The frontend
asks the arms manifest for the discovery capability. Review checks and the
confirmed pre-existing suite failures are recorded in the stage 1 report.

*Merged into `feat/viscous-arm` (2026-10-27, Claude), pushed, pulled on
gavin-linux.* Not yet live anywhere: the gavin-linux service still runs its
2026-10-22 process and the Mac Lab was not running; neither `frontend/dist` was
rebuilt. To go live (with Gavin's OK, no session running): rebuild the
frontend locally, then `systemctl --user restart makermodslab-viscous` on
gavin-linux. Next, in order: (1) first powered try with Gavin at the arm
(swap USB sockets, start with the arm off then power it, Stop during the
wait, the Linux `viscous_waterproof_01` record should warn); (2) GitHub
Actions are not enabled on the `gavingavinchan/makermodslab` fork (no
workflows registered), so the new CI triggers do nothing until Gavin enables
Actions in the fork's Actions tab; (3) stage 2, unit 1's measured profile,
needs Gavin's re-zero vs recapture decision and him at the arm.

**Resume haptics here (2026-10-28, later):** gripper haptics works in Lab teleop (run 4, see "First powered runs"). Next: Gavin's feel, the moving lead cap for impact spikes.

**Resume haptics here (2026-10-28):** gripper haptics is in Lab teleop behind
a checkbox (see "Gripper haptics in Lab teleop"), offline only. Next, with
Gavin at unit 1: rebuild the frontend, restart the service (this also makes
plug-and-play stage 1 live), and do the first powered run. Check the loop
rate with the raw Star transport and his feel against Run B.

**Resume haptics here (2026-10-27):** the gripper proof of concept works
(see "Gripper haptics proof of concept"). Next: get Gavin's feel, fix the
calibration baseline, the follower's closed-stop clamp and the gate chatter,
then move the loop off the script and into Lab teleop for the gripper before
any arm joint. Unit 1 was folded by hand on 10-27 and both arms were powered.

**Resume haptics here (2026-10-21):** goal A2. The Star gripper has been
characterised as an actuator without a hand on it (see "Star actuation check").
`scripts/star-actuation-check.py` (raw transport `makermodslab/fashionstar_raw.py`)
energizes one servo with an explicit power cap and releases all seven on every
exit. The Star's power supply is the hard stop (stops are unacknowledged and
there is no bus-loss watchdog). Gavin has judged the gripper safe to test and
wants such runs done without waiting for him. Next:
1. Put `star_follow.py` on `fashionstar_raw.py` (power on every command, an
   explicit 0x10 release for zero effort, never power 0) and dry-run it.
2. When Gavin is at the arm: the hold and damping phases with his hand on the
   gripper, recording his feel per power. That sets A3's power range.
3. Check the gripper's closed-side travel (-2.2 deg measured vs -16.9 derived).
Unit 1 (the follower) is torque-off and probably tipped forward; it is not
needed for Star tests.

**Open item (Gavin, 2026-10-20):** unit 1's wrist camera image is flipped. The
`viscous_01` record's `wrist` camera (opencv index 0, 640x480 MJPG) has no
rotation field; lerobot's `OpenCVCameraConfig` takes `rotation` (0/90/180/-90),
but check whether the Lab's camera config passes it through before relying on
it. Ask which way it is flipped (180 deg vs mirrored) before fixing: a mirror
is not a rotation. Fix it before recording any dataset or policy run on unit 1.

## Waterproof arm actuator check before the SubConn splice (2026-11-03)

*Intent:* Gavin, before cutting the waterproof arm's cables to splice in SubConn
connectors: "make sure all the actuators are still working". The arm was on
gavin-linux (CANable `205038953846` on `ttyACM0`), the Star leader unplugged.

- *Result: all seven actuators move and their encoders follow.* Shoulder, elbow
  and wrist_flex by `scripts/viscous-point-up.py` (folded to straight up and back,
  3 deg/s: shoulder_lift -88 for -90, elbow 157.9 for 157, wrist_flex 32.1 for 32).
  Gripper, wrist_roll, wrist_yaw and pan by `scripts/viscous-wiggle-joints.py`,
  the Lab's CAN gripper wiggle widened to any joint (one motor on the bus, the rest
  limp): commanded/encoder 12.6/10.7, 20/19.7, 20/17.4 and 10/9.8 deg.
- *Straight up on the URDF* is shoulder_lift -90, elbow_flex 157, wrist_flex 32,
  not elbow 180: the forearm and wrist are cranked ~23 deg. Wrist, roll joint and
  tool0 then sit within 1 mm of the vertical over the shoulder.
- *J1's real negative stop bit:* pan rested at -37 (past the recorded -32.4 stop
  of the post-ROV frame; Lab's soft limit still allows -127). A -15 deg pan
  stroke from there tripped the 10 deg lag check against the stop; one
  `No response ... shoulder_pan` warning came with it. Stroke pan + only from
  there until its soft limit is moved into the new frame.
- *Launch gotcha:* `nohup setsid cmd & echo $!` records setsid's pid, not
  Python's, so the first stop signal went to a dead pid. Both scripts take
  `--pid-file`/handle SIGTERM; signal the pid the script wrote.
- gavin-linux's `viscous_waterproof_01` record still points at unit 1's CANable;
  the scripts took `--port` explicitly. Not yet changed (Gavin's call).

## Waterproof arm after the SubConn splice: CAN check (2026-11-07)

*Intent:* Gavin, after re-doing the cable management with the SubConn
connectors installed: "try to connect to it". Arm on the Mac (CANable
`205038953846`), no Star leader plugged in, Lab running but idle.

- Read-only check: bus opened without the enable handshake, each motor sent the
  same clear-fault state refresh as the Lab's reads, closed with no disable write.
  Nothing was energized.
- *First try: no motor answered* (0/5 on all seven; the adapter itself
  answered). Cause: the GND had come loose on the shore side of the splice;
  Gavin fixed it.
- *After the fix: all seven answer 5/5*, torque 0, MOS 25-26 C. Readings (deg):
  pan +182.8, lift +297.4 (-62.6 one turn down), elbow 0.16, wrist_flex +40.4,
  yaw -2.4, roll -3.6, gripper +242.1 (-117.9 one turn down). Elbow matches its
  2026-10-11 folded reading exactly, so its zero survived the splice; the arm
  was not folded at the time. Gripper -117.9 would be 2.8 deg past its lower soft
  limit (-115.1); confirm its pose before any powered run.
- No powered motion yet.

### First teleop in the filled aquarium: J1 swung toward the back wall (2026-11-07)

*Intent:* Gavin: "i pointed the leader arm straight and unfolded, and the
follower arm started rotating left (like j1 rotating). check the mapping".

- Run: 1:1 posture trial in slow mode (15 deg/s), 17:37 server time. At start the
  trial's worst error was **shoulder_pan 188 deg**. Lift, elbow and wrist errors
  fell below 2 deg within 4 s, so they map correctly. Gavin stopped after about
  60 deg of J1 travel, and the stop path returned the arm to its start pose.
  Read-only check afterwards: pan **+184.4**, which Gavin calls "straight-ish".
- Cause: in the tank the follower points straight into the tank at pan about
  +182 (matches the aquarium model: pan +182 puts tool0 toward -x). The trial
  maps a straight leader to pan about 0, which points over the base's back edge
  toward the back wall 40 mm behind J1. The trial has no follower start gate
  (the warm-up is an indicator, by design), so it slewed toward it.
- Fix: the Star profile `viscous_profiles/star_viscous_waterproof_01.json`
  (under MAKERMODSLAB_HOME, not in the repo) now has
  `joint_offsets.shoulder_pan = 180`. A straight leader maps to pan +180, and
  ±30 deg on the leader maps to 150-210, inside the trial's J1 limits
  (-32.4, 282.0). Backup: `backups/star_viscous_waterproof_01.json.before-j1-offset-180-20261107`.
  Rotation sense is unchanged (sign +1); confirm it with a small leader pan move.
  180 is nominal; tune it by a few degrees if the arms look offset when both are
  straight.
- Same evening, Gavin: "its still rotated a bit to the left" (a render of the
  read pose showed J1 at +187.4, 7.4 deg toward +y), then "let the current
  position of j1 be forward". J1 read **+185.8**, so the offset is now
  **185.8**: a straight leader maps to J1 +185.8. Previous file:
  `backups/star_viscous_waterproof_01.json.before-j1-offset-185.8-20261107`.
- Not yet verified: why the 2026-10-20 trial worked at offset 0. Either the
  arm sits in the tank turned half a turn from its bench orientation, or J1's
  zero moved again (as it did by 94.48 deg on 2026-10-11). If J1's zero moved,
  the trial's J1 limits are wrong by the same amount; check with Gavin.
- Non-trial teleop also applies `joint_offsets`, but against the uncorrected
  profile J1 limits (-126.9, 187.5), so it saturates at 187.5 one way.
- Open: a start gate that refuses when any mapped joint is far from the
  follower (here 188 deg) would have stopped this before any motion.

### Aquarium environment model (2026-11-07)

Offline subtask of A1a/A3 (aquarium contact/haptics) and B4 (sim to real).
Gavin: "create a new environment for the aquarium, aka something like urdf for
simulations ... so that you know the limits of the aquarium walls. we may or may
not need to put up real time boundary limits to the follower arm. lets see".

- Added `configs/viscous/environments/aquarium_landen_75/`: `tank.json` geometry
  source with provenance/status, measurement README, relative-mesh MuJoCo
  `aquarium_scene.xml`, `offline-validation.json` and three small read-pose
  PNG previews. `scripts/viscous_aquarium_env.py` provides importable numpy-only
  exact collision-vertex/plane clearance plus `clearance`, `sweep`, `scene`,
  `render`; `tests/test_viscous_aquarium_env.py` has 14 offline geometry tests,
  including independent Pinocchio FK. No live boundary or Lab changes.
- **Estimated:** nominal LANDEN 750 × 450 × 450 mm, unconfirmed 8 mm glass,
  near inner face x=+40, J1 centred laterally, aligned axes, no mat/spacer.
  Derived inner bounds (mm): x=[−694,40], y=[−217,217], z=[0,442]. Floor is
  base collision minimum; top subtracts one bottom pane. Water 283 mm and margin
  30 mm are design values. Gavin supplied motor readback; no new tape measurements
  or hardware reads. README lists exact J1-to-glass, glass, spacer and support
  tower measurements needed; tower/wiring/fixtures are not modelled.
- Read tool0=(−526.041,−65.063,42.820) mm; folded=(185.167,125.275,148.941).
  Full-arm read clearances (mm): near 0/base, far 112.647/right jaw, left
  142.000/base, right 87.732/left jaw, floor 0/base, rim 301.038/yaw link,
  water 142.038/yaw link. **Base touches floor and near wall by construction**;
  strictly positive clearance on every wall is impossible. All moving links
  are positive (near 8.979, floor 4.551). Folded target penetrates near half-space
  by **174.529 mm**, right jaw. MuJoCo 3.16.0 loads: 0 read-pose arm/glass
  contacts, 32 folded-target near-pane contacts; tool FK matches both models.
- Strict **30 mm all-link sweep: none for J1–J6** (base tangencies). Diagnostic
  **0 mm** sampled intervals, motor deg, all other joints fixed at read:
  J1 [173.02,187.53], J2 [−199.45,−160.83], J3 [170.96,237.96],
  J4 [38.42,82.10], J5 [−86.06,54.22], J6 [−16.46,14.54]. Uses committed
  waterproof `evidence/lab-hardware-profile.json` follower limits, 1° samples
  plus endpoints, no automatic exclusions. Read lift −201.83 and gripper −117.9
  are below profile limits; scans flag this and are **geometry only**, not
  hardware permission or a continuous path proof. Water is optional visual/
  submersion headroom, excluded from the default glass sweep; rim is not a lid.
- Initial clearance timing: **0.464 ms median, 0.482 ms p95**, 1,000 calls on
  gavin-linux, 9,624 distinct vertices, no geometric decimation. Fits the 2 ms
  target; measurements and mount-contact policy still precede a live guard.
- Re-run from repo root: `.venv/bin/python scripts/viscous_aquarium_env.py
  clearance --pose read` (or `--pose folded`), `sweep` (or `--margin-mm 0`);
  use `-o /tmp/result.json` for reports. Tests: `.venv/bin/python -m unittest
  discover -s tests -p test_viscous_aquarium_env.py -v`. Scene: use
  `~/dreamtrial/.venv/bin/python scripts/viscous_aquarium_env.py scene`, then
  the same Python with `MUJOCO_GL=egl` and `render`. MuJoCo is isolated from Lab.
  Offline only; local commit requested, **no push**, no service/session or
  installed-state changes, overriding the usual synchronisation rule below.

*2026-11-07 follow-up — J5 photo inference and moving/static clearance (A1a/A3, B4):*
Gavin: "seems like the j5 is tilted the wrong way". Added shared
`configs/viscous/model/motor_to_urdf.json`: J5 −1, hand test pending; J1–J4 +1
photo-consistent, J6 +1 assumed/unverified, all offsets zero. Aquarium FK and
saved MJCF keyframes use it; gravity's direct mapping is unchanged and needs
the same check. Independent Pinocchio confirms read tool0
(−520.150,+101.963,42.820) mm and folded (207.979,82.008,148.941).
New clearance semantics select all moving descendants of shoulder_pan,
**including yaw_link**; base_link is separate static-mount info, never a
violation, and has MuJoCo contact masks 0/0. Provenanced per-surface margins
in tank.json are walls/rim 30 mm, floor independently 0 mm.
Moving read distances (near/far/left/right/floor/rim, mm):
8.979/116.580/50.564/178.642/4.551/301.038; folded:
−201.280/513.697/78.974/99.975/57.400/173.206.
**Default 30/0 mm sweep remains empty for J1–J6: moving yaw_link is too close
to the assumed near wall**, not the static base or floor. Every near-wall
sample fails; no contiguous passing range contains read. Supplemental 0/0 mm
motor-degree samples: J1 [157.02,187.53], J2 [−199.45,−161.83],
J3 [170.96,235.96], J4 [38.42,82.10], J5 [−54.78,87.32],
J6 [−14.46,16.54]; J2 cannot contain the out-of-limit read angle.
Read lift/gripper remain outside profile limits; geometry only, no path or
hardware permission. MuJoCo 3.16.0: 0 read / 28 folded near-pane contacts;
three mapped previews regenerated/reviewed. 17 offline tests pass; read
clearance median/p95 0.435/0.438 ms (1,000 calls, 9,059 moving vertices;
565 static vertices reported separately). Updated README and
`offline-validation.json` preserve inputs, results and reproduction commands
above; `--margin-mm` now overrides only walls/rim, `--floor-margin-mm` is
independent. Placement/glass estimates are unchanged. Local follow-up to
1e5282b5 only, no ports/services, installed state, frontend edits, network or
push; the original subsection above is retained as history.

### Upside-down camera: a per-camera 180° flip (2026-11-07, goals A and B2)

Gavin installed a USB camera ("USB Camera", uniqueID `0x1300000c45636b`) that
is mounted upside down, and asked for it to be flipped "when we do the preview
in the gui and when we do physical ai autonomous stuff". Each camera in a
robot record can now carry `rotation: 180`, set with the **Flip 180°** button on
the camera's card in Robot settings. The flip is applied by lerobot at capture
(`OpenCVCameraConfig.rotation`), so recordings, local inference, remote
inference and hosting all see the picture upright, and a policy trained on
flipped recordings sees the same picture at rollout. The server-side preview
(`/camera-preview?rotation=180`) turns the frames too, so the tile shows what
gets recorded. Only 0 and 180 are accepted: a quarter turn makes lerobot swap
the capture width and height, which the record's dims and the policy-binding
check do not handle yet. Verified on the live camera: the barcode in frame
reads upright with the flip on. Both Viscous records had no cameras at the
time, so the camera still has to be added to the robot and flipped in the UI.

### Aquarium glass protection and the tank measurement (2026-11-07)

Subtask of A (piloting the arm safely) in the aquarium, and of B4's aquarium
layout. Gavin, with the waterproof arm in the rimless LANDEN tank and about to
fill it: "we already have the slow speed mode as protection. but do we need
anything more for protection so the arm does not break the glass of the
aquarium? give some suggestions."

*What the slow mode is and is not* (`makermodslab/can_speed_limit.py`): it caps
how fast each arm joint's **target** moves (≤30 deg/s, 15 on the monitored
arm) and seeds targets from a fresh reading before torque. It does not cap
force (MIT stiffness pushes up to the RS00's 14 N m peak against a blocked
joint; J1 at kp 60 gets there after ~13° of error), does not limit where the
arm goes (only the full mechanical soft limits, J1's still in the old frame),
does not stop on contact (the 10° lag trip lives only in bench scripts), and
does not rate-limit the gripper. Slow lowers impact energy, not the final push.

*Hazards found while answering* (the readiness thread's read-only checks):
- No follower start gate. Lab teleop connects the Star with `calibrate=False`,
  so even the leader-near-zero check never runs. With the arm stretched in the
  tank (pan +182) and the Star folded, the normal-mode start target puts the
  gripper ~145 mm **through the near glass**; the 1:1 posture mode wraps J1
  +182 → −178, clamps to −32.4 and swings the extended arm ~214°.
- The arm is mounted ~180° from the 2026-10-18 aquarium neutral.
- The bar clamp squeezes the rimless tank's polished top edge; the base tower's
  reaction loads and any tug on the tether lever on it. Uneven support under
  ~135–170 kg filled is the classic way rimless tanks fail.

*Proposed protection list (Claude, ranked; only the tank measurement below is
agreed so far, the rest awaits Gavin's pick):* must-have before water: a
follower-vs-first-target start gate (≤~10° per joint, refuse on a clip or an
out-of-limit start), a J1 wrap guard plus a J1 mapping that fits this mount;
a force cap (leash target-minus-measured to a few degrees, hold and end on
persistent error; keep gravity feedforward off in water, the dry fit
over-lifts under buoyancy); tank joint limits; the clamp off the glass (a
weighted freestanding base); full flat support and a levelling mat; the power
switch within reach of the leader hand; the tether strain-relieved to the table.
Should-have: the all-link keep-out check against the measured walls, a leader
jump filter, a gripper rate limit, soft bumpers, the contact-signal stop after
the wet fit (A1a). Nice-to-have: a live distance-to-wall readout, the RobStride
firmware torque limit as a backstop (unverified in MIT mode), sacrificial liner
sheets. Geometry: the arm's furthest collision point reaches ~0.61 m, so the far
end wall (~0.69 m) is out of reach; the long walls are the real hazard (J1 only
about ±16° either side of straight at full stretch with a 50 mm margin).

*The tank measurement (agreed, being built).* Gavin's design, in his words:
- "i manually move the torque off of the arm and you with the urdf figure out
  the size of the aquairum (its rectangular), and the position of the base of
  the arm relative to the aquarium. cuz everytime i take the arm in and out of
  the aquarium, the position may shift a bit. so i want a way to do it quickly
  (like i might do this once a day)"
- "there are a lot of cabling sticking out of the side of the arms, and not
  represented in the urdf. so ill mostly use the grippers tip to trace the
  aquarium. but you might want to track the rest of the arm as well."
- "lets also measure where the back wall is from the arm as j3 can easily hit
  it." (his back wall = the near face, +x, behind J1)
- "ya, there is the base tower as well. anyways, build this into the gui, and
  record down the intentions of the feature as well, especially the stuff
  coming from me"

How it follows from that:
- **Torque off, read-only**, opened exactly like zero calibration (no
  energizing handshake, explicit disable); it holds the bus as a calibration
  does, so every existing mutex refuses while it runs.
- **The gripper tip is the probe** (closed gripper), because cables stick out of
  the links and are not in the URDF. Touches are recorded when the arm is held
  still for ~1 s, with a beep, so no free hand is needed.
- **Daily walls:** 2 touches on each long wall (≥150 mm apart, so the base's yaw
  is well conditioned), 2 on the near/back wall (the elbow's hazard), 1 on the
  floor, 1 check touch. The fit returns the base's position and yaw in the tank
  and the inner width, refuses to save on a bad fit (residual > 3 mm, width off
  the expected ~434 mm), and reports how far the base moved since last time.
  The far wall is out of reach, so it is placed from the listed length and
  marked estimated.
- **Base tower, once:** front, top and sides touched with the tip; it is rigid
  with the base, so it is stored in the base frame.
- **"Track the rest of the arm":** after re-cabling, each link's cable side is
  pressed against an already-measured wall; the gap between the link's URDF
  surface and the wall is its cable padding. Unmeasured links get a 40 mm
  default. Live per-link clearance is shown during the measurement.
- **Output** is a `tank.json`-shaped file under `MAKERMODSLAB_HOME`, so the
  offline aquarium tool above can load it, and it replaces that model's
  estimated placement (which put yaw_link ~9 mm from an assumed near wall).
- **Measurement only.** Nothing in teleop reads it yet; the keep-out check and
  the start gate are the next steps and will.

Status 2026-11-07: implemented on branch `feat/viscous-tank-measure` (worktree
`../makermodslab-viscous-tank`, kept apart from the readiness work in the main
checkout); not merged, no hardware run. Codex wrote it from Claude's brief;
Claude reviewed it and changed one thing: the Viscous family's refusal to open
its follower for calibration (the re-zero guard) stays untouched, and the
measurement opens the bus through its own torque-off opener instead.
[tank-measure.md](tank-measure.md) has the operator sequence, the mesh-derived
closed-gripper probe, the FK cross-check (0.0 mm against the offline aquarium
script), the fit gates, the tower/cable extras, API and storage, the
never-energize contract and the follow-ups. It measures and displays only;
teleop still has no tank keep-out or start gate.

2026-11-07, after the merge (5c642f32) put the **Measure tank** button in the
running Lab: Gavin, "i will only move the arm manually when calibrating for
now". So for now the arm moves in the tank only by hand, torque off, which is
the one mode the measurement allows. The follower start gate and the J1 wrap
guard stay at the top of the must-haves. They are parked, not dropped, and
must be built before the next powered teleop in the tank.

First real measurement, the same evening: Gavin traced both long walls and the
floor by hand. Only two touches were stored, both auto-captured under the first
step's "right" label: one wall touch at y +207 mm, and one at the pose where the
arm was set down. The motion in between was not kept. Gavin: "thats dumb, at
least record all the positions so an ai agent can review it if needed". Every
reading now goes to a per-session trace (see tank-measure.md, Trace).

First traced run (trace `tank/viscous_waterproof_01/traces/20261108T044404.111395Z.jsonl`,
857 readings over 95 s): Gavin traced the floor, then the +y wall, then the -y
wall. He noticed the base was loose and had slid partway through, and pushed
it back. The fit uses the extremes of every gripper-mesh vertex, not the tip
point: on the floor the tip read 0 mm when the gripper pointed straight down
but 15-22 mm when it was tilted, so a tilted gripper touches with the finger
edge. Results in the J1 base frame:
- **+y wall** (the model's "left"): 210 mm from J1, sd 1.7 mm over 237 mm of
  travel along it, about 1 deg of yaw.
- **-y wall** (the model's "right"): 224 mm, from the readings that cluster
  between -218 and -228 mm. The rest of that pass spreads out to -254 mm,
  which fits the loose base being pushed sideways by up to 30 mm.
- **Inner width:** 210 + 224 = 434 mm, the nominal width for 8 mm glass. This
  supports both the -y plateau and the 8 mm glass assumption, but it is not
  yet confirmed from the box label.
- **J1** sits about 7 mm toward +y from the tank's centre line.
- **Floor:** the lowest finger point reached -4 mm (5th percentile; minimum
  -6), against the model's 0. Gavin confirmed the arm is mounted slightly
  raised above the glass ("the true floor is indeed below the bottom of the
  base of the arm as i mounted the arm slightly up (issues with the
  mounting"). The fingers cannot pass through the glass, so the floor is at
  -6 mm or lower. In tank.json that is `base_spacer_height_mm` of about 6
  (floor_z_mm about -6), and it should be checked with a ruler.
  Later: "lets go with this for now, the true gap is more like 15mm, but
  whatever, the floor of the aquarium is the only part of the aquarium the arm
  is allowed to touch anyways". So: floor about 15 mm below the base bottom
  (the trace's -6 mm is an upper bound). The floor is the one allowed contact
  surface; the walls and rim are keep-out.

Next (Gavin, same evening): "can we now build a bounding box to run dream1?",
in a new thread. Build a workspace box from these numbers that gates every
command sent to the real arm, so Dream-1 can run on the real Viscous in the
tank. Until now B3 has run in MuJoCo only.
- **Back/near wall:** not traced; it remains the 40 mm estimate.

Safety finding: the base slides about 30 mm under a hand push. A powered arm
can shove it the same way, which would invalidate any measured keep-out.
Clamp or weight the base before any powered motion in the tank.

2026-11-07, the bounding box (this thread, branch `feat/viscous-dream1-tank-box`).
Gavin: "can we now build a bounding box to run dream1?", with the rule above
that the floor is the only allowed contact. Built: `makermodslab/tank_guard.py`
plus `keepout_viscous_waterproof_01.json`, which checks every moving-link mesh
vertex (with 40 mm default cable padding and the 30 mm margin) against the
walls and rim, allows the floor with at most a 10 mm press, clips to joint
limits, refuses jumps over 20 deg (a J1 wrap), and proves the straight path
between targets clear, not just its ends. Re-fitting the trace with every
vertex: the arm reached +218.5 and -223..-231 mm, so the accepted 210 / 224
planes are 3-8 mm conservative. Design calls made without Gavin, to check
with him: `yaw_link` is exempt (it rides the base, 30-42 mm behind J1 at every
angle); the upper arm's heel may sit up to the estimated near plane; a start
up to 5 deg outside the soft limits is accepted (the arm rests outside three
of them). Offline validation (Codex, independent checker, 562,508 poses)
found no unsafe grant. It also found that Dream's sim tasks reach 110 mm past
the +y glass and its folded start fails the box, so the tank run needs its own
start pose and an object on the tank's centre line. On the +y side the J1
soft limit (187.5 deg, possibly stale) binds before the glass. Full write-up, validation and the real-run plan with open questions:
[dream1-tank.md](dream1-tank.md). Nothing drives the arm yet.

## Waterproof arm read-only CAN check (2026-11-13)

*Intent:* Gavin: "the waterproof arm is connected, test it, dont move it". Arm on
the Mac (CANable `/dev/cu.usbmodem2050389538461`), no Star leader plugged in, Lab
not running, port free.

- `scripts/viscous-can-ping.py` (bus opened without the enable handshake, the
  clear-fault state refresh only, closed with no disable write). Nothing energized.
- *All seven answer 5/5*, torque 0.00 Nm, MOS 24-27 C. Readings (deg): pan
  +178.1, lift +302.0 (-58.0 one turn down), elbow +0.49, wrist_flex +34.9, yaw
  -2.2, roll -1.4, gripper -0.30 (inside its soft limits [-115.1, -0.03], at the
  closed end; the 2026-11-07 reading of +242.1 is gone).

## Working rules for this project

How Gavin wants the work done. These apply to every agent and tool.

- **Frame every task as a subtask of the goal tree** above. Record the objective
  and Gavin's intent behind it (his words where possible) in this file, adding
  dated lines rather than rewriting history. Commit messages say *why*.
- **Never leave progress unsaved.** At milestones, check `git status -sb` and
  unpushed commits on BOTH machines and all related repos (`makermodslab-viscous`,
  `viscous-arm-simulation` a.k.a. `~/Documents/viscous-arm-model`, the lerobot
  forks). Commit AND push.
- **The repo must be self-contained.** A fresh clone must have every URDF, mesh,
  calibration, profile, fit and config it needs. Committed data stores
  repo-relative paths, never `~/Documents/...`. Never commit `frontend/dist`
  (rebuild locally) or multi-GB render output.
- **Heavy binaries go through Git LFS, not git** (2026-11-06). By then this
  branch had added ~1.9 GB to history against `main` (10 versions of the two
  ~100 MB `.blend` files = 0.8 GB, 102 `.mp4` versions = 0.5 GB), every
  version kept forever, and `final_scene.blend` was 96 MiB, just under
  GitHub's 100 MiB per-file hard limit that would refuse the push. The root
  `.gitattributes` now routes `.blend`/`.mp4`/`.mov`/`.webm`/`.npz`/`.exr`
  under `scripts/viscous-dream1/` and `configs/viscous/` to LFS (both machines
  have git-lfs); files already committed convert the next time they change.
  Text run logs (`log.json`, `trajectory.json`, evidence JSON) stay in git:
  they compress well and are what this doc cites, even though they are most of
  T3's "+5.4M lines". Before committing a run, commit the inputs and the
  evidence, not scratch: no Blender backups (`*.blend1`, `*.prelogo.blend` are
  ignored), no render-frame dumps, no candidate/duplicate videos. Never `git
  add` a non-LFS file over ~50 MB. This branch never merges into `main`
  wholesale: land app code through a curated PR.
- **Renders live on gavin-linux (2 TB), never on the MacBook (256 GB)**
  (2026-11-06, Gavin). Render, encode and run Blender on gavin-linux only.
  The Mac clone is deliberately slimmed: a sparse checkout
  (`.git/info/sparse-checkout`) leaves every `.blend`/`.mp4`/`.mov`/`.webm`/
  `.npz`/`.exr` under `scripts/viscous-dream1/` and `configs/viscous/` out of
  its working tree, `lfs.fetchexclude` keeps LFS from downloading them, and it
  is a partial clone (`remote.origin.partialclonefilter=blob:limit=1m`), so
  old versions of big files are fetched from GitHub only if a command needs
  them. Those files missing on the Mac is expected, not breakage: don't
  "restore" them, and don't `git add --sparse` them there. To show one in chat,
  `scp` it from gavin-linux into `/tmp` and delete it afterwards.
- **No AI attribution trailers** (no `Co-Authored-By`) in commits or PRs.
- **Safe tests: run them, don't wait.** Once Gavin has judged a test safe (for
  example the Star gripper actuation check), run it yourself and report the
  results, rather than handing him commands to run (2026-10-21).
- **Talk during a run with Gavin at the arm** (2026-10-27): "next time you
  should not just run a command and not talk back to me". Never launch a
  powered run as one blocking command: run it in the background, watch its
  output, and tell him in chat as each phase starts and what you see. He only
  sees chat, so a silent run leaves him guessing when calibration starts.
- **Powered hardware runs:** say exactly which joints move before energizing.
  Prove the stop path on the actual launch method first; a process started in the
  background ignores SIGINT unless it installs its own handler. Check liveness
  with `kill -0 <pid>`, not `pgrep -f` inside the same ssh command. Gavin's power
  switch is the hard stop. The thermal ceiling is the driver's 85 °C stop;
  scripts abort just below it (82 °C) and cool at 80 °C. On the dry arm keep
  planned loads ≤ ~3.5 Nm (see Results: the shoulder dropped the arm at ~5.6 Nm).

