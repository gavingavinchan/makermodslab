# Independent gripper haptics attempt, 2026-10-28

Goal A3, one joint. Gavin needs to demo gripper contact ASAP: fast closes
give no felt wall, slow closes work, and the resend change felt only slightly
better. This is an independent controller, offline only. No serial port,
hardware script, service restart, deployment or push.

## Diagnosis from recorded data

All figures are CSV measurements, not inferred hand torque.

- Median host period: 21.4 ms (~47 Hz).
- In 225901Z, three contacts first latch 14.8, 14.7, 12.1 Star degrees
  through the wall. At >=590 mW: current median 48 mA, maximum 83 (209 ticks).
- In 234213Z, >=590 mW current is median 46.5 mA, maximum 82 (52 ticks).
  In newest **234727Z**, it rises to **82 mA median, 279 mA maximum**
  (161 ticks). Fewer resends materially increased loaded current. Calling
  all the newest run's effort "40-77 mA" is incorrect. Nevertheless its four
  fast grasps latch 13.9, 12.5, 11.8, 12.1 degrees through their walls.
- At 9.8973 s in 234727Z, Star +2.1 degrees, follower -64.39 degrees,
  velocity +627.48 deg/s, torque 0.31 Nm. At **9.9187 s**, Star already
  **-3.8 degrees**, follower -52.61, velocity **+295.04 deg/s**, torque
  **5.74 Nm**: observable impact while the jaw is moving. The old stillness
  gate waits until **9.9403 s**, then sends only 186 mW. It reaches 600
  at **10.0478 s**, 129.1 ms after observable impact. Current reaches
  220 mA at 10.1122 s, **193.5 ms after impact**.
- Other fast impacts: 13.4440 / 16.5014 / 20.1592 s. Old latches:
  13.4652 / 16.5229 / 20.1803. Old 600 mW:
  13.5712 / 16.6296 / 20.2866.
- 10-27 v2 Run B, >=590 mW: current median 48 mA, max 72; penetration
  median 9.8 degrees, max 11.1. It removed oscillation by slowing actuation,
  but sacrificed impact effort.
- Free fast motion can itself produce 3.1 Nm (10-27 Run A documentation).
  Lag alone is also normal (~60 ms in Run A). Neither proves object contact.

The contact signal comes after the finger passes the object's equivalent
opening; timed trajectories and the power ramp delay load further. A hold
afterwards cannot recover that initial sensation. Firmware trajectory
restarting remains a hypothesis supported partially by the newest current
data; it is not the entire diagnosis.

## Controller and exact constants

1. **Precontact brake, 0x12 damping.** Star closing >=100 deg/s, lag >6
   Star degrees (about 10.7 follower degrees): P=clip(200+40*(lag-6),
   200,400) mW. Lag hysteresis 6 on / 4 off. Release when opening or speed
   drops below 100. This absorbs motion without inventing an object wall.
   It deliberately adds drag to fast EMPTY closes; slow free motion stays
   released. BRAKE_CLOSE_DEG_S=100, BRAKE_ON_LAG_DEG=6,
   BRAKE_OFF_LAG_DEG=4, BRAKE_MIN_MW=200, BRAKE_MAX_MW=400,
   BRAKE_MW_PER_DEG=40.
2. **Earlier impact detection.** Retain calibrated still-jaw detection;
   additionally detect squeeze >=max(3.5 Nm, 2*threshold) plus closing jaw
   velocity dropping >=80 deg/s since the previous tick and >0.5 Star
   degree penetration beyond measured jaws. Acceleration alone fails this
   test. IMPACT_NM=3.5, IMPACT_DROP_DEG_S=80.
3. **Fixed unilateral wall.** Freeze the measured contact position, rather
   than following a compressing jaw. Target its mapped Star position,
   bounded by calibrated travel (1 degree margin plus 0.1 inset before
   rounding). **MOVE_MS=1**, basic angle command 0x08. This requests an
   immediate internal setpoint; physical motion is still servo/effort limited.
4. **No power slew.** With penetration p in Star degrees and closing speed v
   in Star degrees/s: P=round(clip(200+100*p+3*v,0,800)) mW. Opening makes v
   negative and subtracts effort. Full effort at 6 degrees static penetration
   or sufficient closing speed. MIN_MW=200 is a bias, not a lower clamp;
   MW_PER_DEG=100, MW_PER_DEG_S=3, **MAX_MW=800**. At/outside the wall,
   release. Zero P always sends 0x10 release, never the ceiling encoding.
   ON_DEG=0.5; already-active wall stays eligible down to zero penetration.
5. RESEND_MW=25 relative to last SENT power; resend when mode/target changes.
   A fixed wall and short trajectory avoid host-scale interpolation even
   when changing power on consecutive ticks.
6. Follower closing lead: MAX_LEAD_DEG=8 still, MOVING_LEAD_DEG=12 moving,
   STILL_DEG_S=40. Opening is unrestricted within the normal travel limits;
   calibration is uncapped to avoid the startup baseline failure.
   At kp20 these position terms are ~2.8 / 4.2 Nm; inertia and kd mean these
   are NOT guaranteed torque caps.
7. RELEASE_DEG=3, SQUEEZE_GONE_S=0.3, squeeze refresh at 0.3*threshold.
   CLOSED_STOP_IGNORE_DEG=4 follower degrees for both contact detectors.
8. CALIB_S=8, CALIB_TRAVEL_DEG=20, CALIB_STILL_TICKS=25,
   BASELINE_ERR_DEG=3. Baseline=median still, tracking torque;
   threshold=clip(1.5*p95 absolute deviation+0.1,0.5,1.5) Nm.
9. STAR_TEMP_LIMIT_C=55, FOLLOWER_TEMP_LIMIT_C=82,
   FEEDBACK_FRESH_S=0.1 (also maximum host loop gap).
   LOSS_THRESHOLD=20 for ordinary raw-leader servo read loss; haptics
   disables on the FIRST stale gripper sample.
   STAR_CLOSE=-1; only gripper ID 6; finite nonzero mapping required.

0x11 hold captures CURRENT Star position, already at its closed stop after a
fast close; it cannot move its anchor to the cube. Damping alone has no
static wall. A long move delays effort. Thus damping before contact and
short position control after it.

## Power evidence and remaining uncertainty

**New cap: 800 mW.** The recorded
[10-21 actuation check](viscous-haptics.md#star-actuation-check-2026-10-21-goal-a2-next-step-3)
tested 100/200/400/800 mW: stalled currents 77/79/92/**136 mA**, temperature
29–31 C, release returned all seven to 30 mA. The 50 ms 800 mW test showed
80–100 deg/s and 181 mA stop overshoot. Source files in
configs/viscous/viscous_01/evidence:
star-actuation-20261021T051835.266732Z.json.gz and
star-actuation-20261021T052114.957006Z.json.
10-27 v1 reached 738 mW under a hand, with up to 660 mA and oscillation.
This chooses a previously exercised setting range; it does NOT establish a
calibrated or universally safe hand-force bound. The maximum safe hand
force/current remains unknown.

Expected feel: drag on rapid free closure, earlier firmer object resistance,
lighter effort when easing open. Watch for an opening shove, rebound/hunting
from short trajectories, or brake chatter near 100 deg/s. Velocity feedback
reduces recoil but is not a passivity proof. A 1 ms duration and damping
under a hand are not characterized on this firmware. A moving object can
leave the frozen wall inaccurate until lost-squeeze timeout or opening
past it. Bus loss/SIGKILL can leave the last effort active: physical power
remains the hard stop.

A perfectly stiff first fast contact is not promised with ~60 ms follower
lag, ~21 ms sampling and an effort-limited hobby servo. The brake trades
rapid free-motion transparency for time to render contact.

## Verification and handoff

Committed input: configs/viscous/viscous_01/evidence/
gripper-haptics-20261028T234727Z.csv.gz. The full-trace test feeds recorded
measurements through fake buses and injected monotonic/wall clocks. On the
representative grasp, damping begins at **9.8123 s**, **106.4 ms before
observable impact**, and the wall requests **800 mW at 9.9187 s**, 129 ms
before the old command reached 600 mW. Other fast grasps likewise get their
wall one tick before the old controller latched. The whole newest trace
replays without a fault. These are command decisions on unchanged input,
NOT a simulation or proof of resulting force, penetration or feel.
Physical feedback will change the trace.

27 focused tests pass, no sleeps. Coverage includes early braking/impact,
acceleration rejection, fixed wall, opening effort reduction, lost contact,
thermal/stale/non-finite feedback, loop gaps, partial writes, failed-release
retry and release-all-seven in a worker. Ruff check and format pass.

Public controller/leader/status interfaces and teleoperate.py wiring remain.
CSV appends mode (brake/wall) and star_close_deg_s to the original columns.
Only ID 6 energizes; no motorbridge/raw coexistence. Disconnect releases all
seven twice, never calls RawBus.close(). Pending effort is marked BEFORE
writing and cleared only AFTER successful release, so partial writes and
failed releases receive cleanup retries. Freshness/timing faults disable
haptics until a new session rather than automatically re-energizing.

No powered validation. Claude deploys from the independent worktree branch
for Gavin to judge the feel and the explicit fast-empty-close drag tradeoff.
