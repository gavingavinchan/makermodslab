# Viscous joint mapping keeps going wrong: problem note for a fresh session

Written 2026-10-27, at the end of the gripper haptics session. Read this first,
then `docs/viscous-haptics.md` (goal tree; "Gripper haptics proof of concept"),
then the two investigation reports below. Nothing in this note has been changed
in code or config yet: Gavin wants it solved properly, once, in a new session.

## LIVE HAZARD: deal with this first

On gavin-linux the `viscous_waterproof_01` robot record points at **unit 1's**
CAN adapter (`...207E37A55445`) and Star, the same ports as `viscous_01`
(`configs/viscous/state-snapshots/2026-10-27/gavin-linux/.makermods/makermodslab-viscous/robots/`,
line 11 of each). Starting any session with that record on gavin-linux drives
the DRY arm with the waterproof unit's limits and mapping, e.g. shoulder_pan
allowed -126..187 against unit 1's -185..127. Not changed yet: ask Gavin, then
fix the record or add an adapter-identity check before anything else.

## Gavin's request

"see how we can update all the teleop or urdf or whatever for this dry arm so
this does not keep happening. it this is a recurring problem."
Then: "leave a note about this problem and then I'll ask a fresh new session to
see how we can solve this once and for all so that our workflow is working
properly."

So the goal is not today's gripper number. It is: every physical Viscous unit
has ONE committed, measured description of how its leader maps to its follower
(and to the URDF/sim), every consumer uses it, and drift is caught before an
arm moves.

## What happened today (unit 1 `viscous_01`, dry arm, gavin-linux)

1. In the gripper haptics runs Gavin saw "the follower arm did not move the
   full range".
2. Measured by hand, torque off (`scripts/gripper-range-read.py`,
   `configs/viscous/viscous_01/evidence/gripper-range-20261027T002654.json`):

   | | measured | configured |
   |---|---|---|
   | follower gripper closed stop | -32.6 deg | soft limit -2.8 |
   | follower gripper open stop | -156 deg | soft limit -120 |
   | follower width | 123 deg | 117 + 2 x 2.9 back-off = 123 |
   | Star gripper travel (raw) | -3.1..+60.6 | preset assumes -16.9..+48.9 |

   The soft-limit window has the right width but sits ~35 deg off: the
   follower gripper's zero has moved since its limits were captured
   (2026-08-28). Closing is reachable (the Star preset's -33 offset matches the
   closed stop). The last ~36 deg of OPENING is cut off by the -120 soft limit.
   The preset scale (-1.778) maps the Star's 64 deg onto 113, not 123 (fit:
   follower ~= -1.936 x Star - 38.6). A follower powered up fully open (-156)
   would sit 36 deg outside its limits, past `MakerFollower`'s 20 deg wrap
   grace, and be refused as a stale zero.
3. Also seen: after a power cycle the follower gripper read +327.5 raw (whole
   turn reset; `MakerFollower` and the haptics script correct it by -360).
4. The follower config comment says joint zero = "gripper open"; the data says
   -33 is closed and -156 open. One of them is stale.

## Why it recurs (both investigations agree on the shape)

- **No single owner.** Unit 1 has NO commissioned profile
  (`<MAKERMODSLAB_HOME>/viscous_profiles/` is absent for it on both machines);
  it runs on the lerobot fork's built-in defaults. The waterproof unit has a
  profile, but the Mac copy and the gavin-linux/bundle copy DIFFER (gripper
  -115.1..-0.03 on the Mac vs -115.1..-2.9 elsewhere).
- **Several independent interpretations:** the lerobot Star preset
  (scale x raw + offset), Lab's 1:1 posture mapping (waterproof only), the
  haptics scripts (`gripper-haptics-poc.py`, `star_follow.py`), the sim repo's
  draft mapping profiles (`viscous-arm-model/leader/mapping/profiles/`), the
  URDF (CAD limits), and the gravity model (motor degrees straight to radians).
- **Overloaded fields:** `joint_directions` carries sign AND scale;
  `joint_ranges` is both the follower clamp and the leader's unwrap window, and
  the haptics code inverts it as if it were measured Star travel.
- **Zeros move and limits don't follow:** today's gripper (~35 deg), and the
  waterproof J1 re-zero of 94.5 deg on 2026-10-11 whose saved limits were
  deliberately left unchanged.
- **Nothing checks measured reality against config** before an arm moves: the
  pairing check compares configured bounds only, and Lab connects the Star with
  `calibrate=False`, skipping its start-pose check.

## Investigation reports (read-only, 2026-10-27)

- `codex-report-2026-10-27.md`: Codex gpt-6.1-sol, high effort. Full per-joint
  inventory for both units, runtime precedence, root cause, a proposed
  canonical per-unit profile schema (`configs/viscous/<unit>/viscous_profiles/<unit>.json`,
  measured endpoints instead of scale x offset, one shared mapping library for
  teleop/haptics/sim, offline tests, pre-connect refusal), gripper endpoint
  mapping, risks. It did NOT have today's gripper measurement (it assumed the
  open end is -120; it is -156).
- `claude-subagent-report-2026-10-27.md`: the same brief, run independently by
  a Claude subagent that did get the measurement. Adds:
  - the frame shift happened between 08-28 and 09-10 (stop-to-stop ~33 deg; a
    slipped gripper coupling is the fork docs' usual cause), and the 09-10 fix
    was the leader's -33 offset (fork `058abeb9d`), leaving the open-side limit
    stale;
  - the calibration JSONs' ranges are inert decoys (RobStride ignores them, the
    leader clamps with `config.joint_ranges`);
  - the waterproof directions were computed from unit 1's 08-29 Star capture;
  - unit 1 reads wrist_yaw -13..-15 and wrist_roll -4..-6 when "folded": other
    joints may have shifted too;
  - the identity hazard above;
  - the today's measurement caveat: the Star range (-3.1..60.6) came from the
    follower phase, not the Star phase, so re-measure it.
  It proposes `configs/viscous/<unit>/unit.json` (Codex: extend
  `viscous_profiles/<unit>.json`); both are Lab-only, drop the preset fallback,
  refuse to start on a missing/mismatched file, and add drift tests plus a
  read-only startup check.
- `investigation-brief-2026-10-27.md`: the brief both were given.
- `configs/viscous/state-snapshots/2026-10-27/`: both machines' robot records,
  `viscous_profiles` and lerobot calibration files as they were today.

## Where the two reports agree and differ

Agree: no per-unit owner is the root cause; one committed, measured file per
physical unit; store measured ENDPOINTS, not scale x offset; Lab-only change
(fork optional); drop the silent preset fallback; refuse on missing/drifted
files; one shared mapping used by teleop, haptics scripts and sim; tests plus a
read-only pre-motion check; gripper mapped closed->closed, open->open.
Differ: file name/shape (`unit.json` with identity, stops, rest pose and
camera rotation vs extending `viscous_profiles`); the subagent adds an
adapter-serial identity check and rest-pose drift check at startup and a
lint-style sweep against unit literals in code; Codex adds profile/frame
hashes in datasets and fits, and explicit follower limits on the inference CLI
(rollout passes only type/port/id today).

## Open questions for the new session (ask Gavin, don't guess)

- The live hazard above: fix the record, or add the identity check?

- Which end is "before": did normal teleop ever open the jaws fully? (Would
  date when the gripper zero moved.)
- Fix the follower gripper by re-zeroing it to its original pose, or by
  re-capturing limits around the current zero? Re-zeroing invalidates every
  existing gripper value; re-capturing keeps datasets consistent.
- Same question for the other six joints of unit 1: re-measure all of them
  (stops + matched leader/follower poses), not just the gripper. Codex's table
  shows travel-width scaling also misaligns endpoints on shoulder_pan.
- Where the canonical profile lives and how installed copies are kept in sync
  (Codex proposes repo-canonical, installed copies checksummed).
- Does the sim repo consume the same profile, or a generated copy?

## Side finding: the "read-only" RobStride read is not feel-neutral

The Lab's RobStride state read sends a clear-fault command (0xFB) and decodes
the reply. Polling it at 20 Hz on a torque-off gripper made it feel "not
smooth in a weird way" to Gavin's hand; with no polling it was smooth again.
One observation, not yet A/B-confirmed (plan: 10 s no polling / 10 s at 20 Hz
/ 10 s at 2 Hz, hand on the jaws, read-only). It matters for any flow that
polls a torque-off RobStride arm moved by hand: step calibration, motion
identification, hand-guided measurements, and a measurement procedure the new
session may write.

## Rules that apply (from docs/viscous-haptics.md, "Working rules")

- Discuss the proposal with Gavin before changing teleop config.
- Powered runs: say which joints move; run in the background and talk to Gavin
  through each phase (he only sees chat).
- The repo must be self-contained: measured profiles and evidence are committed
  with repo-relative paths. Commit and push on both machines.
- Never commit `frontend/dist`.

## What waits on this

Gripper haptics Run B (the cube with v2's rendered wall) is on hold: its wall
position comes from the gripper mapping. Resume it from "Gripper haptics proof
of concept" in docs/viscous-haptics.md once the gripper mapping is measured and
committed.
