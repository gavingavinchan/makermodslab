# Brief: design "plug in and teleop just works" for the Viscous arms (read-only)

You are writing a DESIGN, not code. Write exactly one file, your design doc at
`docs/viscous-mapping/plug-and-play-design-2026-10-27.md`. Change nothing else:
no code edits, no commits, no service restarts, no hardware commands of any kind
(no opening CAN or serial ports, not even a "read-only" probe: on RobStride the
state read sends a clear-fault frame). Reading code, docs, logs, `git`, and
read-only `ssh gavin-linux '...'` (journalctl, ls /dev/serial/by-id, cat records)
is fine.

## Gavin's requirement (his words, 2026-10-27)

"For me the most important thing is when I deploy it, it works ... I am running
around going on visits, on travels, to show this to people. Last thing I want is
to go there, press teleop and it doesn't work for whatever stupid reason" and
then have to start an AI agent to debug in front of a guest.

"I don't want to do rehearsal and this kind of stuff ... If I go anywhere, I plug
in, it just works ... each time I plug in stuff, I might switch the USB ports
around ... these things should just work ... why should a rehearsal be needed?"

So: no pre-trip rehearsal, no manual port selection, no checklist. Plugging the
arm(s) into any USB sockets of the laptop and pressing Teleop must work, or show
one plain sentence that a non-engineer can act on in seconds ("Turn on the
arm's power switch"). The testing burden is ours (automatic tests, bench checks
by agents on gavin-linux), never Gavin's.

## Read first

- `CLAUDE.md` (repo conventions; arms registry; mutex/session rules).
- `docs/viscous-mapping/README.md`, `workflow-review-2026-10-27.md` (Codex),
  `fable-workflow-review-2026-10-27.md` (Fable): the three earlier reviews. Fable's
  log analysis lists the real causes of "teleop failed to start": (a) a joint
  parked past a soft limit (fixed in d28d5fc4; unit 1's gripper is the remaining
  case: its limits sit ~35 deg off its real stops), (b) arm not powered/ready at
  the click (CAN handshake: no reply from all 7; a retry 5-60 s later always
  worked), (c) macOS port-path drift on the Star leader's CH340 adapter, which has
  no USB serial number. All three surface as one generic error sentence
  (`teleoperate.py` ~783).
- `docs/viscous-haptics.md` "Handoff" and "Working rules".
- Code: `makermodslab/usb_ports.py`, `makermodslab/utils/config.py`
  (`get_robot_record`, `capture_identities`), `makermodslab/arms/viscous.py`,
  `makermodslab/arms/can_common.py`, `makermodslab/teleoperate.py` connect path,
  `makermodslab/maker_ports.py` (existing CAN/Star probes),
  `makermodslab/can_speed_limit.py`, the installed lerobot fork's
  `maker_follower` / viscous follower and Star leader
  (`.venv/lib/python3.12/site-packages/lerobot/...`), `makermodslab/fashionstar_raw.py`.

## Questions the design must answer (with code/log evidence)

1. **Find the hardware wherever it is plugged.** How the Lab should locate the
   follower's CAN adapter and the Star leader on every connect regardless of
   socket or OS path, on macOS and Linux. The CANable has a USB serial; the CH340
   does not. What is safe to probe, and what is not (Damiao handshake energizes;
   RobStride clear-fault read; FashionStar ping)?
2. **Know WHICH arm is plugged in.** An adapter serial identifies an adapter, not
   an arm. Can the arm itself be identified passively: e.g. RobStride's
   device-ID/MCU-ID query (communication type 0), FashionStar servo IDs or a
   user/EEPROM field? Is each such query truly passive (no enable, no fault
   clear, no motion)? Cite the protocol code/docs. If the arm cannot be
   identified, what is the safe fallback (e.g. exactly one candidate allowed)?
   Today on gavin-linux the waterproof record points at unit 1's adapter: the
   design must make that state impossible to act on.
3. **Settings travel with the arm.** Each physical unit gets one committed,
   measured profile (Codex/Fable agree; reuse the existing
   `configs/viscous/<unit>/manifest.json` + restore machinery and
   `ViscousFamily._commissioned_config`); the Lab picks it by the identified
   arm, not by which record or port was chosen. Note the Mac's live waterproof
   profile is newer than the committed one; say how to reconcile once.
4. **Handle transient "not ready".** Auto-wait/retry on the CAN handshake and Star
   ping with a bounded timeout and a live status ("Waiting for the arm to power
   up..."), without ever leaving motors energized on a partial handshake.
5. **Self-check on every Teleop press** (this replaces the rehearsal): what is
   checked before torque (identity, fresh feedback from every motor, positions
   consistent with the profile's measured stops / rest pose, Star answering),
   how long it takes, and the exact plain message for each failure.
6. **Plain messages in the UI.** Map each failure class to one actionable
   sentence; keep the technical detail available but secondary. Respect the
   i18n rules in `frontend/docs/localization.md`.
7. **How we prove it without Gavin.** A simulated-hardware test harness (fake
   serial enumeration, fake CAN bus) that reproduces each logged failure:
   ports swapped, OS path renamed, arm unpowered then powered, Star slow to
   answer, wrong arm on a record's adapter, joint outside limits, unit 1's
   gripper case. Which tests must run in CI (note CI does not run on
   `feat/viscous-arm` pushes today, and path filters skip `configs/**`,
   `scripts/**`). Plus an agent-run bench check on gavin-linux that needs no
   human (what can be safely exercised with nobody at the arm).
8. **Keeping both machines on the known-good version** without Gavin doing
   anything: how the travel laptop ends up on a version that passed the above
   (e.g. tagged release, auto-update only to tagged versions, version shown in
   the UI). Keep it small: one operator, two machines.
9. **What cannot be automated** and must be done once by Gavin (e.g. measuring
   unit 1's gripper stops, deciding re-zero vs recapture). List it explicitly.

## Deliverable

`docs/viscous-mapping/plug-and-play-design-2026-10-27.md`, under ~200 lines:
the answers above with evidence (file:line, command + key output); then an
implementation plan in small stages, each with its tests and the files it
touches, ordered so the stages that remove the logged failures land first;
then risks and open questions for Gavin. Plain language in the summary at the
top (Gavin is not a software engineer). Final message: a ≤15-line summary.

## Gavin's decision after this brief went out (2026-10-27)

"identifying the arm is good but let me override it with a warning i guess.
cuz im not sure if that measuring and identifying is reliable"

So an arm-identity mismatch (or an arm that cannot be identified) must WARN and
offer an explicit override, not hard-block. The warning names what was found
and which arm's settings will be used if he continues. The override applies to
that one start only, never remembered. Hard refusals stay only where motion
would be unsafe regardless of identity (no feedback from a motor, a partial
handshake).
