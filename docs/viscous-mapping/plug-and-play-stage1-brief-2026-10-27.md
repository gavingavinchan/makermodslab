# Brief: build plug-and-play stage 1 (offline, no hardware)

Implement stage 1 of `docs/viscous-mapping/plug-and-play-design-2026-10-27.md`
(Codex's design), as amended by Claude's review below. Read the design, the
brief it answered (`plug-and-play-design-brief-2026-10-27.md`, including
"Gavin's decision after this brief went out"), `CLAUDE.md`, and
`frontend/docs/localization.md` before you start.

## Where to work

You are in a dedicated git worktree on branch `feat/viscous-plug-and-play`
(branched from `feat/viscous-arm`). Work and commit only here. Set up its own
environment first: `uv venv && uv sync --extra test --extra arm-dynamics`
(the lerobot git pin is cached; do not change the pin), and `npm ci` in
`frontend/` (never edit or regenerate `frontend/package-lock.json`). Record the
baseline of `pytest -q`, `ruff check`, `npm run lint`, and both
`npx tsc --noEmit -p tsconfig.app.json` / `-p tsconfig.node.json` BEFORE you change
anything, and compare against it at the end; do not fix pre-existing failures.

Hard rules: no hardware. Do not open any serial or CAN port, run no script that
talks to an arm, touch nothing under `~/.makermods`, `~/.cache/huggingface`, or
`/dev`, run nothing over ssh, restart no service. Do not push. Do not commit
`frontend/dist`. Commit in small logical commits with messages that say WHY;
no `Co-Authored-By` or any AI attribution trailer. Copyright header for new
files: `# Copyright 2026 MakerMods. All rights reserved.` + Apache boilerplate.

## Claude's amendments to the design (these override it)

1. **Identity mismatch WARNS with a one-start override; it does not refuse.**
   Gavin: "identifying the arm is good but let me override it with a warning".
   When the record names one unit but the adapter found belongs to another
   unit, or the adapter is not enrolled to any unit, the start returns a typed
   error (`robot.unit_mismatch` / `robot.identity_unverified`-style codes per
   the taxonomy rules in `tests/test_api_errors.py`) carrying what was found
   and which unit's settings would be used. The UI shows that as a warning
   with a "Continue anyway" action that re-sends the start with an explicit
   override flag for THAT start only (never persisted). Hard refusals remain
   only for physical faults: a motor not answering, ambiguous adapters, port
   busy.
2. **No "passive query qualification" gate in this stage.** Today's teleop
   already sends RobStride's 0xFB state read on every connect. Stage 1 must not
   add NEW kinds of frames, but may keep using the existing connect path.
   Motor-UID identity (design section 2) is a later stage; leave a clear seam
   for it.
3. **Auto-wait only for the RobStride (Viscous/Maker) follower and the Star
   leader, never Damiao.** Retrying RobStride `connect()` after a handshake in
   which no motor answered is what Gavin does by hand today (it worked within
   5-60 s every time). Verify in the installed fork
   (`.venv/lib/python3.12/site-packages/lerobot/robots/maker_follower/`) that a
   failed handshake enables nothing before relying on this; if any partial path
   can enable, run the existing `de_energize_can_device` before each retry or
   don't retry that case. A partial reply (some motors answer, some don't) is
   NOT retried: that is a physical fault, report it.
4. **Release tooling is out of scope** (design stage 6 is overbuilt for two
   machines). So are the record/replay/rollout/hosting bypasses (stage 5), the
   unit-1 profile (stage 2, needs Gavin at the arm), and moving the stale-zero
   refusal into the lerobot fork.

## Stage 1 scope

A. **Find the adapters on every Teleop press**, for the Viscous family (and
   wherever the code is shared, Maker too, without changing SO-101/Metal
   behaviour). Re-enumerate USB descriptors at each start; locate the CANable by
   USB serial and the Star's CH340 as the single candidate of its model; never
   open a stale saved path when the device is elsewhere; works for records
   with no `port_identities` (Linux records today) and with macOS tty/cu and
   Linux by-id aliases. Descriptor-only; no probing of unknown ports.
B. **Committed adapter enrollment per unit**: add the adapter serials to the
   unit manifests (`configs/viscous/viscous_01/`, `configs/viscous/viscous_waterproof_01/`;
   dry CANable `207E37A55445`, waterproof CANable `205038953846`; verify these
   against the committed state snapshots under
   `configs/viscous/state-snapshots/2026-10-27/`). Reuse the existing manifest
   files; if unit 1 has none, create the minimal one the loader needs. The
   Lab maps record -> unit by the record's follower calibration/profile name
   (check `arms/viscous.py`), adapter -> unit by the manifest, and applies
   amendment 1 on disagreement. The Linux waterproof-record-on-dry-adapter
   case must produce the warning, not a silent start.
C. **Typed failures at the source with one plain sentence each**: use the
   design's section 5 table for the cases in this stage (USB missing either
   side, follower not ready, leader not ready, partial feedback, ambiguous
   adapters, port busy, unit mismatch/unverified, unknown). Keep the technical
   detail in `details`, not appended to the sentence. Follow the `ApiError`
   conventions and the error-code grammar tests.
D. **Bounded auto-wait** (amendment 3): up to ~60 s, cancellable by Stop, with
   a visible status ("Waiting for the arm to power up..." / the Star's
   equivalent) published through the session phase seam so the UI can show it.
   Today `handle_start_teleoperation` connects inside the start request
   (`teleoperate.py:773-815`, thread started ~1179): pick the smallest correct
   way to keep the lease/heartbeat and Stop working during the wait, and say
   why in the commit. Inject the clock/sleep so tests never sleep.
E. **Frontend**: show the plain sentence prominently, details behind a
   disclosure, the waiting status while it waits, and the mismatch warning with
   "Continue anyway". i18n for every new client-side string in every catalog
   (catalog-parity test). Use `ApiError.code`, never parse backend prose.
F. **Fake-hardware tests** (design section 6 rows that apply): socket swap and
   renamed paths, old record without metadata, one vs two CH340s, Linux
   waterproof-record-on-dry-adapter warning + override, unenrolled adapter,
   follower answering at 5 s / 60 s / never with a fake clock, partial reply
   not retried, Star late, Stop during the wait, no Damiao retry, exact codes.
   Follow `CLAUDE.md` test policy (no sleeps, no new dependencies, no
   subprocess/thread happy paths).
G. **CI**: make `tests.yml`, `quality.yml` and `frontend.yml` also run on
   pushes to `feat/viscous-arm`, and widen `tests.yml` path filters to include
   `configs/**`, `scripts/**`, `uv.lock`.
H. Regenerate `docs/api/openapi.json` (`uv run python -m makermodslab.scripts.export_openapi`)
   and update ratchet lists only as the contract tests require for new typed
   surface.

## Report

Write `docs/viscous-mapping/plug-and-play-stage1-report-2026-10-27.md` (commit it
too): what was built, per scope item; the baseline vs final check results with
exact commands; what you verified in the fork about failed-handshake safety
(file:line); anything you deliberately left out and why; anything that needs
Gavin or a powered test. Final message: ≤15 lines.
