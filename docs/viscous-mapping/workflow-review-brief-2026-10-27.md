# Brief: review a proposed "fixes stay fixed" workflow (read-only)

You are reviewing a proposal, not implementing it. Change NO files except your
report, run NO hardware, start/stop/restart NO services, and send no commands
to any arm or CAN/serial port. Reading files, `git`, `systemctl --user show/cat`,
and read-only `ssh gavin-linux '...'` commands are fine.

## Context

Repo: this checkout (`~/Documents/makermodslab-viscous`, branch `feat/viscous-arm`),
MakerMods Lab, driving Viscous arms. Two machines: the MacBook (this one,
waterproof arm `viscous_waterproof_01`) and `gavin-linux` (ssh alias, same repo
path, dry unit 1 `viscous_01`, Lab runs as the systemd user service
`makermodslab-viscous`). Read first: `docs/viscous-mapping/README.md` (the
recurring joint-mapping problem) and the "Handoff" and "Working rules" sections
of `docs/viscous-haptics.md`. Also `CLAUDE.md` for repo conventions.

Gavin (the owner, not a software engineer) says: bugs keep recurring, e.g. the
follower gripper not opening fully when the leader opens, and teleop failing to
start. He wants a standard workflow so a fix stays fixed and reaches both
machines.

## Claims Claude made (verify each independently; say if any is wrong)

1. Both machines are on the same commit (`bfa73b70`), but the gavin-linux service
   has been running since 2026-10-22 17:34 CDT, while the backend fix
   `d28d5fc4` ("Let teleop start with a joint outside its limits and ease it
   inside") was committed 2026-10-22 17:58. uvicorn runs without `--reload`, so
   the running Linux Lab does not have that fix.
2. The arm-defining state (robot records, `viscous_profiles`, lerobot
   calibration JSONs) lives outside git (`~/.makermods/makermodslab-viscous`,
   `~/.cache/huggingface/lerobot/calibration`) and differs between machines.
   Systemd drop-ins (e.g. `gravity-ff.conf`) are also machine-local and untracked.
3. No GitHub Actions workflow runs on pushes to `feat/viscous-arm` (they trigger
   on PRs to main/staging or pushes to main/staging).
4. Nothing compares measured joint positions to configured limits/mapping before
   an arm moves, and nothing checks the CAN adapter identity against the record.

## Proposed workflow (critique it)

1. One-command deploy per machine: pull, `uv sync` from the lockfile, restart the
   Lab, run a quick check; the Lab reports the commit it is running (e.g. in
   `/api/v1/health`).
2. Enable CI (tests) on pushes to `feat/viscous-arm`; rule: no fix lands
   without a regression test that would have caught it.
3. One committed, measured settings file per physical arm unit (single source
   of truth); machines use it, and the Lab refuses to start if a machine's copy
   is missing or differs. (This is the mapping note's fix; it needs Gavin's
   decisions on the open questions there.)
4. Read-only preflight before teleop energizes anything: right adapter on the
   right port, live positions consistent with the committed profile; refuse with
   a clear message otherwise.
5. A short bug log: symptom, root cause, the test now guarding it.

Proposed order: 1, 2, then 3 (after Gavin's answers), then 4.

## What to report

Write a concise report (aim for under ~150 lines):
- Each claim: confirmed / wrong / partly, with the evidence (command + output).
- Is the proposal sound for a two-machine, hardware-in-the-loop solo setup?
  What is missing, overbuilt, or in the wrong order? Concrete alternatives.
- Specific risks (e.g. CI cannot exercise hardware; restarting the service mid-
  session; uv sync of a git-pinned lerobot; machine-specific state that
  legitimately differs, such as port paths).
- Anything else you notice that would make bugs recur that the proposal does not
  address (e.g. scripts outside `makermodslab/` that re-implement mappings).
- Your recommended first concrete step.
