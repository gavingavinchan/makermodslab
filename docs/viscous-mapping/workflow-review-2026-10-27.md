# Review: making Viscous fixes stay fixed — 2026-10-27

Read-only review for goal P (repeatable work) supporting A3 (gripper haptics).
Read CLAUDE.md, the mapping README, and the Handoff/Working rules in viscous-haptics.md.
Only this report was written. No tests, hardware commands, installs, service changes, commits or pushes were run.
Existing dirty frontend/dist files and untracked work on both machines were left alone.

## Verdict

The proposal is sound with three changes: contain the wrong-unit record first; distinguish deployed code from git history; and enforce arm configuration at motion entry points rather than preventing the whole Lab from starting.
For two machines and one operator, a small deployment script, per-unit manifest, shared mapping loader and short acceptance checklist are enough. A deployment platform or duplicated configuration framework would be overbuilt.

## Claims checked independently

### 1. Same commit, old Linux process, therefore missing fix — PARTLY; conclusion unproven

Commands (git commands run in each checkout; service/stat commands on Linux):

```text
git log -1 --format='%H %cI %s'
Mac and Linux: 73ebe8874bf5e4c0bdd244bcb1978eae00eff592 2026-10-27T00:45:28-05:00
git show -s --format='%H %cI %s' d28d5fc4
d28d5fc40573dad53881cd7ef103b94636c430fe 2026-10-22T17:58:04-05:00 Let teleop start with a joint outside its limits and ease it inside
systemctl --user show makermodslab-viscous -p MainPID -p ExecMainStartTimestamp -p ExecStart
MainPID=107095
ExecMainStartTimestamp=Thu 2026-10-22 17:34:42 CDT
ExecStart=.../.venv/bin/python -m uvicorn makermodslab.server:app --host 100.64.91.4 --port 8000
stat -c '%y %n' makermodslab/can_speed_limit.py
2026-10-22 17:34:31.903406470 -0500 makermodslab/can_speed_limit.py
git diff --quiet HEAD -- makermodslab/can_speed_limit.py
exit 0
```

Both HEADs match, but are now **73ebe887**, not bfa73b70. Service timing and absence of `--reload` are confirmed.
Crucially, the fixed file's mtime precedes startup by 11 seconds: the fix could have been running as an uncommitted edit before its commit. Commit time is not deployment time; mtime is supporting evidence, not proof of loaded bytes.
Also `rg -n can_speed_limit makermodslab/teleoperate.py` gives `757: from .can_speed_limit import install`: this module is imported lazily during connection. Process age alone cannot identify its loaded version. Treat runtime revision as unknown, not demonstrably missing d28d5fc4. Adding reload would be inappropriate around live sessions.

### 2. Machine-local arm state and drop-ins differ — CONFIRMED, with committed-backup caveat

Live files are outside the checkout. Reproducible file-only scan, run as `python3 -` locally and `ssh gavin-linux python3 -` remotely, with this stdin:

```python
from pathlib import Path
import json, hashlib
b = Path.home()/'.makermods/makermodslab-viscous'
for p in sorted((b/'robots').glob('*viscous*.json')):
    x = json.loads(p.read_text()); print(p.name, x['follower_port'], x.get('port_identities'))
p = b/'viscous_profiles/viscous_waterproof_01.json'
print(json.loads(p.read_text())['follower']['joint_limits']['gripper'])
c = Path.home()/'.cache/huggingface/lerobot/calibration/robots/viscous_follower/viscous_waterproof_01.json'
print(hashlib.sha256(c.read_bytes()).hexdigest()[:16])
print((b/'viscous_profiles/viscous_01.json').exists())
```

Observed outputs (port paths shortened here; hash prefixes are byte hashes):

| Item | Mac | Linux |
|---|---|---|
| Dry record CAN adapter | /dev/cu.usbmodem207E37A554451 | /dev/serial/by-id/...207E37A55445-if00 |
| Waterproof record CAN adapter | /dev/cu.usbmodem2050389538461 | /dev/serial/by-id/...207E37A55445-if00 |
| Saved follower USB serials | dry 207E37A55445; waterproof 205038953846 | no port_identities in either record |
| Waterproof gripper limits | [-115.10722104178238, -0.03437746770784939] | [-115.10722104178238, -2.9220847551671985] |
| Waterproof follower calibration hash | d79074b685bb7bbd | c95c6a9d184cdf16 |
| Unit 1 profile exists | False | False |

The profile difference is semantic; a calibration byte-hash difference alone does not establish a meaningful calibration difference.
`systemctl --user cat makermodslab-viscous` lists `/home/gavin/.config/systemd/user/makermodslab-viscous.service.d/gravity-ff.conf`, setting `MAKERMODSLAB_GRAVITY_FF=.../configs/viscous/viscous_01/gravity-air-fit.json` and `MAKERMODSLAB_GRAVITY_FF_SCALE=0.5`.
The active unit/drop-in are machine-local; `scripts/makermodslab-viscous.service` is a committed template.
Likewise, `git ls-files configs/viscous/viscous_waterproof_01/manifest.json configs/viscous/viscous_waterproof_01/viscous_profiles configs/viscous/viscous_01/calibration` lists committed manifests/profiles/calibrations. The gap is runtime authority and drift enforcement, not absence of all versioned state.

### 3. No workflow triggered by feature-branch pushes — CONFIRMED for checked-in definitions

`rg -n '^on:|^  (push|pull_request|workflow_dispatch|workflow_run):|branches:|main|staging' .github/workflows/*.yml` shows:

```text
tests.yml: push branches main, staging; pull_request branches main, staging; workflow_dispatch
quality.yml: push/pull_request branches main, staging; workflow_dispatch
frontend.yml: pull_request branches main, staging; workflow_dispatch
build_frontend.yml: push branches [main, staging]; workflow_dispatch
lockfile_registry_guard.yml: push branches [main, staging]; pull_request (path-filtered, no base restriction)
sync_staging.yml: push/workflow_run branches [main]; workflow_dispatch
```

A PR from this branch can trigger checks; a direct push alone does not. The blanket description of all PR triggers was slightly inaccurate. GitHub run history/settings were not queried.
Tests also have path filters omitting `uv.lock`, `configs/**`, and `scripts/**`: merely adding the branch still leaves important fixes untested.

### 4. No measured-position or CAN identity checks — PARTLY; blanket statement WRONG

Evidence commands: `sed -n '170,205p;400,440p;530,550p' .venv/lib/python3.12/site-packages/lerobot/robots/maker_follower/maker_follower.py`; `cat makermodslab/can_speed_limit.py makermodslab/usb_ports.py`; `sed -n '680,810p' makermodslab/teleoperate.py`.
Observed code: ViscousFollower inherits MakerFollower; `connect()` calls `_detect_full_turn_offsets()` before `enable_torque()`. It compares live positions with configured limits plus 20-degree grace, corrects whole turns, and records stale zeros; **the stale-zero refusal is in `send_action()`, after torque may already be enabled**.
The optional speed limiter requires fresh finite starting feedback before preloads/enables. d28d5fc4 changes an outside-limit refusal into a warning. Its gripper branch bypasses the arm-joint slew cap, so “ease inside” is not universal.
`ViscousFamily.build_single_configs()` checks configured leader/follower bounds, not measured mapping accuracy. The Star's `_check_start_pose()` is gated on `calibrate`; Lab passes `calibrate=False` and connects the follower first.
USB identity handling already exists: `usb_ports.resolve_ports()` matches VID/PID/serial and blanks missing/ambiguous matches; `utils/config.py:get_robot_record()` uses it when saved metadata exists. Mac records have metadata; Linux records do not. Legacy start requests and raw scripts need separate coverage.
Thus there is no universal, mandatory **physical-unit-to-profile** check. A record can consistently identify the wrong adapter, and an adapter serial identifies an adapter, not the arm plugged into it. The Linux waterproof record confirms that gap.

## Recommended workflow and order

1. **First contain the known identity hazard.** Before the next powered session or deployment restart, review with Gavin which records should be usable on Linux. Disable/quarantine its waterproof assignment or require the waterproof unit's expected serial, with an offline wrong-unit regression test. Do not silently substitute another profile. This can precede endpoint remeasurement and the final profile-schema decision.
2. **Add runtime evidence and guarded deployment, alongside CI.** Deploy the same chosen, tested SHA to both machines; record separate success/failure for each. Refuse dirty source/config trees and divergent pulls (`--ff-only`); never stash/reset Gavin's edits automatically. Record runtime SHA at process startup, dirty status, dependency revision, profile hashes and effective feedforward settings. `/health` currently reports package version, not SHA; reading HEAD on each request would falsely label an old process as current. Preserve API schema/snapshot conventions.
3. **Prepare before disruption.** Validate dependencies, frontend and profile compatibility before restart. Use `uv sync --locked` with explicit required extras (notably `arm-dynamics`, plus test extras where needed); bare sync can remove opt-in packages. Do not rewrite the active venv or pull code underneath a live session. A staged release/venv is preferable; an idle maintenance window is the small-system alternative. Refuse active sessions/returns/hosting and prevent a new session racing the restart; verify safe shutdown rather than relying on a status snapshot. Roll back code, dependencies and compatible settings together if startup checks fail.
4. **Run useful CI on feature pushes.** Include lockfile, profiles, scripts, mapping library and workflow changes; add frontend typecheck/build for UI fixes. Current Tests installs with pip, not the uv lock, so align or explicitly validate the deployment environment. Keep tests offline: endpoint mapping, unwrap/zero shifts, wrong-unit rejection, stale/missing feedback and no-enable-on-failure ordering. d28d5fc4 already changed a regression test; coverage cannot fix stale deployment or stale measurements. Require a reproducer/test for reproducible logic; use a recorded supervised acceptance procedure for hardware-only faults.
5. **Commission shared unit data, then complete motion preflight.** Reuse `configs/viscous/<unit>/manifest.json` and restore machinery rather than creating another competing store. Keep measured endpoints, safe backed-off limits, zero/frame revision, leader pairing, evidence/date, expected adapter and mapping version together. Unit 1's silent preset fallback must end. Gavin must resolve re-zero versus remeasure and the uncertain Star endpoints; do not promote today's uncertain measurements to authority automatically.
6. **Keep host bindings separate.** Port paths, camera device selection, install/node identity and host overrides legitimately differ. Compare canonical arm fields/hashes, not entire robot-record bytes. On mismatch, start the Lab in a diagnosable state and block that unit's motion, including recording/replay/inference/hosting and bench scripts. Allow deliberate calibration/recovery workflows with explicit scope. A global boot refusal would hide the UI needed to repair one unavailable arm.
7. **Make preflight part of the connection transaction.** Check descriptors first, then fresh feedback through a transport explicitly audited not to enable motors. Recheck under exclusive ownership immediately before energizing; a detached earlier check can go stale. Validate known frame/wrap rules and pose agreement with documented tolerances; do not replace d28d5fc4 with a blanket any-outside-limit refusal. Being inside limits cannot prove correct zeros, signs, scale, or endpoint travel. Those require supervised commissioning. The RobStride clear-fault query has an observed feel effect; “read-only” is not necessarily passive. Do not generalize this connection pattern to Damiao, whose handshake enables motors.
8. **Keep the bug log small and useful.** Link symptom/reproducer, root cause, guarding test or acceptance evidence, fix SHA, profile revision and deployment verification on each host. Reuse the project notes; avoid a second manually synchronized narrative.

## Remaining recurrence paths

- `scripts/gripper-haptics-poc.py:follower_bus/leader_scale` constructs fork defaults directly; `scripts/gripper-range-read.py` imports that script. Lab profile fixes will not reach them automatically. `makermodslab/star_follow.py` inverts configured follower ranges to infer leader travel; `joint_posture_mapping.py` maintains another interpretation. Route these and sim/URDF consumers through the same explicit frame/mapping contract; test the consumers, not just the JSON loader.
- `rollout.py:_single_robot_args()` passes robot type/port/id and cameras, but no commissioned joint limits. The inherited CAN `preflight_ports()` is a no-op. Inference therefore needs explicit profile propagation, not an assumption that teleop's builder covers it.
- Both checkouts have dirty `frontend/dist`; feature pushes do not run the committed-bundle rebuild. A fixed backend can ship with a stale UI. Rebuild and verify locally from the selected frontend source using the repo's feature-branch rules; do not hand-commit dist.
- Git-pinned lerobot requires network/cache access and native dependencies; matching Lab SHAs do not prove matching installed drivers. Both installed `lerobot-*/direct_url.json` files currently report `2a5ea4e581c0d8956b6e58a8e1c6bf8e13486daa`, matching pyproject/uv.lock. Preserve that evidence per deploy; do not silently advance a fork pin. CLAUDE.md still names an older pin, illustrating documentation drift.
- Software tests cannot detect a slipped coupling or changed zero. Remeasure after mechanical/zero changes, version the frame, and mark affected datasets, gravity fits and policies incompatible or migrated. Include profile/frame hashes in new evidence. The mapping README says Run B waits, while HEAD records Run B completed: reconcile handoffs when implementing the workflow.

**Recommended first concrete implementation step:** a narrow offline guard and regression fixture for the Linux waterproof-record/dry-adapter mismatch, after Gavin reviews the intended assignment. Then add startup revision reporting and a non-moving deployment check. A blind restart is not justified by claim 1, and this review authorizes no configuration or service change.
