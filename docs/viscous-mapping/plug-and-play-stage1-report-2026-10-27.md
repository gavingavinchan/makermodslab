<!--
# Copyright 2026 MakerMods. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
-->

# Plug-and-play stage 1 — offline implementation, 2026-10-27

Goal P / A3: Gavin can move the USB plugs and press Teleop, without manually finding ports or repeating a failed power-up start. Work stayed on `feat/viscous-plug-and-play` in this worktree. No hardware port was opened, no arm script/SSH/service operation ran, and nothing was pushed. Runtime state and Hugging Face paths were redirected to temporary directories for validation. The driver pin, lockfiles and `frontend/dist` are unchanged.

## Built, by scope item

| Scope | Implementation and evidence |
|---|---|
| A — rediscover on every start | `teleop_connection.resolve_start_ports` consumes a fresh descriptor snapshot before readiness/config staging. CANable uses VID/PID `5840:4478` and saved/enrolled serial hints; Star uses the sole `6790:29987` candidate. A sole different CANable produces an identity warning, never stale-path fallback. `usb_ports.unique_ports` collapses macOS tty/cu and Linux descriptor LINK/location aliases without opening or following a device path. Old records without `port_identities` work. Both sessions and legacy Teleop use the resolver. Family opt-in covers Viscous and Maker; SO-101 and Metal retain their existing connection behavior. Multiple indistinguishable Star endpoints remain ambiguous, including bimanual Maker leaders without distinguishing descriptors. |
| B — unit enrollment | Extended the **existing manifests for both units** with `adapter_enrollment`: dry `207E37A55445`, waterproof `205038953846`. Verified against committed Mac descriptor snapshots and Linux records/backups under `configs/viscous/state-snapshots/2026-10-27/`. The follower calibration name maps to the expected unit; enrollment maps the observed serial to its unit. The Linux waterproof record on the dry adapter returns `robot.unit_mismatch`, identifying `viscous_01` and the `viscous_waterproof_01` settings. An unenrolled adapter returns `robot.identity_unverified`. `allow_unit_override` applies only to that request and keeps the record's selected settings; it writes no identity exception to disk. Missing enrollment still warns. |
| C — typed failures | Added grammar-conforming `robot.*` codes for missing USB on either side, follower/leader not ready, missing feedback, ambiguous connections, unit mismatch/unverified identity, busy ports and unknown failures. Fault and uncertain-release codes preserve actionable cleanup failures too. Technical context is in `details`. RobStride outcomes are captured at the query source, surviving the fork's generic `ConnectionError` wrapper without parsing its prose. OS errno/cause chains identify busy ports. |
| D — cancellable waiting | One worker owns connect, control and cleanup for opted-in families. The start claims synchronously and returns a session/lease immediately; a private gate releases the worker only after attribution/lease attachment. This avoids both a blocked heartbeat and a fast failure racing the session identity. Stop and lease expiry set the same cancellation event; ownership stays claimed until cleanup. Star readiness runs before normal Star connect and before follower enable. One injected monotonic deadline is **65 seconds**, with cancellable quarter-second sleep slices. Only an all-silent RobStride handshake retries; partial feedback, faults, post-handshake failures and Damiao are terminal. Each failed handshake transport closes before retry. Cancellation/deadline checks also guard each follower enable. Phases `starting`, `waiting_leader`, `waiting_follower`, `running` publish through `session_events`; typed status includes the waiting sentence. |
| E — frontend | Teleop remains clickable when these families need endpoint rediscovery. Start failures show the plain sentence, with structured details collapsed. Identity warning codes alone offer **Continue anyway**, resending the captured options plus the one-start flag. The live dialog displays waiting status, keeps heartbeating and offers Stop; failed asynchronous starts show the plain sentence and collapsed details. All new client labels exist in English and Chinese. The translation-key scan explicitly distinguishes the two API codes from translation keys. Backend sentences remain English as required by the localization contract. |
| F — offline tests | New tests exercise the actual pinned Maker/Viscous connect and RobStride handshake against a fake CAN transport, and existing raw Star ping framing against fake serial bytes. Tests cover renamed/swapped locators, aliases, absent metadata, one/two Stars, duplicate CAN serials, enrollment snapshots, mismatch/unverified warnings and override reset, follower readiness at 5/60 seconds/never, partial/fault feedback without retries, Star late/partial/never, Stop during query/wait/before enable, lease expiry, lease-attribution ordering, typed status, busy/unknown codes, and one Damiao recovery with no retry. Clocks/sleeps are injected; no new dependencies, real sleeps or worker/subprocess happy paths were added. Five client tests cover warning/override/reset, physical refusals, details, waiting heartbeat and Stop. |
| G — CI | `tests.yml`, `quality.yml`, `frontend.yml` now include `feat/viscous-arm` pushes. Both tests-workflow path filters include `configs/**`, `scripts/**`, `uv.lock`. Frontend CI also runs the connection UI and catalog/key tests. A fixture checks branch and profile/script/lock triggers. |
| H — contract | Regenerated OpenAPI. Teleoperation status now has a response model; removed its single entry from `UNTYPED_V1_ROUTES`. Added the request boolean on legacy Teleop and session Teleop options. No routes or ratchet exemptions were added. |

## Failed-handshake safety verified in the installed fork

Inspected `.venv/lib/python3.12/site-packages/lerobot/` at the unchanged pin **`2a5ea4e581c0d8956b6e58a8e1c6bf8e13486daa`**:

- `robots/maker_follower/maker_follower.py:400-423`: `bus.connect()` is at line 402, **before** gains, turn-offset reads, and `enable_torque()` at line 423. A handshake failure exits before any enable. Viscous inherits this implementation (`robots/viscous_follower/viscous_follower.py:42-56`).
- `motors/robstride/robstride.py:227-236,288-318`: the handshake sends only existing `0xFB` clear-fault/state queries, records missing/faulted motors and raises if any are missing/faulted. This is non-enabling, **not passive**. No motor-UID or new frame type was introduced.
- `motors/robstride/robstride.py:215-225`: failed handshake marks disconnected but leaves the CAN transport allocated. Stage 1 closes that transport before an all-silent retry. A failed close terminates waiting with `robot.release_uncertain`; it never retries.
- `robots/maker_follower/maker_follower.py:424-427`: failures after bus connection use the existing disconnect/release path. Lab's existing explicit CAN recovery remains in the terminal setup-error path, including physical partial failures; it is not called on every silent retry because the inspected path cannot enable.
- `teleoperators/rebot_102_leader/rebot_102_leader.py:156-189`: pings precede `configure()`/optional lock, and failure closes the handle. Stage 1 waits using the existing read-only raw ping transport with finite serial timeouts before invoking normal connect once; no repeated unlock/reset/lock sequence during waiting.
- `motors/damiao/damiao.py:209-251`: handshake enables motors; Metal does not opt into discovery/wait, and its existing partial-connect recovery is still used once.

These are code inspection and fake-wire results, not a powered-handshake certification.

## Baseline versus final validation

Installed before edits: `uv venv && uv sync --extra test --extra arm-dynamics`; `cd frontend && npm ci`. npm warned that Node `22.22.1` is just below jsdom's supported `22.22.2` minimum. No dependency or lockfile changes were made.

For Python checks below, the command environment was:

```sh
HF_HOME=/tmp/pnp-hf HF_LEROBOT_HOME=/tmp/pnp-lerobot HF_HUB_OFFLINE=1
```

Pytest's conftest separately redirects application state/output. OpenAPI commands additionally used `MAKERMODSLAB_HOME=/tmp/pnp-state MAKERMODSLAB_OUTPUT_ROOT=/tmp/pnp-output` and the temporary HF paths. All frontend commands ran in `frontend/`.

| Exact command | Baseline before edits | Final |
|---|---|---|
| `uv run pytest -q` | Initial native-import collection was interrupted after 117 s; rerun with `-o faulthandler_timeout=60` completed: **790 passed, 10 failed**, configured first-10 cutoff. | **791 passed, the same 10 failures**. The extra pass is the registry sweep over the new backend module. |
| `ruff check` / `uv run ruff check` | Ruff unavailable on PATH/in venv. Used `uv tool run ruff check` (Ruff 0.17.3): **3,229 findings**. | Same tool/command: **3,229 findings**. All changed/new Python files pass a targeted Ruff check. |
| `npm run lint` | **0 errors, 34 warnings**. | **0 errors, 34 warnings**. |
| `npx tsc --noEmit -p tsconfig.app.json` | Passed. | Passed. |
| `npx tsc --noEmit -p tsconfig.node.json` | Passed. | Passed. |

The common ten Python failures are `test_single_device_configs_carry_the_port_and_id[viscous]` (uncommissioned `cal-f` profile), and the first nine dataset rename tests. Their baseline failure names are unchanged; none was repaired.

Additional final checks:

- `uv run pytest -q tests/test_teleop_connection.py tests/test_teleoperate.py tests/test_sessions.py tests/test_usb_ports.py tests/test_api_errors.py tests/test_api_contract.py`: **227 passed**. This includes **32 new backend cases**.
- Including `tests/test_arm_registry.py` in the targeted check: the pre-existing `cal-f` profile failure remained; the rest passed (the earlier run before the final four new cases had 399 passes).
- `uv run pytest -q --maxfail=0`: **4,340 passed, 19 skipped, 44 failed**. Failures by file: arm registry 1, datasets 17, jobs 20, W&B preflight 6. This broader run extends beyond the initial baseline cutoff, so the additional 34 failures were **not measured in the original before-edit run**. The affected dataset/job/W&B implementation files were left untouched; no full-suite-green claim is made.
- `npm test -- --reporter=dot src/components/launchpad/RobotControls.connection.test.tsx src/components/dialogs/TeleopDialog.connection.test.tsx src/i18n/catalogs.test.ts src/i18n/keyUsage.test.ts`: **15 passed**.
- `npm test -- --reporter=dot`: the existing four-slot `RobotConfigDialog` calibration test timed out at 5 seconds under parallel load (597 passed, 1 timeout); its code was not changed. `npm test -- --maxWorkers=2 --reporter=dot`: **598 passed / 59 files**, including that test.
- `npm run build -- --outDir /tmp/pnp-frontend-build`: passed, output outside the worktree; only the existing large-chunk warning.
- `uv run python -m makermodslab.scripts.export_openapi` and the same command with `--check`: passed. API contract tests pass.
- `git diff --check`: passed. Driver pin, `uv.lock`, `frontend/package-lock.json` and `frontend/dist` have no diff.

## Deliberately deferred and powered acceptance

No measured limits, profile aliases, motor zero, stale-zero gate, record/replay/rollout/hosting/haptics behavior, driver pin or release/update tooling changed. Unit 1's known gripper-profile problem remains stage 2; no guessed endpoints or fallback changes were introduced. Motor UID identity and strict pre-torque freshness/physical-envelope qualification remain later stages; the family enrollment method and `identity_kind=usb_adapter_only` make that limit explicit. Adapter identity cannot detect someone moving the enrolled adapter onto another physical arm.

Powered acceptance still needs Gavin physically present at the arm: confirm real boot timing, USB socket changes and descriptor rebinding on each machine; same-click follower/Star power-up; warning plus explicit one-start override; Stop/lease expiry during connect; and the existing first-motion/return/release behavior. Agents should operate the tools and capture evidence. Unit 1's measured gripper stops/zero decision still needs physical commissioning before its profile repair. Normal driver transport opens/configuration retain their own I/O latency; the 65-second policy is cooperative, with checks between calls and before motor enable, not a hard real-time interrupt of an OS open. Actual cancellation latency and USB-driver behavior therefore remain powered/platform acceptance items.
