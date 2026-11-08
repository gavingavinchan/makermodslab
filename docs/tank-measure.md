<!-- Copyright 2026 MakerMods. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# Lab aquarium measurement

2026-11-07 · `feat/viscous-tank-measure` · A1a/A3 and B4 in
[viscous-haptics.md](viscous-haptics.md).

Gavin: “everytime i take the arm in and out of the aquarium, the position may
shift a bit”; “ill mostly use the grippers tip to trace the aquarium”; “lets
also measure where the back wall is from the arm as j3 can easily hit it.”
This replaces the assumed tank placement, including the +x near/back wall.
It **measures and displays only**. Teleoperation has no tank keep-out or
start gate from this feature. The saved schema-v2 evidence is for that later work.

## Operator flow

Select the waterproof Viscous robot, then **Measure tank → Start measurement**.
Torque stays off throughout. Close the gripper and move the arm by hand.
Follow right wall ×2, left wall ×2, near/back wall ×2 on opposite sides of
the tower, floor ×1, and a held-out check touch on a selected reachable wall.
On each long wall separate the touches by at least 150 mm along that wall.
The SVG labels +y left, −y right, +x near/back, and −x far.

Capture needs a fresh reply from every joint and probe motion under 1.5 mm
across a full second. Joint angles are averaged over the final one-second
interval. More than 5 mm opening per finger blocks tip captures. Default-on
auto-capture advances the guide, beeps when browser audio is available, and
requires at least 20 mm movement before another automatic capture. Manual
capture is available. Touches can be deleted and any guide step revisited.

Save needs at least two left and two right touches, one near and one floor;
max residual ≤3 mm, fitted width within 10 mm of nominal 434 mm, and adequate
long-wall spacing. Search-boundary yaw and J1 outside a wall also fail. The
second near touch is recommended, and the GUI guides it. A check touch is
excluded from fitting but included in residual gates. The UI shows failures,
width, yaw, J1 distances, residuals and change from the previous save.

Tower planes and cable padding are optional. Saved extras survive daily tank
placement measurements. Re-measure all four tower planes if its mount changes;
partial or inverted tower boxes block saving. The base-frame box runs from
its probed front/left/right/top to the fitted near plane and floor. Its rear
edge encloses the near plane at both lateral edges; it remains an approximation.
For cable padding, first fit the walls, select a moving link and face, and
press that link's cable side against that face. Padding is its minimum CAD
vertex distance to the face, clamped at zero; retain the maximum measured
padding for the link. Unmeasured links explicitly use 40 mm default padding.

## Geometry and evidence

`makermodslab/tank_geometry.py` is numpy-only, has no hardware imports and
loads repo-relative CAD/config assets. J1–J6 use `motor_to_urdf.json`, including
J5's −1 sign. Gripper slide uses tank.json's clamped 0° closed / −115° open /
52.4 mm per finger mapping. No measured angle is clamped to CAD joint limits.

The fingertip is derived from collision meshes. Normalize `tool_fixed`'s
translation to get the approach axis in the wrist-roll frame. For each finger,
select vertices within 0.01 mm of its largest projection along that axis and
average that extreme patch. Average the two transformed tips to get the closed
gripper's probe. The local tip centres, in mm, are:

- left `(166.308001, +1.132000, -27.091000)`;
- right `(166.308001, -1.132000, -27.091000)`;
- midpoint `(166.308001, 0, -27.091000)`.

This is a CAD-based virtual centre of the closed fingertip pair, not tool0.
Physical fingertip/contact shape and provisional motor signs still need an
operator check; none was performed during implementation.

FK cross-check against `scripts/viscous_aquarium_env.py`, loaded by path with
importlib: **0.0 mm maximum origin difference across every link**, both
reference poses; **0.0 mm probe difference** using independent mesh selection
and the reference transforms.

| Motor pose | tool0 xyz mm | fingertip midpoint xyz mm |
|---|---|---|
| read | −520.149821, +101.962732, 42.820462 | −549.613716, +119.540241, 44.026655 |
| folded | +207.979186, +82.007720, 148.941164 | +240.091260, +93.470241, 144.944898 |

Yaw is tank +x relative to base +x. Project base points onto the rotated tank
axes; for each yaw each plane's offset is its touch mean. A ±20° coarse scan
at 0.1° followed by bounded golden-section refinement minimizes squared
residuals. Far is near minus nominal 734 mm inner length and explicitly
**estimated**. Rim is measured floor plus nominal 442 mm height, not a lid.

Clearance includes every moving descendant of shoulder_pan, including yaw_link,
both fingers and fixed descendants that carry meshes: **9,059 distinct moving
vertices; no subsampling**. Each link's padding inflates its plane/box distances.
Margins come from tank.json (30 mm walls/rim, floor 0). Floor contact is
informational. Probe distances are also reported. Tower clearance is signed
vertex-to-box distance; it is not an exact triangle intersection or swept-path
test and is not suitable as a motion guard.

After warm-up, 200 calls on this Mac: clearance median/p95/max
**0.406 / 0.415 / 0.507 ms**; fit **7.966 / 8.147 / 8.309 ms**. This supports
the 10 Hz measurement display; it is not a real-time guarantee. Row-wise
vertex transforms use numpy.einsum, retaining the same geometry while avoiding
spurious Accelerate matmul floating-point warnings on this machine.

## Session, API and storage

`makermodslab/tank_measure.py` resolves the saved single-arm robot record,
checks the family's manifest capability and commissioned follower config,
and refuses an adapter owned by the CAN monitor bridge. It uses
`family.open_follower_torque_off(port, config_id)` (bus handshake disabled,
then torque disabled), never follower.connect. Each poll uses the family's
fresh-all-joints reader. It invokes the installed MakerFollower's exact
`_detect_full_turn_offsets` implementation on a context backed by that fresh
read and the commissioned limits, then retains offsets for the session just
like teleop. Missed replies clear the sample window and disable captures.
Every exit attempts disable_torque followed by bus.disconnect(False), even
when disabling raises. No zero, enable or position write exists in the module.

It owns the existing calibration mutex and emits calibration claim/release.
All existing aggregate reciprocal checks see it. Two calibration start paths
previously read manager flags directly; they now also refuse it. The sessions
surface can stop it as kind calibration. Polling stops after 15 minutes without
a status poll, using an injectable clock. A timed-out stop retains the claim
until the worker's real disconnect completes.

All seven routes are v1-only, typed, uniquely named and registered in the API
contract test. Latest works with no hardware:

- `POST /api/v1/tank-measure/start` `{robot_name}`
- `GET /api/v1/tank-measure/status`
- `POST /api/v1/tank-measure/capture` `{label}`
- `DELETE /api/v1/tank-measure/touches/{index}`
- `POST /api/v1/tank-measure/save`
- `POST /api/v1/tank-measure/stop`
- `GET /api/v1/tank-measure/latest?robot_name=...`

State constants live in utils/config.py. Save atomically replaces each file:
`$MAKERMODSLAB_HOME/tank/<robot>/latest.json`,
`history/<UTC stamp>.json`, and `arm_extras.json` (tower, padding and timestamp).
History is written before latest. Export keeps schema_version 2, fills measured
placement/source, regenerates inner bounds and records yaw, residuals, touches,
averaged motor angles, extras, timestamps and nominal dimension provenance.
Fitted width replaces the width used for derived bounds. Nominal far length
and rim height remain estimates. Store yaw in base_rpy_deg even though today's
offline loader refuses nonzero rotation; zero-yaw compatibility is tested.

## Implementation choices and remaining validation

The Viscous family refuses to open its follower for calibration, because there
that path exists only to re-zero, and re-zeroing invalidates its measured
limits. That refusal is unchanged. The bus-only open behind it now lives in
`CanArmFamily.open_follower_torque_off(port, config_id)`, which
`open_for_calibration("robot", ...)` and this measurement both call, so the
measurement reads the arm without loosening the re-zero guard. (Review change:
the first implementation lifted the refusal and moved the re-zero test to the
wizard's start gate; Claude reverted that.) The family capability defaults false
and is true only for Viscous in the manifest, backend schema and frontend type.

No hardware, Lab process, real user state, commit or push was used. A temporary
Vite preview with mocked API data was inspected in Chromium in English and
Chinese with no page errors; temporary preview source files were removed.
Ruff was missing from the required shared interpreter, so it was installed
under /tmp and invoked with PYTHONPATH; the shared venv was not modified.
Temporary prettier/Playwright tooling also lived under /tmp. No repository
dependency or lockfile changed. Built frontend/dist was restored after checks.

Physical contact-point/sign verification and a first real daily measurement
remain operator validation, as hardware access was explicitly excluded.
There are no unresolved implementation questions. Nonzero-yaw support in the
offline script, a teleop keep-out and a follower start gate remain follow-up work.

## Files changed for review

| Files | Purpose |
|---|---|
| `makermodslab/tank_geometry.py` | CAD FK/probe, plane fit, gates, extras, clearances and schema-v2 export |
| `makermodslab/tank_measure.py` | Read-only bus/session lifecycle, fresh samples, stillness, timeout and atomic persistence |
| `makermodslab/schemas/tank.py`, `server.py` | Seven typed v1-only routes |
| `makermodslab/arms/base.py`, `arms/can_common.py`, `arms/viscous.py`, `arms/manifest.py`, `schemas/system.py` | Measurement capability and the shared torque-off follower opener |
| `makermodslab/calibrate.py`, `step_calibrate.py`, `sessions.py` | Reciprocal bus exclusion and calibration-session stop dispatch |
| `makermodslab/utils/config.py` | Tank storage constants |
| `frontend/src/lib/tankApi.ts`, `lib/armsApi.ts` | Typed versioned client and manifest capability |
| `frontend/src/components/launchpad/TankMeasureDialog.tsx`, `RobotControls.tsx` | Operator workflow and capability-gated entry point |
| `frontend/src/i18n/locales/{en,zh-CN}/tank.ts`, both `index.ts` files | Translated prose and catalog registration |
| `tests/test_tank_geometry.py`, `test_tank_measure.py` | Offline numeric, safety, lifecycle, API and persistence tests |
| `tests/test_arms_manifest.py`, `test_api_contract.py` | Capability and route ratchet |
| `frontend/src/lib/tankApi.test.ts`, `components/launchpad/TankMeasureDialog.test.tsx`, `RobotControls.connection.test.tsx` | v1/data-label contract, auto-capture/stale/open/close behavior and manifest gate |
| `docs/api/openapi.json` | Regenerated API snapshot; only existing schema changed is ArmCapabilities |
| `.gitignore`, `docs/tank-measure.md`, implementation note in `docs/viscous-haptics.md` | Keep the new documentation reviewable; intent, method, evidence, resumption notes and remaining work |

Other concurrent aquarium-protection additions in viscous-haptics.md were
preserved. Only the separate implementation note above “Working rules” was
added by this task. The protected offline script, its tests, environments and
model files were read but not edited.

## Check output

Required interpreter resolves makermodslab to this worktree:
`/Users/gavin/Documents/makermodslab-viscous-tank/makermodslab/__init__.py`.
All Python checks ran from this worktree with the specified other-checkout
interpreter; tests redirect MAKERMODSLAB_HOME to temporary state.

Baseline, before edits:

- `python -m pytest -q -x`: **1 failed, 125 passed**; stopped at
  `test_arm_registry.py::test_single_device_configs_carry_the_port_and_id[viscous]`.
- Full baseline: **1 failed, 4452 passed, 19 skipped**. Same failure: existing
  fixture asks for commissioned profile `cal-f`, which does not exist.
- Frontend lint: **0 errors, 34 warnings**. Both TypeScript projects: **exit 0**.

Final checks:

- New geometry/session tests plus arms manifest, Viscous and API contracts:
  **113 passed**. Includes 40 deterministic noisy synthetic tanks, ±5° yaw,
  ±30 mm offsets and 0.5 mm noise, with the requested accuracy assertions;
  every-link/probe FK cross-check; gate refusals; never-energize fake bus;
  all busy holders; both reciprocal calibration managers; injected-clock idle
  cleanup; persistence round trip and zero-yaw offline load.
- Calibration/error-contract regression selection: **109 passed**.
- Full pytest (`--assert=plain --tb=short`): **1 failed, 4496 passed, 19 skipped (91.72 s)**; same baseline `cal-f` fixture failure. A normal-reporting retry stalled formatting an existing launcher socket-test assertion while its shared sleep-call list grew; it was terminated. The launcher file passes in isolation (**49 passed**), and the complete plain-reporting retry ran every test with no exclusions and no new failures.
- `ruff check` on all 17 changed Python files: **All checks passed!**
- `ruff format --check`: **17 files already formatted**.
- Frontend lint: **0 errors, 34 warnings**, unchanged from baseline.
- `npx tsc --noEmit -p tsconfig.app.json`: **exit 0**.
- `npx tsc --noEmit -p tsconfig.node.json`: **exit 0**.
- Full Vitest: **61 files, 610 tests passed**, including new flow/client tests
  and catalog key/placeholder parity.
- `npm run build`: **passed**; normal large-chunk advisory only. Restored
  tracked dist and removed generated untracked dist assets afterwards.
- `python -m makermodslab.scripts.export_openapi --check`: **exit 0**.
- `git diff --check`: **exit 0**.
- Mocked Chromium screenshots in English and Chinese: **no page errors**.

No backend test failure was suppressed or excluded to obtain these results.
The pre-existing commissioned-profile fixture failure is left for its owner.
