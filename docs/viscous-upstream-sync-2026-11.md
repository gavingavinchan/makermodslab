# Viscous fork: sync with makermods-robotics, 2026-11-13

**Goal-tree place:** the to-do "Update the fork from makermods-robotics" in
[viscous-haptics.md](viscous-haptics.md). Gavin's intent: "about time to update
our fork ... without breaking the stuff for our viscous arm" (2026-11-07), then
"update the current makermods labs to have all the features from the latest
version" (2026-11-13). New upstream CAN behaviour for Viscous is "off for now",
and each one is listed below so he can test it on the physical arm when he is
there.

**Status:** merged and tested offline only, on branch
`feat/viscous-upstream-sync` (not merged into `feat/viscous-arm`). Nothing here
has driven a motor. The running Lab on either machine is unchanged. The powered
acceptance run the to-do asks for is still needed before this replaces the
running Lab.

## What was merged

| | |
|---|---|
| Upstream | `makermods-robotics/makermodslab` (remote `upstream` in this repo; our parent project, not Hugging Face leLab) |
| Merged ref | `upstream/staging` at `232f8894` (2026-11-04, PR #175, Maker gravity feed-forward) |
| Merge base | `a031174e` (2026-09-14) |
| Our side | `feat/viscous-arm` at `bcb15bf0` |
| Size | 121 upstream commits, 244 ours |
| Merge commit | `7888ed7e` (a real merge, no rebase or squash, so `feat/viscous-arm` keeps shared history) |

`upstream/main` (`e3771494`) is a strict subset: staging adds only the three
Maker gravity feed-forward commits (`0881672d`, `be4e3ee8`, `c83bdf8d`).

Commits on the branch after the merge:

- `a52f0863`: Viscous opt-outs for upstream CAN behaviour (table below), with
  `tests/test_viscous_upstream_optouts.py`.
- `ac2519ac`: fixes from the first gavin-linux run (see "Testing").
- `e250411e`: OpenAPI snapshot regenerated.
- `ae7877d3`: the haptics to-do status line. (This file was meant to be in
  that commit; `.gitignore`'s `docs/*` skipped it silently. It now has an
  exception and is committed with the review fixes.)
- The review-fix commit after it: see "Independent review" below.

What arrived, by area:
- the `makermodslab-sdk` Python SDK (`sdk/`, its own CI workflow);
- Maker: gripper holding on RS00, teleop firmware-fault guard and CAN fault
  journal, gravity feed-forward, fitted URDF inertials;
- Metal: gs_usb adapters, a speed cap while following a leader, late gripper
  replies, a vertical-grip leader;
- hosting fixes (RPCs registered before Portal connects, Damiao read
  deadline, uncapped hosted gripper, a featured station camera);
- camera reconnect mid-recording, an MJPG preview default, stopping the
  heartbeat when the server says the session is gone;
- routes scoped to a session id (recording status and prompt, calibration
  step, GPU stop by launch id);
- `GET /api/v1/policy-config`, `release_now` on teleop stop, the trigger grip
  mapped by distance pulled, and the Maker URDF sync check in CI.

## Conflicts and how each was resolved

Git reported content conflicts in four files. The other 17 files changed on
both sides merged automatically; each was then read for semantic clashes.

| File | Resolution |
|---|---|
| `makermodslab/arms/can_common.py` (`return_to_rest`) | Combined. Kept our return, which honours a teleop speed cap (a slower speed and a longer ceiling). Took upstream's per-arm `(ok, reason)` verdicts and `ensure_target=True`. `ensure_target` now comes from a new family flag, `rest_return_ensures_target`: True on `CanArmFamily`, False on Viscous. |
| `makermodslab/arms/maker.py` | Kept both flags: ours, `supports_teleop_speed_limit`, and upstream's, `supports_gripper_effort_control` (Maker gripper holding). |
| `makermodslab/remote_can.py` (`limit_action`) | Combined. Kept our `speed_deg_s` parameter and finite-value checks (`can_speed_limit.py` relies on them). Took upstream's uncapped hosted gripper as the default, through a new `exempt=` parameter. `remote_host.py` asks the family (`hosted_speed_cap_exempt`, new flag `hosted_gripper_skips_speed_cap`), so a Viscous hosted gripper stays capped. |
| `makermodslab/teleoperate.py` | Combined. `_connect_can` keeps our env gravity switch and connect recovery, then calls upstream's `prepare_teleoperation` / `prepare_leader_following` hooks. The worker's `global` lines are the union of both sides. |
| `frontend/dist/**` (no textual conflict) | The JS/CSS bundle and `index.html` kept ours: never hand-commit a bundle. CI's `build_frontend.yml` runs only on `main`/`staging`, so on this fork the committed bundle is still the merge base's (2026-09-14) and lacks upstream's UI changes; rebuild it locally before running the Lab (see the last section). `dist/maker-urdf/{robot.urdf,README.md}` take upstream's, because they are vite's verbatim copy of `public/maker-urdf/`. `maker_gravity.py` reads that copy in a wheel install, and the old one failed `test_wheel_layout_reads_the_urdf_from_dist`. |
| `CLAUDE.md` (auto) | Both sides' text kept. Added one sentence to the new `maker_gravity.py` bullet saying it does not reach the Viscous follower, with a link here. |
| `docs/api/openapi.json` (auto) | Regenerated on gavin-linux after the merge, with no diff. After `rest_failed` was declared (below) it changed by that one field. |
| `tests/test_api_contract.py` (auto) | The ratchet lists are the union of both sides; none were widened. Upstream's four new v1 routes went into `V1_ONLY_ROUTES` with their reasons. |
| `pyproject.toml` | Upstream did not change it. The lerobot pin stays ours: `gavingavinchan/lerobot-MakerMods@2a5ea4e`, which already contains upstream's `5ce2fe41a`. The `metal-leader` extra already pointed at that same SHA, which is consistent: a direct reference must match the base dependency. Upstream added no new Python dependencies outside `sdk/`. |
| `frontend/package-lock.json` | Upstream did not change it. Untouched. |

Semantic fixes after the merge (no textual conflict):

- **`/teleoperation-status` dropped `rest_failed`.** Our fork had typed this
  route, and `response_model` silently filters undeclared fields. Declared it in
  `schemas/sessions.py`. The SDK's recovery test caught this.
- **`tests/test_maker_gripper_hold.py::test_teleoperation_installs_maker_adapter_before_connect`**
  (upstream's) used `/dev/` ports that do not exist, so our USB-presence check
  refused before the code under test ran. It also stubbed
  `de_energize_can_device` to return `None` where our code adds to a list. The
  test now uses non-`/dev/` port names and a list. What it asserts is unchanged.
- **SDK ratchets** (`sdk/tests`): the fork's seven tank-measurement routes go in
  `PLANNED["system"]`, and the four Viscous teleop options (`allow_unit_override`,
  `gripper_haptics`, `joint_posture_mapping`, `speed_limit_deg_s`) in
  `EXCLUDED_OPTIONS["teleoperation"]`. Both are the registers' designed path for
  surface that lands through a merge. Adding SDK methods for them is a separate
  job.

## How upstream behaviour could reach Viscous silently

There are two inheritance paths, and neither shows up as a conflict:

1. `ViscousFamily` subclasses `CanArmFamily`. Any flag or hook upstream puts
   there applies to Viscous.
2. `lerobot.robots.viscous_follower.ViscousFollower` subclasses `MakerFollower`,
   and `ViscousFollowerConfigBase` subclasses `MakerFollowerConfigBase`.
   `maker_can.install()` runs in every Lab process (`robot_factory`, `calibrate`,
   `maker_ports`, the rollout/DAgger/eval/DRTC children) and patches
   `MakerFollower.__init__`. **Without a gate, upstream's Maker gravity
   feed-forward was attached to the Viscous follower:** the Maker URDF model at
   scale 1.0, up to 15 N m on `shoulder_lift`. It would also have stacked on
   Viscous's own feed-forward. That was the most serious finding of this sync.
   It is now gated (`maker_gravity._family_scale`: only a follower whose class
   `name` is `maker_follower`) and pinned by a test that constructs a real
   `ViscousFollower`.

Viscous's own gravity feed-forward is unchanged. It is still the env switch
`MAKERMODSLAB_GRAVITY_FF` / `MAKERMODSLAB_GRAVITY_FF_SCALE`
(`gravity_feedforward.py`); gavin-linux's systemd drop-in turns it on for
`viscous_01` teleop at 50%. `install_from_env` now also refuses to install on a
follower that already carries upstream's feed-forward. That can only happen on
a Maker, so the env var means exactly what it did before.

## Upstream features available for Viscous, currently OFF

Each is pinned OFF by a test in `tests/test_viscous_upstream_optouts.py`. "Turn
on" means the exact code change; do it on a branch, run the test file (it will
fail on that line, on purpose), update the test, then do the hardware check.

| Feature | Upstream commit(s) | How it would reach Viscous | How to turn it on | What to test at the arm | Risk |
|---|---|---|---|---|---|
| **Teleop tracking failsafe**: a failed landing holds torque until an explicit release; error exits (including a thermal stop) also attempt the return; a second Stop, a lease-expiry stop or a new start during the return no longer abort it (idempotent stops). With it OFF, as before, any of those aborts the return and releases torque where the arm is. | `d1974ebd` (PR #165) | `CanArmFamily.requires_verified_rest = True` | Delete `requires_verified_rest = False` in `arms/viscous.py` | All 6 arm joints. Stop teleop normally: it returns and releases as now. Then make a landing fail (hold one joint by hand during the return): the arm must stay energized, the UI must show the recovery notice, and "release torque" must release it. Also trip an error exit and watch it try to return. | **High.** It keeps a stalled arm energized with no one driving. The RS00 J2/J3 went 38→56 °C in 3 min of dry teleop. The 85 °C thermal stop is designed to drop the arm, and with this on, a thermal error exit tries to return and hold instead. |
| **Return re-commands the pose even when already within tolerance** | `d1974ebd` | `CanArmFamily.rest_return_ensures_target = True` (new flag; upstream passes `ensure_target=True`) | Delete `rest_return_ensures_target = False` | Stop teleop with the arm resting at its start pose. Expect a short ramp with no visible motion, then release. Watch all 6 arm joints for a twitch. | Low |
| **Hosted gripper follows the operator uncapped** (remote teleop) | `cc3b4cd6` | `remote_host` → `remote_can.limit_action` default `exempt=GRIPPERS` | Delete `hosted_gripper_skips_speed_cap = False` | Gripper (joint 7) only, in a station-mode hosting session. A fast leader open/close is followed immediately rather than at 30 deg/s. Close it on an object and watch the grip force (gripper kp is 20). | Low to medium |
| **Maker URDF gravity feed-forward** (`maker_gravity.py`) | `0881672d`, `be4e3ee8`, `c83bdf8d` (PR #175) | `maker_can.install` patches `MakerFollower.__init__`, which `ViscousFollower` inherits | **Do not turn on as is.** The model is the Maker arm's (RS02 joints, its masses), not this arm's. Enabling it would need a Viscous URDF in `maker_gravity.shipped_model`, `_family_scale` to accept `viscous_follower` and return a Viscous scale (no Viscous flag reads it today; the class-name gate is the only switch), and `MAKERMODSLAB_GRAVITY_FF` unset (the two refuse to stack). Our own fitted FF already covers this. | J2-J4 (shoulder_lift, elbow_flex, wrist_flex) hold a raised pose without sag or push. Compare against the env FF at 50%. | **High** with the Maker model (wrong torques, up to 15 N m cap) |
| **RS00 gripper holding** (`maker_gripper_hold.py`: holding torque, capped closing, 30 Hz writes, capped gains during feedback polls) | `dc6b9301`, `17df5f0b`, `821e61e3`, `2cffb3c1` | `supports_gripper_effort_control` → `metal_gripper.install_metal_gripper`, which also requires `config.type == "maker_follower"` | Set `supports_gripper_effort_control = True` in `arms/viscous.py` **and** add `"viscous_follower"` to the type check in `install_metal_gripper` (route it to `MakerHoldingBus`). Check it does not fight `supports_gripper_haptics`: both drive the gripper. Viscous's gripper is RS00 like Maker's, so this is the most plausibly useful feature here. | Gripper (joint 7) only. Close on an object: it holds at the set torque (default 0.5 N m) and does not creep. Watch the gripper temperature (the controller stops at 55 °C, resumes at 45 °C). Then repeat with gripper haptics on. | Medium |
| **Metal gripper current limiting** | `8426d060` era, `supports_gripper_current_control` | Flag on `ArmFamily` | Not applicable: Damiao current mode, and the Viscous motors are RobStride. | n/a | n/a |
| **Maker teleop safety + CAN fault journal** (`maker_teleop_safety.py`, `maker_fault_diagnostics.py`): checks every action for feedback no older than 250 ms and for a latched firmware fault, and on a failure saves CAN evidence under `MAKERMODSLAB_HOME/logs/teleoperation/` | `d1974ebd` | `MakerFamily.prepare_teleoperation` (Viscous overrides it with a no-op, so a later move of the hook to `CanArmFamily` cannot reach it) | In `ViscousFamily.prepare_teleoperation`, do what `MakerFamily` does: `MakerTeleopSafety(arm)` for each `maker_follower_arms(robot)`. Pair it with `requires_verified_rest`: its `hold()` / `recovery_drive()` only matter on that path. | All 7 joints during normal teleop. Watch for false trips: feedback goes through our CAN monitor proxy (`monitored_can`), an extra socket hop the 250 ms check has never seen. Pull the CAN cable mid-teleop and check the evidence folder is written. | Medium (false trips end a session) |
| **Follower speed cap while chasing a leader** (280 deg/s, gripper exempt) | `3114dfa4` | `prepare_leader_following`, called by teleop and recording (Viscous overrides it with a no-op) | In `ViscousFamily.prepare_leader_following`: `install_follower_speed_cap(maker_follower_arms(robot), 280.0, math.inf, exempt=("gripper",))`, as `MetalFamily` does. It is mostly redundant next to our supervised ≤30 deg/s teleop limit. | All 6 arm joints during teleop and recording, with fast leader moves. With our speed limit also on, check the two caps do not interact badly. | Low |
| **Trigger grip mapped by distance pulled** | `450d4b49` | Patches `RebotArm102Leader.get_action` for every Star leader, but only remaps one whose `joint_directions` is a `TriggerJointDirections`. The Viscous leader's is a plain dict. | Only relevant if a Viscous unit gets the trigger grip: add a `star_trigger`-style leader kind to `ViscousFamily.leader_options`. | Gripper (joint 7): pull the trigger both ways. | n/a today |
| **Maker gripper open limit narrowed to −96°** | `2cffb3c1` | Patches `MakerFollowerConfigBase.__init__` / `MakerFollowerConfig.__init__`; `ViscousFollowerConfig` is its own dataclass with a generated `__init__`, so it is not reached. Pinned by a test. | Not applicable: this is the Maker jaw's measured stop. | n/a | n/a |
| **Maker URDF fitted inertials and friction** | `84bece22`, `4d55104b` | Only the Maker 3D view and `maker_gravity`. Viscous telemetry is `degrees`. The Dream-1 scripts keep their own copy under `scripts/viscous-dream1/assets/` (not touched). | Nothing to turn on. If the Dream-1 translation layer should use the new Maker inertials, refresh that copy in its own task. | n/a | None |
| **Recording camera reconnect**: a camera whose reader dies mid-recording is reconnected (5 tries, 1 s apart) instead of ending the session | `34b9a697` | Ungated in `record.py`'s observation wrapper | Delete `recording_recovers_dead_cameras = False` in `arms/viscous.py` | All 7 joints hold their last command while the camera reconnects (up to ~5 s); Stop interrupts it. Unplug a camera during a recording with the arm in a raised pose: the arm holds, the camera comes back, recording continues. Press Stop during a reconnect: it stops within ~1 s and returns. | Low to medium (the arm holds a pose unattended for a few seconds) |
| **Recording re-alignment targets clamped to the follower's soft limits** | `2cffb3c1` (`maker_targets_from_action`) | Ungated in `maker_rest_pose.py`, used by recording's leader catch-up | Delete `recording_clamps_alignment_targets = False` | All 6 arm joints: start (or resume) a recording with the leader past a follower soft limit (for example `shoulder_lift` above its −2.4° stop) and watch the catch-up. The commands should not change, because `send_action` clamps the same way; what changes is the convergence judgement (with the clamp, the catch-up stops waiting for a pose the follower can never reach). Codex reproduced 17.6° → −2.4° on the real Viscous config. | Low |
| **Metal over gs_usb, Linux kernel-driver detach** | `8426d060`, `9d47a343` | Only for a `gs_usb:<serial>` port. Both Viscous records on gavin-linux use the CANable2 over slcan (`/dev/serial/by-id/usb-Openlight_Labs_CANable2_...`). | Change the record's follower port to `gs_usb:<serial>` (the kernel detach is then automatic) | Read-only CAN check first (all seven answer, nothing energized), then a normal teleop | Low |

## Upstream changes taken that do touch Viscous (judgement calls)

These merged in. Each changes behaviour on the motors only in a fault case.
They are listed so Gavin can overrule them.

1. **The return to rest refuses to move on missing or non-finite joint feedback**
   (`d1974ebd`, `maker_rest_pose.return_maker_to_pose`). Before, a joint missing
   from the reading was silently left out of the return. A NaN start value
   interpolated into a NaN setpoint, and `MakerFollower.send_action`'s clamp
   (`max(lo, min(hi, nan))`) turns that into the joint's HIGH soft limit, which is
   a jump. Now the return reports `missing or invalid joint feedback` and the
   stop path releases torque, as it already did after any failed return. On the
   Viscous that means that in this fault the arm is released where it is,
   instead of moving the readable joints first. Kept because the old path could
   command a joint to its limit.
2. **A return to rest that raises now holds torque, visibly** (the
   `_return_before_release` wrapper from `d1974ebd`). If the return itself
   raises on Viscous (rather than reporting a stalled verdict), teleop sets
   `rest_failed` and torque stays on until a second Stop or the lease expiry
   releases it. Before the sync that exception escaped the worker's `finally`
   and skipped the release with no status at all, so this is not worse: the
   arm is held, and the UI says so. Pinned by
   `test_failed_return_that_raises_holds_torque_until_an_explicit_release`.
3. **Teleop worker clears `teleoperation_active` before the return** (part of
   `d1974ebd`). It changes status only: during an error-exit cleanup the status
   now reads not-active and releasing. Motor commands are unchanged.

Bug fixes taken as-is with no Viscous motor effect: hosting RPCs registered before
Portal connects (`a2925312`), MJPG
before preview dimensions (`d8cf7f2a`), stop heartbeating a gone session
(`cd57c722`), step-calibration session scoping, auto-calibration batch status, the
LAN-node `policy_device`, the Modal launch id, the SDK and its routes.

## Testing (gavin-linux only)

Two worktrees beside the live checkout, each with its own Python 3.12 venv
(`uv venv` + `uv pip install -e '.[test,gs-usb,arm-dynamics]'`). The
`arm-dynamics` and `gs-usb` extras are there so the Viscous gravity and
transport tests run instead of skipping; both trees used the same extras.

- merged: `~/Documents/makermodslab-viscous-upstream-sync` (kept);
- baseline: `~/Documents/makermodslab-viscous-sync-baseline` at `bcb15bf0`
  (removed afterwards).

The runner script is `~/tmp/viscous-sync/run_checks.sh` on gavin-linux. pytest
ran with `-o addopts=` and no `--maxfail`, so every failure is counted. Ruff is
`ruff@0.14.1`, the pre-commit pin.

| Check | Baseline `bcb15bf0` | Merged (final, `e250411e`) |
|---|---|---|
| `ruff check .` | 4,673 findings (pre-existing, mostly `scripts/`) | 4,673, the identical set (same file, rule, message) |
| `ruff format --check .` | 199 files would reformat | 199, the identical list |
| `pytest` (`tests/`) | 4,653 passed, 1 failed, 17 skipped | 4,891 passed, 1 failed (the same test), 17 skipped |
| `pytest sdk/tests` (new) | n/a | 427 passed |
| New `test_viscous_upstream_optouts.py` | n/a | 11 passed |
| OpenAPI export | already up to date | already up to date (`rest_failed` committed in `e250411e`) |
| `npm ci` | ok | ok |
| `npm run lint` | 0 errors, 34 warnings | 0 errors, 34 warnings |
| `tsc -p tsconfig.app.json` / `tsconfig.node.json` | clean / clean | clean / clean |
| `npm run build` | ok (output not committed) | ok (output not committed) |
| `vitest run` (extra) | 610 passed | 621 passed |
| `tools/sync_maker_urdf.py --check` (new CI step) | n/a | ok |

The first merged run (`a52f0863`) had 3 new pytest failures and 3 SDK failures.
All were fixed in `ac2519ac` as described under the conflicts. Nine baseline test
ids are absent from the merged run because upstream removed or renamed them
(for example `test_new_robot_default_is_metal_only`, now that Maker also
defaults to holding, and the Maker wiggle cases at the old −120.1° limit).

The one pytest failure common to both trees,
`tests/test_arm_registry.py::test_single_device_configs_carry_the_port_and_id[viscous]`,
predates this sync. The generic registry test builds a follower with the id
`cal-f`, and `ViscousFamily` refuses any follower id without a commissioned
hardware profile (by design, so it never runs on Maker defaults). It is left
as it was.

Ruff check reports the same 4,673 findings, file for file and rule for rule,
in both trees, almost all under `scripts/`. Ruff format lists the same files
in both, once the three this sync wrote were formatted.

## Independent review (2026-11-14)

Three reviewers read the branch at `ae7877d3` independently, each given the
code and the requirements, not the author's conclusions: Codex (gpt-6.1-sol,
conflict correctness), Astra (gpt-6-astra, Viscous motor safety) and Claude
Fable 5.2 (tests, API contract, dependencies, hygiene). None found a blocker,
and none found a path by which Maker gravity, gains, gripper holding or the
fault guard reach Viscous in normal operation. Fable reproduced the test counts
exactly and ran ten mutations against the opt-out tests (nine caught; the miss
is fixed below). Fixed after review:

- **Camera reconnect could delay, and hide, motor faults** (Astra). The
  wrapper caught every exception whenever any camera reader was dead, and
  retried for ~5 s without servicing Stop. The follower's `get_observation`
  runs its bus-silence and thermal checks before reading cameras, so a motor
  fault coinciding with a dead camera was retried, and could disappear if the
  second read worked. Now (all families) only a camera read error is
  recovered, everything else propagates unchanged, and Stop ends the retries
  between attempts; Viscous is opted out entirely
  (`recording_recovers_dead_cameras = False`). This narrowing applies to the
  Maker and Metal too and is worth sending upstream.
- **Re-alignment clamp reached Viscous** (Codex, Astra): now opted out
  (`recording_clamps_alignment_targets = False`), with a test of the
  17.6° → −2.4° case Codex reproduced.
- **The hosted-gripper cap was pinned only by one call's argument** (Fable):
  deleting `exempt=` in `remote_host.py` kept every test green. `exempt` is now
  a required keyword on `remote_can.limit_action`, so no call site can get the
  uncapped gripper by default.
- **This file was not committed** (all three): `.gitignore` exception added.
- **SDK Tests never ran on this fork** (Fable): `sdk_tests.yml` push trigger
  now includes `feat/viscous-arm`.
- **`VISCOUS.gravity_feedforward_scale = 0.0` was inert** (Fable): nothing
  reads it; removed, with a comment pointing at the class-name gate (which the
  real-follower test pins).
- **`rest_failed` comment was wrong for Viscous** (Fable): corrected and the
  raising-return case pinned by a test (judgement call 2 above).
- **CLAUDE.md named makermods-robotics' lerobot pin** (Fable): it now names
  ours and says to keep it when merging makermods-robotics.

After the fixes (`c9875e9b`, gavin-linux, same venv and commands as above):
`tests/` 4,897 passed, 1 failed (the same pre-existing
`test_single_device_configs_carry_the_port_and_id[viscous]`), 17 skipped,
which is the 4,891 plus the six new tests; `sdk/tests` 427 passed; the OpenAPI
export has no diff; ruff check the same 4,673 findings and ruff format the
same 199 files as the baseline.

Noted, not changed: 50 of the 121 merged upstream commits carry
`Co-Authored-By: Claude` trailers from upstream's own history. Gavin's
no-attribution rule is about commits written for him; only a squash would drop
them, at the cost of the shared history this merge exists to keep.

## Before this replaces the running Lab

- Gavin reviews this branch and merges it into `feat/viscous-arm`, not into
  `main`. No PR has been opened.
- Rebuild the frontend locally on the machine that will run it
  (`cd frontend && npm ci && npm run build`; never commit `dist`). The fork's
  CI does not rebuild it, and the committed bundle predates upstream's UI work.
- Do a powered acceptance run on unit 1 with every opt-out above in place:
  teleop start and stop (return then release), the env gravity FF at 50%,
  recording with a mid-session re-alignment, and gripper haptics. Talk through
  each phase, per the working rules.
- Only then turn on, one at a time, any feature from the OFF table, each with
  its own hardware check.
