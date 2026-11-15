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

# Dream-1 page for the aquarium Viscous arm

Goal B3 in [viscous-haptics.md](viscous-haptics.md). Gavin missed the camera problem because the terminal runner had no preview. `/dream` now follows the Playground's camera stage and prompt flow, with the runner's exact request JPEGs and status. It is fork-only, enabled by the family's `supports_dream` capability. The engine remains `scripts/viscous-dream1/real/dream_real.py`; its safety settings and graceful return remain in the runner.

Validated offline on gavin-linux, on `feat/viscous-dream-page`, based on the November upstream sync (`d2cdb986`). No hardware, billed calls, real camera reads, or input approvals were performed. The Viscous upstream opt-outs remain pinned OFF. Review and supervised Mac acceptance are still required.

## On the Mac

Use a source checkout of this branch; the runner and its configuration files must be present. Stop the previous Lab instance through its normal shutdown before starting this one. Run Lab from a desktop Terminal so its child can open the standalone Tk stop button. Use the existing Viscous Lab state home and commissioned record, not an empty home.

```bash
cd <this-branch-checkout>
uv sync --extra test
(cd frontend && npm ci)
MAKERMODSLAB_HOME="$HOME/.makermods/makermodslab-viscous" uv run makermodslab --dev
```

Open the frontend address printed by the launcher (normally `http://localhost:8080`); select **Dream-1 Playground** on the Launchpad, or open `/dream`. Development mode serves the new source without changing `frontend/dist`. This branch does not ship a hand-built dist bundle.

1. Select the single Viscous follower record. This UI exposes the existing aquarium runner, whose tank model, tool point and boxes must already be commissioned for this rig. Adding another Viscous record is not commissioning another environment.
2. Enter the task. Open **Settings** or **Cameras** and map Front (`overhead`), Left (`wrist_l`) and Right (`wrist_r`). Camera names are record data. Rotations come from Robot settings. Black slots are deliberate; the right slot can also copy Left.
3. Idle tiles use Lab's existing rotated camera preview and mark the runner's integer-pixel 16:9 crop. Check that the arm is inside that band. These are labelled **live camera, crop shown**.
4. Click **Check cameras**. Lab first reserves the cameras and closes its previews; the child runs `--preview`, opening only the record's cameras. The tiles now show the exact 320×176 JPEGs from the client's request observation, plus the inputs id. This does not connect the arm or contact Dream.
5. Review the images, then click **Inputs look right** yourself. This is the only action that invokes `dream.input_check <id>`. A changed prompt or mapping clears the page's review, and a new run requires a fresh page camera check, so do the final review with the final settings. A failed/expired review requires another check.
6. Click **Run**. This runs the dry run with the live settings on fakes, then opens a confirmation dialog containing the runner's verbatim **Joints that may move (ALL SEVEN)** block and live-readiness refusals. A plan expires after 20 minutes or a record change; it also expires at server restart. Only **Confirm and run** starts a session.
7. After confirmation, Lab claims a `dream` session and attaches the tab's lease before letting the worker launch anything. It starts the standalone red stop button first. If the button cannot stay open, it reports how to start it by hand and never starts the runner. The live runner receives the plan id, pid file, and exactly its own Lab session id; every other idle/readiness refusal remains.
8. Watch **Working on**, the phase/call count, last reason, peak torque/temperature, and the tiles. The page shows the newest JPEG in each actual request; it is not a continuously refreshed raw camera during a run. Telemetry is available below. The page heartbeats its own lease; leaving it abandoned causes the server to request a graceful return.
9. **Return to rest**, lease expiry, shutdown and supervision errors request a remote key press from the independent red button. The button alone signals the runner: SIGTERM means STOP only; SIGUSR1 means physical-click release-all. Repeated remote requests and Space/Enter/Escape never release torque, including during a hold or with stale status. A physical click at least one second after STOP, or a click while the runner is holding, sends SIGUSR1. Every remote request logs its source in the button window and supervisor log. If the button disappears, the runner initiates its normal return internally (`stop button gone`), without a signal. A frozen button also triggers this internal stop after its first heartbeat becomes more than two seconds old (`stop button not responding`).

The normal safety preparations from the [runner README](../scripts/viscous-dream1/real/README.md) and [tank plan](dream1-tank.md) still apply, including the base placement and supervision. This page does not commission the arm, validate the physical camera placement, or approve its own inputs.

## Deployment settings and saved data

All settings below are server environment variables, set before starting Lab. No API key is accepted by Lab's page or endpoints; Dream retains its existing credentials path.

| Variable | Default / purpose |
| --- | --- |
| `MAKERMODSLAB_DREAM_VENV` | `~/viscous-dream-real/.venv`; Python 3.12 with `dreamlabs==1.0.37`, `dreamlabs-client==1.1.3` |
| `MAKERMODSLAB_DREAM_ARM_FILE` | `~/viscous-dream-real/station/arms/viscous-waterproof-tank.json`; the existing tool-calibrated tank arm file |
| `MAKERMODSLAB_DREAM_LAB_URL` | `http://127.0.0.1:8000`; the runner's read-only check of this Lab server; set it if its port changes |
| `MAKERMODSLAB_DREAM_RUN_DATA` | `run_data.RUN_DATA`, currently `~/.makermods/makermodslab-viscous/run-data`; the existing ring of the last ten raw bundles |
| `MAKERMODSLAB_DREAM_REPLAY` | A particular saved bundle; otherwise the newest inactive bundle with frames, ticks, chunks, console and plan under the run-data root |

Lab's supervisor work directories/logs live under `$MAKERMODSLAB_HOME/dream/work`; plans and pending input-review tokens are in memory. Raw bundles retain the runner's existing ring/KEEP behavior. Replay reconstruction uses a hidden `.dream-replay-cache` under the run-data root, ignored by the ring; it is disposable and does not contain source media committed to the repository. Supervisor work directories and the replay cache are separate from the ten-run ring and are not automatically pruned.

For terminal-free normal runs, the Mac must already have the commissioned profile, tool calibration, model approval, cameras, Dream environment and credentials. Installation, `dreamcli tcp`, model/profile commissioning, changing the deployment paths, and recovering a lost display/process still require the terminal or existing setup tools. Beta is the runner's only checkpoint; there is no model flag and Flash is not offered.

The family has `telemetry_kind="degrees"` and no `frontend/public` Viscous URDF registration. The stretch 3D/ghost/path viewer is therefore skipped; measured/commanded degrees are shown. The requested TCP path is still present in the status API for a later viewer.

## Replay without an arm

No replay media is bundled in git. On gavin-linux the provided source is:

```bash
MAKERMODSLAB_DREAM_REPLAY="$HOME/dream-page-ref/run-data/live-20261114T091037Z" uv run makermodslab --dev
```

Open `/dream?demo=1` (or click **Replay demo**). Play/pause and scrub the 59-second aquarium run; swap camera tiles and expand telemetry/phases. Replay opens no camera, imports no Dream client, starts no subprocess and claims no session. Motion/approval controls are disabled. It never changes the current live-run pointer. An active Dream session takes priority over demo display, including a stale `?demo=1` URL.

Raw saved JPEGs already have the record rotation applied, so replay uses the same crop/resize/JPEG helpers without rotating twice. The old bundle predates exact-request outputs: replay images are explicitly labelled **reconstructed Dream crop**, not original request bytes. Telemetry follows recorded monotonic timestamps. Console phases are recorded; call timestamps are aligned from the chunk ledger, so the reconstruction is approximate between console events.

## API and safety review notes

The six new routes are v1-only: `POST /api/v1/dream/plan`, `/preview`, `/inputs-approval`, and `GET /api/v1/dream/status`, `/view/{run}/{slot}`, `/replay?at_s=...`. Responses have schemas in `schemas/dream.py`; the JPEG route is the reasoned FileResponse exemption. All images/status have `Cache-Control: no-store`; reads only use run-data files. Live starts and stops use the sessions surface, with reciprocal busy checks and `robot.busy.dream` in the taxonomy. The SDK's Dream sugar also requires an already reviewed plan id; page-specific routes are registered as fork additions in its coverage ratchet.

Remote presses are atomic JSON files with target runner pid, request wall time and source under `<pid-file>.press/`, consumed on the button's existing 250 ms Tk timer. This serializes physical and remote presses on the Tk thread, avoids Tk/signal-handler interactions and logs each request's source. Lab never signals the runner. The button is the only new remote input; its window retains the same labels and click lockout. Keys/remotes send at most one STOP per runner pid, preserving the count across transient lookup failures. Requests predating either the runner or current button process, or targeting a different pid, are discarded. A reopened button cannot replay old requests. Terminal runs without `--stop-button-pid` retain the runner's first/second signal semantics.

The button launch check proves the process stays alive for a second, not that the window is physically visible to Gavin. Lab passes the button pid **and creation time** to the runner so pid reuse in the launch gap cannot defeat the dead-man. Verify that on the Mac desktop. Both button and runner are launched through a fresh double-fork helper interpreter, reparented before Lab continues. Each has its own process session; neither appears in the launcher's descendant shutdown snapshot. The detached monitor reaps its child and records exit status. Lab shutdown writes one remote press, logs that the stop continues independently and returns without waiting for a held arm. Lab never terminates either process, including the button after runner exit. The systemd template uses `KillMode=process` so stopping its cgroup signals only the Lab main process, and newly generated macOS LaunchAgent plists set `AbandonProcessGroup=true`. For service deployment, copy/reload the updated systemd template or regenerate the LaunchAgent before powered acceptance; this fix round does not change installed services. Desktop `--dev` needs no service update.

Each Tk tick atomically writes `<pid-file>.button.json` (`pid`, process `start_time`, `wall`, `mono`). After the first matching heartbeat, an age above `--stop-button-timeout-s` (default 2.0 s) causes the runner's ordinary internal graceful stop in the Dream loop; it never aborts an existing return/hold. Missing/corrupt heartbeats expire too. The check does not preempt a blocked call. A stop request left unconsumed for one second appears in the page's existing error alert: **“the red button is not responding: use it directly or cut power”**, even if its process still exists.

On macOS the button uses the existing PyObjC Foundation bridge to hold an `NSProcessInfo` / `NSActivityUserInitiated` activity token for the window lifetime. [Apple documents this as preventing App Nap and deferred work](https://developer.apple.com/library/archive/documentation/Performance/Conceptual/power_efficiency_guidelines_osx/PrioritizeWorkAtTheAppLevel.html). No new dependency, system preference or service change is needed. If the assertion is unavailable, the button logs this and relies on the heartbeat; real Mac locked-screen/covered-window acceptance remains pending. Runner status filters non-finite telemetry to null; write failures are loud in its log, remove the old status when possible, and show “no status” or expired status on the button. The release signal is independent of status freshness.

Closing the desktop Terminal (SIGHUP) follows SIGTERM shutdown in both the dev launcher and Uvicorn worker: Lab writes its normal shutdown press before exiting. Normal return-to-rest cleanup still releases torque at rest. Only aborting a supervised return/hold requires the physical-click SIGUSR1; terminal runs without `--stop-button-pid` retain two-SIGTERM/Ctrl-C semantics.

`$MAKERMODSLAB_HOME/dream/active.json` records runner/button pids and process creation times, pid file, view status path, run/robot and session id. Startup and every `dream_is_active()` reconcile that record, checking the runner's start time and exact script command. A surviving runner retains the robot mutex and appears on the page under its original session id; stop still routes to the button mailbox. The record is removed only after the original runner exits (including pid reuse). Unreadable records/identities fail closed with a logged error. Monotonic browser leases cannot be restored across restart; the original button remains the independent supervisor and dead-man. Status/mutex reads reconcile exit and release the adopted claim.

`--dev` watches only `makermodslab/`; saving runner scripts does not reload Lab. **Do not edit code during a run.** Editing backend files can still reload the server and request a stop.

The server's camera-input approval is bound to the preview token, inputs id,
robot-record digest and expiry, not to evidence of a particular browser click.
An API caller with a valid token can approve it directly; the page's explicit
"Inputs look right" action is an operator workflow, not server-side proof of
a click. This design is unchanged. Port probing and both identification routes
now refuse every robot holder with `robot.busy.*`, before opening adapters.
Idle camera open and Dream reservation share a lock, so a check/run cannot
claim cameras between the final busy check and capture acquisition.
