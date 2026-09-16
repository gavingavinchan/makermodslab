# Viscous Arm in MakerMods Lab

This fork adds the commissioned Viscous Arm v1 and its purple Star Arm 102
leader to Lab's arm registry. Recording, dataset management, ACT training,
replay and local policy execution use Lab's existing workflows.

The driver pin combines Lab's `5ce2fe41a` baseline with Viscous branch
`00830b582`, preserving the Maker and Metal families. A small driver change
keeps the default policy state at seven joint positions and the action at
seven joint targets. Temperature and torque remain available in raw
observations, and thermal monitoring remains active. Diagnostic datasets
can explicitly opt into `telemetry_in_state=true`; those use a different
state shape and should not be mixed with the default Lab datasets.

## Setup

Install this branch with Python 3.12 and uv:

```sh
git clone --branch feat/viscous-arm https://github.com/gavingavinchan/makermodslab.git
cd makermodslab
git lfs pull
uv sync --extra test
cd frontend
npm ci
npm run build
cd ..
uv run makermodslab --lan --bind tailscale0
```

Building locally is necessary for this feature branch; Lab's existing CI
rebuilds its packaged frontend only on `main` and `staging`. Do not commit
the generated bundle.

Create a single-arm robot and choose **Viscous Arm v1**. Assign its CANable
follower port and CH340 Star leader port. On the commissioned Linux machine,
select the existing `viscous_01` follower calibration and `star_viscous`
leader calibration. Prefer the corresponding `/dev/serial/by-id/` paths.
Add the wrist and front cameras in the usual camera settings.

Follower re-zeroing is deliberately refused before opening the bus: the
arm's measured limits and mapping depend on its commissioned zero. Existing
calibration files can be selected or imported through the normal library.
The Star leader retains the existing zero-pose procedure.

## One-command launcher for Gavin's machines

Run `makermods-viscous` on whichever machine the arms are connected to.
On the MacBook it starts the local Lab at `http://127.0.0.1:8000`; on Linux
it opens `http://100.64.91.4:8000`. It waits for Lab and opens the browser,
without starting a hardware session or restarting an already running server.

To use the Linux desktop from the MacBook instead, run
`makermods-viscous --remote`. This uses the existing `gavin-linux` SSH alias
over Tailscale. The arms and cameras must then be connected to Linux;
datasets and training also live on that machine. `--no-browser` prints the
address without opening a browser.

The installed command links to `scripts/makermods-viscous` in this checkout:

```sh
mkdir -p ~/.local/bin
ln -s "$PWD/scripts/makermods-viscous" ~/.local/bin/makermods-viscous
```

Linux also needs the persistent, on-demand service (already installed on
Gavin's desktop). This template assumes the checkout is at
`~/Documents/makermodslab-viscous` and uses its separate Lab settings:

```sh
mkdir -p ~/.config/systemd/user
cp scripts/makermodslab-viscous.service ~/.config/systemd/user/
systemctl --user daemon-reload
```

On the MacBook the launcher installs its own on-demand LaunchAgent on first
use (`~/Library/LaunchAgents/com.gavin.makermods-viscous.plist`), using this
checkout's virtual environment. Both machines keep Lab settings separately
under `~/.makermods/makermodslab-viscous`; the Mac server log is
`~/.makermods/makermodslab-viscous/server.log`.

The services start through the command, not automatically at login. Local
Mac use does not require Tailscale. Remote use requires Tailscale on both
machines. No `makermods` alias is installed.

## Supported behavior

- Seven RS00 motors, Viscous limits and follow gains, matched Star mapping,
  and the existing 85 C thermal stop come from the Viscous driver.
- Teleoperation, recording and replay reuse the CAN return-to-rest and
  torque-release path; local inference uses the registered Viscous type.
- A missing calibration refuses startup. Bimanual Viscous configurations
  are refused because no bimanual Viscous driver is registered.
- Joint telemetry uses the numeric view; no Viscous URDF is claimed.
- DAgger handover and remote GPU inference remain unavailable for this
  family. Local ACT training and local policy execution are available.

The arm has no brakes. Start folded, with the leader at its starting pose.
The existing Viscous driver notes still apply, including the unresolved
folded wrist-flex stop and untuned shoulder/elbow gains. A passing software
test does not validate a physical return path or a learned policy.

## Validation

`tests/test_viscous_arm.py` checks registry/UI metadata, preserved calibration,
missing-file refusal, matched configs, seven-value policy state, inference
arguments, and rejection of follower re-zeroing and unsupported layouts.
The GUI test creates a Viscous record using the server manifest and checks
that bimanual selection is disabled.

The full backend suite passed (4,162 tests, 19 skipped), as did all 593
frontend tests, the production build, lint/type checks and the OpenAPI
snapshot check. A subsequent fresh-process import regression and the arm
registry suite passed (174 tests). On the Linux RTX 5070 Ti, a synthetic
two-camera ACT batch passed forward/backward with finite gradients and
produced seven joint targets. That is a software smoke test, not a trained
policy or a hardware rollout.

Linux hardware checks on 2026-09-15 read all seven follower motors without
enabling torque, checked the Star's starting pose, and captured both cameras
at 640 x 480. Motor temperatures were 34–38 C. These were read-only checks.
Gavin subsequently tested powered teleoperation in Lab on the Linux desktop
and reported that it works. Recording and physical policy execution still
require separate bench validation.

MacBook checks after moving both arms on 2026-09-15 found the CANable at
`/dev/cu.usbmodem2050389538461` and the Star at `/dev/cu.wchusbserial10`.
Both calibration files matched Linux byte-for-byte. All seven follower
motors answered with temperatures of 34–38 C, and every leader joint passed
the preset's 30-degree starting-pose tolerance. Lab's live port-probe API
correctly identified both ports, and the local `viscous_01` record reports
both sides ready. These checks did not enable torque or command motion.
Only built-in Mac cameras were detected; wrist/front cameras remain
unassigned on the Mac. Local and remote launcher modes passed, and repeating
the local command preserved the running server's PID.
