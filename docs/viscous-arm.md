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
at 640 x 480. Motor temperatures were 34–38 C. These were read-only checks;
powered teleoperation, recording and physical policy execution require a
separate bench validation.
