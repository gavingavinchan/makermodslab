# Gavin's Viscous Arm unit 1 recovery bundle

This is the **original, non-waterproof Viscous Arm v1**, `viscous_01`, with its
purple Star Arm 102 leader, calibration id `star_viscous`. It is not a generic
calibration for other arms, and it is not the waterproof arm
(`../viscous_waterproof_01/`): never select `viscous_01` for that one, or the
reverse. `manifest.json` records the driver pin and the SHA-256 of every file the
restore needs plus the commissioning evidence.

Unit 1 was commissioned in August 2026 (ROM capture and mechanical zero on
2026-08-28, Star leader ranges on 2026-08-29). It currently sits on gavin-linux,
on the CANable 2 adapter with serial `207E37A55445` and a CH340 (`1a86`) USB
serial for the Star leader, and it was the arm used for the dry-bench gravity
identification on 2026-10-20.

## No hardware profiles: the built-in presets are this arm's

Unlike the waterproof bundle, there is no `viscous_profiles/` here. Unit 1's soft
limits and leader mapping are the driver's built-in presets
(`ViscousFollowerConfig` defaults and `RebotArm102LeaderViscousTeleopConfig`),
which were captured on this arm (the follower limits are
`evidence/commissioning/joint-limits-zeroed-frame.json`). `makermodslab/arms/viscous.py`
(`_commissioned_config`) uses those built-ins when the ids are `viscous_01` /
`star_viscous` and no profile file exists, so do **not** create profile files for
these ids. Because the presets live in the driver, the driver pin matters: use
`2a5ea4e581c0d8956b6e58a8e1c6bf8e13486daa` (see `pyproject.toml`).

## Restore on another machine

1. Clone `https://github.com/gavingavinchan/makermodslab.git`, branch
   `feat/viscous-arm`, and follow `docs/viscous-arm.md` to install Lab with its
   pinned LeRobot dependency.
2. With Lab stopped, find the ports for the CAN adapter and the purple Star leader
   on the destination machine (`/dev/serial/by-id/...` on Linux, `/dev/cu.*` on
   macOS). The template ships with both ports blank.
3. From the repository root, replace the example ports and run:

   ```sh
   python3 scripts/restore-viscous-waterproof.py \
     --bundle configs/viscous/viscous_01 \
     --follower-port /dev/cu.YOUR_CAN_ADAPTER \
     --leader-port /dev/cu.YOUR_STAR_LEADER
   ```

   Despite its name the script restores any Viscous bundle; without `--bundle` it
   restores the waterproof arm. The default Lab home is
   `~/.makermods/makermodslab-viscous`; pass `--lab-home` if your server uses
   another `MAKERMODSLAB_HOME`. `--dry-run` prints the destinations without
   creating them. Existing matching files are kept; a differing file makes the
   whole restore refuse before writing anything. It never opens hardware.
4. Start Lab, select `viscous_01`, and check the ports and the cameras before any
   powered test. Support the arm, fold both arms and clear hands/tools first.

The restore writes three files:

- `~/.cache/huggingface/lerobot/calibration/robots/viscous_follower/viscous_01.json`
- `~/.cache/huggingface/lerobot/calibration/teleoperators/rebot_102_leader/star_viscous.json`
- `<lab-home>/robots/viscous_01.json`

**Cameras are per-machine.** The template keeps the record's two OpenCV cameras
(`wrist` at index 0, `front` at index 2, 640x480 @ 30 fps MJPG), which are
gavin-linux's indices. On any other machine, check and fix the indices in Lab's
robot settings before recording.

Do not re-zero this follower or recalibrate the leader just because the computer
changed: motor IDs, zero and watchdog live in actuator flash.

## Gravity feedforward

`gravity-air-fit.json` is the fit from the 2026-10-20 dry-bench identification
(`evidence/gravity-air-*.json` are its two runs; the URDF is
`configs/viscous/model/`). It is off by default. To enable it for teleop, start
the Lab server with:

```sh
MAKERMODSLAB_GRAVITY_FF=configs/viscous/viscous_01/gravity-air-fit.json
MAKERMODSLAB_GRAVITY_FF_SCALE=0.5   # optional, 0..1, default 1
```

The fit is measured in air; it is not valid with the arm in water. These files are
versioned by git rather than the manifest, so refitting does not break the
restore.

## Contents

- `calibration/` — the two LeRobot calibration files the restore installs.
- `robot-record-template.json` — the Lab robot record with ports blanked.
- `evidence/commissioning/` — the August 2026 commissioning captures, original
  names kept:
  - `rom-raw-2026-08-28.json` — first manual ROM sweep, all seven joints, raw
    encoder frame (radians, 0.05 rad stop backoff).
  - `rom-raw-j5-2026-08-28.json` — J5 (`wrist_yaw`) recaptured alone, because the
    first sweep reached only part of its travel.
  - `joint-limits-zeroed-frame.json` — those limits re-expressed in degrees against
    the mechanical zero; these are the built-in follower preset.
  - `star-ranges-2026-08-29.json` — the Star leader's measured servo ranges.
  - `viscous_arm_v1.yaml` — the final SDK capture profile, zeroed frame.
    `viscous_arm_v1.yaml.bak` is the same profile with the first sweep's raw-frame
    limits; `viscous_arm_v1.yaml.prezero` adds the J5 recapture, still in the raw
    frame, just before the zero was set. The YAML gains are capture placeholders,
    not the teleop profile.
- `evidence/gravity-air-*.json`, `gravity-air-fit.json` — see above.

The bundle holds no datasets or policies.
