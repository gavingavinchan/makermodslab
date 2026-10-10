# Gavin's waterproof Viscous Arm recovery bundle

This is the **same commissioned physical arm**, `viscous_waterproof_01`, with the
same purple Star Arm 102-HD leader. It is not a generic calibration for other arms.
The bundle includes both calibrations, hardware profiles, a portable robot record,
measured limits, SDK YAML and commissioning/test evidence. `manifest.json` records
the driver/SDK pins and SHA-256 checksums.

Gavin reported small powered movement across all seven joints through Lab's GUI.
Wider tracking and direction checks are pending cable management. J4's rearward
travel is restricted by the current logo; remeasure J4 if that clearance changes.
See `evidence/first-small-motion-test.json` and `evidence/j4-logo-clearance-note.txt`.

## Restore on another machine

1. Clone `https://github.com/gavingavinchan/makermodslab.git`, branch
   `feat/viscous-arm`, and follow `docs/viscous-arm.md` to install Lab and its pinned
   LeRobot dependency and build the frontend. The required driver pin is
   `2a5ea4e581c0d8956b6e58a8e1c6bf8e13486daa`.
2. With Lab stopped, find the ports for the CAN adapter and purple Star leader on
   the destination machine. Linux ports normally use `/dev/serial/by-id/`; macOS
   ports use `/dev/cu.*`. The old Linux port strings are retained only in evidence.
3. From the repository root, replace the example ports and run:

   ```sh
   python3 scripts/restore-viscous-waterproof.py \
     --follower-port /dev/cu.YOUR_CAN_ADAPTER \
     --leader-port /dev/cu.YOUR_STAR_LEADER
   ```

   The default Lab home is `~/.makermods/makermodslab-viscous`, matching the Viscous
   launcher. If your server uses another `MAKERMODSLAB_HOME`, pass that exact
   directory with `--lab-home`. `--dry-run` prints the destination paths without
   creating them. Existing matching files are kept; different files cause the
   entire restore to refuse before writing. Preserve any conflicting file before
   restoring. The script needs only Python's standard library.
4. Start Lab with `makermods-viscous`, select `viscous_waterproof_01`, and check both
   ports before any powered test. Secure/support the arm, fold both arms and clear
   hands/tools before pressing Teleop. Finish cable management before wider moves.

The restore writes these five files:

- `~/.cache/huggingface/lerobot/calibration/robots/viscous_follower/viscous_waterproof_01.json`
- `~/.cache/huggingface/lerobot/calibration/teleoperators/rebot_102_leader/star_viscous_waterproof_01.json`
- `<lab-home>/viscous_profiles/viscous_waterproof_01.json`
- `<lab-home>/viscous_profiles/star_viscous_waterproof_01.json`
- `<lab-home>/robots/viscous_waterproof_01.json`

Calibration JSON alone does not replace the driver's limits: restore the profiles
as well. Do not re-zero this commissioned follower or recalibrate the same leader
just because the computer changed. IDs, motor zero and watchdog live in actuator
flash; the restore script writes only disk files and never opens hardware.

The SDK YAML has zero gains for manual capture; use the Lab hardware profile for
teleop. Policy rollout needs explicit matching limits on its subprocess CLI path
and remains unverified. Camera bindings are empty. This bundle does not contain
datasets, policies or the entire Linux machine state. Original raw capture logs
remain on the offline Linux computer; captured extrema are preserved in
`evidence/commissioning-progress.json`.
