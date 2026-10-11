# Viscous Arm on the ROV: safe-workspace envelope (plan)

Status: planned, not started. Written 2026-10-11.

## Problem

`viscous_waterproof_01` is mounted on the ROV and can hit the hull. Joint
limits alone can't prevent that, because whether the arm collides depends on
the combination of joints. The ROV CAD is out of date, so the envelope is
painted by hand rather than computed from the hull model.

## Current state

- Follower joint 1 (shoulder_pan) was re-zeroed on the ROV mount on
  2026-10-11. Only motor 1 was zeroed. Just before the write it read 265.52 deg,
  which is -94.48 deg in the old table-calibrated frame. Datasets recorded on
  this unit before that date have J1 in the old frame. The unit's record is
  `~/Documents/viscous-arm-calib/waterproof-2026-10-08/j1-rezero-2026-10-11.json`.
- Soft limits and the leader mapping were left unchanged. If J1 really sat a
  quarter turn from its old zero (near-certain: 265.52 deg is outside the
  captured stops, so the reading is a whole turn off), the measured stops are
  now about -32.4 / +282.0 deg, while the soft limit still allows -127 deg.
- Powered teleop from the Lab GUI works after the re-zero.

## Approach: paint the envelope by hand

1. **Record.** With torque off, move the arm by hand through everything it may
   reach, up to and gently touching the hull. Lab reads all seven joints at
   30-50 Hz without enabling the motors. Painting by hand was chosen over
   teleop because it is safer near the hull.
2. **Mark safe space.** Forward kinematics through the arm URDF
   (`~/Documents/viscous-arm-model/model/viscous_arm.urdf`, which has collision
   geometry) places every link's collision shape for each sample. Each link's
   shape is pre-voxelised once in its own frame (1-2 cm cubes). Each sample
   transforms those points and marks the cubes they land in. The union of all
   samples is the safe envelope, stored in the arm's base frame, so the mount
   transform and ROV model are not needed.
3. **Enforce.** On every teleop or policy command, place the commanded pose's
   link shapes the same way. They must lie inside the envelope, eroded by a
   margin. Otherwise hold the last safe pose. The check is on the command, not
   on where the arm actually is, so the margin must also cover tracking lag,
   which is larger in water.

## Live visualisation (VR play-space style)

- Show the live 3D Viscous arm in Lab, driven by its actual joint readings.
  This needs the Viscous URDF wired into Lab's URDF view, which is currently
  numeric only for this family.
- Draw the painted region growing in real time as translucent cubes or a
  surface.
- Show holes in red: pockets enclosed by painted space that nothing has
  passed through yet, found by a flood fill from the outside. This is the
  "missed a spot" cue.
- Separate the hull from unfinished edges: pressing a marker, for example the
  Star leader's lock button, while touching the hull marks that edge as
  confirmed wall. The remaining open edges show as unexplored.
- During teleop, show the envelope and highlight any link that nears or
  crosses its edge.
- Compute on the server, so the stored envelope and the enforced one are the
  same file, and stream new cubes over the existing joint-data WebSocket.

## Order of work

1. Verify the encoder-to-URDF mapping per joint (sign, zero), including
   today's J1 re-zero. Map gripper degrees to the URDF's prismatic fingers. The
   model README states the mapping is unverified. Pose the arm by hand and
   compare it against the model viewer.
2. Add the Viscous URDF to Lab's live 3D view.
3. Add the read-only painting recorder with the live envelope, hole and hull
   display.
4. Enforce the envelope on teleop and policy commands.

## Caveats

- Coverage near the hull decides everything: space never swept is forbidden.
- Re-paint after any change to the ROV (payload, thruster, re-mount).
- Self-collision is a separate check, done with the URDF.
- Paint on land, mounted on the ROV: water doesn't change the geometry.
- Use the updated ROV CAD (`~/Documents/viscous-rov-model`) later to
  cross-check the painted envelope, or to allow regions that are hard to reach
  by hand.
