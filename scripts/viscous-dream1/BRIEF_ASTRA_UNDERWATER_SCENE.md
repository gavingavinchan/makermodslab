# Task: an underwater BlueROV flange scene for the Astra harness (no Astra runs)

Work on gavin-linux in `~/dreamtrial/viscous_sim`, same env and hard rules as the earlier Astra briefs: write only in
`astra_op/`, `runs/astra_flange/`, `~/dreamtrial/astra_ws`; no Dream calls, ledger, runner lock or hardware; do not
edit original flange/propeller sources, `viscous_arm.json`, or the other session's
`~/Documents/makermodslab-viscous/scripts/viscous-dream1/cinematic/` (read it freely). Reuse what you learned in
`astra_op/underwater/` (the replay attempt: native ROV meshes as OBJ, mount transforms, the mirrored right tool0
frame, sand, fog).

## Why (Gavin, 2026-10-25)
Astra bolted the flange autonomously on a tabletop (episode `astra_g3_20261025_006`, six attempts, three nut drops).
Replaying that motion on the real BlueROV failed: the native mounts (493.05 mm apart, front of the frame) cannot
track the tabletop hand poses without the arms passing through the ROV. Gavin: "Let's do it properly." Astra will
do the task again, truly autonomously, on the BlueROV2 Heavy with the two Viscous arms on their native mounts,
underwater, with a seabed where the table was. It will get its own lessons from episode 6, and the hope is one
clean success with no dropped nut.

## The scene (`--scene rov`, new `--profile astra_rov`)
1. The BlueROV2 Heavy from `~/Documents/viscous-rov-model` (native URDF, meshes, mount transforms unchanged),
   level (zero pitch and roll), fixed in the world (resting on or just above the seabed; no hydrodynamics). The two
   Viscous arms on their native mounts with native kinematics and limits, driven by the same controllers and
   physics as now (same actuator limits as `--profile astra`, same jaw model, grasp welds, thread model). **The ROV
   body, frame, thrusters and foam must be collision geometry** (simplified convex hulls or boxes are fine) so an
   arm that hits the vehicle is stopped by physics and the stall guard reports it.
2. The pipe run with flanges, supports, studs, the loose bolt and nut: same geometry and materials. Place the
   flange relative to the ROV so the right arm can reach the empty hole's bolt tip coaxially with useful wrist roll
   range for `screw`, and the left arm can help (hand-offs) without hitting the vehicle. Prove it with the native
   IK and a clearance check before anything else.
3. Seabed: sand at a sensible height below the work (a dropped nut lands on it and stays reachable). If the
   geometry allows, the same height relative to the pipe as the old table top.
4. Underwater look in every camera Astra sees: blue-green colour and depth fog, light falloff, sparse marine snow.
   Keep it moderate (clear water): the nut, bolt thread end, jaws and flange holes must stay easy to see in the
   wrist cameras at working distance. Check with a contact sheet before validating.
5. Astra's cameras: `rov` = the BlueROV's own front camera (inside the dome, looking forward and down at the work;
   pick an angle that sees both hands and the flange joint), replacing `top`; `wrist_l` and `wrist_r` rigid to
   the hands as now. `robot point` works for all three. Update `README_OPERATOR.md` for this scene: what the
   robot is (a BlueROV2 Heavy with two Viscous arms), the frame (ROV-relative axes described in words), the
   cameras, limits, and that the vehicle body is solid. No object coordinates.
6. G3 start as in `--profile astra` (bolt through both rings, 33 mm protrusion, right-hand thread, nut in the
   right gripper held across the flats), with a sensible starting arm pose clear of the vehicle and fixture.

## Also fix `screw` (from `BRIEF_ASTRA_SCREW_EFFORT.md`, unresolved)
The seated nut keeps spinning without a torque rise, so `screw` cannot detect snug. Add a seat stop to the
harness thread model: when the nut face reaches the flange face (or the nut contacts the flange), further
tightening advance is blocked, so rotation stops and wrist torque rises (a stiff but finite seating, e.g. a short
compliant stop). Then the adaptive snug detector (baseline + 1.5 Nm, 0.3 s) or a simpler one must stop
`screw` with `stall` when seated, and must not false-stall on an imperfect but valid grasp (test 2 mm off centre,
6 deg tilt). This applies to both scenes.

## Validate
1. Native IK reach and clearance report for the placement (both arms, start pose, hole, bolt tip, hand-off zone).
2. Contact sheet of the three Astra cameras at the start and at the bolt tip.
3. Scripted CLI-only G3 validation in the ROV scene (privileged targets allowed only to choose targets, marked as
   a cheat): align, catch, `screw right 3600`, which stops `stall` when seated; nut seated; no arm/ROV
   intersections. Video `runs/astra_flange/validation/rov_g3/run.mp4` (three panes: an external wide shot of the
   ROV at the flange, the flange close-up, the right wrist; underwater look).
4. Tabletop `--profile astra` still passes its scripted G3 with the new seat stop; released-nut and pull-off
   checks still pass.
5. Short section at the top of `runs/astra_flange/harness_report.md` with the exact launch command for an
   `astra_rov` episode. Leave no server running.
