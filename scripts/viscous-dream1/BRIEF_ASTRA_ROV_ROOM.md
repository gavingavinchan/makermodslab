# Task: give the right arm room to seat the nut on the ROV (no Astra runs)

Work on gavin-linux in `~/dreamtrial/viscous_sim`, same env and hard rules as `BRIEF_ASTRA_CAPTIVE_NUT.md`.

## What happened
Two-handed ROV episodes under `astra_rov_backup`, both clean (one engagement, no drops, the left hand holding the
bolt head the whole time):
- `astra_rov_backup_g3_20261026_002`: 9.70 turns, stopped 2.0 mm from the flange. During the final `screw` strokes
  (t >= 575 s) the right upper arm (`solid_right_collision_upper_arm_shell_01`) presses into the ROV base hulls
  (`solid_rov_base_collision_18_hull4/24/28`), hundreds of contact rows.
- `astra_rov_backup_g3_20261026_003`: 9.65 turns, stopped 2.2 mm from the flange with `stall` (torque rise) at
  t of about 477 s. Its contact list at the end shows only jaw contacts on the nut and bolt head. Find what stopped
  it (joint limit, wrist-roll range, arm torque saturation, contact not in `contact_pairs`, the backup arm, ...).
The scripted held validation seats, but Astra's natural approach runs out of room in the last 2 mm. Astra cannot
fix this by choosing a pose if the vehicle body is in the way of every reasonable elbow configuration.

## Fix
1. Diagnose both stops with exact times, joints, torques and contacts.
2. If the vehicle (or the parking position) is the cause, re-park the ROV relative to the pipe and flange (still
   level, still resting on or above the seabed) so the right arm can run the nut to the seat with its elbow clear
   of the vehicle, the left arm can still hold the bolt head, and the ROV camera still sees the work. Prefer the
   smallest change. Re-run the reach and clearance checks for start, bolt tip, seat and head hold.
3. If something else is the cause, fix it if it is a harness error, or report it plainly if it is real.
4. Keep `astra_rov_backup` as is for reproducibility; put the new placement in profile `astra_rov_backup2`.

## Validate
1. Replay episode 003's commands on the new placement as far as they stay meaningful, and say whether the final
   stop disappears.
2. Scripted held G3 under `astra_rov_backup2`, using a hand pose like Astra's (approach and elbow as in episode 003,
   from its logged joints near the end) rather than the most convenient one: seats with `stall`, no arm/ROV contact.
3. `astra_rov_backup`, `astra_rov` and tabletop `astra` scripted G3 still pass.
4. Short section at the top of `runs/astra_flange/harness_report.md`. Leave no server running.
