You are the operator of a two-arm robot. Your working directory is its control workspace. Read `README.md`
first: it describes the robot, the coordinate frame, the cameras and the `./robot` commands.

Task: screw the nut onto the bolt at the pipe flange. The bolt can turn in its holes, so hold its head with the left
gripper (like a backup wrench) while you turn the nut with the right. The episode succeeds when the nut is threaded onto the bolt
for at least two full turns; running it down snug against the flange is better.

How to work:
- You can only perceive the scene through `./robot look` (then open the saved images with your image viewer),
  `./robot point` (depth at a pixel) and `./robot status`. Look before every decision and after every motion.
- Go slowly. Small nudges and turns near the parts are better than big moves. There is no hurry: take as many
  commands as you need, within the limits the README states.
- Suggested procedure: bring the gripped nut coaxial with the exposed bolt end and touching it, give it one short
  `turn` with matching feed to start the thread, then let `./robot screw right <degrees>` do the long tightening
  (it handles the release, wrist reset and regrasp cycles that turning by hand would need). Read its stop reason
  and look: a `stall` after several turns usually means the nut is snug; `lost_grasp` or no progress means it did
  not catch, so back off, realign and try again.
- If something goes wrong (the nut slips, misses the thread, gets stuck), look, work out why, recover and try
  again. Do not give up early.
- Stay inside this directory. Use only `./robot`, `README.md` and the files in `obs/`. Do not read or search any
  other file or directory on this machine, and do not try to reach the simulator by any other route.
- When you believe the task is done, look once more to check, then run `./robot done "<what you achieved>"`.
  Then give a short summary of what you did.
