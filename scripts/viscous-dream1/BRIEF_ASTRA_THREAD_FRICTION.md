# Task: a released nut must not spin itself down the thread (no Astra runs)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, same env and hard rules as `BRIEF_ASTRA_ROBUST.md` (write only in
`astra_op/`, `runs/astra_flange/`, `~/dreamtrial/astra_ws`; no Dream calls, ledger, runner lock, hardware, propeller
files or original flange sources).

## What happened (Astra episode 2, `runs/astra_flange/astra_g3_20261025_002*`)
Astra engaged the thread with one `turn right 30 --feed-mm 0.29` (command 81, 0.02 turns), then opened the
gripper (83). From there the nut screwed itself on while the hand was open or only nudging and closing:
0.55 turns at 83, 1.66 at 86, 3.0 at 90, 5.33 at 100, 7.58 at 109, about 0.55 turns per simulated second, with
no `turn` command. In total Astra commanded 30 + 60 degrees of turning. The right-hand thread surrogate in
`astra_op` freewheels: once engaged, nothing resists spin (gravity, contact jitter or a sign or energy error drives
it). A real nut stays where it is when released. This invalidates the episode as an autonomous result: the video
would show the nut turning by itself.

## Fix
1. Find the cause and say what it was (energy injection in the right-hand subclass, missing damping on the screw
   coordinate, gravity torque through the helix, or something else). Check whether `--thread-hand legacy` has the
   same problem.
2. Make the engaged nut behave like a real thread with friction: it does not move along the screw unless the hand
   applies a torque (or a meaningful axial force) beyond a breakaway level, and it stops when that drive stops.
   Use a physically plausible friction or damping on the screw coordinate (state the value and why). Turning by
   the gripper must still work at the operator's normal speeds, and the arm torques it needs stay sensible.
3. No change to entry gates, pitch, hand, release-on-pull, or anything outside the thread model.

## Validate
1. Release test: engage the nut by CLI, open the gripper, wait 10 s of simulated time (`status` repeatedly):
   turns and advance change by less than 0.01 turn. Repeat with the hand nudged around near the nut without
   touching it.
2. Replay Astra episode 2's commands 1-109 against a fresh `--profile astra` server: the nut must now stay near
   0.1 turns after command 83, not climb to 7.6.
3. The scripted G3 validation (`--profile astra`) still passes, >= 2 turns, ideally seated; report the turn count
   per commanded degree of hand rotation (it should be close to 1:1 while gripped and 0 while released).
4. Short section at the top of `runs/astra_flange/harness_report.md` with the cause, the fix and these numbers.
   Leave no server running.
