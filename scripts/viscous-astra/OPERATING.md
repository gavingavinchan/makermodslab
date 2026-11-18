<!--
# Copyright 2026 MakerMods. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
-->

# Operating the waterproof Viscous arm

Gavin owns the red button. Claude starts one server. Astra chooses one move at a time using the MakerMods Metal arm observe–move–observe procedure. Use the `viscous` tool `op` with a single command string. Start with `observe`. After every reply, call `frames` to open both JPEGs named in it. Describe what changed before the next move. The server reserves its last budget slot for shutdown (40 commands by default). The launcher also counts attempted tool commands, paired frame reads and Codex turns against that budget.

## Coordinates and views

- The tip is the arm file's calibrated TCP. `dream_base` shares the physical URDF base_link origin on the J1 axis at the base mounting datum, 57.4 mm below the J1 joint origin in the November model. +x goes forward down the tank, +y to your left looking forward, +z up. Physical base_link x and y are reversed. Use metres for tip/nudge and degrees for angles. Motor telemetry is in unwrapped motor degrees.
- `head` is the KD overhead. The arm should appear on the right. Refraction through the water surface shifts apparent positions. Use it for gross alignment. Trust the already 180°-rotated `left` wrist view near the object. Keep both jaw tips visible. Glare, bubbles or a missing cube warrant another observation or a stop.
- Physical base_link planes in mm are floor −28, rim +414, sides +227/−207, post face +106 and estimated far glass −616. Back glass lies 12 mm beyond the post. Gavin places the post flush on the back glass and between its marks before each session. Wall/rim margin is normally 30 mm with 40 mm cable allowance on most links. The tank backstop allows 20 mm of modeled floor press to accommodate CAD error at rest. The operator restricts the lowest hand vertex to recovery upward once below glass. Reported mm distances are mesh-to-plane estimates.
- The operator caps arm commands at 15°/s and slows to 5°/s within 76 mm of the floor. One move has a 45° default budget summed across the seven motors and all its legs. The gripper's full stroke is about 112°, so change its opening fraction by at most 0.4 per command. The server checks all waypoints before the first write. After a refusal, read the verdict and inspect both views. Hand back if the view stays occluded or the same boundary rejects a smaller move.

## Commands and a 25 mm cube

1. Read `status` or `observe`. Use `clearance.hand_floor_mm` for the lowest hand vertex at the measured joints. `commanded_clearance` gives the same distances at the setpoint. Positive `sag_mm` means the lowest hand vertex is that many millimetres closer to the floor than commanded. The planner carries the current joint errors through the proposed motion to estimate floor clearance. In passive preview the difference includes unexecuted motion. Replay images cycle through a recording independently of virtual motion.
2. `tip X Y Z [--pitch DEG] [--yaw DEG]` requests an absolute TCP. Pitch defaults to −90° (straight down), zero is horizontal. Yaw is the approach azimuth from +x toward +y. At exactly vertical pitch, yaw sets the horizontal jaw-opening axis to yaw + 90°. `nudge forward=0.01 left=-0.005 up=0.005 pitch=2` adds deltas in dream_base to the last **commanded** tip. Omit unused axes.
3. `open`, `close` or `gripper 0.6` set the jaw opening. Hover about 60 mm above the intended grasp. Use a steep reachable pitch, for example −70° to −80°. Center the cube in the wrist view, then descend in 5–10 mm increments, slowing to a few mm near contact. A cube resting on level glass has its middle 12.5 mm above the floor. Check the lowest hand clearance as well as the TCP.
4. Close once. `holding_something` combines a closing target, stationary jaws short of it and measured gripper effort. Confirm in the wrist view with a small vertical test lift. Lift clear before translating, preferably to 120 mm clearance if reachable. Keep enough clearance to put it down before releasing. Avoid sweeping a suspended load near the wall.
5. `rest` retraces the approved arm route toward the measured startup rest while the session stays up. Hand back to the supervisor when finished. The launcher leaves the server running and the red button starts the return. If it reports HOLDING, Gavin physically supports the arm before pressing RELEASE. Read `stop.json` for arrival and release.

Never open devices or cameras, run Python against the arm, start another server, change a calibration, edit outside the session folder, or alter the guard, gains, limits or budget. Only `op` reaches the server. Never assume a grasp from a still image alone.

## Operator setup

Run Claude's server from this checkout with the repo Python environment. Leave the Lab running and idle. Confirm the waterproof unit, camera identity and rotation, mounting marks and torque-off supported starting rest. Open the existing stop button with the same pid file before starting. Gavin gives separate approval before an armed session. Copy the absolute directory printed after READY into the launcher command. Both the shell client and trusted agent bridge read its private connection.json. Use `--tcp 127.0.0.1:0` on serve for an assigned loopback port. The launcher uses a narrow stdio bridge for either transport and disables model shell tools in production.

For a new floor measurement, start an **unarmed** lab session with the motors already torque off. Have Gavin hand-seat the closed jaws gently on the floor, holding still. From a human terminal issue `astra_op.py op --session "$SESSION" calibrate-floor`, lift and re-seat, then issue it again. The lowest jaw-vertex heights must agree within 3 mm. The result applies in that session. Pass its `--floor-calibration "$SESSION/floor.json"` to the next serve only while the mount and model are unchanged. The checked-in keep-out stays at −28 mm.

### Tank simulation

Start the server with the Lab Python. The simulated motors start at the supplied arm file's `rest` with closed jaws. The child runs in the Dream trial environment with EGL. The same `op` and paired `frames` tools work for `ball`, `screwdriver` and `cube`. The simulated overhead camera has an assumed pose. Use it for coarse XY and use measured hand clearance for depth.

```bash
python scripts/viscous-astra/astra_op.py serve --backend sim --armed \
  --sim-variant current --sim-task ball --arm-file "$ARM_FILE" \
  --run-root "$RUN_ROOT" --pid-file "$RUN_ROOT/serve.pid" --tcp 127.0.0.1:0
```

Point `ARM_FILE` at the matching tank arm profile and use a separate `RUN_ROOT` for each campaign. The server polls trips while the agent thinks. The idle timer starts at READY and restarts after each operator reply. After `--idle-deadline-s` (45 by default), the server enters its ordinary protected stop. Set 240 explicitly for a foamed simulation needing the longer interval. A torque or motion trip can stop either variant sooner. The supervisor's sim-only `op truth` and `op mark LABEL` report ground truth in physical base_link metres and annotate the child log, respectively. Both leave the operator budget and idle deadline intact.

### Reproduce the offline fixture pick

From the worktree root, with the repo Python on PATH, use the calibrated TCP from the archived November 17 run. The replay has no physical cube observations. Its 25 mm contact fixture attaches when the simulated jaws close around the specified point.

```bash
archive="$HOME/dream-run-archive/mac-run-data/live-20261117T082827Z"
op=scripts/viscous-astra/astra_op.py
python "$op" serve --backend fake --armed --environment tank --fake-cube .26 0 \
  --arm-file "$archive/inputs/viscous-waterproof-dry-air.json" --frames "$archive/frames"
```

In a second terminal at the same checkout, run this deterministic operator. The close should report a 25 mm opening and holding. The two upward nudges lift the fixture about 60 mm. After `rest`, use the stop button and check `stop.json` for arrival and release.

```bash
export ASTRA_SESSION="$(cat /tmp/viscous-astra.pid.session)"
python scripts/viscous-astra/astra_op.py ops --json \
  'gripper .3' 'tip .435 0 .108 --pitch -35' 'tip .375 0 .087 --pitch -47' \
  'tip .317 0 .066 --pitch -59' 'tip .26 0 .0445 --pitch -70' \
  'nudge up=-.030' 'nudge up=-.030' 'close' 'nudge up=.030' 'nudge up=.030' 'rest'
```
