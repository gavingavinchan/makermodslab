<!-- Copyright 2026 MakerMods. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# Tank wall auto-calibration

Use this optional tool after moving the mount if a tape measurement or the Lab’s hand-guided **Measure tank** is inconvenient. It probes the floor and long walls and writes proposed geometry for Gavin to approve. The default runner, Astra operator and tank guard have no dependency on it.

## Offline use

Run from the repository root on Linux.

```bash
lab=~/Documents/makermodslab-viscous/.venv/bin/python
sim=~/dreamtrial/.venv/bin/python
tool=scripts/viscous-tank-autocal/autocal.py
"$lab" "$tool" plan --spares --out /tmp/autocal-plan.json
OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl "$sim" "$tool" sim --tanks 20 --workers 12 \
  --out /tmp/autocal-validation --video /tmp/autocal.mp4
OPENBLAS_NUM_THREADS=1 "$sim" "$tool" sim --seed 2 --repeat 1 --workers 6 \
  --scenario no_flex flex_severe model_error cable rock rock_wide floor_trap post wrist_post buckle red_button second_signal stale limp missing_wall
OPENBLAS_NUM_THREADS=1 "$sim" "$tool" sim --scenario centred --repeat 20 --workers 12 --out /tmp/autocal-centred
OPENBLAS_NUM_THREADS=1 "$sim" "$tool" sim --scenario finite_panes --repeat 3 --workers 3 --out /tmp/autocal-finite
"$lab" scripts/viscous-tank-autocal/survey.py --workers 8 --out /tmp/autocal-reach.json
"$lab" -m pytest -q tests/test_viscous_tank_autocal.py
"$sim" -m unittest discover -s tests -p test_viscous_tank_autocal_sim.py
```

The dry plan prints each guarded pose assuming preceding touches found nominal glass. Live sessions replan from measured touches. Use fresh output paths. A valid run writes `candidate-tank.PENDING-APPROVAL.json` and `candidate-keepout.PENDING-APPROVAL.json` in its own folder, alongside poses, residuals, rejected probes and the stop record.

## What the jaw measures

1. The post touches the back glass at base (118, 10) mm. The CAD prior places +y at 227 mm, −y at −207 mm and the floor at −28 mm. Back glass is 12 mm behind the post face.
2. The arm seeks four floor points, three on −y, three on +y and one independent check per face. The first −y probe starts about 73 mm inside the wall at x = −470 mm, height 135 mm. Later probes can start 42 mm inside a known wall if the full uncertainty and clearance checks pass.
3. The closed jaw approaches at 3 mm/s. Only the probed face gets a contact allowance. Fingers must lead every other moving link and its cable padding by 15 mm. Measured sag at the lift and standoff compensates the approach within the commissioned joint bounds. Divergence from the local baseline must exceed 3 mm and six standard deviations, grow with command travel and persist for five readings.
4. The two settled press depths must produce the same measured-joint FK plane point within 1.5 mm. The point is the mean finger patch within 0.3 mm of the leading vertex. The trace records lateral slide. A cable can move between depths. A rigid rock can pass both presses, so the spatial fit and held-out checks are essential.
5. The shared `fit_tank` estimates offsets and yaw. If the real +y span is below 150 mm, the tool can derive +y from −y plus 434 mm and reports the number of real +y checks on both proposals, including zero. It flags J1 displacements above 15 mm from centre, yaw above 1.5°, and plane shifts above 10 mm.

## Force, speed and stop

Normal stiffness is `1 / (nᵀ J K⁻¹ Jᵀ n)`. The deep press is capped at 14 mm and 8 N using the worst stiffness along the line. A pose that cannot accommodate 6 mm is refused. Pan uses 30/2, lift and elbow 60/3, wrists 30/2. A 1.5 Nm sustained torque rise stops the advance. At a 3 N wall press the pan sees about 1.0–1.5 Nm. At initial contact its signal can overlap 0.1 Nm friction and 0.2 Nm model error. Divergence is primary.

Retreat is 8 mm/s, slews and returns 5°/s, with a 15°/s command cap and a 15.1°/s measured trip over two readings. The existing red button re-anchors, releases the gripper, checks the return and releases torque at rest. Lost feedback or a limp motor requires HOLD and support. A second press during return releases at that position.

## Reach and offline results

After the floor and −y measurements predict +y, rotating the wrist gives the following guarded reach at nominal yaw. The grid spans six x coordinates and heights of 80, 120 and 160 mm.

| J1 to +y | Feasible cells | x extent |
|---|---:|---:|
| 227 mm, CAD | 14/18 | −440 to −280 mm |
| 217 mm, centred | 15/18 | −440 to −280 mm |
| 207 mm | 18/18 | −480 to −280 mm |

J1 reaches +187.529°. At 227 mm only 9–12 mm of search remains beyond the prediction after reserving the deep press. Moving the mount toward +y by 10 or 20 mm improves this side and moves −y farther away. A longer rounded probe needs measured FK, swept geometry and new force leverage. The simulation uses the commissioned command limits and omits mechanical end stops. Physical J1 and cable clearance need verification.

| MuJoCo trial, paired seeds 1–20 | Valid, rested and released | +y measured / derived |
|---|---:|---:|
| CAD mount with ±30 mm displacement | 20/20 | 9 / 11 |
| Centred mount with ±30 mm displacement | 20/20 | 12 / 8 |

Yaw varied ±3° and floor height ±10 mm. The width was 434 mm in both trials. Seventeen derived candidates had zero +y checks. Worst CAD errors were 2.47 mm floor, 1.07 mm +y and 1.10 mm −y over 600 mm reach, 0.078° yaw and 1.08 mm lateral centre. Across both trials, maximum first-contact speed was 3.18 mm/s and maximum recontact speed 10.85 mm/s. Peak force was 6.20 N over 20 ms and 7.52 N instantaneous.

The 1 ms wet simulation uses 0.15 Nm torque noise, joint friction, water drag and 25% gravity compensation. Three finite-pane repeats of the default inner-plane collision model gave 3/3 fits and 5.87 / 6.86 N. Seed 2 floor errors were 0.05 mm with no flex/play, 2.44 mm with nominal flex, 5.12 mm with severe flex and 4.64 mm with ±0.5° joint-zero error. A failed shoulder hit **15.01 / 35.15 N and 24.37°/s**. The wrist-on-pin impact reached 8.50 N. [Per-run measurements and fault outcomes](evidence-2026-11-18.json).

## Hardware status

Only Claude starts `autocal.py run` on Gavin’s Mac with his explicit go. It checks that the Lab and follower port are idle. The default reads and plans passively. Torque requires `--armed`. Use the run’s PID file with the existing `stop_button.py --pid-file`. Start supported, jaw closed.

Observed wet sag is 27–53 mm. Compensation assumes the load stays comparable during the approach. The start support unloads over three seconds in simulation. Pan 30/2, wet press loads and the powered stop remain untested here. The arm-file TCP is 132 mm from the flange and the CAD fingertip is 166 mm. The tool uses the CAD fingers. A gap at the back post or errors in the jaw geometry, encoder zero or structural flex bias the inferred planes.
