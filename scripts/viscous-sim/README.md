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

# Waterproof tank simulator and validation, 2026-11-18

**The fitted foam still lets the jaws strike the floor.** In two real-time MuJoCo runs, the arm held the November rehearsal target for two seconds and then released all motors. The left finger hit the floor about 0.175 s later in both. The foamed shoulder carried 4.66 Nm at release. The measured hardware rehearsal carried about 5.1 Nm. These runs use still water, the archived distal fingertip TCP and the current and foamed buoyancy sidecars.

| Held release at rehearsal target | J2 / J3 / J4 motor torque just before release, Nm | First floor normal speed | First floor total speed |
|---|---:|---:|---:|
| Current | 4.80 / 3.18 / 1.15 | 0.933 m/s | 0.942 m/s |
| Foamed | 4.66 / 3.15 / 1.15 | 0.926 m/s | 0.949 m/s |

The initial motor degrees were 176.59, −122.90, 99.34, 8.56, −9.00, −4.00 and −2.922. The arm settled under its own PD controller before release. The clip plays at 4× wall speed. Each local output directory contains the 20 Hz joint/truth trace, contact events and picture timestamps. [Current release video](outputs/hold-release-current/sim-video.mp4) · [Foamed release video](outputs/hold-release-foamed/sim-video.mp4)

A second experiment initializes those angles exactly with **zero velocity and zero motor torque**, eliminating differences in pre-release sag. Both first contacts occur at 0.220 s. Nominal normal impact speeds are **0.969 m/s current and 0.974 m/s foamed**. Independently multiplying drag and added mass by 0.5 or 2 gave ranges of **0.667–1.225 m/s** and **0.695–1.220 m/s** over the four corners. These are single-pose scenario ranges. Workspace extremes and uncertainty in displacement, rotor inertia, cables and surface conditions can fall outside them. The retained clips show 0.5–0.7 mm penetration on the first contact step. The solver reports contact speed, and glass fracture strength remains unmeasured.

## Thermal evidence

The sampled shoulder torque falls from 5.13 to effectively 0 Nm at 121.952 s from the first tick of rehearsal 082911Z, **115.497 s after the hold began**. The reported case temperature was 48 °C. The model updates every motor's excess torque-squared integral above 4.5 Nm, with exponential cooling time 300 s. The integral at the measured collapse is 595.936 Nm²·s. Crossing that value latches the individual actuator off for the remainder of the session, including after another engage. Each run starts with zero dose. The other six RS00s inherit the shoulder budget because only the shoulder has a measured trip. The 300 s time constant represents slow recovery and cannot be identified from one complete failure trace.

- **Recorded-torque replay.** Of 30 copied run directories, 25 contain at least two torque samples. The model trips on the rehearsal collapse sample and gives zero trips in the other 24 recorded segments. The highest other peak is 508.376 Nm²·s in rehearsal 222855Z on November 8. The 134 s terminal hold in live 084532Z has no torque samples. Its last recorded dose was 253.764 Nm²·s. A hypothetical constant 4.926 Nm throughout the missing hold reaches the fitted threshold at 134 s. The actual torque and time of collapse are unknown.
- **End-to-end command replay.** Feeding the November 14 rehearsal's commands to the simulated current arm gives 4.80 Nm mean shoulder torque during seconds 100–115. Hardware measured 5.11 Nm in that interval. The simulated shoulder remains energized through 260 s when the last command is extended. A few tenths of a Nm near the knee materially change trip time. The model cannot certify a 45 s or 240 s hardware hold. [Rehearsal command replay](replay_rehearsal.py)
- **Displayed temperature.** A separate first-order model approaches 22 °C plus 1.1573 °C/Nm² times torque squared with a 63.506 s time constant. Least squares on that rehearsal's shoulder trace produced 0.87 °C RMS error. Across the 25 recorded segments with observed initial temperatures the median shoulder RMS error is 2.97 °C, range 0.38–5.08 °C. In the simulator every motor starts at 22 °C. Winding temperature and the manufacturer's trip logic were unobserved. [Torque replay and thermal model](thermal.py)

## Child protocol and outputs

Run `MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python scripts/viscous-sim/child.py`. Each stdin line is one JSON request with `op`. Each stdout line is one response carrying `ok` and the requested fields, or `ok: false` and `error`. All seven motor dictionaries use shoulder_pan through gripper in harness order. All truth coordinates are metres in physical **base_link**. Negate both x and y for the harness's dream_base.

| Operation | Input | Response or action |
|---|---|---|
| `init` | `variant` current/foamed, `task` ball/screwdriver/cube, absolute `arm_file`, absolute `run_dir`, `start` motor degrees | Compiles CAD and sidecar, renders initial camera pair, starts wall clock with motors released. Optional `object_xy` and `target_xy` are base_link metres. |
| `read` | | `pos` degrees, `torque` Nm in motor sign, `temp` °C |
| `engage` | | Sets goals to measured angles and returns `pos`. Thermal cutouts stay latched. |
| `goal` | `target` complete motor-degree dictionary | Updates MIT position setpoints. The parent plans, limits and guards travel. |
| `gripper_off`, `release` | | Disable J7 or all seven motors respectively. Physics continues. |
| `frames` | | `head`, `left` base64 JPEGs and `frame_t_s` for the latest synchronized 480×360 pair |
| `truth` | | `object`, `tip`, `target`, task state and cumulative metrics below. For the screwdriver, object origin is handle midpoint. For lifts, target is initial object origin plus 50 mm in z. For the cube it is the square's center at floor height. |
| `mark`, `close` | arbitrary JSON `label` for mark | Marks are recorded at sim time. Close finalizes the video and summary and exits. EOF also finalizes. |

| Output in run_dir | Contents |
|---|---|
| `sim-telemetry.jsonl` | 20 Hz physical truth, `wall_s`, `t_s`, motor readings and on/off bitmask. `torque_peak_nm` and `torque_abs_integral_nm_s` cover all motion. `hold_peak_nm`, `hold_abs_integral_nm_s` and `hold_time_s` count each enabled motor while its motor-space speed is below 1°/s. `overload_nm2_s` and `limp` expose thermal state. |
| `sim-contacts.jsonl` | Onset of each link/surface contact sampled at 500 Hz. `actor` arm/object, `surface`, `link`, `geom`, `xyz_m`, `t_s`, `normal_m_s` closing speed and `speed_m_s` total speed at that point before the impulse. A continuing slide can have speed at zero normal approach. |
| `sim-summary.json` | Final truth, arm `first_floor_contact`, `peak_floor_normal_m_s`, `peak_glass_contact_speed_m_s`, object `object_peak_glass_contact_speed_m_s`, `min_glass_clearance_m`, `camera_frames`, `missed_camera_snapshots`, `max_backlog_s`. Signed glass plane distances in `clearance_m` come from the arm collision hulls at 20 Hz. Penetrations also update the running minimum every solver step. Post contacts are named separately. |
| `sim-model.json`, `sim-marks.jsonl` | Inputs, simulation source hashes, sidecar byte and content hashes, object properties, water parameters, timestamps and operator labels |
| `sim-video.mp4`, `sim-video-frames.jsonl` | Paired overhead/wrist at 5 Hz of simulated wall time encoded at 20 fps, overlaid with time, shoulder torque, label and success. A separate renderer owns its MjData. Missing camera slots hold their preceding rendered image in the encoding. The frame ledger identifies the actual source times and overwritten snapshots. |

`bilateral_contact` requires contact on both articulated fingers. For the screwdriver, both must touch the handle. `grasped_lift_m` is the highest bilateral lift clear of the floor. For a ball or screwdriver, `success` latches after 0.2 s continuously grasped at ≥50 mm lift. We chose 50 mm for the screwdriver to match the ball and give a visible floor separation. The cube must first lift ≥10 mm with both fingers, then rest on the floor touching neither finger at <10 mm/s and within 20 mm xy of the square for 0.2 s. `peak_lift_m` includes ungrasped motion. All scoring reads simulated bodies and contacts.

## Physical and optical assumptions

- The scene extends the checked-in aquarium MJCF in memory. CAD dry masses, inertias, joint axes, the two coupled prismatic jaws and decomposed convex collision surfaces are preserved. The existing aquarium collision policy excludes arm self-contact. The visible arm uses the repository's Viscous CAD display meshes. Obstacle locations come from the November 13 mount model. Floor −28, rim +414, left +227, right −207, far −616, glass back +118 and post face +106 mm. The post is 80 mm across, centered at y=+10 mm. Pane thickness is the nominal 8 mm. Cable loops, corner fin, water waves, base movement and glass flexure have no physical representation. [Mount evidence](../../configs/viscous/environments/aquarium_landen_75/keepout_viscous_waterproof_01.json)
- Dry weight remains in the mass matrix. Per-body upward force is ρg times immersed volume at its wet centroid, with the COM moment arm applied through `xfrc_applied`. Each link consumes the sidecar's equal-volume points. Foam uses a rotated 4³ midpoint grid, plus a real dry mass and box inertia. Separate gripper/object components have their own inertia and displacement. Fresh water uses 1000 kg/m³, 0.001 Pa·s and surface z=+255 mm. The object waterline approximation clips an ellipsoid fitted to its dimensions, exact for the sphere. A stationary neutrally dense sphere in native MuJoCo's ellipsoid fluid model alone accelerates downward at 9.81 m/s², measured by the included test. [Buoyancy sidecars](../../configs/viscous/model/buoyancy/)
- Water drag uses local projected elliptical areas, component-wise quadratic drag, linear viscous drag, and spin resistance from broadside pressure. Assumed drag coefficient is 1.05 for links, blocks, cube and tool, 0.47 for the ball. The open flooded cavities expose some foam area to flow. Counting the whole block is an upper estimate for shielded surfaces. The default added mass is 0.5ρV at the COB with ellipsoid rotational inertia and compensated weight. This sphere-based coefficient is a proxy for an articulated nonspherical arm. Added inertia stays fixed during partial emergence. The simulator omits fluid coupling, wakes, free-surface effects and ground effect. These approximations motivate independent ×0.5/×2 coefficient sweeps.
- MIT gains (kp in Nm/rad, kd in Nm·s/rad) are (60,4), (90,4), (90,4), three times (30,2), then (20,0.5), from the installed follower config and November 14 tank overrides. Optional arm-profile `gains` and `torque_limits_nm` override defaults. The archive specifies the TCP and rest pose and leaves those fields absent. The 14 Nm peak for J1–J6 permits the measured transients above the nominal 5 Nm continuous rating. For J7 the modeled 10 N URDF effort per pad gives 0.535 Nm total motor torque by virtual work. Actual jaw linkage efficiency, gearbox drag and physical pinch force are unmeasured.
- One symmetric slide transmission maps −2.922° closed to −115.107° open. The tank geometry helper returns up to 52.4 mm travel **per finger**. The full opening increases by 104.8 mm, and the distal CAD faces are about 105.1 mm apart at full travel. The 42.7 mm golf ball fits. The empirical J5 sign reversal comes from `motor_to_urdf.json`. The original base_link pan origin takes full motor pan angle. The operator's shifted pan coordinate belongs to its rotated IK URDF. FK is checked independently against the tank geometry helper. Joint stops assume 5° beyond the soft profile to accommodate known out-of-profile physical rest poses. Assumed rotor inertia is 0.003 kg·m² per motor, reflected through J7 gearing. Static friction Nm are 0.05, 0.11, 0.04, 0.07, 0.03, 0.03, 0.015. J2/J3/J4 use the prior bench measurements. The others are assumed small losses. There are no brakes.
- The cube is 25 mm, 21.875 g at 1400 kg/m³. The golf ball is 45.9 g and displaces 40.765 cm³, leaving 5.135 g downward equivalent. The screwdriver has a Ø6×100 mm steel shaft at 7850 kg/m³ and a solid Ø28×95 mm polymer handle at an assumed 1150 kg/m³. Total is 89.47 g and 61.32 cm³. The shaft initially points along base_link −x. Every object has a free joint and sinks when released above the floor. The nominal grasp feature starts at xy (−0.325,+0.060) m. The cube marker is a 40 mm square at (−0.345,+0.005) m. These are assumed reachable fixture positions for repeatability. Wet contact friction coefficients 0.60–0.65 and 8 ms contact time constant approximate compliant pads and firm glass. Actual wet grip and collision impulse require measurements.
- Head is a downward pinhole at (−0.248,+0.010,+0.710) m, 46° vertical FOV, arm base at image right. No overhead tank calibration exists. The RGB/depth pair is inverse-warped per pixel by iterating ray intersection against its depth map across a flat interface at refractive index 1.333. Single-layer raster depth cannot reconstruct occluded surfaces. Wrist uses the Maker-style bracket estimate, 79.35 mm above and 87.49 mm behind the TCP, pitched down 20°, rotated 180° in the delivered image. A submerged flat port shrinks its assumed 76° air vertical FOV to 55.0°. Air-side reflections, bubbles and lens distortion are omitted. The visually chosen depth tint uses RGB attenuation coefficients (1.3,0.52,0.22) m⁻¹. Neither camera estimates millimetric depth from a pixel alone.
- Physics integrates 2 ms steps against a monotonic deadline. Every owed step executes after scheduler delay. The automated protocol check explicitly pauses and resumes the child for 250 ms and verifies elapsed physics and integrated holding torque. In the two retained 4.5 s captures, both encoded 23 distinct frames and missed zero camera snapshots. Maximum observed physics backlog was 14 ms.

## Reproduction

The drop CLI takes the same `variant`, `task`, `arm_file` and `start` as init. It runs faster than wall time with initial velocities zero and motors released. `--drag-scale` scales the quadratic terms, `--added-scale` scales the entrained-water inertia, and `--telemetry` writes an optional 20 Hz JSONL. A caller performing its own sweeps can use `physics.Tank(req, sidecar=None, drag_scale=1, added_scale=1)`, `step()`, `first_floor` and `truth()`. A perturbed sidecar can be supplied in memory. Final submerged fractions use the sidecar volume sample coordinates. Early development runs before this change used an ellipsoid envelope to estimate arm submersion. Regenerate any sweep made with that earlier version before comparing worst-pose results.

```sh
sim_py=~/dreamtrial/.venv/bin/python
export MUJOCO_GL=egl
"$sim_py" scripts/viscous-sim/test_tank.py -v
"$sim_py" scripts/viscous-sim/thermal.py ~/sim-tank-evidence/mac-tank-runs
"$sim_py" scripts/viscous-sim/replay_rehearsal.py \
  ~/sim-tank-evidence/mac-tank-runs/rehearse-live-20261114T082911Z --end-s 260
"$sim_py" scripts/viscous-sim/drop.py --seconds 3 --drag-scale 1 --added-scale 1 \
  --telemetry /tmp/drop-telemetry.jsonl < init.json
```

Seven tests exercise independent hydrostatic torques on both variants, FK, mass preservation, jaw span, negative buoyancy of all three objects, analytic initial ball acceleration, native fluid behavior, motor disable and thermal latch, actual floor impacts, protocol, camera JPEGs, H.264 video and missed deadlines. The independent hydrostatic torque check uses an absolute tolerance of 10⁻⁷ Nm and aligns the two FK models' gripper endpoint conventions.
