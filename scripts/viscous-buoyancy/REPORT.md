<!--
Copyright 2026 MakerMods. All rights reserved.

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

# Waterproof arm buoyancy and loss of torque · 2026-11-18

**The internal foam does not meet Gavin’s glass-impact rule.** The four CAD-clear blanks displace 252.8 cm³. They remove 0.238 kg of apparent weight from a 1.725 kg shoulder load when immersed. At the failed November 14 rehearsal pose the measured shoulder held 5.10 Nm. Adding the modeled change of −0.14 Nm to that measurement gives **4.97 Nm**, effectively at the RS00’s roughly 5 Nm continuous rating. Releasing all joints from that pose puts a jaw on the floor after **0.174 s** at **0.907 m/s current and 0.904 m/s foamed**. These are simulated releases from rest. The recorded hardware event involved a limp shoulder.

The foamed variant supports comparisons of those specific inserts. The evidence does not justify a longer powered idle deadline for the arm. A useful design target would be 0.05–0.15 kg downward apparent mass beyond J2, with lift distributed near the mass on each articulated segment. Slight negative buoyancy favors a supported floor rest. The target is a design choice, and the descent still depends on each joint’s moment and damping. With 60 kg/m³ foam, reaching that total force alone would take 1.68–1.78 litres starting from the current arm. The inserts leave **1.487 kg** downward beyond J2.

## Where the foam fits

Dimensions are X × Y × Z. Centres and dimensions are in millimetres in the named URDF link frame, with zero roll, pitch and yaw. The solver minimized the largest absolute J2, J3 or J4 torque over sampled tank poses, allowing each candidate from empty to full. It selected all four. These fixed pocket candidates model machinable rectangular blanks. Other cuts or external floats change the optimization.

| Link / block | Dimensions (mm) | Centre (mm) | Volume (cm³) |
|---|---|---|---:|
| Upper arm, between J2 and J3 | 96 × 46 × 36 | (−80, 0, 0) | 158.98 |
| Forearm, between crossbars | 40 × 46 × 36 | (79, 0, 58.79) | 66.24 |
| Forearm, proximal end of central blank | 22 × 46 × 20 | (48, 0, 58.79) | 20.24 |
| Forearm, distal end | 8 × 46 × 20 | (103, 0, 58.79) | 7.36 |

The three forearm blanks total **93.84 cm³** and form a stepped insert. The URDF’s visual parts are present in this repo’s propeller-scene assets. They show two upper-arm side plates with inner faces at about Y ±25.5 mm, a distal motor extending to X −131.5 mm, and four forearm crossbars around X 54/104 and Z 44/74 mm. Every blank clears every same-link visual part’s *bounding box* by at least 2.0 mm (2.5 mm upper arm). Every blank is also inside the union of the link’s convex collision pieces. The checks account for nominal CAD solids. Waterproof cable routes, potting, adhesive, straps, tolerances, adjacent-link sweep and service access have no measured allowance. The dimensions describe candidates for the simulation.

A sealed pocket changes the answer. At full immersion, filling a currently flooded void displaces water equal to the foam volume. Filling an already air-filled sealed cavity adds the foam’s **15.2 g** with zero increase in external displacement. The guard’s upper/forearm shell unions enclose 658/644 cm³ while their CAD-part union estimates are 246/206 cm³. Even assigning the entire difference to usable flooded space gives only about 850 cm³ total. Actual insert space is smaller. Treating the whole collision shell as a watertight link would credit substantial lift before inserting anything.

## Holding loads and workspace

The dry values are CAD masses. The next columns give fitted apparent loads in kg-equivalent, summed over the whole chain that each joint supports at full immersion. Multiply by 9.81 for newtons. A moment also needs the pose and lever arms.

| Joint | Dry downstream kg | Apparent kg current | Apparent kg foamed |
|---|---:|---:|---:|
| J1 shoulder pan | 2.906 | 1.996 | 1.758 |
| J2 shoulder lift | 2.456 | 1.725 | 1.487 |
| J3 elbow flex | 1.970 | 1.427 | 1.338 |
| J4 wrist flex | 1.447 | 1.133 | 1.133 |
| J5 wrist yaw | 1.042 | 0.819 | 0.819 |
| J6 wrist roll | 0.644 | 0.499 | 0.499 |

A scrambled Sobol sequence sampled 65,536 poses over the seven commissioned motor ranges. The measured-mount wall rules and cable margins, zero floor penetration and actual finger slides admitted 718 poses. Water surface Z is +255 mm (−28 mm floor plus 283 mm), density 1000 kg/m³. At 183 poses every distal link was immersed. The table uses that common 718-pose envelope before excluding self contact. Its medians describe that sample distribution. They do not estimate operator dwell time or certify a continuous workspace maximum.

| Joint | Current median / worst (Nm) | Foamed median / worst (Nm) | Fully immersed worst, current → foamed (Nm) |
|---|---:|---:|---:|
| J1 shoulder pan | 0 / 0 | 0 / 0 | 0 → 0 |
| J2 shoulder lift | 4.28 / 6.14 | 4.05 / 5.82 | 5.77 → 5.44 |
| J3 elbow flex | 2.43 / 3.97 | 2.38 / 3.95 | 3.17 → 3.09 |
| J4 wrist flex | 0.88 / 1.48 | 0.88 / 1.48 | 1.20 → 1.20 |
| J5 wrist yaw | 0.12 / 0.53 | 0.12 / 0.53 | 0.39 → 0.39 |
| J6 wrist roll | 0.014 / 0.033 | 0.014 / 0.033 | 0.023 → 0.023 |
| J7 empty opposed slides, ideal transmission | 0.00015 / 0.00937 | 0.00015 / 0.00937 | below 0.001 |

After MuJoCo excludes initial contacts, 618 poses remain, 162 fully immersed. The optimizer again selects all four full-size blanks. Across these 618 the median J2/J3/J4 holding torques are 4.51/2.58/0.89 Nm current and 4.27/2.52/0.89 Nm foamed. Their maxima match the table. J2 exceeds 5 Nm in 190/618 current poses and 127/618 foamed poses, including 67/162 and 44/162 fully immersed. Even at the least demanding of the 618 foamed starts, at least one pitch joint needs 1.21 Nm. Internal foam cannot exert a static moment beyond its attachment joints. J7 uses the documented linear slide-to-motor conversion and the difference in finger apparent weights. A grasp, drive friction, cable pull or a finger stop adds torque. The vertical J1 axis has zero modeled gravitational moment.

Foam densities 30 / 60 / 100 kg/m³ give worst J2 of 5.810 / 5.820 / 5.833 Nm. The result depends far more on **available volume, lever arm and immersion**. With the pocket-size constraints removed, the fully immersed minimax solution requests equivalent lift from **1.84 litres at X +103 mm** on the forearm. Its best worst J2/J3 pair is 1.36/1.36 Nm, and J4 still reaches 1.20 Nm. This virtual point lift exceeds the entire forearm envelope.

**Distal alternative.** Two external 90 × 45 × 50 mm floats on the wrist roll link, centres (28, ±105, 0) mm, add **405 cm³** to the internal design. They lie outside the roll link’s ±80 mm width and finish at X +73 mm before the finger meshes begin. Their guard width is **255 mm**. Only 369/718 sampled poses still clear their enlarged guard and the floor, including 58 with the floats fully immersed. In those 58, the worst J2/J3/J4 holds are **3.72 / 2.00 / 0.67 Nm**. Across all 369 they rise to **5.35 / 3.43 / 1.24 Nm**. This is an external-float concept with unknown mount mass, cable routing, lens occlusion and grasp access. The sidecar and actual enlarged guard geometry are included to let those tradeoffs be tested.

## Evidence behind the displacement

The November 7 local evening tank trace (November 8 UTC) recorded hand-guided, torque-off positions and supplies geometry evidence only. For the powered November 8 and 14 runs, 25 distinct tick logs produced 1,197 nonoverlapping stationary windows of at least 0.5 s. The repo selection routine requires less than 1°/s span on every arm axis, no trip, no long gaps and one second since the phase transition. We retained 1,116 windows at least 15 mm from glass/post on the corresponding mount. Median aggregation in 3° J2–J6 bins within each run formed 69 observations in 19 runs. Each run carries equal total fit weight to prevent a two-minute hold overwhelming other poses. Source SHA-256 values and the bin measurements accompany the model.

| Joint | Measured median / max absolute (Nm) | Dry CAD error RMS on wet holds | CAD displacement prior error RMS | Calibrated leave-one-run-out RMS |
|---|---:|---:|---:|---:|
| J1 | 0.08 / 0.32 | 0.10 | 0.10 | 0.10 |
| J2 | 4.29 / 5.15 | 1.28 | 0.97 | 0.21 |
| J3 | 2.88 / 3.17 | 1.19 | 0.49 | 0.27 |
| J4 | 1.82 / 2.41 | 0.48 | 0.99 | 0.75 |
| J5 | 0.08 / 0.37 | 0.11 | 0.12 | 0.11 |
| J6 | 0.09 / 0.23 | 0.12 | 0.12 | 0.12 |

The rehearsal cutout pose measured J2/J3/J4 at 5.10/3.07/1.45 Nm. Dry CAD predicts 6.38/4.18/1.47 and the calibrated wet model predicts 4.80/3.18/1.14 Nm. The second cutout’s 134 s hold was unlogged. Only its recorded pre-stop windows enter the fit. J4 has a +0.61 Nm mean measurement-minus-model error. A wrist model predicting only 1.48 Nm over a workspace sweep cannot bound the real 2.41 Nm measured max. The largest held-out absolute J2/J3/J4 errors were 1.04/1.40/1.24 Nm. Fitting November 8 alone and testing November 14 gives 0.33/0.19/0.73 Nm RMS on those three axes. Some November 14 poses repeat November 8. This tests repeatability across days more directly than extrapolation to new arm shapes.

| Link | Dry kg | CAD occupied cm³ | Effective cm³ |
|---|---:|---:|---:|
| Base | 0.602 | 232.8 | 232.8 |
| Yaw | 0.450 | 178.9 | 178.9 |
| Upper arm | 0.487 | 246.0 | 188.5 |
| Forearm | 0.523 | 206.1 | 229.4 |
| Wrist pitch | 0.405 | 165.4 | 91.2 |
| Wrist yaw | 0.398 | 163.9 | 77.5 |
| Wrist roll | 0.565 | 207.9 | 72.8 |
| Left / right finger | 0.041 / 0.039 | 37.6 / 34.9 | 37.6 / 34.9 |

* CAD prior. The integrator unions parts on a 2 mm voxel-centre grid with ray/triangle intersections so overlapping solids count once. A 3 mm repeat changes the estimated forearm by 8.95 cm³, the largest change. A watertight positive-volume part excludes water from its material. Each non-volume mesh uses its own convex envelope as a sealed proxy, including the RS00 bodies, two small finger parts and one upper-arm plate. The sidecars list the actual part IDs. This convention can close holes or count motor cavities. No physical hull or trapped gas was measured. The CAD COB is the first moment of this integrated volume. Thirty-two equal-weight spatial centroids per link preserve that first moment and approximate the waterline.
* Calibration. Five relative displacement scales from upper arm to wrist roll fit measured motor torques with nonnegative bounded least squares and a CAD prior. The fixed base and yaw gravity torques cannot identify their volumes. Small fingers keep their priors because their opposite slide travel is weakly observable. Fitted scales span 0.35–2.0 of CAD, capped at the guard volume less the modeled open pockets. These deliberately broad bounds and a 35% prior scale keep the inverse problem physical enough for comparison. Wrist-roll displacement reaches the 0.35 lower bound. Changing the prior scale to 15% or 70% moves upper-arm volume between 86 and 225 cm³ and forearm between 177 and 339 cm³. Foamed worst shoulder across those fits is 5.79–5.87 Nm. Those are sensitivity results, not confidence bounds.
* Identifiability. Torque measures net first moments. Dry CAD mass, missing potting, wires, cable forces, water displacement and motor bias can trade against one another. We retain original dry masses/inertias, fix COB to geometry, and assign residual wet loading to effective displacement. The fitted 73 cm³ for the wrist roll is smaller than the 131 cm³ convex envelope of its RS00 alone. If that motor is sealed, the missing apparent load must be explained elsewhere. Using the unscaled geometric displacement for every link still gives a 5.33 Nm worst foamed shoulder. The non-volume upper plate is another sizeable uncertainty: its convex envelope is 82.6 cm³ versus 26.0 cm³ from its unclosed triangle surface integral. Actual air measurements on the waterproof unit also disagree with dry CAD. Across the 619 archived dry windows the CAD J2/J3/J4 errors are 1.55/1.64/0.85 Nm RMS. No wet bin lies within 4° on every arm joint of a dry bin (closest 5.58°). Pairwise subtraction would conflate pose and immersion. Wet J2 covers only −151.7° to −97.1° and J3 +50.4° to +151.6°. The measured loads and run-held-out errors are stronger evidence than wide-pose extrapolation.

## What falls when torque goes away

The independent MuJoCo 3.16 release calculation starts empty-handed at zero velocity, with every actuator off. Of 718 wall-rule-admissible poses, 100 start in collision and are excluded. Both nominal variants reach the floor in **615 of the 618 free starts** within 1.7 s. The metric is the component of the actual first contact point’s velocity toward the floor, including angular motion, before the contact solver applies the first impulse. Sampling can miss a worse pose. The numbers give neither a certified maximum nor a glass fracture threshold.

| First floor contact | Current (m/s) | Internal foam (m/s) |
|---|---:|---:|
| Nominal median across impacts | 0.761 | 0.753 |
| Nominal 95th percentile | 1.020 | 1.026 |
| Highest nominal, fully immersed start | 1.016 | 1.021 |
| Highest nominal, any sampled start | 1.248 | 1.205 |
| Highest across weak-resistance case | 1.689 | 1.782 |
| Highest across strong-resistance case | 1.018 | 1.000 |

The fastest nominal contacts are the current right jaw at 0.228 s and the foamed wrist pitch link at 0.324 s. The exact motor vectors, contact link, 3D speed and time are in the drop results. The two maxima occur at different partially emerged poses. Repeating those starts at 1 ms instead of 2 ms gives 1.245 and 1.203 m/s. At the hardware rehearsal pose the tool0 reference descends about **95 mm** and shifts sideways. Six seconds later the model still has 3/4 floor contacts current/foamed, respectively. Floor-supported rest is the observed passive outcome in these two examples. A small residual motion remains in both; arbitrary unpowered poses did not persist.

* Dynamics. Exact dry URDF masses and inertia tensors, real foam box inertia, locked-base kinematics, independent opposed jaw slides with their equality constraint, convex-part contacts. A fixed child at each COB carries added translational mass and ellipsoid rotational inertia, with its added weight canceled. A nominal added-mass coefficient 0.6 approximates entrained water in the absence of coast-down data. The same inertia remains while crossing the waterline, which can understate acceleration in air. Full dry gravity always acts. Archimedes force acts only on immersed volume samples, and torque uses each sample’s lever about the receiving body COM.
* Drag. Still water with quadratic, axis-dependent drag on volume samples, using ellipsoid projected areas from link extents and local point velocities. The samples distribute drag over the link and resist rotation. Nominal Cd is 1.0. Internal inserts add no *external* area in this approximation. Flow through the plates can change substantially when a pocket fills. At these impact speeds the model uses pressure drag; it omits linear viscous skin friction, fluid memory, surface waves and a tank-wall squeeze film. Coulomb losses of 0.11/0.04/0.07 Nm on J2/J3/J4 follow the October dry bench values. Other rotary axes assume 0.04 Nm. Actual wet friction, gearbox backdrive and joint rotor inertia are unknown.
* Sensitivity. Weak resistance pairs Cd 0.5, added-mass coefficient 0.25 and zero joint friction. Strong resistance pairs 1.6, 1.2 and twice nominal friction. These bracket assumptions deliberately move together. They do not identify separate coefficients from the two cutouts. The 11-14 full-release trajectory, loads after glass contact, payload torque, first-shoulder-only cutout, post geometry away from the arm, hardware joint stops, self collision with uncaptured cabling and foam attachments remain unvalidated. The model uses a full-height plane at the closer +106 mm post face, hardware soft limits plus a 5° stop allowance, dry CAD frictionless finger rails and the source scene’s fixed base contact mask.

## Integration and reproduction

Data live under `configs/viscous/model/buoyancy/`. The sim consumes `current.json` or `foamed.json`. The optional `wrist_alternative.json` describes the separate external concept. For dynamics, start with the **original dry URDF**. In each sidecar, link mass, COM, effective volume and COB refer to the **bare** link. Add every foam block once, using its own mass, inertia and displaced volume. `volume_samples` is an array of 32 equal-volume XYZ points in link-local metres. Their mean equals `cob_m`. Mesh paths in `foam_blocks` are repo-relative. STL vertices are centred at zero, and `center_m` and `rpy` provide placement. `inside_existing_hull` means the guard envelope contains the blank. All foams here assume their occupied space was initially flooded.

The three `*_gravity_only.urdf` files reproduce fully immersed static motor moments for neutral (closed) finger slides in `arm_gravity.GravityModel`. They include the foam collision geometries for guard/FK consumers. Use the original motor map, including negative wrist yaw. For each link the encoded mass is dry minus displaced mass and its first moment is dry first moment minus the water first moment. When net mass is zero or negative, the exporter transfers a point mass at the hinge origin from its parent to the child. Potential energy derivatives are invariant under that transfer, including for the gripper slides locked at neutral. It preserves a pure buoyancy couple with separated COM and COB. The redistributed masses and their inertias have **no dynamics meaning**.

From the worktree root, the saved CAD integration and selected wet bins suffice to regenerate the sidecars on a fresh checkout. Raw regeneration of the November 14 observations also needs the supplied run archive.

```sh
DREAM_PY="$HOME/dreamtrial/.venv/bin/python"
LAB_PY="$HOME/Documents/makermodslab-viscous/.venv/bin/python"
export OPENBLAS_NUM_THREADS=1
"$DREAM_PY" scripts/viscous-buoyancy/evidence.py --archive "$HOME/sim-tank-evidence/mac-tank-runs"
"$DREAM_PY" scripts/viscous-buoyancy/build.py --remeasure
"$DREAM_PY" scripts/viscous-buoyancy/drop.py --workers 6
"$LAB_PY" -m pytest -q tests/test_viscous_buoyancy.py tests/test_arm_gravity.py tests/test_tank_geometry.py tests/test_tank_guard.py
```

Omit the evidence step and `--remeasure` to rebuild from the stored intermediates. `cad.json` gives per-part hashes and 2/3 mm integrations, `wet_observations.json` contains source hashes, counts, motor poses and torques, `analysis.json` records fit errors, pose coverage, density/prior sensitivities and alternatives, and `drops.json` records dynamic settings, collision outcomes, worst starts and convergence checks. Static verification used 192 arbitrary independent six-joint poses per variant. Maximum disagreement with Pinocchio was 3.11 × 10⁻¹⁵ Nm. Six new tests and 62 existing gravity, geometry and guard tests passed. The Lab venv lacked a Ruff executable or module. A locally cached Ruff 0.17.5 binary ran with this repo’s configuration. No hardware, model service, venv installation or Git mutation was needed.
