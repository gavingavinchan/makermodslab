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

# LANDEN 75 aquarium: waterproof Viscous arm environment

Offline model for `viscous_waterproof_01`, supporting haptics goals A1a/A3 and
aquarium sim-to-real goal B4. Gavin (2026-11-07): "create a new environment for
the aquarium, aka something like urdf for simulations ... so that you know the
limits of the aquarium walls. we may or may not need to put up real time
boundary limits to the follower arm. lets see".

`tank.json` is the geometry source. `aquarium_scene.xml` and `previews/*.png`
are generated artifacts. `offline-validation.json` records the offline results.
The script imports no Lab, driver, serial or CAN code and reads no user state.
No real-time boundary is installed or enabled by this work.

## Numbers and coordinate conventions

All JSON distances are **mm**, in the repo URDF's **base_link frame**; MJCF uses
metres. Every parameter carries its source and status. No tank placement has
been tape measured. The arm angle readback was supplied by Gavin; this tool
never reads the arm.

| Parameter | Current value | Provenance/status |
|---|---:|---|
| Outer length × width × height | 750 × 450 × 450 mm | LANDEN Amazon listing supplied in brief; nominal estimate |
| Uniform glass thickness | 8 mm | Unconfirmed third-party LANDEN 75H listing; estimate |
| Near inner face (+x, behind J1) | x = +40 mm | Prior base-back-edge design assumption; estimate |
| Far inner face (−x, arm reach direction) | x = −694 mm | 40 − (750 − 16); derived estimate |
| Left/right inner faces | y = +217 / −217 mm | J1 centred in 434 mm inner width; estimate |
| J1 offset from inner width midpoint | 0 mm | Estimate; positive means J1 toward the +y face |
| Base orientation relative to box | roll/pitch/yaw = 0/0/0° | Estimate; current implementation rejects nonzero rotations |
| Floor inner surface | z = 0 mm | Lowest base collision-mesh z, assuming direct glass support |
| Mat/spacer height | 0 mm | Estimate; a spacer of height h gives floor z = mesh minimum − h |
| Glass rim | z = 442 mm | Floor + outer height − bottom glass thickness |
| Optional water height | 283 mm above floor | Design in `docs/shackle-sim2real.md`; not filled; `null` disables |
| Default margin | 30 mm | Design target from this brief |

Inner length/width are outer minus **two** panes: 734 × 434 mm. The requested
open-top floor-to-rim height subtracts **one** bottom pane, hence 442 mm
(outer-minus-two-glass nominal height would be 434 mm). There is **no lid**.
The six half-space clearances bound the inner box; `rim` reports upper
headroom and is not an overhead physical collision surface. `water_line`
reports headroom below the proposed water surface, for a fully submerged arm;
it is informational and excluded from the default glass sweep.

Motor degrees convert **directly** to URDF radians for J1–J6, without wrapping,
offsets or sign changes; shoulder_pan already has the URDF −Z axis. Gripper
uses an **approximate** linear map: 0° closed, −115° → 52.4 mm displacement
per finger, clamped to [0, 52.4] mm. Thus readback −117.9° is fully open.
The CAD slide limit is 52.4125 mm. The geometry is the repo's CAD collision
model, not a new measurement of waterproof seals, wiring or other hardware.

## What Gavin must measure

Use the **J1 axis**, not the edge of the base plate, as the horizontal datum:

1. J1 axis to the **near short-wall inner glass face**, perpendicular distance
   in mm. This replaces `placement.near_inner_x_mm` (positive +x).
2. J1 axis to **each long-wall inner face**, separately. Mark which physical
   front/back face corresponds to base +y (called left) and −y (called right).
   If those distances are L and R, inner width = L + R and
   `j1_y_from_inner_midline_mm = (R − L)/2`. Confirm the arm's long direction
   is perpendicular to the short wall; measure base yaw if not aligned.
3. Glass thickness, including whether bottom and side panes have the same
   thickness. Current schema assumes uniform thickness; split pane thicknesses
   in the schema/code if they differ. Confirm actual outer length/width/height
   (or measure J1 to the far inner face to cross-check inner length).
4. Does the base plate sit **directly on the inner glass floor**, or on a
   mat/spacer? Measure spacer height under the lowest base collision surface.
   Set spacer height h and `floor_z_mm = base mesh minimum − h`.
5. The black rim-clamped **support tower footprint and height** near the base:
   measure its x/y extents relative to J1, lowest/highest z relative to the
   inner floor, and any clamp overhang into the tank. Record photos/handedness
   so it can be added as an obstacle; it is currently absent.
6. When filled, measure the water height above the inner glass floor.

For each replaced parameter set `status: "measured"` and record date, datum
and method in `source`. Recompute `derived_inner_bounds_mm` and `rim_z_mm`;
`load_tank` rejects stale derived values. Regenerate scene/previews and rerun
tests/clearance/sweeps after changing measurements. For a base yaw/tilt, extend
the plane transforms first: silently treating rotated walls as axis-aligned
would be incorrect.

## Offline commands

Run from repository root. Core commands need only the existing repo Python
and numpy; tests additionally use its installed Pinocchio. `-o` writes JSON
results; sweep also prints a Markdown table. Flags override the selected
reference pose; `--angles` accepts a complete seven-angle JSON object or file.

```bash
.venv/bin/python scripts/viscous_aquarium_env.py clearance --pose read -o /tmp/aquarium-read.json
.venv/bin/python scripts/viscous_aquarium_env.py clearance --pose folded -o /tmp/aquarium-folded.json
.venv/bin/python scripts/viscous_aquarium_env.py clearance --shoulder-pan=182.02 --gripper=-117.9
.venv/bin/python scripts/viscous_aquarium_env.py sweep -o /tmp/aquarium-sweep.json
.venv/bin/python scripts/viscous_aquarium_env.py sweep --margin-mm 0 -o /tmp/aquarium-zero-margin.json
.venv/bin/python -m unittest discover -s tests -p test_viscous_aquarium_env.py -v
```

Importable, load-once interface (no shared mutable FK workspace):

```python
from scripts.viscous_aquarium_env import load_tank, load_model, clearance
tank = load_tank()
model = load_model(tank)
pose = tank["reference_poses_motor_deg"]["read"]["value"]
distances = clearance(model, tank, pose)  # wall -> {distance_mm, link}
```

Only scene generation/check/render use the separate MuJoCo environment:

```bash
~/dreamtrial/.venv/bin/python scripts/viscous_aquarium_env.py scene -o /tmp/aquarium-scene.json
MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python scripts/viscous_aquarium_env.py render -o /tmp/aquarium-render.json
```

MuJoCo is **not** added to Lab's dependencies. MJCF copies URDF joint origins,
axes, inertias, collision geometry and CAD limits, plus the gripper mimic
constraint; mesh paths are relative to the scene. Arm masks 1/2 and glass
masks 2/1 enable arm/glass contacts and disable arm self contacts. The static
base's intentional tangencies do not create static/static MuJoCo contacts.
Water is a visual-only finite plane (thin box). The scene's read/folded
keyframes are kinematic snapshots; **mj_forward only**, no dynamics/actuation
validation. CAD joint limits differ from the waterproof motor soft limits;
these snapshots may exceed CAD limits. Sweep uses the committed waterproof
`evidence/lab-hardware-profile.json` **follower.joint_limits**, without
substituting the rounded calibration ranges or reading installed state.

## Results and interpretation (2026-11-07)

FK tool0 at read: (−526.041, −65.063, 42.820) mm; folded target:
(185.167, 125.275, 148.941) mm. Every link transform agrees with Pinocchio;
MuJoCo tool0 agrees within 1e−9 mm. MuJoCo 3.16.0 loads 8 qpos/175 geoms,
with **0 read-pose arm/glass contacts**, **32 folded-target near-wall
contacts** (deepest finite-pane contact approximately −36.088 mm). Those
finite-pane depths differ from half-space violation: parts already entirely
beyond the glass no longer contact the pane, but remain outside the box.

Full-arm read distances (mm): near **0 (base)**, far **112.647 (right jaw)**,
left **142.000 (base)**, right **87.732 (left jaw)**, floor **0 (base)**,
rim **301.038 (yaw link)**, water line **142.038 (yaw link)**. Folded near
clearance is **−174.529 mm (right jaw)**. No read-pose vertex penetrates;
strictly positive clearance on every wall is impossible with the specified
base-floor tangency. Excluding only the base for diagnosis gives moving-link
near/floor clearances **8.979 / 4.551 mm**; no exclusions apply by default.

The requested **30 mm all-link margin produces no safe samples on any joint**.
The fixed base touches the floor and near wall at every pose. A supplementary
**zero-margin** scan shows the sampled geometric envelope below; it is not a
recommended hardware range. All other joints remain at the supplied read pose.

| Joint | Profile soft limits (deg) | Safe samples at 30 mm | Geometric samples at 0 mm (deg) |
|---|---:|---|---:|
| J1 shoulder_pan | −126.85 to 187.53 | none | 173.02 to 187.53 |
| J2 shoulder_lift | −199.45 to −2.81 | none | −199.45 to −160.83 |
| J3 elbow_flex | 2.86 to 250.04 | none | 170.96 to 237.96 |
| J4 wrist_flex | −122.27 to 82.10 | none | 38.42 to 82.10 |
| J5 wrist_yaw | −86.06 to 87.32 | none | −86.06 to 54.22 |
| J6 wrist_roll | −154.99 to 150.00 | none | −16.46 to 14.54 |

Samples are anchored to each read angle at 1° steps, plus exact soft endpoints;
intervals summarize consecutive passing samples, not a continuous swept-volume
proof. Read shoulder_lift −201.83° is below profile −199.4466°; gripper −117.9°
is below −115.1072°. Those facts are flagged, not normalized away. Other joints
remain at this read pose even when outside their limits.

The initial 1,000-call read benchmark was **0.464 ms median / 0.482 ms p95**
on gavin-linux, including input validation, FK, reductions and result creation,
excluding loading. All **9,624 distinct collision vertices** are retained;
only duplicate coordinates are removed. This fits the 2 ms target, but these
are observed timings, not a hard deadline guarantee or deployed 100 Hz guard.
Before a real boundary, measurements and explicit mount-contact policy are
needed; this work leaves the policy decision open as Gavin requested.

Previews: `previews/read_top.png`, `read_side.png`, `read_three_quarter.png`.
Each is under 1 MB. Glass outlines use the inner faces; no water volume is
rendered because the real tank has not been filled. Tower, wiring, fixtures,
water forces and self-collisions are outside this environment model.
