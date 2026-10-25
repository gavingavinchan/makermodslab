# Native BlueROV propeller-cleaning cinematic

An editable 28-second, 1280×720, 24 fps Cycles presentation of the original
BlueROV2 Heavy with its two natively mounted Viscous arms, working on the approved
960 mm, four-blade helical propeller. The environment is a generic single-screw
workboat stern with a shaft boss, hull aperture and aft rudder. There is no seabed.
The stern was rebuilt from additional [SMH Marine dry-dock photographs](https://smh-marine.com/projects.html)
and the [Palumbo Malta shipyard photographs](https://www.palumbogroup.it/palumbo-shipyard-malta).
It now has a closed, fuller hull, broad transom, rising centreline run, shaft boss,
separate rudder and sacrificial anodes. Source links and modeled structural
features are in `assets/stern_references.json`. This is original illustrative
geometry, not a dimensional reconstruction or an imported ship CAD model.

Open `results/final_scene.blend` or play `results/cleaning_preview.mp4`.
`results/contact_sheet.jpg` summarizes the film. The three `comparison_*.png`
images use exactly the same camera to show dirty, partially cleaned and final
surfaces. The film contains no engineering overlays. Reports are separate.
`results/horizontal_profile.png` is an eye-level view of the same saved scene,
so the vehicle's horizontal frame can also be judged without a tilted camera.

## What is verified

- The source is the original **2026-09-18** bimanual ROV, not the later concept.
  `assets/native_rov.blend` is byte-identical to the MakerModsBrain handoff:
  SHA256 `5eeb60fba40c2f6fc5e4f6744a9f94274fb8e361310a61aa9e573cf82ad721ef`,
  97,262,641 bytes. Original project: `viscous-rov-model`. Original user-owned CAD
  archives: `Viscous ROV with Arms-2.step` and `.f3z`, in OneDrive
  `RYGN/viscous robotics/Viscous Robotics Files/CAD/2026-09-18_Bimanual_ROV/`.
  The large archives are deliberately not duplicated here.
- All 100 native mesh instances, their mesh coordinates, local transforms and
  parents remain unchanged, including all eight thrusters and camera/brackets.
  Original arm mounts are 493.05 mm apart. Only the whole-vehicle transform and
  native joint coordinates change. Native rig joint origins match the original
  URDF to float precision. The desktop's unrelated collision-proxy changes did
  not change any of its 28 joint definitions; neither source repository was edited.
- The scene imports the exact four approved STLs from
  `../assets/propeller_corrected/`. Linear subdivision of the working blade adds
  mask resolution without changing its surface. No alternate blade design is used.
- `results/geometry_audit.json` checks the final scene at every film frame and
  intervening half-frame (1,343 samples), with evaluated native FK assertions and
  triangle BVH intersection tests. It includes arms/chassis/thrusters/ship/rudder,
  mutual arms, nonadjacent links, and tool casing. It explicitly lists intended
  grip, pad and native bearing interfaces excluded from intersection rejection.
  This is an intersection audit, not a signed-distance clearance certificate.
- Tool v1.1 is appended from `tool/polisher.blend`, SHA256
  `5e5f76d54c99232f463a3004f1babd84705d8ff6bac6b4582cc9245306124b87`.
  The root is mounted **+20 mm along native right_tool0 Z**; its face-contact
  anchor is therefore **101 mm from tool0**. The clear sleeve is D32×70 mm,
  z=−34…+34 mm in tool coordinates; the pad is D52 mm. Native jaw displacement is
  12 mm per right jaw and 0.8 mm per left jaw. The audit reports actual sleeve
  samples against native jaw triangles (both jaws within 0.0095 mm at tool-local
  z≈−5.10 mm, well inside the clear band) and intended left-jaw/blade contacts.
  The rear cap/seal/index ring/fasteners were narrowed by the tool owner to clear
  the original finger backs. Native fingers were not changed.

## Motion and scope

The frozen observed bench contract is vendored as `assets/coverage_plan.json`,
SHA256 `b18d8c4e36ad3d784e0d4e498b34cf3c4bfb0410bec9e2fc9d1c628dc98df7db`.
It has 570 samples at 20 Hz over 28.5 seconds, four offset lanes and a continuous
shift at every reversal. The selected bench run maintained pad and both holds
through all 25,500 scrub substeps. Its measured radial span was 143.10 mm, with
1.9006 mm conservative nominal rim margin and 18.6605 mm left-arm clearance
bound. **Those bench margins are not native-vehicle clearance measurements.**

The native vehicle uses new inverse-kinematic solutions, never copied bench joint
angles. Gavin rejected the earlier pitched presentation and requires the BlueROV
to be horizontal. `selected_pose.json` now fixes **zero vehicle pitch and roll**,
with native mounts unchanged. The level solution uses these choices:

- `t_native = observed_t`, `u_native = observed_u`: the full **143.10 mm radial
  span** and four overlapping alternating lanes are retained without the earlier
  chord narrowing or inboard shift. Contact anchors follow the analytic surface
  projection of the observed t/u track; this does not reproduce the physics pad's
  full 3D centre pose, tilt or forces.
- The circular pad's axis follows the same analytic normal. Roll about that axis
  is free in the IK solve, with continuity maintained from the previous joint
  state; the cylindrical tool remains rigidly mounted in the native hand.
- The left hand holds the same outer-edge surface location at `t=.97,u=0`.
  Its approach turns by 30.19° within the tangent plane, with the closing axis
  aligned to the blade normal. Both actual CAD jaws are checked against the blade.
- Source scrub time 25.5 seconds is retimed to 22 seconds, film seconds 3–25.
  The initial three seconds hold an initialized contact pose; the final three
  hold the endpoint. No pickup or approach is demonstrated.
  The close shot spans seconds 5–26 continuously, then cuts to the full vehicle.
  No camera pullback travels through the ROV. No lane reset, time reversal,
  branch jump or hidden repositioning is used.
- Vehicle pitch and roll are **0°**. Minimum joint-limit margin is **0.02541 rad
  (1.456°)**. Peak joint speed is **0.4745 rad/s**, peak frame step **0.01977 rad**.
  Full transform, surface residuals and every joint margin are in `report.json`.

This is a **kinematic Blender presentation**. It does not validate wet dynamics,
stationkeeping, thrust, forces, torque, joint backlash, autonomy or hardware
feasibility. No Dream calls or hardware commands are involved.

## What the green-to-copper reveal means

Only the accumulated, contact-gated footprint of the **actual native-FK pad path**
reveals copper. Each texture sample stores its first touch frame; later passes
retain that copper. Cleaning is gated by the source's simultaneous contact duty
and scripted scrub stage. Settle and finish do not grow the reveal. The shader
transitions over two frames; surface bump and apparent fouling thickness are
visual effects, not an abrasion or material-removal simulation.

The presentation uses a **nominal compliant D52 mm footprint**, tangent radius
26 mm and ±3 mm normal tolerance. It is additionally clipped to the source run's
exported nominal union using `assets/allowed_nominal_mask.npz` (conservative
four-nearest surface samples; provenance and source mask hash alongside it).
No mask sample lies outside the native pad footprint or the frozen source nominal
union. Backfaces and the three other blades stay green.

The level presentation reveals **135.41 cm²**, about **10.8% of the 1,250.31 cm²
working face**. This is distinct from the source bench's actual geometric
ellipsoid-touch union **57.40 cm²** (versus 24.22 cm² for the single-strip baseline,
2.37×), and its broader nominal disk union **137.09 cm²** (versus 97.97 cm²,
1.40×). It does not claim a whole blade was cleaned. This presentation footprint
is clipped to the same frozen exported nominal union.

## Reproduce from a fresh repository clone

The original ROV blend, reduced kinematic URDF, observed motion contract, selected
pose, permitted mask, tool asset and approved propeller STLs are all in this
repository. No sibling project under `~/Documents` is needed for rebuilding.
`vendor/bimanual_common.py` comes from the original ROV project; only its local
URDF path and an unused output-directory side effect were changed.
`assets/source_original.urdf` preserves the original source for provenance;
`assets/kinematics.urdf` retains its links/joints without unused mesh paths.

Use Python with **bpy 4.5.3**, NumPy and SciPy for scene work, and a Python
with Pillow for media packaging (the desktop system `python3` has Pillow); install `ffmpeg`/`ffprobe`
on PATH. Rendering uses NVIDIA OptiX. The existing desktop environment was
`~/Documents/viscous-arm-model/.blender-venv/bin/python`; this is an interpreter
location, not a required source checkout. The original rig's inspected drivers
are simple joint expressions and must evaluate when opening the editable file.

From this directory, with `PY` pointing to that interpreter:

```sh
$PY plan_motion.py
$PY build_scene.py --no-render
$PY validate_scene.py
$PY audit_geometry.py
$PY render.py --comparison --frames 1 350 672
$PY render.py --frames 1 350 590 672
$PY render_profile.py
$PY render.py --scratch /tmp/viscous-cinematic-frames
python3 package_media.py --scratch /tmp/viscous-cinematic-frames
python3 manifest.py
```

For the isolated rebuild comparison, run
`$PY verify_rebuild.py --scratch /tmp/viscous-isolated-rebuild` with a new,
nonexistent scratch path. This rebuild does not change the selected blend.

`render.py` resumes existing frame files. Use an empty scratch directory when
changing the scene; cached frames are bound to scene/motion hashes. Camera key
angles use compatible Euler representations, and every film frame checks its
camera aim and up direction. Packaging also rejects nearly uniform raster frames.
Full PNG frames remain in Linux scratch storage, outside the
repository. The final scene packs its image resources and uses relative paths.
Do not re-save or modify the original source blend. Source CAD belongs to its
owner; inclusion here does not invent or grant a new CAD license. Authored scene
scripts follow the repository code notice; vendored helper provenance is retained.

`results/manifest.json` and `results/SHA256SUMS` inventory the compact final
package, including dependency hashes. `report.json`, `geometry_audit.json`,
`mask_report.json`, `provenance.json` and `media_verification.json` hold the
numerical evidence and exact artifact identities.

The isolated-package rebuild reproduced motion and mask bytes exactly, and all
scene mesh coordinates and object/camera transforms matched at twelve keyframes
(`results/rebuild_verification.json`). Extremal motion screenshots are selected
in `results/evidence_selection.json`; no collision worst frame exists because
the dense audit reported zero unintended intersections.
