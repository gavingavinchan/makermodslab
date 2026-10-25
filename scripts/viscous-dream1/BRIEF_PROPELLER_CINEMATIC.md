# Edge coverage and a realistic BlueROV Blender presentation

## Why and current authorization

Goal B3a/B3b, Gavin on 2026-10-24 after reviewing the 12–13 cm strokes:
extend the motion toward the blade edge if possible, and say when it is too
difficult. Now improve the grasped power drill/polisher model, show a dirty
greenish propeller becoming copper/bronze where cleaned, and bring in the actual
BlueROV with its mounted arms. Cover different areas instead of repeatedly
scrubbing one strip. Make the environment realistic, principally in Blender.

Gavin explicitly refined this to a **zigzag/serpentine path** later in the same
session: sweep along one lane, shift across the blade, reverse along the next
lane, and repeat. Throughout the video the pad should enter new dirty areas,
rather than polish one strip repeatedly. Use overlapping lanes to avoid gaps;
report unique swept area and lane spacing. The dirt reveal must follow these
actual passes. Keep the left-hand hold and edge-clearance constraints intact.

Gavin's follow-up explicitly directs us to **MakerMods Brain for the CAD** and
to do **Blender and most heavy work on gavin-linux**. The Mac is the orchestration,
code review and artifact-viewing machine. This authorizes the environment phase
previously held for motion review. Earlier rejected table-underwater/service-frame
and generic-blob-hull previews are not the scene template.

## Authoritative vehicle located in MakerMods Brain

Read on 2026-10-24 through the vault's OpenKnowledge workflow:

- `Side Projects (Gavin)/Viscous Robotics/Viscous Bimanual ROV Model.md`.
- Its source record `Sources/viscous-bimanual-rov-2026-09-18.json`.
- `Side Projects (Gavin)/Viscous Robotics/Large Files.md`.

The documented native **BlueROV2 Heavy with two Viscous arms** lives on Linux at
`/home/gavin/Documents/viscous-rov-model`. The scene is
`blender/viscous_rov.blend`; kinematics are in `model/viscous_rov.urdf` and
`scripts/bimanual_common.py`. The arm mount axes are **493.05 mm apart**. This is
not the separate `concept_2026_10_11` vehicle.

The scene was checked against the Brain source hash on the desktop:

```text
viscous_rov.blend — 97,262,641 bytes
SHA-256 5eeb60fba40c2f6fc5e4f6744a9f94274fb8e361310a61aa9e573cf82ad721ef
```

Original STEP/F3Z exports are indexed under OneDrive Personal,
`RYGN/viscous robotics/Viscous Robotics Files/CAD/2026-09-18_Bimanual_ROV/`;
the handoff is under `Models/2026-09-18_Bimanual_ROV/`. Preserve source archives,
the native Blender file, and unrelated dirty collision/concept work in that
Linux project. Use copies/append operations for the new presentation.

Blender 4.5.3 is already installed in the desktop's existing simulation Blender
venv. The old `viscous-arm-model/cinematic/rov_ship` film offers useful native-rig,
lighting and footprint-reveal techniques, but its different propellers and vessel
layout are not the current approved scene. Keep the corrected 960 mm four-blade
shape and its scale unless a new constraint is explicitly explained.

## Work and integration contracts

1. **Physical edge/coverage exploration:** retain the corrected bench physics
   model and test outward/offset paths, accounting for the hand holding the edge.
   The pad footprint can approach the edge without its centre exiting the face.
   Report the closest safe reachable margin and any inaccessible held region.
   Export observed pad poses, normals, blade coordinates, contact duty and
   coverage in `runs/propeller_edge/coverage_plan.json`, schema
   `propeller-edge-coverage-v1`, surface `sole-inspired-helical-v1`, blade 0,
   +Y face. More than one overlapping lane should clean a visibly wider patch.
2. **Polisher asset:** a detailed, credible sealed rotary/drill-style tool with
   readable housing, machined interfaces, rubber grip, fasteners, backing disc
   and abrasive pad. Preserve a 32 mm diameter grasp zone and 52 mm pad. Blender
   collection `CleaningTool`, root `CleaningToolRoot`, grip anchor at origin,
   pad contact anchor at local `(0,0,0.081)` m, local +Z toward the blade; rotor
   child `ToolRotor`. New surface details are presentation geometry, not a new
   calibrated dynamics model. Own files under `cinematic/tool/`.
3. **Blender integration:** preserve the actual complete ROV CAD hierarchy,
   eight thrusters, camera/bracket and native arm mounts. Solve/retarget to the
   actual URDF frames; bench angles and bench mount spacing cannot be copied
   directly. Position the whole vehicle appropriately rather than remounting
   the arms. Use a coherent referenced ship stern/shaft/propeller/rudder layout,
   open water, work lights and restrained haze/particles. No desk, risers,
   standing service-frame legs or seabed.
4. **Cleaning reveal:** green biofouling initially covers the work surface.
   Reveal copper/bronze only inside the accumulated, contact-gated pad footprint
   along the final retargeted multi-pass path. Cleaned regions persist and
   untouched areas stay fouled. Avoid a global colour fade or invented cleaning
   of unvisited blade regions. Show matched before/after detail views and record
   the cleaned area/path source.

The [previous correction brief](BRIEF_PROPELLER_CORRECT.md) contains the inspected
manufacturer and Palumbo shipyard references. Read their geometry, rather than
inventing a generic hull blob. Full source scenes and rebuild assets should be
self-contained with provenance and original notices; original large CAD archives
stay in their documented home. Keep multi-GB frame caches outside tracked output.

## Deliverable and truthfulness

Provide an editable packed Blender scene, an attractive short cleaning film,
tool preview, clear ROV/ship establishing and working views, and progressive
dirty-to-clean comparisons. Inspect an early scene render before the full movie.
Verify source/mount identity, joint bounds/IK, visible contacts and collisions,
coverage-mask locality, render resources and complete video decode.

The bench motion remains scripted physics with initialized grasps, a fixed shaft
and limited torque margin. Native-ROV retargeting in Blender is a **kinematic
presentation**, not validated wet dynamics, new policy execution or autonomy.
No Dream calls or hardware activity are part of this work. State limitations in
the report and delivery; keep the movie itself visually clean. Parent owns the
goal record and all repository commits/pushes; workers own their separate paths.
