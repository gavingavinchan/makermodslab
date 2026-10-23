# Pipe-flange scene — 2026-10-23

**Scene preview complete; scripted assembly incomplete.** The right arm lifts the nut, but bolt pickup stalls. No insertion, engagement or nut turning was achieved. Zero inference calls; ledger and both arm JSON hashes are unchanged.

[Front](overview_front.png) · [Top](overview_top.png) · [Close-up](overview_closeup.png) · [Policy sheet](contact_sheet.jpg) · [Demo](scripted_demo.mp4)

Overviews: 1280×720. Policy sheet: actual decoded **320×176** input JPEGs, top/wrist_l/wrist_r, labels outside images. Demo: H.264/yuv420p, 1280×400, 10 fps, 27.8 s, top + inspection camera; visually reviewed and fully decoded.

## Hardware

| Part | Dimensions |
|---|---|
| Flanges | **DN50 PN16 pattern enlarged 2×**: OD 330 mm, PCD 250 mm, four 34 mm clearance holes, thickness 36 mm each; 2 mm face gap |
| Pipes | Two 190 mm sections, OD 120.6 mm, bore 105 mm; assembly x ±226 mm |
| Fixture | Fixed V-block stands; world-x axis, centre (0, −160, 965) mm; table z=760 mm |
| Target hole | Arm side: (0, −35, 965) mm; left arm inserts along −x |
| Bolt | M30×3.5×120; 46 mm AF, simplified 20 mm head; assigned mass 650 g |
| Nut | M30×3.5; 46 mm AF, 26 mm thick, simplified 31 mm bore; assigned mass 180 g |

This is a nonstandard enlarged fixture. References: [DN50 PN16 manufacturer dimensions](https://shawvalves.co.uk/4_reference_data/page/pn16_fl.htm), [M30 dimensions](https://brooksforgings.co.uk/technical-data/industrial-components/hexagon-bolts-and-set-screws/iso-4017-din-933-hexagon-bolts). The 34 mm holes are a modelling choice.

Blue flanges, satin steel tubes, dark stands and bright zinc fasteners replace fruit/bowl/bottle/plate clutter. Tube normals are smooth with sharp rims; hex faces remain flat. Original table, lights, B0 arms/bases/actuators and policy cameras are retained. Only object materials and inspection cameras were added; flange render buffer/constraint arena are 1280×720/512 MB. Bolt/nut have free joints; the fixture has none.

## Physics and native IK

Each flange uses **261 convex collision prisms** preserving all holes. Pipe/nut bores are open. The aligned bolt clears both flanges; a 5 mm radial offset contacts both. No insertion snap or guide force acts on the bolt.

Nut engagement requires <3 mm radial error, <10° axis error, and its leading face within 3 mm of the tip. Every 1 ms an ideal screw manifold preserves contact-driven spin, projects radial displacement/swing and couples advance at **3.5 mm/revolution**. Reverse past the start releases it; cumulative attempted withdrawal exceeding engagement advance +3 mm also releases it. This is a pull-off surrogate, not a stripping-force model. Isolated tests give **7 mm after two turns** and pass entry/release gates. Cosmetic bands are not helical collision threads; torque, backlash, cross-threading and tightening are omitted.

Nominal MJCF jaw opening: **104.825 mm**, versus 46 mm hexes. Final local native client-IK results, using multistart numerical seeds:

| Pose | Arm | Position error | Rotation error |
|---|---|---:|---:|
| Bolt pickup | Left | 0.421 mm | 0.152° |
| Hole entrance | Left | 0.722 mm | 0.325° |
| Through both flanges | Left | 0.543 mm | 0.213° |
| Nut pickup | Right | 0.150 mm | 0.056° |
| Far-side nut | Right | 0.652 mm | 0.865° |

Insertion approaches 15° below horizontal over the arm-side rim. Static held-bolt checks verify axis error <0.12° and no bolt/flange collision. Endpoints have no unintended fixture/table contacts; diagnostic pickup contacts are intentional. Intended division: left inserts, right installs nut. These endpoints do not prove a loaded path or one-arm completion. [IK evidence](validation/ik_reach.json).

## Scripted result and remaining issues

The contact-only attempt executed **555 rows**, without object setters or grasp welds. The shank grasp contacted the bolt, but lift stayed below 0.5 mm. Shoulder tracking error reached 13.5° while TCP height rose only 2.4 mm. Pickup also produced finger/table contacts. Original ±5 Nm joint and ±10 N gripper actuator limits were retained. This is an unstable/stalled physical pickup under B0 controls; grasp geometry, torque and actuator-load contributions are not isolated. The video marks failure and skips insertion.

The right arm lifts the nut **52.5 mm**, then loses it during transport. Insertion **0 mm**, engagement **no**, turns **0**. Command replay reproduces both trajectories exactly. Reliable pickup, transport and physical thread starting remain unfinished: **full dynamic feasibility is not proven**. The unchanged top view sees the flange edge-on; wrist rest fingers partly obscure it. Simplified hex edges, cosmetic thread bands and some rim/shadow aliasing remain visible.

Rows log insertion depth/through-both, engagement/turns/advance, lift/between-finger/contact grasp, object/fixture and gripper/fixture contacts, finger/table contacts, and requested-hand distances to object centres/hole. Contacts accumulate over every physics substep.

## Validation and reproduction

[Audit](validation/final_audit.json): four 48-row stubs through the existing runner, exact interrupted replay/finished skip, **all ledger access forbidden**. [Dry episode](dry/B0/flange/ep_00/log.json). All existing no-call suites pass: sweep check, fake-client runner, 18 baseline modes, one-arm/isolation, propeller IK and four full propeller stubs/replay. [Results](validation/legacy_checks.json). Flange live execution is rejected before scene/client construction.

Use the requested venv/environment: `flange_render.py`, `flange_demo.py`, `flange_demo_replay.py`, `flange_check.py`, `flange_ik_check.py`, `flange_runner_check.py`; runner: `sweep.py --task flange --config B0 --episodes 1`.

Changed: `cross_scenes.py`, `sweep_scene.py`, `sweep.py`, `SWEEP_README.md`. Added: six flange scripts above, `flange_scene.py`, `assets/flange/`, `runs/flange/`. No billed calls, ledger migration, hardware access, package/arm-JSON edits or external-repository edits.
