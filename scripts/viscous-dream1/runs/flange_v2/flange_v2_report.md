# Subsea pipework preview — 2026-10-23

**Scene complete; pickup unresolved. Zero inference/billed calls, real-ledger access or hardware.** `runs/flange/` remains the before.

[Front](overview_front.png) · [Top](overview_top.png) · [Close-up](overview_closeup.png) · [Before/after](before_after.png) · [Call-1 images](contact_sheet.jpg) · [Scripted attempt](scripted_demo.mp4)

## Geometry and materials

| Part | Dimensions / treatment |
|---|---|
| Pipes | Two **600 mm** sections; bare OD **120.6 mm**, bore **105 mm**; x=±117…717 mm. Total run **1.434 m**, overhanging the table; two grounded saddles per section. |
| Weld necks | **80 mm** tapered hubs, OD **184→120.6 mm**; weld beads and **51 mm** exposed coating cutback. |
| Flanges | OD **330 mm**, PCD **250 mm**, **eight 34 mm holes**; chamfered 33 mm rings, 8 mm root gap, 3.4 mm raised faces and 1.2 mm annular gasket. Original outer faces x=±37 mm preserve the **74 mm task stack**. |
| Installed hardware | **Seven 156 mm M30-style studs**, each with two chamfered **46 mm AF** hex nuts and four exposed thread pitches per end. |
| Loose parts | **M30×3.5×120 bolt**, 46 mm AF head, **650 g**; separate **46 mm AF × 26 mm nut**, **180 g**. Free bodies on the table. A bolt avoids requiring a second loose nut. |
| Placement | World-x pipe axis, centre **(0, −240, 965) mm**. Empty lower arm-facing hole **(0, −124.515, 917.165) mm**. Bolt-circle pattern offset 22.5°; pipe run not rotated. |

This is a nonstandard enlarged training fixture, without a DN/PN rating claim. Moving the assembly **80 mm away from the arms** exposes the reachable target to both wrists. Loose spawns, before settling: bolt (120, −40, 787), nut (−70, −40, 774) mm.

Grey epoxy flanges, red-brown coated pipe, exposed steel/welds and blue fluoropolymer-style fasteners use four deterministic PNG textures under `assets/flange/`, sparse wear/chips and satin specular highlights. The **3.3 mm coating envelope** is a visual simplification, not a commercial FBE specification. Explicit split OBJ normals smooth turned surfaces and preserve crisp hex faces. Table, lights, arm models, actuator limits, policy cameras and inspection-camera transforms are unchanged.

## Reach, physics and visibility

[Native IK](validation/ik_reach.json): all five poses pass without unintended contacts. Position/orientation errors: bolt pickup **0.54 mm/0.22°**, entrance **0.82/0.39°**, insertion **0.65/0.31°**, nut pickup **0.52/0.27°**, far-side nut **0.73/0.52°**. Angled pickups clear the table; insertion follows one elbow branch; the right approaches from the side.

[Clearance](validation/clearance_paths.json): **seven paths, 1,368 samples**, ≤0.01 rad joint increments, no arm/fixture, arm/table or inter-arm penetration. Rest is clear. This proves sampled kinematic clearance, not loaded dynamics.

[Physics](validation/physics_checks.json): aligned clearance, offset contact on both rings, fixed fixture, engagement gates, **7 mm advance after two turns**, pull-off release and live rejection pass. Each ring has 320 convex contact prisms. The thread model is **byte-identical** to v1. Visual-only meshes carry zero mass; assigned part masses are preserved.

The contact sheet contains actual decoded **320×176** call-1 JPEGs. Loose parts appear in top plus a wrist; both wrists show the empty hole mouth. [Pixel/occlusion evidence](validation/visibility.json), [marked locations](validation/policy_annotations.jpg).

**Unmet top-view requirement:** the flange remains edge-on and hides the empty hole. Proposed, **not applied**: rotate the pipe run about world z by roughly 25–35°, then repeat reach, clearance and visibility checks. The gasket is only a thin dark ring in the gap, unreadable at policy resolution. Pipe ends crop in the retained inspection framing; small thread crests and shadows still alias.

## Pickup and checks

The contact-only [demo](scripted_demo_log.json) has **555 rows**, no object pose overrides or grasp welds: bolt lift **1.1 mm**, nut **9.9 mm**, insertion/turns **zero**. Jaw contact peaks at **10.4 N**; μ=0.6 gives **6.22 N** friction capacity versus **6.38 N** bolt weight. The jaw reaches **10 N**, arm drives **5 Nm**, and lift tracking error **14.6°**. Finger/table contacts occur; gripper/fixture contact rows are **zero**.

Separate μ=1.2 jaw-liner and **326 g solid-aluminum** diagnostic trials also fail sustained pickup. Neither change was adopted. Friction is marginal, but friction or mass alone is insufficient: arm saturation and grasp/trajectory geometry remain unresolved. **No honest pickup fix was completed.**

Video: **H.264/yuv420p, 1280×400, 10 fps**; full decode passes and command replay has **zero object-position error**. [QA](validation/artifact_qa.json).

All existing no-call suites pass: sweep/FK/clamp, fake-client replay, **18 baseline modes**, one-arm/isolation and propeller IK/metrics/full stub replays. Final suites block the real ledger and remote connections, supply SDK metadata locally, and use synthetic accounting fixtures. [Results/retries](validation/legacy_checks.json). The flange runner passes **4×48 stub rows**, exact replay/skip and its CLI. [Evidence](validation/runner_checks.json). Protected arm JSON hashes are unchanged; no installed packages or external workspaces were edited.

Reproduce with the requested venv, `MUJOCO_GL=egl`, `TMPDIR=$PWD/.cache/tmp`: `flange_render.py`, `flange_check.py`, `flange_ik_check.py`, `flange_clearance_check.py`, `flange_visibility_check.py`, `flange_runner_check.py`. CLI: `sweep.py --task flange --config B0 --episodes 1`. Current flange outputs default to `runs/flange_v2/`; original scripts are in `source_before/`.

## References

[Oil States product/photo](https://oilstates.com/offshore/subsea-pipeline-products/swivel-ring-flange/) informed hub/ring proportions; this model has fixed weld necks, not a swivel mechanism. [Offshore’s subsea tie-in article](https://www.offshore-mag.com/subsea/article/16756366/bolting-unit-allows-flange-tie-in-of-deepwater-pipelines) supplied bolting context. [EPCLand’s RTJ illustrations](https://epcland.com/rtj-flange-engineering-guide/) informed seal comparison; the implemented joint uses a raised face and annular gasket, **not an RTJ groove**. [Hwashin piping hardware](https://www.hwashinbolt.co.kr/product-service?tab=piping-bolt) supplied coated stud/nut examples.
