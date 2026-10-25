# Edgeward strokes and offset coverage on the unchanged bench

Goal B3a: Gavin asked to go toward the blade edge and cover different dirty strips.
This experiment changes only scripted motion on the corrected 960 mm propeller.
The 52 mm ellipsoidal pad, bases, fixtures, joint limits, friction, forces,
left outer-edge pregrasp and fixed shaft are unchanged. It makes no Dream calls
and accesses no hardware. The parent owns goal-doc updates and commits/pushes.

The final results and their numerical limits are in `runs/propeller_edge/check.json`,
`coverage_metrics.json` and `validation.json`. `contact_detail.mp4` and
`diagnostic.mp4` replay the physical run in real time using the original cameras.

## Selected result: four fresh lanes, near the edge

The final **28.5 s** continuous run makes an initial outward half-pass, then
shifts laterally at every reversal: inward, outward, inward. There are no pose
resets between lanes. Commanded `t=.54–.944`, `u=.13,.17,.21,.25`; six-second
full strokes and 1.5-second smooth lateral shifts preserve contact. The initial
half-pass spans 70.52 mm; the following lane spans are **142.24 / 142.41 /
138.71 mm**. Overall actual radial span is **143.10 mm**. Both complete
reversal-to-reversal strokes are 142.81 / 143.10 mm at 100% simultaneous contact;
the final inward stroke ends in a hold, so it is not counted as another reversal.

- **All 25,500 scrub substeps:** pad contact and both bilateral holds 100%.
- **Conservative swept pad-to-rim clearance:** 1.901 mm. The actual oriented
  nominal disk has 2.514 mm minimum sampled rim clearance (polygon error bound
  0.008 mm). The enclosing footprint reaches `t=.98765`, versus `.8547` in the
  previous wide run. Its maximum radius is 475.12 mm on the 480 mm propeller.
- **Tool/right-finger to left-arm clearance:** 18.661 mm conservative interpolated
  lower bound; 18.705 mm minimum sampled Euclidean distance. The +u offset reaches
  alongside the held region without entering the fingers.
- No unintended contacts. Maximum intended penetration 0.799 mm. Peak pad force
  10.505 N. Left/right peak arm efforts 5.000 / 4.791 Nm. Release drops the free
  tool 620.94 mm. Left edge contacts remain 453.60–478.83 mm radius.
- Exact outer-rim contact is **not claimed**. The remaining 1.9 mm conservative
  margin is intentional; further outward movement risks crossing the rounded
  rim. A prior three-lane candidate left only 0.22 mm and was rejected. Broader
  `.0/.18/.36` lanes lost contact and incurred 4.87 mm IK error near the inner
  endpoint; they are not the presented result.

The four lane centre lines are **7.91 mm apart** at `t=.74`, **23.74 mm** from
first to last. They overlap heavily because the pad is 52 mm across. This adds
real coverage but does not cover the whole blade:

| Unique union measure | Single-strip rerun | Selected four lanes |
|---|---:|---:|
| Actual ellipsoid/surface overlap area | 24.22 cm² | **57.40 cm² (2.37×)** |
| Nominal whole-disk swept footprint | 97.97 cm² | **137.09 cm² (1.40×)** |
| Actual overlap global chordwise width | 33.12 mm | **56.63 mm** |
| Nominal footprint global chordwise width | 68.72 mm | **95.70 mm** |

The face area is 1,251.20 cm²: only approximately **4.59%** is geometrically
touched, or **10.95%** under the nominal disk footprint. The touched overlap
sample nearest the rim is 9.72 mm away; the much closer **pad footprint** does
not imply the entire rigid ellipsoid is loaded. This is exactly why the two
coverage masks are separate. Area resolution refinement changes actual overlap
by 0.026 cm² and nominal footprint by 0.047 cm².

The comparison uses an instrumented rerun of the unchanged wide sine command
(24 s, 131.83 mm actual radial span, 99.9667% contact), versus 25.5 s scrub for
this run. The historical wide result remains untouched at 131.64 mm. Minor
floating-point trajectory evaluation differences lead to slightly different
contact evolution; the rerun is identified separately, not substituted for the
historical evidence. See `coverage_comparison.json`, `coverage_map.png` and
`candidate_summary.json`. All rejected trials remain under `candidates/`.

## Portable contract

`runs/propeller_edge/coverage_plan.json` uses schema
`propeller-edge-coverage-v1`. It identifies the exact corrected blade mesh with
SHA-256, source function `surface(t,u)`, blade 0 and the +Y face. Every 20 Hz frame
contains observed world pad centre, pad rotation, surface normal, inferred actual
`t,u`, commanded `t,u`, lane index, time and actual contact duties from the
preceding 50 one-millisecond physics samples. `left_hold` includes the unchanged
left target and rotation at `t=.97,u=0`.

Only the `scripted scrub` stage contributes coverage. The contact gate is
`frames[i].contact_duty.simultaneous`. The exact 1 ms samples and rotations are
in `substeps.npz`; 20 Hz recorded generalized positions/velocities are in
`physics_replay.npz`. The latter is for source-bench replay, not for copying joint
angles onto a different mount. Retargeting pad poses onto the BlueROV2's native
mounts is **kinematic presentation**, not validated ROV or wet dynamics.

## Coverage means two different measurable things

`coverage_mask.npz` supplies surface triangle centroids (`surface_xyz_m`), their
blade coordinates (`surface_tu`), quadrature areas (`area_m2`) and two first-hit
timestamps. Both only advance when real pad contact and both bilateral holds
exist simultaneously at a recorded 1 ms sample:

- `first_touch_time_s`: the actual oriented pad ellipsoid geometrically contains
  the surface sample. This estimates the area of contact overlap. It is the
  conservative choice for a cleaning-reveal mask.
- `nominal_first_touch_time_s`: the sample lies under the projected nominal
  52 mm circular disk. This is a broader swept-footprint estimate; the rigid
  ellipsoid does **not** bear uniformly over this entire circle.

Infinite timestamps mean untouched. Neither measure is an abrasion simulation
or proof of fouling removal. Treating every contact frame as whole-blade cleaning
would be false. Coverage is integrated on subdivided triangles of the **actual
collision-matched visible mesh**, with 1.5 mm and 1 mm mesh-edge resolutions and
reported convergence differences. Cumulative areas are monotone and overlap is
counted only once.

## Clearance and safety criteria

Every 1 ms substep checks the 26 mm enclosing disk against **every straight rim
segment of the existing collision mesh**, including the rounded tip and both
chord edges. Signed centre containment is also checked. This encloses the pad
at every orientation; a radial-only margin would miss the sloping rounded rim.
A half-step translation bound supplies a conservative clearance for the
piecewise-linear sweep between samples. The exporter additionally measures the
actual oriented nominal disk's rim distance with 128 perimeter vertices and
reports its polygon approximation bound.

`mj_geomDistance` measures all collidable tool geoms and right-finger geoms
against all collidable left-arm geoms at every substep, including separated
pairs that never enter the contact list. Translation and bounding-radius
rotation bounds estimate a lower clearance bound for interpolated motion.
These are numerical simulated-geometry checks, not a hardware collision proof.
The passing gate requires >1 mm enclosing-pad rim clearance and >3 mm left-arm
clearance, >99% simultaneous contact, both holds preserved, <1 mm intended
penetration, no unintended contact, and <0.1 mm positional IK error.

All motion comes from the existing joint servo physics. The only pose setters
are the explicitly initialized pregrasp and separate IK scratch data; live state
is asserted unchanged across IK fits. The tool remains a free body with no weld,
mocap or external forces. Opening the right jaws after the recorded motion tests
release separately. Left-arm 5 Nm saturation remains a limitation; this does not
establish torque margin, pickup, approach, free-shaft anti-rotation, hardware
feasibility, abrasive spin or learned/autonomous behavior.

## Reproduction

Run on gavin-linux from `scripts/viscous-dream1` with the established venv:

```sh
export OPENBLAS_NUM_THREADS=1
export MUJOCO_GL=egl
export TMPDIR="$HOME/dreamtrial/viscous_sim/.cache/tmp"
"$HOME/dreamtrial/.venv/bin/python" propeller_edge_demo.py --centre .742 --amplitude .202
"$HOME/dreamtrial/.venv/bin/python" propeller_edge_export.py
"$HOME/dreamtrial/.venv/bin/python" propeller_edge_compare.py
"$HOME/dreamtrial/.venv/bin/python" propeller_edge_media.py
"$HOME/dreamtrial/.venv/bin/python" propeller_edge_check.py
```

The comparison reads the included instrumented `candidates/single_lane_baseline`
evidence. Regenerate it with `--lanes= --centre .64 --amplitude .19 --seconds 24`
under a fresh candidate name and run the exporter with that same name.

The demo refuses to overwrite an existing `check.json`. For a new experiment,
use `--name candidates/new_name`; accepted evidence is copied to the root result
directory only after selection. All trials are physics-only; only the selected
passing motion is rendered. Baseline source/assets/results remain byte-for-byte
unchanged and are verified against their recorded commits.

Final portable-contract SHA-256 (identical on both machines):
`b18d8c4e36ad3d784e0d4e498b34cf3c4bfb0410bec9e2fc9d1c628dc98df7db`.
