# Corrected marine screw propeller — bench motion only

Goal B3a, implementing [Gavin's correction](BRIEF_PROPELLER_CORRECT.md): correct
the propeller first, prove motion, and show it before further environment work.
The new `propeller_corrected` profile leaves the entire large bench unchanged.
The original cardinal, large and underwater modules and results are historical
and have not been edited. No BlueROV, hull, seabed or scenery work is included.

## Inspect this result

All artifacts are in [`runs/propeller_corrected`](runs/propeller_corrected/):

- Propeller only: [`propeller_head_on.png`](runs/propeller_corrected/propeller_head_on.png),
  [`propeller_oblique.png`](runs/propeller_corrected/propeller_oblique.png),
  [`propeller_side.png`](runs/propeller_corrected/propeller_side.png).
- Actual settled bench and bilateral interaction:
  [`ready_front.png`](runs/propeller_corrected/ready_front.png),
  [`ready_detail.png`](runs/propeller_corrected/ready_detail.png),
  [`scrub_detail.png`](runs/propeller_corrected/scrub_detail.png).
- [`scripted_feasibility.mp4`](runs/propeller_corrected/scripted_feasibility.mp4):
  15 seconds, real-time, 10 fps, 1600×940. Full bench, contact close-up and all
  three policy images are shown together. This is **scripted simulation**.
- Exact 640×360 policy images:
  [`ready_top.png`](runs/propeller_corrected/ready_top.png),
  [`ready_left.png`](runs/propeller_corrected/ready_left.png),
  [`ready_right.png`](runs/propeller_corrected/ready_right.png).
  `*_top_sensor_full.png` retains the original 640×480 pre-crop sensor image.
- Physical release: [`released_front.png`](runs/propeller_corrected/released_front.png)
  and [`release_trace.json`](runs/propeller_corrected/release_trace.json).
  Release is the separate 1.5-second check immediately following the 15-second clip.
- Machine-readable evidence: [`check.json`](runs/propeller_corrected/check.json),
  [`geometry_check.json`](runs/propeller_corrected/geometry_check.json),
  [`media_check.json`](runs/propeller_corrected/media_check.json),
  `trajectory.jsonl` and `physics_replay.npz`.

The propeller-only diagnostics use a separate model containing just the same
rotor and existing lights. Shadows are disabled **only for those three diagnostic
renders**, to expose actual surface curvature rather than hard self-shadow
boundaries. The dynamics model, bench images, lights and policy rendering are
unchanged. Hard shadow boundaries remain visible in the bench views; they are
not folds or missing blade faces.

## Shape correction and reference

The primary silhouette reference is the manufacturer
[Solé four-blade shaft-fixed propeller photo](https://soleadvance.com/Fotos/FA1628_001.jpg),
checked against [France Hélices FH4-65](https://www.francehelices.fr/wp-content/uploads/helices-4-pales-fh4-65.png)
and the [Palumbo Malta four-blade shipyard photograph](https://images.squarespace-cdn.com/content/v1/584eca8b9de4bb74d3231a73/1544529006672-3EQ3S7EONKSZNYJ565CS/14.JPG).
These were visually inspected before constructing the new sections. Reference
photographs are not used as model assets. This is a conventional illustrative
marine screw family, **not an exact Solé product or validated hydrodynamic design**.

The previous large model scaled a sine-width leaf profile, retained a finite
chord at its terminal station, and pulled the terminal corners backwards with
an arbitrary quadratic radius offset. That produced conspicuous pointed/chopped
ends. The earlier surface was a locally tilted strip rather than a coherent
constant-pitch helicoid. We do not claim its normals were inverted: the visible
problem was principally its section construction and silhouette; hard lighting
also made the surfaces appear creased.

`propeller_corrected_scene.py` replaces that construction with cylindrical
helical sections. At radius `r`, the mean axial advance is `-P*theta/(2*pi)`
with selected pitch **P = 754 mm** (P/D ≈ 0.785). Mean pitch angle decreases
from **54.69° at the buried root to 14.04° at the tip**, with the same sign on
all four blades. Smooth chord growth places broad working area farther out;
chord and thickness taper together to a rounded terminal point. The section
has camber and an elliptical thickness distribution, **28 mm maximum full root
thickness**, at most **6.88° skew**, and **15 mm tip rake**. Roots begin at
85 mm radius, inside the unchanged 102 mm-radius hub. These are selected model
parameters, not dimensions extracted from manufacturer engineering drawings.

The diameter is **960 mm**. Tip directions remain +X, +Z, −X, −Z; all four blades
are exact 90° rotations of one mesh, with no mirrored copies. The old hub, nose,
shaft location and bronze material are retained. The projected blade area ratio
of this model is approximately **0.615**; it is not claimed to match Solé's
published product BAR.

## Collision and environment verification

Each blade has 48 radial × 16 chord convex cells, **3,072 cells total**. Both
surfaces come from the same section grid. Crucially, the visible mesh is assembled
from the cells' actual exterior triangles: **every visible triangle is a triangle
of a real collision cell**. There is no separate smooth decoration concealing a
flat or oversized contact proxy. Interior vertical cell walls match, and the
sum of cell volumes equals the visible solid volume to floating-point precision.

The audit checks every blade and cell for watertightness, consistent outward
winding, positive volume and nondegenerate faces. It checks the rotational
copies, convex projected tiling, section injectivity and a 10.43° minimum angular
separation between neighbouring blade envelopes. Roots are embedded in the hub;
the bottom blade remains **50.000 mm above the table**. Both compiled visual and
collision X/Z extents are 960 mm to float precision.

The analytical surface used by IK and its tessellated mesh have a maximum
sampled nearest-surface discrepancy of **0.440 mm** over all cell centres and
less than **0.008 mm** in the pad working patch. This is discretization of the
same helical surface; the visual and collision triangles themselves match.

The stripped XML trees are **byte-identical after removing blade mesh assets and
blade geoms only**. A second check compares compiled named attributes of 23
non-propeller bodies, 475 non-blade geoms, 17 joints, 14 actuators, five cameras
and two lights. Their values are exactly equal. This includes the table,
risers, stand, arm bases, original policy/bench cameras, tool dimensions,
friction and actuator caps. Vendored arm asset hashes also pass.

## Physical result and remaining limits

A single explicit pregrasp is initialized using IK recomputed from the new
surface. The unchanged servo pipeline then closes the jaws for two seconds,
scrubs for twelve seconds, and holds for one second. It executes two sinusoidal
command cycles. The tool is a free body retained by contact with the right jaws;
the left jaws hold the outer edge of the same left-pointing blade being scrubbed.
No grasp weld, mocap drive, external force or trajectory-time object pose setter
is used. IK scratch pose evaluations restore the simulation state before stepping.

The final run measures:

- Pad contact, bilateral left hold and bilateral right hold: **100% of all
  12,000 scrub physics substeps**, simultaneously.
- Left contact radius: **453.601–478.827 mm**, or **94.50–99.76%** of the 480 mm radius.
- Actual pad radial span: **31.506 mm** (X span 31.481 mm), **three** strict
  continuous-contact reversals, 117.465 mm accumulated continuous-contact travel.
- Maximum intended penetration: **0.772 mm**, dominated by the right handle/jaw
  contact; no unwanted contacts or penetration.
- Peak pad normal force: **8.122 N**. Maximum scrub IK residual: **0.000067 mm**.
- Left arm: **5 Nm cap occupied throughout the scrub**. Right peak: **4.039 Nm**,
  no 5 Nm saturation. Force caps are unchanged.
- Opening the right jaws after the clip drops the free tool **561.829 mm**;
  30 release states and the resulting tabletop image are saved.
- Video: **150/150 frames fully decoded**, 300 finite, monotonically timed
  motion states, plus 30 release states. Hashes are in `media_check.json`.

Compared with the previous leaf geometry's 34.14 mm/four-reversal measurement,
the corrected surface gives slightly less actual travel and three strict
reversals under the same command amplitude and force limits. This is reported
rather than moving the bases or increasing the forces to recover old numbers.

The initialized pregrasp does **not** demonstrate pickup or approach. The shaft
is fixed, so the hold does not establish anti-rotation capability on a free shaft.
The persistent left-arm 5 Nm saturation means this is **not hardware-ready**.
There is no abrasion, material removal, tool-spin or wet-dynamics validation.
No Dream calls, policy execution, autonomy claim or hardware activity occurred.

## Reproduce and ownership

Use the existing simulation environment and versions from the cardinal/large
READMEs. From `scripts/viscous-dream1` on the rendering machine:

```sh
mkdir -p .cache/tmp
MUJOCO_GL=egl TMPDIR="$PWD/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_corrected_demo.py
MUJOCO_GL=egl TMPDIR="$PWD/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_corrected_check.py
MUJOCO_GL=egl TMPDIR="$PWD/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_corrected_media_check.py
```

`--physics-only` omits all rendering; `--no-render` omits video but saves stills.
The new scene builder reuses the old large builder, replacing blade nodes only.
All source meshes are in the repository; there is no runtime sibling checkout.
`generate_assets()` in `propeller_corrected_scene.py` regenerates the new blade
STLs and profile manifest, and runs automatically if the profile is absent.
No package, service or system changes are required or were made.

Only new `propeller_corrected_*.py`, `assets/propeller_corrected`, this README
and `runs/propeller_corrected` belong to this implementation. The parent owns
the project goal record, reference brief and commits/pushes. No commits or
pushes were made by this implementation task.
