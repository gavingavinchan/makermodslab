# Larger strokes on the unchanged corrected propeller

Goal B3a, [Gavin's larger-motion request](BRIEF_PROPELLER_WIDE.md): “i want larger
motions, the side way motions is not enough.” This is scripted physical motion
on the same corrected 960 mm propeller, from an explicitly initialized pregrasp.

**Actual radial span is 131.641 mm, 4.178× the 31.506 mm baseline.** Three complete
reversal-to-reversal strokes measure **128.354, 122.144 and 125.432 mm**, each with
**100% simultaneous pad contact and both bilateral holds**. The full 24-second
scrub has 99.9667% simultaneous contact: one 8 ms interruption at 2.062–2.069 s,
just after motion begins. Both jaw holds stay at 100%, including that interruption.

## Review the motion

- [Clean fixed-camera contact video](runs/propeller_wide/contact_detail.mp4):
  27 seconds, 10 fps, real time. It uses the **original `cardinal_detail` camera**.
  Captions occupy an added lower margin; simulation pixels are unmodified.
- [Diagnostic video](runs/propeller_wide/diagnostic.mp4): the full bench, original
  detail camera and all three exact 640×360 policy images. Same 27 physical seconds.
- Unaltered contact stills: [start](runs/propeller_wide/start_detail.png),
  [mid](runs/propeller_wide/mid_detail.png), [end](runs/propeller_wide/end_detail.png).
- The visible sweep endpoints: [outer reversal](runs/propeller_wide/outer_reversal_detail.png)
  and [inner reversal](runs/propeller_wide/inner_reversal_detail.png).
- [Released tool on the table](runs/propeller_wide/released_front.png).

The original corrected code and results remain in place. No new geometry,
fixtures, scenery, mounts, camera settings, friction, force limits or assets were
introduced. These are physical servo trajectories, not zoom, accelerated playback
or a kinematic animation. The separate renderer assigns recorded states solely
to replay physics; it never generates simulation motion.

## What changed and why

`propeller_wide_demo.py` imports `CorrectedEnv` unchanged and reuses its existing
servo step, contact audit and local IK. The new default command moves the working
patch inward to `t = .64 + .19 sin(2π time/12)`, `u = 0`, over two cycles. This
commands 150.1 mm of radial surface excursion; the actual free pad travels less,
as measured above. The left target and orientation are exactly the original
`LEFT_T=.97` outer-edge hold. Right pregrasp IK is recomputed for the new patch.

The right jaw-opening direction is rotated into the surface-projected vertical
axis (90° roll), while the approach continues to follow `surface(t,u)` normals.
Physics-only trials with the previous radial jaw-opening direction allowed the
handle to rotate gradually, lost pad contact, and exceeded the penetration
threshold. The vertical grip avoids that failure in the selected run. A smooth
one-second normal-loading ramp changes the TCP gap from 6 to 3 mm. This is a
motion target, not a modification of the pad or contact model.

The two-second jaw-closing settle and one-second finishing hold remain. The
24-second scrub uses slower six-second end-to-end strokes. All trajectory-time
movement comes from joint servo commands and MuJoCo contact dynamics. There are
no welds, mocap bodies, external forces, or object pose setters in the motion
loop. LocalIK runs on a separate scratch `MjData`; its scratch qpos is restored
by the existing solver, and live state is asserted unchanged across every fit.

## Measured evidence

| Quantity | Selected run |
|---|---:|
| Actual radial span during simultaneous contact | 131.641 mm |
| Maximum 3D point-to-point span | 132.842 mm |
| X / Y / Z peak-to-peak span | 132.646 / 8.647 / 11.566 mm |
| Projection on centre-patch radial tangent | 132.816 mm |
| Projection on centre-patch chordwise tangent | 9.887 mm |
| Complete radial stroke amplitudes | 128.354 / 122.144 / 125.432 mm |
| Complete projected tangent amplitudes | 129.333 / 123.113 / 126.597 mm |
| Actual radial reversals, 10 mm prominence | 4 |
| Accumulated continuous-contact tangential path | 494.182 mm |
| Simultaneous contact, all 24,000 scrub substeps | 99.9667% |
| Both bilateral holds, individually | 100% / 100% |
| Maximum intended penetration, including settle | 0.769 mm |
| Unwanted arm/tool contact pairs | None |
| Peak pad normal force | 12.175 N |
| Maximum positional IK residual | 0.000035 mm |
| Left hold contact radius | 453.601–478.827 mm (94.50–99.76%) |
| Left / right peak arm effort | 5.000 / 3.690 Nm |
| Left / right 5 Nm cap occupancy during scrub | 100% / 0% |
| Tool drop after opening right jaws for 1.5 s | 578.599 mm |

Span metrics use actual pad-centre positions at **every 1 ms substep**, restricted
to simultaneous contact. Individual stroke extrema are found on the measured
radial trace with 10 mm prominence; each selected complete stroke is entirely
in contact. Projected spans use the orthonormal surface tangent axes at the
commanded patch centre. Accumulated travel removes the local surface-normal
component at every step and excludes intervals crossing a contact gap. It is
reported separately and is **not** the stroke metric.

The pad's conservative 26 mm enclosing sphere, sampled in XZ at 128 perimeter
points at all 24,000 scrub steps, remains within blade 0's face: footprint
`t=.3898–.8547`, `u=-.1416–.1914`. Its radial clearance to the hub is at least
136.978 mm and to the left hold's contact zone is 30.985 mm. These are conservative
radial geometric margins, not an all-body Euclidean clearance measurement;
the complete contact audit separately finds no unwanted arm/tool collisions.

## Assertions and files

- [check.json](runs/propeller_wide/check.json): motion, forces, release, comparison
  and entire-model fingerprints.
- [validation.json](runs/propeller_wide/validation.json): recomputes metrics from
  raw substeps, checks all three complete strokes have 100% simultaneous contact,
  verifies the original left target and orientation, and validates the footprint.
- The validation checks **3,563 frozen source, asset and baseline-result files**
  byte-for-byte against commit `81d64c1e`. The complete corrected XML is identical;
  all exposed numeric compiled model arrays/options have the same fingerprint
  before setup, after the run/release, and in a fresh `CorrectedEnv`.
- Canonical XML SHA-256 (repository path normalized):
  `e0d3d3d52cc34e1a07d36c8001cbc4ee6ff6d0c8bbaf74770acf085217e62448`.
- Compiled numeric model SHA-256:
  `c2b3433db75dd703f990065e3c3d7d21347ca5217c0bb0e69632ee4fd93ce21a`.
- `substeps.npz`: all 27,000 contact samples and pad positions, including settle
  and finish. MuJoCo's `mj_step` contact/geom evaluation precedes integration;
  timestamps denote the end of that 1 ms step, as explicitly recorded in the
  summary. `trajectory.jsonl` and `physics_replay.npz` hold 540 servo-row records
  and physical states. `release_trace.json` holds the subsequent 30 release states.
- [media_check.json](runs/propeller_wide/media_check.json): **270/270 frames fully
  decoded in each video**, verified 27 seconds at 10 fps, plus content hashes.
- [candidate_summary.json](runs/propeller_wide/candidate_summary.json): six
  physics-only trials. No rejected candidate was rendered. The 137.9 mm candidate
  had recurring contact gaps; the selected 131.6 mm run gives three uninterrupted
  strokes over 120 mm. Failed-trial videos were never produced.

## Reproduce

From `scripts/viscous-dream1` using the established Linux simulation venv:

```sh
MUJOCO_GL=egl TMPDIR="$HOME/dreamtrial/viscous_sim/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_wide_demo.py
MUJOCO_GL=egl TMPDIR="$HOME/dreamtrial/viscous_sim/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_wide_media.py
MUJOCO_GL=egl TMPDIR="$HOME/dreamtrial/viscous_sim/.cache/tmp" \
  "$HOME/dreamtrial/.venv/bin/python" propeller_wide_check.py
```

The demo is physics-only. The renderer refuses failed candidates. Optional
`--name candidates/example`, `--centre`, `--amplitude`, `--u`, `--u-amplitude`,
`--roll`, `--load`, `--period` and `--seconds` isolate motion experiments under
`runs/propeller_wide`; they do not change the model. The evidence validator is
intentionally strict for this selected 24-second, >120 mm run. Package versions
and source/evidence hashes are recorded in validation. No packages or services
were installed or changed.

**Limits:** left-arm 5 Nm saturation persists throughout. This does not establish
hardware feasibility or torque margin. It starts pregrasped, has a fixed shaft,
and does not demonstrate pickup, approach, free-shaft anti-rotation, tool spin,
abrasion, material removal or wet dynamics. It is scripted simulation, not a
learned policy or autonomous result. No Dream calls, hardware/serial/CAN access
or environment/scenery work occurred. The parent owns the goal record and
commit/push; this implementation makes neither commits nor pushes.
