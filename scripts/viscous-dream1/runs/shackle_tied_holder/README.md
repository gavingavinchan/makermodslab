# Freestanding shackle holder — 2026-11-06, goal B4

Gavin requested a printed shackle holder with “2or3 channels/holes for zipties”.
He clarified: “cant tape or glue to the tank when its filled with water, so the
base of the printed part should be big enough to not topple, the shackle is
heavy enough to not really move around”. This supersedes the old glued cradle.

## Design

The example has a **150 × 130 × 6 mm base**, two separated curved saddles, and
**two 6.2 × 2.8 mm tie passages**. Overall printed envelope at the example tilt:
**150 × 130 × 27.56 mm**. The shackle's lowest bow point reaches through a
relief in the base and rests on the tank floor: **zero added shackle height**.
The base and shackle carry their own weight; neither adhesive nor tank fixings
are part of this design.

Each tie enters from an opening beside its saddle, travels through a short
tunnel under the bow, and returns through the opposite opening. The openings
lean inward so the ties wrap only the lower bow stock. The two loops are about
65 mm apart. This avoids threading ties through the full plate width or across
the upper ears/pin. Orange in the preview shows **loose routing envelopes**;
actual ties tighten onto the steel. Put the locking heads on the accessible
outside of the saddles. Heads are not included in the envelope mesh.

Allow ties up to **4.8 mm wide × 1.5 mm thick**; **250 mm length** leaves room
for the approximately 163 mm illustrated loop and a tail. Tie size is a design
assumption, not a measurement of Gavin's stock. Feed the ties before seating
the shackle. Clearance at the modeled steel is nominally **0.7 mm**; the forged
crown curve is still estimated from photos. All exposed convex rims receive
R0.5 treatment; the bed perimeter retains a 0.5 mm chamfer.

## Tilt and environment handoff

**75° from vertical is an example, not an approved task layout. Do not choose
the print angle from this preview alone.** The environment thread is still
checking the tank wall, water line and arm reach. Angle is configurable in CAD
and fixed once printed; there is no mechanical adjustment joint.

Interim handoff from the orientation session (2026-11-06): likely **75–85°**,
bore midpoint **450–500 mm** ahead of J1, head toward the arm; 283 mm water
line, J3 ahead of J1 and all moving geometry ahead of the back-of-base wall.
Do not fix any of those provisional task dimensions here. A **60 mm diameter,
300 mm long corridor** from the clear ear along the pin axis toward the arm
is clear of the holder and both tie envelopes at 75°, 80° and 85°.

Rebuild from the repository root, with dependencies isolated and pinned by
the script's inline metadata:

```sh
uv run --script scripts/viscous-dream1/shackle_tied_holder.py --tilt 75
```

The current station arrangement supports 55–85° from vertical. `--no-render`
skips previews; `--output-dir /tmp/holder-check` redirects generated files.
The old holder, scene, arm files and original shackle meshes are untouched.

The `tilt_75deg/` directory contains STL and geometry-only 3MF in **mm**, three
preview PNGs, and `holder_checks.json`. Print coordinates are `(s, w, h)`:
`s` points away from the arm, `w` across, `h` up, tank floor `h=0`; the bore
midpoint is **(0, 0, 96.9843) mm**. Add the selected bore distance to `s` when
placing the assembly in the tank.

`shackle_tied_holder_body_frame.obj` and `zip_tie_envelope_body_frame.obj` use
**metres in the original shackle body frame**. Place these with exactly the
shackle's transform; do not apply the tilt twice. The original body frame has
the clear ear at +Z, tapped ear at -Z, bow toward -X. The environment acceptance
test should use this actual holder mesh, tie envelopes and actual locking-head
locations, and check the complete arm motion. No arm or task feasibility is
claimed by the holder checks.

## Verification and stability

The 75° design passes: single connected watertight outward-wound mesh; STL/3MF
round trips, explicit 3MF millimetres; no holder/steel intersection; nine clear
vertical drop-in samples; 33 clear pin insertion/extraction samples over
160 mm; both complete tie routing envelopes clear of holder and steel.
55°, 80° and 85° rebuilds also pass the geometry/fit checks (temporary validation
outputs in `/tmp`, not committed). The newly requested approach corridor passes
at 75°, 80° and 85°. Ruff lint/format pass. The previews were visually inspected.

The modeled stainless body is 694 g and pin 241 g. A conservative 40%-solid
PETG holder estimate adds 68 g. Gravity-only support checks, with and without
the pin and with water buoyancy, put the effective load at least **64.5 mm
inside the bottom support rectangle**. Minimum restoring moment is **0.393 N m
submerged without the pin**, or **0.526 N m submerged with it**. This supports
freestanding gravity stability; it is not a claim that the fixture resists an
arbitrary robot push or the old glue design's assumed 2 N m screw load. Sliding
on the actual wet floor and torque reaction require the physical setup check.

Print flat, with PETG, 0.2 mm layers, 5 walls, 6 top/bottom layers and around
40% infill. Designed for support-free printing with **6.2 mm bridges** across
the tie passages; inspect those bridges and clean the openings before fitting.
The 3MF contains geometry and units, not a tested slicer profile. No slicing,
physical print or load test has been performed. Let the flooded print fill
with water before judging stability; the buoyancy estimate assumes no trapped
air. The bow-to-floor fit and tilt must be checked against the actual forged
part before the robot trial.

Cross-machine handoff: Linux is the authoring machine. The Mac's Tailscale SSH
address did not respond (connection timeout), so its checkout could not be
synchronised in this session. The local arm-model and LeRobot checkouts were
clean with no commits ahead of their upstreams; this task changes neither.
