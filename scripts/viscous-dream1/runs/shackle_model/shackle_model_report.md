# Gavin’s bow shackle — model report, 2026-10-27

**Provisional identification: 19–20 mm / 3/4-inch stainless bow shackle with a matching-size pin.** Manufacturer and thread standard remain unknown. All 14 photos were opened. m287 measures **22.23 mm thread length**, not diameter; m293 appears to measure **19.69 mm pin shank diameter**, not ear thickness. m283 is the **7.92 mm cross hole**, m285 is **9.19 mm tab thickness**, and m295 reads **98.50 mm**. Collar OD, tab height and outside ear span remain provisional. [dimension_check.md](dimension_check.md) gives every reading, mesh measurement, difference, interpretation and confidence.

The [Crosby G-209 table](https://kitocrosby.com/wp-content/uploads/2026/04/01_Shackles_IMP_24.pdf), 3/4-inch row, gives approximately 19.05 mm stock, 22.35 mm pin, 31.75 mm ear gap, 71.37 mm inside length, 88.90 mm outer width and 126.24 mm outside length. The body size is plausible; several dimensions disagree with the photos.

The [Suncor stainless table](https://suncorstainless.com/wp-content/uploads/2022/02/Forged-Shackles-1-17-22-1.pdf), S0119-0020, explicitly uses matching body/pin diameters: approximately 19.56 mm pin, 41.148 mm ear gap, 78.74 mm inside length and 130.30 mm outside length. **Only its 41.148 mm gap is borrowed, as an ESTIMATE.** Photos override conflicting table dimensions. This supports a matching-size stainless pattern, without establishing the brand. Source copies/drawings: `shackle/reference/`.

**Pitch estimate: 2.50 mm, low-to-moderate confidence.** Against m287’s 22.23 mm thread length, m288 shows about nine crest traces including partial terminal/runout traces, roughly eight pitches; counting uncertainty gives about 2.5–2.8 mm/turn. m286 is blurred and m287 has glare. M20x2.5 is more consistent with the apparent shank diameter than M22x2.5 or 7/8-9 UNC (2.822 mm). 3/4-10 UNC (2.54 mm) cannot be distinguished photographically in pitch. No thread standard is confirmed.

Every unmeasured dimension is marked ESTIMATE in `shackle_params.json`; ambiguous measurements are PROVISIONAL_MEASURED. Dimensions below are mm unless stated.

| Estimated input or derived dimension | Value | Basis |
|---|---:|---|
| Ear inside gap / each thickness | 41.148 / 19.351 | Table gap; (79.85−gap)/2, symmetric ears |
| Ear radial roll-in | 5.0 | Elliptical barrel profile, m289/m296 |
| Clear bore / thread major diameter | 20.60 / 20.0 | Provisional nominal 20 mm fit |
| Pitch / tapped visual minor bore | 2.50 / 17.29367 | Crest count; basic metric minor-diameter formula |
| Bore mouth chamfer, radial/axial | 0.60 | m289 appearance |
| Pin under-collar / plain shank / total length | 79.85 / 57.62 / 107.90 | Flush seated tip; provisional span plus head |
| Pin tip chamfer, radial/axial | 0.70 | m286/m288; flat standing tip retained |
| Collar thickness / bevel | 4.0 / 0.80 | m286/m288 silhouette |
| Tab width / hole center above collar top | 27.68 / 13.0 | Width assumed equal to collar OD; m288 |
| Tab base-width / shoulder-height fractions | 0.68 / 0.75, unitless | m288 taper/crown |
| Neck length / curve handle length | 22.0 / 13.0 | m289/m296 outline |
| Inside length / maximum inside bow width | 69.42 / 59.68 | Derived outer geometry and diameters |
| Ear centers Z / crown radius / crown center X | ±30.2495 / 39.545 / −49.425 | Derived symmetric construction |
| Collision diametral allowance / neck end X | 0.60 / −11.0 | Deliberate clearance and conservative cutback |

**Measurements Gavin should take:** remove the pin unless stated. Use the large outside jaws for external dimensions, small inside jaws for openings. Include both jaw contacts and display in each photo.

1. **Diameters:** outside jaws across the loose pin’s middle shank, then opposing thread crests, perpendicular to its axis. Repeat after rotating 90°. This resolves 20 versus 22 mm.
2. **Pitch:** outside-jaw tips axially at the first and eighth successive crest peaks on the same side; divide that span by **seven intervals**. Expected: 17.50 mm for 2.5 pitch, 17.78 for 3/4-10, 19.76 for 7/8-9. Use a thread gauge to separate 2.50/2.54.
3. **Pin lengths:** outside jaws along the axis, tip to last full thread/runout; tip to collar **underside**; tip to tab tip. Record separately.
4. **Collar:** outside jaws across OD, avoiding the tab, then across its two axial faces. This separates collar diameter and thickness from tab dimensions.
5. **Tab:** outside jaws across maximum width, narrow thickness, base width immediately above collar, collar **top** to tip, and collar top to widest station. Inside jaws across cross hole. Outside-jaw tips from collar top to the far hole edge, minus half hole diameter, give hole-center height.
6. **Clear ear:** inside jaws at mid-bore, beyond chamfer, in two directions; then at mouth. Outside jaws along the bore axis across its flat faces, beside the hole. Half the mouth-minus-bore diameter gives radial chamfer; depth rod to chamfer end gives axial extent if accessible.
7. **Tapped ear:** outside jaws across its flat faces, separately from the clear ear. Inside jaws lightly across opposing internal thread crests beyond the lead-in give approximate minor diameter; report pitch-gauge result separately.
8. **Spacing:** pin removed, inside jaws between opposing inner seating faces along the common bore axis; then outside jaws across outer seating faces. Avoid rounded rims and collar contact. This resolves m290 and ear symmetry.
9. **Inside bow:** pin installed, inside jaws from inner crown tangent to nearest pin tangent along the bow symmetry direction; separately across the widest internal bow opening.
10. **Outline/rounding:** square-on bow-plane photo with ruler in that plane; mark pin axis, neck-to-crown transition and widest crown station. Outside jaws across stock at those stations and ear OD after a 90° rotation. A side profile photo with ruler constrains ear roll, tip chamfer and collar bevel; scalar caliper readings cannot fully determine those curves.

**Grip check:** existing MJCF sliders each travel 52.4125 mm: **104.825 mm nominal opening**. CAD pad sections at `viscous_arm.json`’s TCP give 0.2641 mm closed gap, hence **105.0891 mm maximum face gap**. [grip_check.json](grip_check.json) records the static calculation; no simulator was loaded.

| Feature | Required separation, mm | Geometric result |
|---|---:|---|
| Shank / collar | 19.69 / 27.68 | Both fit; collar has only 4 mm axial contact length |
| Tab thickness / width | 9.19 / 27.68 | Both fit; broad-face pinch offers flats |
| Bow stock / ear OD / ear thickness | 19.41 / 40.07 / 19.351 | Local grips fit, subject to approach clearance |
| Whole bow width | 98.50 | Fits with 6.325 mm nominal total margin |
| Whole body length | 118.71 | Exceeds nominal opening by 13.885 mm |

**Exports:** two connected, watertight, outward-oriented visual STLs and smooth-normal OBJ companions in `assets/shackle/`; **212 convex, watertight collision pieces**, kept separate. Parameters are mm; meshes are metres. Body origin: pin-axis midpoint, clear ear +Z, tapped ear −Z, bow −X. Pin origin: threaded tip, +Z toward head. Seated translation: (0,0,−0.039925) m.

Both collision holes preserve a **20.60 mm passage**: 0.60 mm diametral / 0.30 mm radial clearance over thread proxy, 0.91 / 0.455 mm over shank. The visual minor bore overlaps the plain major-diameter thread when seated; collision bores omit engagement intentionally. STL cannot store smooth shading; OBJ corner normals do. Forging asymmetry and surface marks are omitted.

Rebuild locally from the repository directory:

```bash
PYTHONDONTWRITEBYTECODE=1 ~/dreamtrial/.venv/bin/python shackle_assets.py
PYTHONDONTWRITEBYTECODE=1 ~/dreamtrial/.venv/bin/python runs/shackle_model/validate_model.py
PYTHONDONTWRITEBYTECODE=1 ~/dreamtrial/.venv/bin/python runs/shackle_model/render_model.py
PYTHONDONTWRITEBYTECODE=1 ~/dreamtrial/.venv/bin/python runs/shackle_model/make_comparisons.py
```

No simulation, episodes, hardware, billed model calls, holder or MuJoCo task scene. The installed Blender build returned black PNG/EXR stills even with factory settings. Final images use the included deterministic CPU triangle renderer: depth buffer, smooth corner normals, projected shadows, exact model geometry. Comparisons use approximate orthographic viewpoints, independently scaled photo crops, and no camera calibration.
