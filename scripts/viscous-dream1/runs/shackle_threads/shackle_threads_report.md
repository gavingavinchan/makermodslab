# Shackle visual threads — MuJoCo check

Default `assets/shackle/shackle_body.obj/.stl` and `shackle_pin.obj/.stl` now have matching, single-start right-hand helices. Original visuals are retained byte-for-byte as `*_plain.obj/.stl`. Clear ear remains a plain bore. No collision geometry was regenerated: SHA-256 matches for all 212 convex cells and their manifest.

Current provisional inputs: **20 mm major diameter, 2.5 mm pitch, 22.23 mm pin thread length**. Change `THREAD_MAJOR_DIAMETER`, `THREAD_PITCH`, lengths, or `THREAD_START_ANGLE` in `shackle_assets.py`; both mating surfaces derive from them. The 60° axial profile has P/8 truncated external crests, truncated roots at the ISO external minor diameter, a chamfer starting below the root cylinder, and a smooth 0.9-pitch runout. Female crests use the basic ISO internal minor, plus 0.1 mm radial running clearance. Both female mouths have 45° countersinks beyond the thread major radius.

| Visual mesh | Triangles | Watertight / connected |
|---|---:|---|
| Body | 62,826 | Yes / one component |
| Pin | 44,994 | Yes / one component |

Both are below 150k faces. Helical edges follow profile breakpoints with 192 angular segments per revolution. OBJ exports independent per-corner normals with a 38° crease threshold: circular surfaces shade smoothly, while 60° crest/flank transitions remain sharp. Normals are calculated in millimetres and verified finite/unit after import. MuJoCo 3.16.0 preserves every OBJ normal index (pin: 134,982 normals); STL instead loads 22,497 generated vertex normals. Independent OBJ normal indices suffice; positional vertex duplication is unnecessary. Use OBJ with `smoothnormal="false"`. [Same-lighting OBJ/STL comparison](normals_obj_vs_stl.png).

The new standalone [MJCF](shackle_threads_check.xml) uses two contact-disabled visual meshes and mocap poses, upright with clear ear +Z, tapped ear −Z. Lighting/headlight and quality values are copied from `yam_env.build_xml()`, inherited by the flange scene; stainless specular/shininess/reflectance use flange `pipe_steel` values. All object pixels in the [four stills](review_contact_sheet.png), [comparison](compare_m288.png) and [video](screw_in.mp4) come from MuJoCo/EGL. The video is 1280×720, H.264/yuv420p, 30 fps, 240 frames / 8 seconds. Main view stays three-quarter; the inset renders the same pose at the tapped ear. Motion is entirely kinematic with no physics steps.

**Phase convention:** pin local +Z points from tip toward head; body seated tip is `(0,0,-0.039925)` m, spin zero. Crest phase is `q = (z_body-z_seated)/pitch - (azimuth-start_angle)/(2*pi)`, with crests at integer q. `THREAD_START_ANGLE = 0` radians changes the shared start phase. For remaining axial travel Δ, use `z_tip=z_seated+Δ` and `spin_about_+Z=2*pi*Δ/pitch` with an unwrapped angle. **Pin turned clockwise by θ, viewed from head toward tip, advances downward by θ/(2*pi)*pitch.** These relations and units are recorded in `assets/shackle/shackle_params.json`.

Validation: zero tapped-ear intersection for seven actual exported-solid poses (including fractional turns); the deliberately wrong half-turn control intersects by 386.27 mm³. Dense sampling across 23 engagement positions preserves at least 0.1 mm radial gap. A second 20.2 mm / 2.54 mm / 0.731 rad parameter set also builds watertight helices and preserves the gap. Every engaged video frame satisfies the screw relation, maximum residual 5.2e-18 m. [Full checks](thread_validation.json).

Remaining limitations: dimensions and standard await the real measurements; clean uniform stainless omits the photo's scratches and forging marks. The inherited bow neck reaches 0.0295 mm above the clear ear's seating plane, producing 0.023 mm³ collar-only overlap when fully seated; there is no thread overlap. Frozen collision bores retain their original 20.60 mm passage, so a larger future diameter needs a separate clearance review. Physically incompatible shank/thread/ear-length edits will be caught by engagement validation.

Rebuild assets, MJCF, stills, video, comparison, checks and this report in one command from the repo:

```bash
PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python runs/shackle_threads/rebuild.py
```

No billed calls, episodes, hardware, holder/task scene, or server operations. Writes are confined to `shackle_assets.py`, `assets/shackle/` and `runs/shackle_threads/`.
