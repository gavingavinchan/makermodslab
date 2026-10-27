# Physical Viscous Robotics badges — 2026-10-26

Gavin's intent: the video arms should carry the same 3D-printed brand badges as
the physical arms. The native asset and final scene now have four physical parts
per arm: October 6 handed nameplates and **October 9 extended V** badges. No old
far-elbow V or circular variant is imported. Each part has separate blue/white
triangle meshes, so `LOGO_BADGES` contains 16 mesh objects total (eight parts).

Open [contact_sheet.jpg](contact_sheet.jpg). Rows are frames **1, 336, 672**;
columns are left +Y, left −Y, right +Y, right −Y, and a wide view. The close
views preserve the link's local up direction so lettering can be read on all
four sides. These are 720×480, 8-sample Cycles/OptiX stills in this folder only.
The check camera and its disposable scene copy were deleted without saving.
The saved scene's camera-bound markers and cameras were never edited. The wide
check camera copies the existing establishing camera transform into the check
scene; it does not change that existing object.

## Files changed

- `assets/native_rov.blend` and `results/final_scene.blend`: new badge geometry,
  parenting, and two new materials only.
- `add_logo_badges.py`: idempotent importer; rerunning replaces `LOGO_BADGES`.
- `build_scene.py`: calls the importer after native appearance setup. The scene
  was **not rebuilt** during this task.
- `assets/logo_badges/`: self-contained source mesh snapshots, OBJ equivalents,
  bracket/arm reference meshes, provenance hashes, and fitted registration.
- `reproduce_logo_reference.py`, `register_logo_badges.py`,
  `check_logo_badges.py`: reference stills, independent registration/packing and
  idempotence checks, preservation proof, stills, and interference audit.
- `.gitignore`: excludes `*.prelogo.blend`; this includes the requested
  `results/final_scene.prelogo.blend` and the extra original native baseline
  `assets/native_rov.prelogo.blend`.
- `README.md` and the project goal record describe this geometry revision.

Source reference images are copied as `source_Mounted_*.png`. The reconstructed
source assembly uses the exact mounting transforms from `render_preview.py`
and `render_variants.py`; `reproduced_mounted_extended.png` and
`reproduced_mounted_bearing.png` show the current nameplate + extended-V pair
on the source arm before ROV integration. Source reference colors/lighting are
retained in these comparison renders; the actual scene uses the requested blue.
All runtime badge geometry is local to each blend, without external mesh files.
There are zero linked libraries and no unpacked file images. The appended
polisher's existing weak source-library paths are inert provenance, not links
or file-open dependencies; they were preserved.

## Mounting transforms and mesh registration

The carrier is `left_forearm_link` / `right_forearm_link`. The actual matched
scene meshes are `{arm}_forearm_link__{arm}_forearm_link_00`. Registration uses
source bracket `arm_61`, independently cross-checked against plates `arm_60`,
`arm_62`, and `arm_63`. It enumerates signed axis permutations using component
extents, then refines translation with 30 iterations of closest-triangle
surface ICP on the bracket. It fits the carrier geometry, never the badge or
an arm's overall pose. Geometry tessellation differs between the CAD exports;
residuals below are source-vertex to target-triangle distances, not a clearance
certificate. Full matrices, all four plate residuals and source file hashes are
in [registration.json](registration.json) and the assets' provenance JSON.

Source preview-arm coordinates are millimetres. Its forearm origin is
`[-160, 0, 111]`. The fitted source-preview-to-ROV-link maps are:

```text
left:  [ x + 160.001440381430,
         y -   0.000000734858,
         z - 110.998804144036 ] mm
right: [ x + 160.001440381430,
        -y +   0.000000734858,
         z - 110.998804144036 ] mm
```

For outward side `s = +1` (PosY) or `−1` (NegY), the exact source print-to-preview
transform is:

```text
[ -s  0  0  X-160       ]
[  0  0  s  s*P         ]  (all translation/mesh coordinates in mm)
[  0  1  0  58.78775+111 ]
[  0  0  0  1           ]

nameplate: X=79,       P=29.5
extended V: X=33.94112, P=37.5
```

After registering and choosing the correct handed part, both arms use rotation
`[[-s,0,0],[0,0,s],[0,1,0]]`, with link-local centres:

| Part | X (mm) | Y (mm) | Z (mm) |
|---|---:|---:|---:|
| nameplate | 79.001440381430 | s×29.5 + arm epsilon | 58.788945855964 |
| extended V | 33.942560381430 | s×37.5 + arm epsilon | 58.788945855964 |

Arm epsilon is −0.000000734858 mm left, +0.000000734858 mm right. Geometry and
translation are converted to metres for Blender. The backs lie directly on the
29.5/37.5 mm aluminium faces, with no artificial stand-off or front-face overlap.
The V's fixing centres remain 32 mm apart. Tiny fitted corrections in X/Z
(~0.002 mm) come from mesh registration rather than an invented visual offset.

| Actual target mesh | arm_61 RMS (mm) | max (mm) |
|---|---:|---:|
| native, left | 0.00809429 | 0.05529597 |
| native, right | 0.00809431 | 0.05529597 |
| film, left | 0.00809461 | 0.05533099 |
| film, right | 0.00809333 | 0.05535861 |

The right arm's mesh reflects Y (registration determinant −1), while both
forearm objects have determinant +1. That reflection is **not** applied to the
badge. Each target outward side gets its PosY/NegY handed part and a rigid
rotation with determinant +1. `register_logo_badges.py` also applies the importer
twice in memory, verifies the same 16 objects/meshes/transforms and two materials,
and verifies positive badge world determinants. Every close still was inspected
for readable, unmirrored VISCOUS / ROBOTICS lettering.

Materials are `BADGE | blue PLA` and `BADGE | white PLA`. Blue is sRGB
**#2D6BFF**, converted to Blender linear RGB; white is (1,1,1). Both use metallic
0, roughness .62, coat 0, and reduced specular. The source's separate colored
preview triangles preserve blue through print Z=4.6 mm and white glyphs from
4.6 to 5.2 mm; no lettering is redrawn or textured.

## Preservation and interference

[unchanged.json](unchanged.json) compares both saved files against their ignored
local backups. It passes: all 161 pre-existing native objects at all 180 native
frames, and all 226 pre-existing film objects at all **672** film frames, have
**exactly zero world-transform difference**. Local basis/parent inverse,
parents, constraints/modifiers, custom properties, drivers, action keyframes and
handles, mesh-coordinate/face hashes, material slots, camera/light data, timeline
markers and camera bindings, saved active camera/frame, frame range, render,
Cycles and color-management settings match. Only Blender's runtime session UID
is excluded from camera/light data comparisons. No video, edit, light, camera,
motion, original material assignment, or pre-existing mesh was modified.

[interference.json](interference.json) checks every integer frame 1–672:
16 badge meshes against all 100 native arm/ROV/thruster mesh objects, excluding
the badge's own rigid mounting link. There are 1,568 candidate object pairs and
10,649 broad-phase survivors. **No triangle intersections were found.** These
are triangle-surface checks, not a signed-distance or containment certificate;
subframes were not requested or checked. Motion was left unchanged.

## Scope and reproduction

Use the existing `viscous-arm-model/.blender-venv/bin/python` (bpy 4.5.3), from
`cinematic/`, for the commands in the main README. Badge insertion, rebuilding
and checks use only repo-local geometry. The two large baseline blends are
local review artifacts and deliberately excluded from git. The old manifest
and finished movie describe the pre-logo render; the current geometry hashes
are in `unchanged.json`. No scene rebuild or video render was performed.

The MuJoCo report and sheet are also copied under
`../../runs/astra_flange/logo_check/` relative to cinematic. Its full capture
audit found an early left-nameplate/vehicle crossing at 27.0–29.4 s; that is
reported there without changing dynamics or commands.

No hardware, Dream-1, Astra inference, accounting ledger, commit or push was
used. No Blender or simulation process is left running. All requested work was
completed; finite sampling and triangle-test limits are stated above.
