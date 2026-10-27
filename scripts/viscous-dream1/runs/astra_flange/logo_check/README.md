# Physical Viscous Robotics badges — ROV flange scene, 2026-10-26

Gavin's intent: the video arms should carry the actual printed Viscous badges.
The `--scene rov` builder now adds the October 6 handed nameplate pair and the
**October 9 extended V** pair to both native forearms. All `astra_rov`,
`astra_rov_backup`, and `astra_rov_backup2` profiles get them. Tabletop `astra`
is unchanged. No video was rendered or edited; the existing social cut remains
the pre-logo cut.

[contact_sheet.jpg](contact_sheet.jpg) has three rows: the episode's first
captured state (0.1 s), the seated state (544.0 s), and a diagnostic state from
the early crossing interval (27.0 s). Columns are left +Y, left −Y, right +Y,
right −Y, and the native ROV camera. These are 960×640 stills only. They use
exact `video_states.npz` qpos captures, with zero simulation steps during still
creation and temporary external render cameras. No model camera, light or
render setting is edited by the scene integration.

## Files changed and integration

All live code changes are under `viscous_sim/astra_op/`:

- `rov_scene.py`: calls `logo_badges.add(root)` immediately before serialization.
- `logo_badges.py`: idempotent XML insertion, correct handed parts and colors.
- `logo_assets/`: eight source color-separated OBJ meshes in metres, bracket
  snapshots, registration/provenance JSON, and an immutable pre-logo builder
  snapshot for before/after verification.
- `check_logo_badges.py`: compiled-model equality, 2,000-row command replay,
  tabletop initialization parity, and captured-pose stills.
- `audit_logo_badges.py`: source-to-compiled-mesh registration checks and badge
  interference audit over every captured episode state.

The changed/new code and assets are mirrored in the Lab repo's
`scripts/viscous-dream1/astra_op/`; `rov_scene_logo.patch` records the two-line
integration against the previous builder. This is the badge change bundle for
the existing Linux harness, whose other operator modules are unchanged. The
check sheet, this README, individual stills and JSON evidence are copied to
`scripts/viscous-dream1/runs/astra_flange/logo_check/`. The original flange and
propeller sources and `viscous_arm.json` were not edited.

From the live `viscous_sim` directory, using the existing harness environment:

```sh
MUJOCO_GL=egl TMPDIR=$PWD/.cache/tmp ../.venv/bin/python astra_op/check_logo_badges.py physics
MUJOCO_GL=egl TMPDIR=$PWD/.cache/tmp ../.venv/bin/python astra_op/check_logo_badges.py replay
MUJOCO_GL=egl TMPDIR=$PWD/.cache/tmp ../.venv/bin/python astra_op/check_logo_badges.py stills
MUJOCO_GL=egl TMPDIR=$PWD/.cache/tmp ../.venv/bin/python astra_op/audit_logo_badges.py
```

## Registration, mounting and handedness

The carrier bodies are `left_forearm_link` and `right_forearm_link`. The source
carrier plates are `arm_60`, `arm_61`, `arm_62`, `arm_63` in the branding preview
assembly; actual ROV matching uses the bracket component in
`{arm}_forearm_link_00`. Signed-axis candidates are identified using that
component's extents, followed by 30 iterations of closest-triangle translation
ICP. Registration is then checked independently on both compiled MuJoCo visual
meshes, undoing the compiler's mesh centering/principal-axis transform with the
actual geom pose. Full four-plate residuals are in
[interference.json](interference.json); original fit matrices and source hashes
are in `astra_op/logo_assets/registration.json` and `provenance.json`.

| Actual compiled carrier mesh | arm_61 RMS (mm) | max (mm) |
|---|---:|---:|
| native_left_forearm_link_00 | 0.00967539 | 0.08069760 |
| native_right_forearm_link_00 | 0.00984907 | 0.08115185 |

These are source-vertex to target-triangle residuals. The source/ROV CAD exports
have different tessellation; residuals are not physical clearance measurements.
The source-preview assembly's forearm origin is `[-160,0,111]` mm. Its fitted
registration is:

```text
left:  ( x+160.001440381430,  y-0.000000734858, z-110.998804144036 ) mm
right: ( x+160.001440381430, -y+0.000000734858, z-110.998804144036 ) mm
```

The mounting reference scripts give print-to-preview rotation
`[[-s,0,0],[0,0,s],[0,1,0]]`, translation
`[X-160, s*P, 58.78775+111]` mm, for outward side `s=±1`.
Nameplate: `X=79, P=29.5`; extended V: `X=33.94112, P=37.5`.
The actual local centres used in both target forearms are:

```text
nameplate:  [79.001440381430, s*29.5 + epsilon, 58.788945855964] mm
extended V: [33.942560381430, s*37.5 + epsilon, 58.788945855964] mm
epsilon = -0.000000734858 (left), +0.000000734858 (right)
```

The right ROV mesh reflects Y, with registration determinant −1. The badge
itself is never reflected: each target outward side gets its proper
PosY_output/NegY_bearing part, with the rigid rotation above (determinant +1).
The local centres stay correctly associated with the reflected bracket.
No negative mesh scale or negative-determinant badge transform is used. The
four arm-side close views were inspected for readable VISCOUS / ROBOTICS.
Both parts lie directly against their aluminium faces; the V retains its
32 mm fixing-hole spacing and sits 8 mm farther outward than the nameplate.

Separate source CAD blue/white preview triangle meshes preserve blue through
print-frame Z=4.6 mm and white glyphs at 4.6–5.2 mm. Blue is #2D6BFF; white is
white PLA. Materials use metallic 0, roughness .62, reflectance 0, low specular
and shininess. The actual source-mounted layout was reproduced first in the
Blender check folder with the current extended V, using the reference scripts'
exact print-to-arm transforms. No substitute lettering or circular/old V
variant is used.

## Physics and scene preservation

[physics.json](physics.json) passes for all three ROV profiles:

- `nbody` remains **54**; only `ngeom` increases **4700 → 4716**.
- The 16 added geoms are on the two existing forearm bodies. Each has
  `contype=0 conaffinity=0 mass=0 density=0 group=2`.
- Removing only the new `badge_*` elements from generated XML gives the
  **byte-identical previous XML**. Thus cameras, lights, render configuration,
  original geometry, inertials, joints, controllers and constraints are intact.
- All compiled body mass/inertia and joint/DOF/actuator/equality/pair/exclusion
  fields compare exactly, except the nonphysical body geom/BVH indexing fields
  that necessarily change when visual geoms are inserted. Every original geom's
  physical attributes compare exactly by name; numeric geom IDs may shift.
- Tabletop `--profile astra`, G3 initialization, matches in body/geom counts,
  qpos, controls and mass, has no badge geoms, and never invokes the ROV hook.

[replay.json](replay.json) passes: replayed the recorded CLI commands through
the **first 2,000 source log rows**, comprising 44 commands through simulated
92.35 s. All joint/object qpos, qvel and controls are **bit-identical** before
and after, with maximum difference **0.0**. Both state streams hash to:

`b942340eb362812654d77ef6b23b74836ee38cf9ecf5ca54604a799f99fd78ab`

This runs the actual `Episode.execute` command/controller/IK/thread/grasp paths
and 1 ms physics integrator. It disables video capture/render/export and
workspace setup. RGB `look` output is replaced with blank scratch stills;
`point` still uses real depth rendering. Recorded numeric command arguments
are replayed unchanged, so point results are not fed back into target selection.
The compared streams also have identical row kinds and simulation times. This
is deterministic command replay, not a fresh Astra or Dream inference episode.

## Interference findings

Every one of the **5,855** recorded 10 Hz qpos captures was checked with FCL
native triangle BVHs. Each badge is checked against all other arm bodies and
all ROV/thruster visuals; the rigid mounting body is the intentional interface
and is excluded. No physical collision geom was added or changed.

One pair crosses: **left PosY/output nameplate vs ROV**, at **24 captured poses
from 27.0 through 29.4 s**, capture indices 269–292. There is no 28.4 s capture
in the source; the list of every actual sample is in `interference.json`.
This occurs during the episode's early exploratory left-arm move, before the
seated work. `interference_27s_left_PosY.png` and the contact-sheet third row
show a diagnostic pose. No other badge/native-link crossings were found at the
captured states, including the start and seated poses. Motion and physics are
unchanged; the badges remain visual-only as requested.

The test detects triangle surface crossings, not containment or signed
penetration/clearance, and does not certify the intervals between 10 Hz captures.
The Blender film separately passes all 672 integer film frames without badge
crossings. Its preservation proof is in `cinematic/results/logo_check/`.

No hardware, Dream-1, Astra inference, accounting ledger, commit or push was
used. No Blender or simulation process remains running. No requested video was
rendered. All requested work is complete; the finite-sampling limits and the
reported early interference are retained for review.
