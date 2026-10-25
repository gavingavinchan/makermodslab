# Compact robotic rotary polisher

Original cinematic asset for the corrected-propeller / native BlueROV2 Heavy
presentation. Built and rendered with Blender Python 4.5.3 on gavin-linux.
The long, plain rubber barrel is deliberate: the robot needs an unobstructed
cylindrical grasp surface. Rounded marine-blue housing, stainless hardware,
sealed seams, collet, compliant backing, abrasive disc and rear connector give
the visible ends a manufactured appearance without filling the jaw space.

## Scene integration

Append collection `CleaningTool` from `polisher.blend`. This file contains only
the tool, with no preview lights, camera, staging geometry or rigid bodies.
All meshes and procedural material nodes are embedded; no external files are
needed. Keep mesh parenting intact and transform **only `CleaningToolRoot`**.

- Units: metres; root identity at the handle centre; +Z points toward the pad.
- `ToolGripAnchor`: root child at `(0, 0, 0)`, identity orientation.
- `ToolPadContactAnchor`: root child at `(0, 0, 0.081)`, identity orientation.
- `ToolRotor`: root child at the origin. Rotate local Euler Z to spin the
  collet, spindle, backing and pad together; the contact anchor remains fixed.
- Grip: 32 mm diameter, 70 mm nominal length, z = −35…+35 mm.
  End bevels occupy 1 mm each; z = −34…+34 mm is a straight 32 mm cylinder.
  No other geometry overlaps that band. Fine rubber texture is shader bump,
  not displacement. No protruding trigger, ribs or labels enter the grip zone.
- Pad: 52 mm maximum diameter; abrasive contact plane z = +81 mm.
  The original physics pad centre +75 mm and half-thickness 6 mm are reference
  values, not a claim that this layered visual pad is the original ellipsoid.
- Total visual envelope: 52 × 52 × 145.12 mm;
  z = −64.12…+81 mm, including the rear connector.
- Version 1.1 narrows the rear seal/cap/index ring to at most 10.84 mm radius
  and moves smaller captive screw heads inward. The previous rear package
  intersected native finger components when seated +20 mm along native tool0;
  narrowing it preserves the D32 grip, all axial dimensions and both anchors.
  The initially suggested 14 mm envelope still intersected the finger backs;
  the final smaller package passes a frame-350 native-hand append test.

The scene coordinator owns the transform to the actual native `right_tool0`.
The bench's 20 mm grasp-TCP approach offset is **not baked into the asset**;
resolve the native axes and grasp anchor before attaching it. No rig, arm mount
or 493.05 mm native base spacing has been changed by this asset task.
For a target world grip matrix `G`, use
`root.matrix_world = G @ grip_anchor.matrix_local.inverted()`.
Parenting at `right_tool0` alone is correct only if that frame already equals
the intended grip frame. Check the actual finger contact in the integrated scene.

An alternative to appending is importing the generator in Blender Python:

```python
import importlib.util
from pathlib import Path

path = Path("scripts/viscous-dream1/cinematic/tool/build_polisher.py").resolve()
spec = importlib.util.spec_from_file_location("build_polisher", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
tool = module.create_tool()
root = tool["root"]
rotor = tool["rotor"]
```

`create_tool(collection=None)` returns the Blender objects under keys `root`,
`grip_anchor`, `pad_contact_anchor`, `rotor`, `collection`. It adds the tool to
the active scene without resetting the scene, changing render settings or
writing files. An optional collection must be empty and named `CleaningTool`.
Existing canonical object/collection names cause a clear error, never silent
replacement or `.001`-suffixed anchors. Meshes, profiles and shaders use no
random generation; a repeated build has the same geometry and materials.

## Rebuild and focused verification

From the repository root **on gavin-linux**, using the existing environment:

```sh
~/Documents/viscous-arm-model/.blender-venv/bin/python \
  scripts/viscous-dream1/cinematic/tool/build_polisher.py --render
~/Documents/viscous-arm-model/.blender-venv/bin/python \
  scripts/viscous-dream1/cinematic/tool/verify_polisher.py
```

The standalone generator resets only its own Blender process, saves the clean
asset before creating a temporary studio, and renders three 1280×720 PNGs with
48 Cycles samples on six CPU threads. No GPU renderer is requested. Omit
`--render` to regenerate only the blend, manifest and geometric verification.
`asset_manifest.json` is the machine-readable interface, materials, license and
envelope; `verification.json` records geometric checks; `saved_asset_check.json`
records reopening and appendability checks. `sha256.json` records source/asset
hashes. The three previews are hero, abrasive-face and side-profile views.
`verify_native_fit.py` optionally appends this asset into the coordinator's
`../results/final_scene.blend` in memory and tests the native right hand at frame
350 with the root seated +20 mm on native tool0. It writes `native_fit_check.json`
only in this directory and never saves the coordinator's scene. This focused
check does not replace the coordinator's all-frame geometry audit.

## Engineering limits and provenance

This is an illustrative sealed-tool design, with no commercial branding or
claim of a manufactured product, pressure rating, motor performance or approval.
The rear connector suggests a detachable potted power lead; it does not include
a routed tether, pinout or validated underwater connector design.

The existing free physics tool totals **117 g**. That is a historical baseline,
not this asset's measured/estimated mass. No mass, inertia, collision shapes,
motor physics, abrasion or buoyancy are assigned. The visual housing, connector,
pad surface and envelope differ from the baseline; the scene must not imply
that prior physics checks validate these additions. Native-robot jaw clearance
and scene mounting still need the coordinator's assembled-scene check.

Geometry, code and procedural shaders are original MakerMods work, Apache-2.0
(repository LICENSE). No external textures, CAD geometry or third-party assets
are used. This task changes only this directory and makes no commits or pushes.
