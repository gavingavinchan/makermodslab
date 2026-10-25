# Correct propeller geometry and motion before changing the scene

## Gavin's correction (2026-10-24)

"The blades of the propeller is wonky. Make that correct." The current stage
is **propeller geometry and motion only**, then show him the result. Keep the
current bench environment, fixtures, arm bases, cameras and tool unchanged;
only replace the propeller and recalculate interaction targets for its surface.
Keep the enlarged 960 mm diameter and four cardinal blades. Left hand grips
the left blade's outer edge; right hand scrubs that same blade.

The previous underwater preview is **rejected as the environment direction**.
After geometry/motion review, use the actual BlueROV2 Heavy with both Viscous
arms attached at their real mounts. The arms must not stand on arbitrary desk
or service-frame posts. The later view should show the submerged underside/
stern of a ship, its shaft/propeller/rudder arrangement, and open water; do not
insert a seabed beneath the cleaning scene. Research the hull from real images
before modelling it. This brief does not authorize a new scenery iteration now.

## Primary propeller references

Retrieved 2026-10-24. Manufacturer images are visual design references, not
dimensional drawings or permission to claim an exact product/certified design.
Third-party reference photos are not part of our generated geometry assets.

- [France Hélices — conventional fixed-pitch propellers](https://www.francehelices.fr/en/fixed-pitch-propellers-for-motor-boats/),
  particularly the [FH4 65 four-blade photo](https://www.francehelices.fr/wp-content/uploads/helices-4-pales-fh4-65.png).
- [Solé — four-blade shaft-fixed propellers](https://soleadvance.com/en/sole-4-blades-shaft-fixed-propellers-fa1628),
  with the [clear product image](https://soleadvance.com/Fotos/FA1628_001.jpg).
  The parent visually inspected this image: broad marine blade surfaces,
  consistently rounded perimeter, smooth root-to-tip shape and common handedness.
- [ZF fixed-pitch propeller brochure](https://www.zf.com/products/media/industrial/marine/brochures_1/ZF_Marine_Propellers_18_EN.pdf):
  additional manufacturer context for blade section, skew, rake and cupping.
- [Fountom four-blade fixed-pitch propeller](https://www.fountom.com/products/4-blade-fixed-pitch-propeller):
  commercial marine product photos and design-drawing context.

Use a smooth marine screw-propeller parameterization with coherent pitch,
rounded tips, credible chord/thickness/root transitions and consistent handedness.
Inspect front, oblique and side geometry, rather than accepting a favourable
camera angle. Visual and decomposed collision surfaces must represent the same
blades. Four blades are exact 90-degree copies around the same shaft.

## Current deliverable

Provide a clearly viewable corrected propeller, unchanged bench overview,
grip/scrub close-up, three policy camera frames and a short physical motion clip.
Check dimensions, mesh validity/normals, pitch handedness, root continuity,
clearance and scene invariants. Recheck actual bilateral outer-edge hold,
pad contact, retention/release, stroke reversals, collisions and torque limits.
Save new results without overwriting the rejected geometry's historical evidence.

The motion remains a scripted simulation unless a policy has actually executed
it. Explicit pregrasp, fixed shaft and left-arm torque saturation must remain
visible where still applicable. No new billed Dream calls or hardware activity.
Do not move arm bases or increase actuator limits to conceal a propeller problem.

## Later environment references (record only; no modelling yet)

- [Blue Robotics — BlueROV2](https://bluerobotics.com/store/rov/bluerov2/):
  manufacturer vehicle reference. Our actual custom bimanual mounting comes
  from `viscous-rov-model`, not a generic replacement vehicle.
- [Palumbo Shipyards Malta](https://www.palumbogroup.it/palumbo-shipyard-malta):
  [this dry-dock photograph](https://images.squarespace-cdn.com/content/v1/584eca8b9de4bb74d3231a73/1544529006672-3EQ3S7EONKSZNYJ565CS/14.JPG)
  was visually inspected. It clearly shows a four-blade propeller on a shaft
  emerging from a shaped stern boss, hull plating curving above the propeller
  aperture, and the rudder aft of the propeller. It is a useful coherent
  single-screw structural reference; its dry-dock floor/scaffolds are excluded
  from the later submerged view. The blades also show smooth root transitions
  and the physical depth/pitch absent from a flat fan silhouette.
- [Damen Shiprepair](https://www.damen.com/services/shiprepair) and its
  [shiprepair brochure](https://medialibrary.damen.com/m/2e739159eb4c7216/original/damen-shiprepair-brochure.pdf):
  further primary shipyard references.

Before that later stage, choose one coherent single- or twin-screw vessel layout
from the references; do not mix appendages from unrelated ships. Dry-dock floors,
blocks and scaffolds show structural context but do not belong in the submerged
open-water composition requested by Gavin.
