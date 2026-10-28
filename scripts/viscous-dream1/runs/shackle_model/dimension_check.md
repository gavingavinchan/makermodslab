# Dimension check

All 14 originals m283–m296 were opened before modeling. Displays m283, m286 and m287 were read after mentally rotating 180°; m295 was also saved rotated for inspection. Values below are mm. Differences are model minus display, conditional on the interpretation; matching a chosen input is not independent validation. Mesh extents/sections use reloaded metre STLs converted to mm; small differences are polygon/float32 error.

| Photo | Display | Model | Δ | Measurement on model | Photo interpretation and confidence |
|---|---:|---:|---:|---|---|
| m283 | 7.92 | 7.9200 | -0.0000 | Cross hole: X extent of inner closed loop in pin mesh section Y=0. | 7.92. Inside jaws are in the cross hole. Disagree with tab thickness; moderate/high confidence in feature, moderate in contact. |
| m284 | 27.68 | 27.6800 | +0.0000 | Collar OD: X extent of mesh section halfway through collar thickness. | 27.68. Outside jaws span the collar/head envelope. Collar OD is the working interpretation; tab width cannot be separated confidently. Moderate. |
| m285 | 9.19 | 9.1900 | -0.0000 | Flat tab thickness: Y extent of mesh vertices above the collar. | 9.19. Outside jaws clamp the narrow edge of the tab, not its cross hole. High. |
| m286 | 24.05 | 24.0500 | +0.0000 | Tab height above collar top; underside-to-tip alternative is 28.05 mm. | 24.05. Axis-wise tab height, collar top to tip. Left contact is partly hidden; if it is under the collar, model is 4.00 too long for that reading. Moderate/low. |
| m287 | 22.23 | 22.2300 | -0.0000 | Maximum Z of full-major-radius thread vertices below mid-shank, measured from tip Z=0. | 22.23. Jaws run from threaded tip to runout. Disagree with shank diameter. High. |
| m290 | 79.85 | 79.8500 | -0.0000 | Z extent of forward annular seating-face mesh vertices, excluding bow. | 79.85. Outside ear span is provisional: the right jaw may land on the collar lip. If it includes collar thickness, true ear span would be 75.85 with the current 4 mm collar. Moderate/low. |
| m291 | 19.41 | 19.4100 | -0.0000 | Y extent of mesh section at the leftmost crown centerline station. | 19.41. Bow rod diameter at crown. High. |
| m292 | 40.07 | 40.0700 | +0.0000 | Boss OD: body Y extent; axis Z. | 40.07. Ear boss OD viewed along pin axis; matches jaw locations. High. |
| m293 | 19.69 | 19.6900 | +0.0000 | Exposed shank diameter from mesh section Z halfway along plain shank, X extent. | 19.69. Jaws straddle the exposed pin shank between ears (see inspect_m293.png). Disagree with ear thickness or bow diameter. Moderate/high; remeasure loose pin. |
| m294 | 118.71 | 118.7100 | +0.0000 | Full body X bounding box: crown to forward ear edge. | 118.71. Crown outside to forward ear outside, not inside length. High. |
| m295 | 98.50 | 98.4997 | -0.0003 | Maximum body Z bounding box across bow, not assembled pin/head. | 98.50, final digit reads 0 after 90° rotation. Bow maximum outside width, high confidence in reading, moderate in tangent alignment. |

The nominal 7/8-inch interpretation would make m293 22.225 mm, a +2.535 mm discrepancy; m287 does not measure diameter. The model therefore uses the observed 19.69 mm shank and a provisional nominal 20 mm thread. Ear thickness is **not measured**: the 19.351 mm value follows the table gap and provisional outside span.

Extra static checks: both visual STLs are connected, closed and outward oriented; all 212 collision STLs are convex and closed. The reloaded collision set guarantees a circular passage of at least 20.60000 mm. STL has no smooth-shading attribute: companion OBJ files contain explicit split corner normals; Final CPU renders interpolate smooth corner normals with a 38° crease threshold.

Body origin is at the pin-axis midpoint; clear ear +Z, tapped ear -Z, bow -X. Pin origin is its flat threaded tip; +Z points toward the head. Seated pin translation is (0, 0, -39.925) mm. A minor-diameter visual bore and major-diameter plain pin necessarily overlap at the tapped ear: this is the requested thread simplification. Collision geometry uses the enlarged bore instead.
