# Dry air desk — 2026-11-16
Waterproof arm on the same base mount, out of the aquarium on a desk; retain the post rules and floor (-28 mm, 20 mm press; Claude: the desk replaces the tank floor).
Gavin 2026-11-16: "the only boundary that is still important here is the back post, the rest of the bounding box does not matter here". Left/right/far/rim are finite ±2000 mm: conservative base-frame mesh reach 789.15 mm = guard J1 bound 731.75 + J1 origin norm 57.4, plus at most 70 mm clearance; point-box bounds use the same distance.
The `tank` reference still supplies only the arm URDF, motor map and gripper travel through `geo.load_model`; no tank geometry enters the guard from it.
