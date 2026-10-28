# Gavin's shackle holder - edge update, 2026-10-28

Gavin's triangular holder is retained, with the shackle hot-glued to the holder and the holder hot-glued to the desk. Local geometry/rendering only: no billed calls, hardware or sim server. Protected source-file hashes are unchanged.

**Dimensions and edges.** Finished envelope **21.00 x 40.00 x 15.75 mm**; body-frame X=-89..-68, Y=+/-20, bottom Z=-49.24987, top Z=-33.5 mm. Named **`EDGE_CHAMFER = 0.5` mm** supplies equivalent R0.5 rounds on the top tip flat's perimeter, side/top transitions, vertical edges and every cradle rim. Existing **R1 outline corners** are retained. All bottom edges have a **0.5 mm, 45-degree chamfer**, including rounded footprint corners, to retain the flat bed face and avoid a lower fillet overhang.

**How edges were found and checked.** Rolling-ball erosion then dilation processes all convex edges together, so the treatment does not depend on manually selecting a few rims. The round kernel spans radius **0.500..0.503 mm**. The functional cradle is a concave surface and stays in place except at its rounded boundary. Triangle adjacency and face normals identify physical creases: turns over 20 degrees, with shared edge and both triangle altitudes over 0.10 mm. The 0.1 mm resolution is half the suggested print layer. Smaller triangle lines are curve tessellation or numerical slivers, not design edges; the rounding operation itself is unfiltered.

The audit found **106 original upper feature segments** and tested **530 points**, including endpoints, confirming that the original edges have been relieved. **0 points left unbroken; 0 untreated upper creases remain.** Maximum resolved upper normal turn is **9.03 degrees**, across round tessellation; bottom transitions are **45.00 degrees**, the intentional blunt chamfer boundaries (135-degree included angle). Thus all handling edges have the requested round/chamfer or the larger existing R1 round. Bottom-face insets are **0.500 mm** or greater. Audit samples and removal depths are saved in `edge_audit_samples.npz`.

**Cradle and fit.** Same mesh-derived saddle, **0.5 mm nominal glue gap**, same +/-35-degree wrap limit and vertical drop-in cutter. At 7941 protected points at least 1 mm from the rim, maximum surface movement is only **0.00098 mm** (mesh tolerance); the final core gap is **0.50046..0.50383 mm**. The round only relieves the rim; it does not tighten the seat. Global minimum gap **0.50039 mm**; 12655 samples on the full-fit patch give **0.50053..0.50591 mm**. The relieved rim has a deliberately larger gap. Body intersection **0 mm^3**; nine raised-body checks are clear. Current full-fit bond area ~**253 mm^2**. Minimum floor/rear/side thickness **3.02/2.74/13.71 mm**.

**Print verification.** Single connected, outward-wound, watertight solid; exported STL/3MF/OBJ roundtrips also pass. Flat face at **Z=0** in STL/3MF; maximum non-bed overhang **45.00 degrees from vertical**. Print **without supports**: 0.2 mm layers, 4 walls, 5 top/bottom layers, 35-45% infill, PLA or PETG. Import STL as **mm**; 3MF explicitly stores millimetres. Sim OBJ remains **metres in the shackle body frame**.

**Glue-joint loads, for information.** Stainless density 7.9 g/cm^3: body **693.9 g**, pin **240.7 g**; seated total **934.6 g**. COM without pin **(-38.75, 0.001, -0.35) mm**; seated **(-28.77, 0.000, 3.00) mm**. Bow/table contact near **(-49.37, 0, -49.25) mm**, saddle bond centroid **(-77.18, 0, -40.12) mm**, desk bond centroid **(-78.5, 0, -49.25) mm**, desk bond area **780 mm^2**. With 20 N down at pin axis X=0:

| Load carried by the glued fixture | Body only | Pin seated |
|---|---:|---:|
| Gravity moment about bow contact, N m | 0.072 | 0.189 |
| Gravity + 20 N moment, N m | 1.060 | 1.176 |
| Equivalent saddle hold-down, N | 38.1 | 42.3 |
| Equivalent desk-bond hold-down, N | 36.4 | 40.4 |

These use a bow-contact pivot and bond-centroid lever arms, neglecting beneficial holder weight and friction. The **2 N m screw torque** passes through both glue joints: equivalent to 100 N opposing forces over 20 mm at the saddle, or 50 N over 40 mm at the base. Current-area elastic-layer estimates tau=T*r/J give peak shear **2.36 MPa saddle / 0.35 MPa desk**; seated average hold-down force/area is **0.167/0.052 MPa**. These numbers describe the accepted glue load path; adhesive peel, voids and creep are not resolved by this simplified statics model.

**Real-part checks.** The m291/m294/m295 dimensions do not bound the exact forged crown curve. Retain the previous +/-2 mm local trial allowance (not a measured tolerance): hot glue fills an open gap, while interference needs fitting. Dry-fit to confirm lowest-bow contact, vertical pin axis and actual pin-tip/table clearance (~9.33 mm). Hold the shackle vertical through glue cure; the circular crown can slide along its arc before bonding. Avoid prolonged glue-gun heat on the plastic. The whole vertical bore cylinder remains clear: holder radius >=68 mm versus bore radius 10.3 mm.

**Files/rebuild.** STL, 3MF, sim OBJ and `holder_params.json` are under `assets/shackle/`. All requested images and the 1:1 PDF are under `runs/shackle_holder/`; print the PDF at 100% and check its 50 mm bar. Complete fit, edge, export and statics data are in `holder_checks.json`; full-fit contact mesh in `contact_patch_mm.ply`. Rebuild with `PYTHONDONTWRITEBYTECODE=1 ~/dreamtrial/.venv/bin/python shackle_holder.py`. Existing top/toe truncations and the sketch mapping are unchanged.
