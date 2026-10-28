Created the model and all deliverables.

- [Comparison renders and overview](/home/gavin/dreamtrial/viscous_sim/runs/shackle_model/review_contact_sheet.png)
- [Report, grip check and requested measurements](/home/gavin/dreamtrial/viscous_sim/runs/shackle_model/shackle_model_report.md)
- [Caliper dimension check](/home/gavin/dreamtrial/viscous_sim/runs/shackle_model/dimension_check.md)
- [Parametric builder](/home/gavin/dreamtrial/viscous_sim/shackle_assets.py)
- [Exported parameters](/home/gavin/dreamtrial/viscous_sim/assets/shackle/shackle_params.json)

Both visual meshes pass watertightness checks. All 212 collision pieces are convex and preserve 0.60 mm diametral pin clearance.

Key finding: m287 measures thread length; m293 appears to measure a **19.69 mm shank**. Pin size and pitch remain provisional.

Blender returned black frames, so comparisons use local CPU rendering. No simulation, hardware access or billed calls.