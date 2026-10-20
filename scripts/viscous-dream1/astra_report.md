Updated [viscous_dream_sim.py](/home/gavin/dreamtrial/viscous_sim/viscous_dream_sim.py): grey speckled desk, 2 cm white cube, 4.5 cm black square with 4 mm lines, softer lighting, and no cup/platform. Updated the task prompt and success check to require the released cube resting fully inside the square.

Arm rest pose, initial gripper opening, IK/control logic, and `viscous_arm.json` are unchanged. Third-person remains human-only. Dry mode now also stubs metadata lookup and blocks network access. No live calls or hardware access occurred.

Both Dream cameras render **640×480**. Final coordinates are metres:

| Camera/frame | Position | Aim point | Up vector | Vertical FOV |
|---|---|---|---|---|
| Overhead/world | `(0.40030, -0.16871, 0.28048)` | `(0.29481, -0.15448, 0)` | `(0.39047, 0.91512, -0.10043)` | 60° |
| Wrist/tool0 | `(0, 0.055, -0.09)` | `(0, 0.025, 0.08)` | `(0, -1, 0)` | 90° |

Comparisons, **real | sim**:

- [Overhead comparison](/home/gavin/dreamtrial/viscous_sim/runs/astra_preview/comparison_overhead.png): square centre and left-edge fingertip placement are close; the square is slightly taller, and extra arm geometry remains visible.
- [Wrist comparison](/home/gavin/dreamtrial/viscous_sim/runs/astra_preview/comparison_wrist.png): fingertips sit near the reference’s vertical position, but the retained open gripper spreads the jaws considerably. The square appears lower.
- [Closed-jaw diagnostic](/home/gavin/dreamtrial/viscous_sim/runs/astra_preview/comparison_wrist_closed_diagnostic.png) isolates the camera framing difference; normal runs still start open.

These are approximate visual matches. Bench clutter, lens distortion, reflections, and some CAD details remain different.

Preview passed. A separate scripted physics check lifted the cube to **6.6 cm** and released it successfully inside the square. All three dry calls reported `"clipped_rows": 0, "limit_rows": 0`, ending with:

```text
3 calls (none billed); log in runs/astra_dry
```

[Full dry-run output](/home/gavin/dreamtrial/viscous_sim/runs/astra_dry/output.txt) · [Validation results](/home/gavin/dreamtrial/viscous_sim/runs/astra_preview/validation.json) · [Camera parameters](/home/gavin/dreamtrial/viscous_sim/runs/astra_preview/camera_parameters.json)