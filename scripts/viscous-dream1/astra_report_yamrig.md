Implemented `--rig yam`; `--rig bench` remains the default. YAM now has two Viscous arms, a parked right arm with actual joint observations and wrist video, Dream’s oak table/lighting, and reachable cube/square placement. Both previews passed. No live calls or hardware access occurred.

[Overhead comparison: Dream YAM | Viscous YAM](/home/gavin/dreamtrial/viscous_sim/runs/yamrig_preview/compare_overhead_yam_vs_viscous.png)

The camera framing matches numerically; arm forward points toward image-up. Positions below are metres, quaternions **w,x,y,z**, in Dream’s transformed MuJoCo world:

| Item | Position | Quaternion |
|---|---|---|
| Left base | `(0.305, 0.24, 0.76)` | `(0.7071067811865476, 0, 0, -0.7071067811865475)` |
| Right base | `(-0.305, 0.24, 0.76)` | Same |
| Overhead | `(-0.008999999612569809, 0.4064948785305023, 1.5543205499649049)` | `(0, 0, 0.30070322265070765, 0.953717763223208)` |

Base poses come from `sim-eval-yam.json`, transformed using `yam_env.py`: `(x,y,z) → (y,0.4925−x,z)`. The overhead uses Dream’s camera builder, including **0.15 m lowering**, **5° upward tilt**, and calibrated intrinsics:

- `fx=392.195617675781`, `fy=391.722351074219`
- `cx=318.389434814453`, `cy=237.876312255859`
- Derived vertical FOV **62.989841846744326°**, rendered **640×480**.

Both wrists retain the Viscous mount: tool-local position `(0,0.055,-0.09)`, target `(0,0.025,0.08)`, up `(0,-1,0)`, FOV **90°**. Policy images follow Dream’s crop/resize to **320×176**, with nine-frame windows at 5 Hz.

Remaining differences: Viscous geometry/dynamics/wrist optics, closed starting grippers, parked right arm, and cube/square task objects. Calibration bases are at **0.760 m**, while Dream’s YAM model bodies use **0.772 m**; that 12 mm discrepancy is preserved and documented.

Production IK, servo equations, and `viscous_arm.json` are unchanged. The YAM dry stub generates reachable poses from a small joint bend because the original Cartesian stub produced tiny joint-limit excursions near neutral.

**Both final dry runs passed:** all three calls on each rig reported `"limit_rows": 0, "clipped_rows": 0`.

```text
3 calls (none billed); log in runs/yamrig_dry
3 calls (none billed); log in runs/benchrig_dry
```

[Full report and parameter provenance](/home/gavin/dreamtrial/viscous_sim/yamrig_report.md) · [Validation results](/home/gavin/dreamtrial/viscous_sim/runs/yamrig_preview/validation.json)