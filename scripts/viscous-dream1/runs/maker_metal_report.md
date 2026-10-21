**Neither Maker nor Metal grasped an object.** All live runs had zero object contact. **60 billed calls total**; Metal box was skipped because Metal never came close.

| Run | Calls | Nearest requested hand → object, L/R | Object → target XY | Grasp / success |
|---|---:|---:|---:|---|
| Maker apple | 20 | 23.09 / 44.13 cm | 20.86 cm | No / No |
| Maker box | 20 | 22.66 / 41.13 cm | 20.86 cm | No / No |
| Metal apple | 20 | 29.94 / 70.32 cm | 20.86 cm | No / No |

Frames show Maker adjusting and raising its hands near home; Metal mostly stays high and waves near the bottle area.

FK passed 102 poses per side: maximum error across both arms **0.000291 mm / 0.000027°**. Scripted pinches lifted both objects about **10 cm** on both arms. Nine existing dry variants passed.

[Maker camera previews](runs/makers_apple_dry/camera_comparison.png) · [Metal camera previews](runs/metal_apple_dry/camera_comparison.png) · [Full report and frames](runs/codex_maker_report.md)

Changed `yam_baseline.py`; added `supported_arms.py`, `supported_arm_check.py`, `analyse_supported_runs.py`, and local CAD copies. These trials reproduce the wrong-hand-intent pattern across the supported arms, leaving the Viscous-only appearance explanation unconfirmed.