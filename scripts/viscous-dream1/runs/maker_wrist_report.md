**Apple succeeded. Both runs grasped their objects. Exactly 40 billed calls: 20 per run.**

| Run | Closest requested hand → object, L/R | Object → target XY | Grasp / success |
|---|---:|---:|---|
| Box | 2.40 / 31.94 cm | 1.81 cm | Yes / No |
| Apple | 2.19 / 31.12 cm | 0.23 cm | Yes / Yes |

The left arm lifted the box 12.9 cm and carried it toward the square, but kept holding it above the table. It released the apple into the bowl during call 12; the apple remained there through call 20. Right stayed high near home.

Both cameras now sit on the CAD camera assembly on `link_007`: position `(0.07934769, 0, −0.08214495)` m, quaternion wxyz `(-0.696364, 0.122788, -0.122788, 0.696364)`. They look 20° down along the fingers, with VFOV 77.2°, native 16:9 rendering, and Dream’s unchanged 320×176 capture path.

[Sim/real comparison](runs/maker_wrist_match/compare.png) · [Report, frames and videos](runs/codex_maker_wrist_report.md)

All 13 dry variants passed; scripted pinches lifted both objects about 10 cm. Changed `supported_arms.py`, `supported_arm_check.py`, and `yam_baseline.py`; added `maker_wrist_check.py`.