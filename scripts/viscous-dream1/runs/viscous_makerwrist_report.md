**Yes for apple:** Dream-1 grasped, lifted, and released it into the bowl. Box targeting improved, but it mostly dragged/tipped the box and failed placement. **Exactly 40 billed calls: 20 each.**

| Run | Closest requested hand → object L/R | Closest object → target XY | Success |
|---|---:|---:|---|
| Apple | 0.40 / 29.67 cm | 0.10 cm | **Yes** |
| Box | 2.00 / 39.06 cm | 9.73 cm | No |

Both cameras now use the Maker-relative pose and 77.2° VFOV, with native 16:9 rendering through Dream’s capture path. Legacy is selectable with `--viscous-wrist legacy`.

All **18 dry variants passed**; scripted pinches lifted both objects. Finger/table collisions remain: 418 apple rows and 292 box rows. `viscous_arm.json` is unchanged.

[Camera comparison](/home/gavin/dreamtrial/viscous_sim/runs/viscous_wrist_match/compare.png) · [Report with camera pose, IK errors, frames, videos, and changed files](/home/gavin/dreamtrial/viscous_sim/runs/codex_viscous_makerwrist_report.md)