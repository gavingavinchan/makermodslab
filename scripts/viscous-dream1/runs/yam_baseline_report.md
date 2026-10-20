Both modes succeeded on YAM. **40 billed calls total**, with no API errors or retries. Both dry runs passed, including vision IK compilation, with zero inference calls.

Commands used `MUJOCO_GL=egl ~/dreamtrial/.venv/bin/python`:

- `yam_baseline.py --mode station --live --max-calls 20 --out runs/yam_station_live1`  
  **20 billed calls.** Left arm reached, grasped, lifted, carried, and released the apple into the bowl by call 8. Right arm stayed near home. Closest horizontal distance: **0.008 m**. It subsequently also placed the orange in the bowl, unprompted. Repeated joint-limit clipping notices; no flailing. [Frames](/home/gavin/dreamtrial/viscous_sim/runs/yam_station_live1/contact_sheet.jpg)

- `yam_baseline.py --mode vision --live --max-calls 20 --out runs/yam_vision_live1`  
  **20 billed calls.** Left arm completed the same apple sequence by call 6. Right arm drifted slightly near home. Closest horizontal distance: **0.004 m**. It then repeatedly attempted the orange without placing it. Minor gripper clipping notices; no arm joint-limit notices or flailing. [Frames](/home/gavin/dreamtrial/viscous_sim/runs/yam_vision_live1/contact_sheet.jpg)

Changed `yam_baseline.py`: fixed the namespace-package import and TCP height lookup; added dry policy validation, call accounting, trajectory/gripper logging, per-call frames, renderer cleanup, and workspace cache/log routing. The two request modes remain unchanged.

This trial shows vision-only mode can succeed on YAM; it does not reproduce Viscous’s failure.

One hard-rule miss: Dream’s import-time logger wrote to `~/.dream/logs` during early dry/station runs. I corrected log routing before the vision run. No hardware or installed packages were touched.