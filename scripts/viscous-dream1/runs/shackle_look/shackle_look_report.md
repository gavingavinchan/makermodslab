# Stainless shading fix — offline, 2026-10-28

The jagged white patches are boundaries in the key light's **shadow-map self-shadowing**, not the stainless material or mesh normals. Same-pose EGL ablations show that removing shadows eliminates them; removing reflectance, specular, or collision group 3 does not. Raising the shadow map from 4096 to 8192 or 16384 only changes the serration. Recomputing smooth normals also left the patches. See [ablation](ablation.jpg), left to right: original, no reflection, no shadows, no specular, no collision cells, 8192, 16384. The broad smooth highlights were already present underneath the faulty shadow mask.

Changed **only `shackle_scene.py` among existing source/assets**: `add_scene()` explicitly sets the station lights' `castshadow="false"` for the shackle task. Only the key light changes behavior; the fill already had shadows disabled. This gives clean studio-style steel shading. **Tradeoff: cast shadows on the table and robot are also removed in this task.** The original diffuse/ambient lights and stainless RGBA `.63 .67 .70 1`, specular `.7`, shininess `.55`, reflectance `.06` remain intact. Authored OBJ split normals and `smoothnormal="false"` remain intact, preserving the machined edges and male/female thread detail. No geometry regeneration or renderer patch. See [exact source diff](scene.diff).

Review images (before above, after below): [detail](detail_comparison.jpg), [front three-quarter](front_comparison.jpg), [top](top_comparison.jpg), [policy contact sheet](contact_sheet_comparison.jpg). Individual 1280×720 frames and 1280×720 matched image-space crops are in `before/` and `after/`: `detail.png`, `front.png`, `top.png`, and their `_crop.png` counterparts. Camera poses, optics, scene layout, and crop rectangles are identical across each pair. Detail uses the same logged insertion pose as the original report; front/top use the original initial state. Baseline detail/front match the archived stills pixel-for-pixel; top differs by one channel level at one pixel.

`before/contact_sheet.jpg` and `after/contact_sheet.jpg` contain the actual top/left/right 320×176, quality-85 policy JPEG observations, using the existing capture function and contact-sheet layout. They use seed 20261028 and the same 33-row/1.65 s call-1 warmup as the runners. The before sheet exactly matches the archived local dry sheet. Individual JPEGs are `policy_top.jpg`, `policy_left.jpg`, `policy_right.jpg`. These are offline reproductions, not observations captured from either live run. Astra uses the same scene lighting; its active server was not instantiated or changed.

Physics verification: of **482 compiled model arrays**, only `light_castshadow` differs. Apart from that attribute, the full shackle XML is identical, including all cameras, transforms, contacts, masses, inertia, actuators, grasp/thread/seat constraints, and meshes. Warmup and another **100 hold rows / 5 s** produce bit-identical positions, velocities, accelerations, controls, actuator forces, and constraint forces. All **10 non-shackle task/wrist XML variants** remain byte-identical; **318 other source/asset files** match their starting hashes. [Validation](validation.json).

All new deliverables, scripts, diagnostics, and source snapshots are under `runs/shackle_look/`; [file inventory](files_created.txt). `shackle_render.py` is unchanged. No Dream/Astra calls, networking, hardware, new servers, or live-run/accounting/lock access. Neither `astra_op/` nor `astra_learn/` was written. Already-imported running processes retain their existing scene/model; the change applies when a future shackle model is built.

Reproduce the comparison without any runner:

```bash
MUJOCO_GL=egl TMPDIR=$PWD/.cache/tmp PYTHONDONTWRITEBYTECODE=1 \
  ~/dreamtrial/.venv/bin/python runs/shackle_look/compare.py
```

Use `runs/shackle_look/diagnose.py` instead for the retained ablations. Both scripts deny remote socket connections and protected accounting/live-output access.
