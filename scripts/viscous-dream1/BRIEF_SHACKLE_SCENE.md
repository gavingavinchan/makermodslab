# Task: shackle pin task scene in MuJoCo, for Dream-1 and Astra (scene + scripted demo, NO billed calls)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`,
`TMPDIR=$PWD/.cache/tmp`. Read first:
- the shackle work: `runs/shackle_model/shackle_model_report.md`, `runs/shackle_threads/shackle_threads_report.md`,
  `runs/shackle_holder/shackle_holder_report.md`, `assets/shackle/shackle_params.json`, `shackle_assets.py`;
- how the flange task was built and run: `BRIEF_FLANGE_SCENE.md`, `BRIEF_FLANGE_DREAM.md`, `BRIEF_FLANGE_DREAM2.md`
  and their reports in `runs/`, the flange scene code, `sweep.py`;
- the Dream-1 findings on the Viscous arm: `BRIEF_VISCOUS_MAKERWRIST.md`, `runs/viscous_makerwrist_report.md`,
  `runs/viscous_onearm_isolate_report.md` (which camera/arm configuration actually worked: the Viscous with a
  Maker-style wrist camera put the apple in the bowl 6/6, also with one arm and the second parked);
- the Astra harness: `astra_op/README.md`, `astra_op/server.py`, `ASTRA_OPERATOR_PROMPT*.md`, `astra_learn/README.md`.

## Why (Gavin, 2026-10-28)
Goal B4, task 3 (Lab repo `docs/shackle-sim2real.md`). The shackle stands upright in Gavin's printed holder,
hot-glued to the holder and the holder hot-glued to the desk. The pin starts standing upright on its threaded end
on the table. The arm picks the pin up, puts it down through the top (clear) ear, and screws it into the bottom
(tapped) ear. Gavin: "throw it into a MuJoCo table, like a proper setup, so it looks like a table and stuff ... ask
Astra to do it. But before that, ask Dream-1 to do it, three attempts." Later the same setup is built for real, so
keep everything at real scale, and the video must look nice ("it has to look nice"; the threads must be visible).

## The scene
- A proper table setup in the station look the existing tasks use (Dream-1 was trained on that look; keep the
  table, lighting and cameras the runner already uses unless something reads badly, and say what you changed).
- The Viscous arm configuration that worked best for Dream-1 (Maker-style wrist camera; one active arm, the second
  parked if the runner needs two). Use the same configuration for the Astra profile so the two are comparable.
- Holder (`assets/shackle/shackle_holder_sim.obj`) fixed to the table; shackle body fixed in the holder exactly as
  designed (glued: weld both to the world). Pin a free body (stainless, ~7.9 g/cm^3), standing upright on its
  threaded end on the table. Place shackle and pin where the active arm reaches both comfortably at a sensible
  wrist orientation, with the pin's start a clear distance (~15-20 cm) from the shackle, and both well inside the
  top camera's view. Prove reach with the native IK: tool point at the pin shank grasp, above the top ear with the
  pin axis vertical, at the seated pose, and through the wrist turns needed to screw it in.
- Looks: threaded OBJ meshes with `smoothnormal="false"` (see the threads report), a stainless material, the
  holder as a printed part (PLA in a plain colour, matte), real proportions. Render it so the threads and the pin
  entering the ears read clearly from the cameras that make the video.
- Physics: the clear ear and the tapped ear are real clearance holes (the existing collision cells), so insertion
  is real contact, no snapping. Thread engagement as in the flange task: when the pin axis is within ~10 deg of the
  bore axis and its tip is in the tapped bore mouth, couple rotation to advance at the real pitch (right hand,
  phase as in `shackle_params.json`, so the visual threads stay meshed); release if it is pulled back out. Seated
  when the collar reaches the top ear. Use the flange task's gripper model (jaw drive, contact-gated grasp) and its
  lifted arm torque limits for this task only, logging peak torques the same way.
- Hidden log per row (never shown to Dream-1 or Astra): pin pose, insertion depth through the top ear, engaged
  yes/no, turns, gap to seated, grasp and contact signals, peak joint torques.

## Deliverables (under `runs/shackle_scene/`)
1. `overview_*.png`: 2-3 nice 1280x720 renders (front three-quarter, top, a close-up of the ears and pin), plus
   `contact_sheet.jpg` of the images Dream-1 will receive at the resolution sent.
2. **Scripted feasibility demo** (IK waypoints, no inference): pick the pin, insert it through the top ear, engage
   the tapped ear, screw it to seated with wrist-roll regrasp cycles. `scripted_demo.mp4` (H.264 yuv420p, the top
   view plus a close-up side by side). If it cannot finish, say exactly where and why; fix the script, not the
   physics or the fixture glue.
3. **Dream-1 task** in the same runner (`sweep.py`) as a new task `shackle`, with a short plain instruction (for
   example "pick up the pin and screw it into the shackle"; say what you chose), the same per-episode call cap
   mechanism, and success = seated, partial credit logged (inserted, engaged, turns). One **dry run** with the
   local stub to show it plugs in. **No live calls**: Claude reviews the scene first and launches the three live
   attempts.
4. **Astra profile** `shackle` for `astra_op/server.py`: the same scene through the existing `robot` CLI, and an
   operator prompt `ASTRA_OPERATOR_PROMPT_SHACKLE.md` modelled on the flange one (task described as a human would,
   no object poses, cameras and depth only). Test it in-process or with the scripted operator **only if it cannot
   collide with the running A/B** (see hard rules); otherwise leave the server smoke test for Claude and say so.
5. `shackle_scene_report.md` (short): layout and dimensions, reach results, thread/seat model, what the scripted
   demo achieved, peak torques versus the real 5 Nm, the Dream-1 instruction, the Astra profile, and what still
   looks wrong.

## Hard rules
- **No billed calls**: no `--live`, do not touch `runs/sweep/ledger.json`. No Astra or Dream-1 episodes. No hardware.
- **An Astra A/B chain is running** (`astra_learn/ab.sh`, episodes `astra_ab_*`, an `astra_op/server.py` process):
  do not stop it, do not start another `astra_op/server.py` while it runs (its runner refuses to start an
  episode if it sees a second server, which would wreck the A/B), and do not edit anything it uses
  (`astra_op/` existing files and profiles, `astra_learn/`, the flange scene). Add new files or new, separate code
  paths only; if you must touch a shared file, make the change additive and prove the existing tasks and profiles
  behave exactly as before.
- Existing tasks (`apple`, `box`, `propeller`, `flange`, ...) must behave exactly as before: rerun their quick
  checks. Only write inside `~/dreamtrial/viscous_sim`. Never print the API key.
