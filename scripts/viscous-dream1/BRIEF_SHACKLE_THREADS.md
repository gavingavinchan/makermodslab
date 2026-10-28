# Task: visible threads on the shackle pin and tapped ear (visual meshes only, NO billed calls)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
Read first: `shackle_model_codex_report.md`, `runs/shackle_model/shackle_model_report.md`, `shackle_assets.py`.
You built that model earlier today; this extends it.

## Why (Gavin, 2026-10-27)
"I want the threads to be modeled or at least visualized ... in the physics simulation how you actually simulate
the threads I don't really care. What I do care is that when we record a video out of the simulation, it has to
look nice." The videos of this task (Astra inserting the pin and screwing it in) are recorded with MuJoCo's own
renderer, like the flange videos, so the threads must look right **in MuJoCo**, not only in an offline render.

## What to build
- **Pin:** a real helical external thread (ISO 60° profile, right hand, rounded or truncated crests and roots so
  it does not look like a knife edge) over `PIN_THREAD_LENGTH`, with a chamfered start at the tip and a natural
  runout into the plain shank. Driven entirely by `THREAD_MAJOR_DIAMETER`, `THREAD_PITCH` and the length params:
  Gavin will send the real diameter and pitch soon, and changing them must be a one-line edit plus a rebuild.
- **Tapped (bottom) ear:** a matching internal helical thread through the bore, with a countersunk/chamfered
  mouth on both faces, visible when the camera looks into the hole or at the pin entering it. The clear (top) ear
  stays a plain bore.
- **Phase:** when the pin is at its seated pose in the shackle frame, the external and internal threads must mesh
  with no visible interpenetration. Expose the thread phase (start angle) as a parameter and record the relation
  "pin turned by theta advances by theta/(2*pi)*pitch" in `shackle_params.json`, so the sim's screw constraint
  later can keep them meshed during the whole screw-in.
- **Visual only.** The collision geometry stays exactly as it is (plain bores with clearance; the sim will couple
  rotation to advance). Keep the old plain visual meshes as `*_plain.*` for comparison, write the threaded ones as
  the default `shackle_body.*` / `shackle_pin.*`.
- Mesh budget and shading: keep each visual mesh small enough for interactive MuJoCo rendering (say so if you go
  over ~150k faces) and make the crests and flanks shade crisply in MuJoCo, not as smeared or faceted helix bands
  (check how MuJoCo treats normals from OBJ vs STL, and split vertices at sharp edges if needed).

## Deliverables (under `runs/shackle_threads/`)
1. A minimal standalone MJCF (new file, `shackle_threads_check.xml`; do not touch existing scenes) with the
   shackle upright (clear ear on top, tapped ear below), the pin, a steel/stainless material in the style of the
   flange scene, and the flange scene's lighting.
2. MuJoCo renders, 1280x720: pin thread close-up; the tapped ear seen from below into the hole; the pin halfway
   screwed in; the pin seated.
3. `screw_in.mp4` (H.264, yuv420p, ~8 s): the pin above the top ear descends through it, engages the tapped ear
   and screws in at the real pitch (kinematic animation, no physics needed), threads meshing throughout, rendered
   in MuJoCo from a three-quarter view with the threads clearly visible.
4. An updated `compare_m288.png` style image (photo m288 beside the threaded pin).
5. `shackle_threads_report.md` (short): face counts, how normals/shading were handled, the phase convention, the
   rebuild command, and anything that still looks wrong.

## Hard rules
- No billed calls, no `--live`, no Astra or Dream-1 episodes. No hardware. Only write inside
  `~/dreamtrial/viscous_sim`: `shackle_assets.py`, `assets/shackle/`, `runs/shackle_threads/`, the new MJCF.
  Do not edit other scenes, scripts or assets. Another Codex job (`astra_learn/`) may be running in this folder:
  do not touch its files and do not start or stop any sim server.
- Don't build the holder or the task scene: Gavin is designing the holder himself.
