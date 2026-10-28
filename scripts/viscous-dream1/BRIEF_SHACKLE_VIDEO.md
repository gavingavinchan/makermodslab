# Task: polished videos of Astra's successful shackle episode (render only, NO model calls)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, venv `~/dreamtrial/.venv/bin/python`, `MUJOCO_GL=egl`.
The episode: `runs/shackle_astra/shackle_S001_20261028_001/` (`run.mp4`, `video_states.npz`, `episode.jsonl`,
`timeline.txt`, `outcome.json`; Astra log `..._astra.log`, final message `..._astra_final.md`). Read how the flange
success was cut: `runs/astra_flange/astra_g3_20261025_006/` (`astra_flange_autonomous_10x.mp4`,
`astra_flange_social.mp4`, `social_edit.py`, `grid10x.mp4`) and the shackle look reports
(`runs/shackle_look/shackle_look_report.md`, the v2 shadow fix).

## Why (Gavin)
Astra (`gpt-6-astra`) seated the pin fully autonomously on its 2nd shackle attempt, using only notes it wrote
itself after failing the 1st: 8.04 turns, collar seated, claim matches the simulator. Gavin wants to show it, and
"it has to look nice"; the threads must be visible. The rules that keep "fully autonomous" true (same as the
flange video): one continuous episode, no human input, resets or edits of the robot's actions; decisions from
cameras and depth only; any speed-up labelled; label it as simulation.

## Make (under `runs/shackle_astra/shackle_S001_20261028_001/video/`)
1. `shackle_autonomous_10x.mp4`: 16:9 1920x1080, the whole episode at a labelled 10x, re-rendered from
   `video_states.npz` with nicer cameras than the inspection view (a three-quarter view of the arm and shackle and
   a close-up on the ears/threads that follows the action), plus a caption panel showing the current robot
   command and phase (pick, carry, align, insert, thread start, screwing with turn count, seated) taken from the
   timeline. The two pin drops and recoveries stay in (they are part of the honest story).
2. `shackle_social.mp4`: a ~45-60 s cut in the style of `astra_flange_social.mp4` (title card "Fully autonomous ·
   simulation", the key moments at varying labelled speed, the screw-in in close-up with the threads visible, the
   final seated shot held for 2-3 s, an end card with: attempt 2, learned from its own notes after attempt 1 failed,
   8 turns, ~55 min wall time). Both 16:9 and a 9:16 vertical version.
3. Stills: `hero_seated.png` (1920x1080 close-up of the seated pin), `before_after.png`.
4. Fix render quality where it hurts: use the v2 shadow settings, anti-aliasing (MSAA/offsamples) at 4x or more,
   and check no white shadow-acne patches or jagged shadow edges show in the close-ups; if they do, render the
   close-up camera with shadows tuned for it. Physics is not re-simulated: replay the saved states only.
5. `video_report.md`: what each file shows, the speed labels, the render settings, and confirmation that every
   frame is a replay of the recorded episode states (no edited motion).

## Hard rules
No model calls of any kind. No hardware. Don't start any sim server; don't touch `astra_learn/`, `astra_op/`, the A/B
runs, `shackle_learn/lessons/` or the episode's original files (write only inside `video/`, plus any new render
script, e.g. `shackle_video.py`). Only write inside `~/dreamtrial/viscous_sim`.
