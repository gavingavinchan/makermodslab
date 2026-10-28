# Astra shackle video delivery

Rendered offline from `../video_states.npz`, episode `shackle_S001_20261028_001`.

| File | Format / duration | Contents |
|---|---|---|
| [shackle_autonomous_10x.mp4](shackle_autonomous_10x.mp4) | 1920×1080, 30 fps, 94.73 s | Entire saved episode at labelled 10×, without editorial cuts. Three-quarter arm view and a tracking ears/threads close-up; current command, phase, turns, drops, source clock and progress bar. Both drops and recoveries remain. |
| [shackle_social.mp4](shackle_social.mp4) | 1920×1080, 30 fps, 49.47 s | Chronological highlights, title, labelled speed changes, both drops, recovery, insertion, visible thread advance, release and verification. |
| [shackle_social_vertical.mp4](shackle_social_vertical.mp4) | 1080×1920, 30 fps, 49.47 s | Same edit, independently composed for portrait with stacked camera views and larger captions. |
| [hero_seated.png](hero_seated.png) | 1920×1080 | Clean macro of the final recorded state, showing the collar seated on the upper ear. |
| [before_after.png](before_after.png) | 1920×1080 | Recorded approach at 380.00 s beside the final seated state at 946.85 s. The approach panel exposes the male and female threads. |

The videos are silent H.264, CRF 17, YUV420p, with fast-start MP4 metadata.

## Edit and speed labels

The social edit follows the flange video's paired camera views, navy caption panel, amber progress / green success language and per-beat commentary. It adds the requested opening and closing cards. The title explicitly says **“Fully autonomous · simulation”**. This label also remains in the footage header. Every replay beat displays its speed, relative to **simulation time**, not model wall time.

| Saved simulation interval | Label | Moment |
|---|---|---|
| 0.10–12.10 s | 4× | Pick and lift |
| 56–68 s | 3× | First drop |
| 84–96 s | 4× | Pick the pin back up |
| 310–340 s | 10× | Carry and align |
| 376–400 s | 4× | Insertion, second drop on release, thread engagement and regrip |
| 400–520 s | 20× | Start screwing; exposed threads visibly descend |
| 520–730 s | 42× | Continued screw / reset / regrasp cycles |
| 730–916 s | 31× | Approach the collar seat |
| 932–946.85 s | 5× | Release, clear the gripper and inspect |

Opening card: 3 s. Final recorded state: **2.5 s held**, explicitly labelled “FINAL HOLD”. End card: 5 s, with **attempt 2**, **learned from its own notes after attempt 1 failed**, **8 turns**, and **~55 min wall time**. The exact timing and captions are in [edit_plan.json](edit_plan.json). Cuts only remove time between chronological intervals; no action is rearranged or changed.

## Recorded facts and captions

The outcome records 494 commands, two drops, 8.038018934 turns, seating and success, with 3290.61 s wall time (54.84 min). The second logged drop is the short fall following release while the pin is already through the upper ear; the lower thread catches at 393.906 s, and Astra regrips. It is shown and described explicitly rather than presented as a second tabletop recovery. Seating occurs at 915.189 s; the final image uses the actual final saved state.

Command text is identical between `timeline.txt` and `episode.jsonl`: all **20,727 rows** match the corresponding **495 timeline command entries**, including initialization. See [timeline_check.json](timeline_check.json). The structured episode timestamps select the active command and measured turn count at each displayed state. Phase headings are editorial descriptions derived from those commands and the timeline's insertion / engagement / seating transitions: pick, carry, recovery, align, insert, thread start, screwing and seated. Turn counts and phase captions are presentation annotations, not information supplied to the robot during the episode.

## Rendering and visual checks

- MuJoCo 3.16.0 with EGL; requested **8× offscreen MSAA**, confirmed **8 actual samples** in `MjrContext.offSamples` ([msaa.json](msaa.json)).
- Preserved the v2 key light at `(0.8, -0.7, 2.0)`, direction `(-0.6, 0.8, -1.2)`, shadow map **8192**, shadow scale **0.8**, original stainless material and authored mesh normals.
- The arm camera retains v2 cast shadows. Testing the closer camera with those shadows exposed a serrated self-shadow stripe on the pin. **Cast shadows are disabled only for the close-up render**, including stills, while the same lighting and material remain. This removes the white shadow-acne boundaries and jagged shadow edges on the visible threads. It also removes close-up cast shadows on the table and robot, a deliberate studio-lighting tradeoff.
- Camera views are rendered at **1280×960**, then downsampled with Lanczos into their panels; the hero is rendered directly at **1920×1080**. Output canvases are native 1080p landscape / portrait. Overlays are rendered at output resolution.
- The tracking camera follows the recorded pin and fixture, widening when they separate. Its final angle exposes the collar and pin with the gripper clear. The arm view provides context throughout.
- Reviewed initial pickup, both drop regions, recovery, exposed threads before insertion, screw-in close-up, final seating, both card layouts and portrait caption fit. Reviewed encoded samples as well as render stills. Close-ups show smooth steel shading and legible thread ridges without the shadow-acne patches.

## Replay integrity and validation

**Every frame containing the robot is a render of an exact saved episode state. No robot motion was edited, interpolated, synthesized or re-simulated.** Title and end cards are graphic-only frames. The final hold repeats the last recorded state. Camera movement, layout, captions, time sampling and the disclosed social cuts are presentation changes only.

The archive contains **9,465 qpos states of 23 values**, from **0.10 to 946.85 s** at approximately 10 samples/s. The continuous export spans that complete available range. At 10× and 30 output fps, it samples the archive in chronological order; it does not display every 0.1-s source sample. There are no temporal cuts in this export. Ordinary nearest-state sampling is also used for the social cut, with no frame interpolation or optical flow. The 0.00–0.10-s initialization interval was not present in the saved video archive and is not invented.

For each scene render the script copies `qpos` verbatim from the archive, sets the recorded time, and computes only kinematics, center-of-mass transforms and camera/light transforms. It asserts exact qpos equality. It creates no environment, controller, thread observer or episode runner. `mj_step`, `mj_step1`, `mj_step2` and remote socket connections are explicitly blocked. No hardware, server or inference client is used.

Per-output frame manifests (`*.mp4.frames.json`) map every output frame to its saved-state index and time, speed label, or graphic-card classification. [stills.frames.json](stills.frames.json) maps the PNG imagery. [validation.json](validation.json) records successful full-video decodes, frame counts, chronological saved-state checks and video SHA-256 values. [source_hashes.json](source_hashes.json) confirms the episode's original files, Astra log and final message remain byte-identical.

**Zero model calls. Zero physics steps. Zero network connections.** Writes are confined to this `video/` directory and the new root script `shackle_video.py`. No changes to `astra_learn/`, `astra_op/`, A/B runs, or `shackle_learn/lessons/`.

Reproduce with:

```bash
MUJOCO_GL=egl PYTHONDONTWRITEBYTECODE=1 \
  ~/dreamtrial/.venv/bin/python shackle_video.py all
```

Use `preview`, `stills`, `full`, `social`, or `verify` for individual stages. The retained previews document camera and shadow review; they are not additional episodes.
