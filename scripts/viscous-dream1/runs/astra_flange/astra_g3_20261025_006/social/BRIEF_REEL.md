# Brief: vertical Instagram Reel + covers for the Astra flange "six attempts" video

Work only in this directory on the Mac:
`/Users/gavin/Documents/makermodslab-viscous/scripts/viscous-dream1/runs/astra_flange/astra_g3_20261025_006/`
(write all outputs into its `social/` subfolder). No hardware, no gavin-linux, no billed API calls, no git commits.

## Source
`astra_flange_social.mp4`: 1280x720, 25 fps, H.264, no audio, 115.9 s. It is a 2x2 grid of 640x360 panes:
- top-left: overhead "TOP" camera
- top-right: "FLANGE JOINT (inspection)" camera
- bottom-left: right wrist camera
- bottom-right: a dark caption panel (GPT-6 Astra · Fully autonomous / headline like "Attempt 4 ✗" / body text /
  "Nut drops: N  Failed attempts: N" / "Simulation | sped up"). The panel changes per beat.
`social_edit.py` next to it is how that cut was made (on Linux; read it for the beat list, colours and fonts). The raw
run.mp4 is NOT on this machine; build from `astra_flange_social.mp4` by cropping its panes with ffmpeg.

## Deliverables (in `social/`)
1. `astra_flange_reel_9x16.mp4` — 1080x1920, H.264 High, yuv420p, 30 fps, `+faststart`, CRF ~18, plus a silent
   stereo AAC 48 kHz track (some Instagram uploads misbehave with no audio track). Same full length and timing as the
   source (keep the ending hold). Layout is your call, but:
   - Use the caption panel AND at least the overhead pane and the wrist pane; the inspection pane too if it fits
     without making everything tiny. Panes must not be distorted (keep 16:9 per pane; crop or letterbox, never stretch).
   - Add a hook line at the top, rendered as an image (PIL is fine; use a macOS system font such as
     /System/Library/Fonts/Helvetica.ttc or SFNS, check the ✗/✓ glyphs render): "We gave an AI a robot and one job:"
     then "put a nut on a bolt." in the amber (255,196,92). Background (15,19,24) like the source.
   - Instagram covers roughly the top 220 px and bottom 380 px with UI. Keep the hook, the caption panel text and the
     counters out of those bands.
   - The caption panel is only 640x360 in the source; if upscaled text looks soft, it is acceptable to re-render the
     panels at full resolution from the BEATS table in social_edit.py, timed with the same out_t() mapping, so they
     line up with the video exactly. Your choice; say which you did.
2. `cover_reel_1080x1920.jpg` — a cover for the reel: a late frame where the nut is seated, with the hook text.
3. `cover_linkedin_1920x1080.jpg` — a 16:9 thumbnail for LinkedIn/X: same idea, hook text over a good seated frame
   from the source (upscaled is fine), readable at phone size.

## Verify before reporting
- ffprobe both video streams (size, fps, duration, codec, pix_fmt, audio present) and the source duration match.
- Extract frames at ~2, 20, 45, 70, 95, 112 s and LOOK at them (contact sheet is fine): text readable, nothing clipped,
  ✗ and ✓ glyphs not tofu boxes, safe bands respected, panes not stretched. Fix and re-render if not.
- Save the build script as `social/make_reel.py` (reproducible, paths relative to this run dir) and the contact sheet as
  `social/reel_contact_sheet.jpg`.

## Report
Short: files with sizes, layout chosen, panel approach (crop vs re-render), anything you could not do.
