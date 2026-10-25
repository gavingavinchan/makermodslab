# Task: stop the thread engage/release chatter at the bolt tip (no Astra runs)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, same env and hard rules as `BRIEF_ASTRA_THREAD_FRICTION.md`.
**Astra episode `astra_g3_20261025_003` may still be running with its own server: do not touch that server, its
socket, its workspace or its output files.** Read them only.

## What happened (Astra episode 3)
With the friction fix in place, Astra brought the nut coaxial onto the bolt tip several times (e.g. commands
358-366 and 513-517) and turned it with matching feed (`turn right 30 --feed-mm 0.291667`). The nut never advanced.
`episode.jsonl` shows the thread model chattering every 50 ms: `engage`, then `release` with `pull_m: 0.003`, then
`engage` again, with turns stuck at about 0.00. The engagement gate (nut face within about 3 mm of the bolt tip)
and the pull-off release gate (pulled back 3 mm) overlap, so a nut sitting at the entry is engaged and released in
the same instant and its rotation never accumulates. The scripted validation never hit this because it pushes
the nut further on before turning.

## Fix
1. Make the gates consistent, with hysteresis, like a real nut starting on a thread: once engaged, release only
   when the nut is pulled back clearly beyond where it engaged (for example the release point measured from the
   engagement position, at least one pitch behind it), or when it is backed off by unscrewing past the tip. A nut
   at the tip that is being turned clockwise with light forward pressure must catch and advance.
2. Keep entry gates (3 mm radial, 10 deg), pitch, hand, friction and the release-on-pull behaviour otherwise as
   they are. State exactly what changed.

## Validate
1. Replay Astra episode 3's commands up to and including 366 against a fresh `--profile astra` server: after
   360 (`turn right 60 --feed-mm 0.583333`) the nut is engaged and has advanced about 1/6 turn; no chatter events.
   Do the same for commands up to 517.
2. Release test from the friction brief still passes (released nut does not drift), and a genuine pull-off
   (pull the nut back 10 mm without turning) still releases.
3. The scripted G3 validation (`--profile astra`) still passes, ideally seated.
4. Short section at the top of `runs/astra_flange/harness_report.md`. Leave no server you started running.
