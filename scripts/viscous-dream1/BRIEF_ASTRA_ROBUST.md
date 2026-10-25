# Task: make the Astra harness survive a stalled move and an interrupted client (no Astra runs)

Work in `~/dreamtrial/viscous_sim` on gavin-linux, same env and hard rules as `BRIEF_ASTRA_FIXES.md` (write only in
`astra_op/`, `runs/astra_flange/`, `~/dreamtrial/astra_ws`; no Dream calls, ledger, runner lock, hardware, propeller
files or original flange sources).

## What happened (first Astra episode, `runs/astra_flange/astra_g3_20261025_001*`)
At command 29 Astra sent `move right -0.122 -0.120 0.9244 0.995 0 -0.10`. The right arm stalled: several joints sat at the
25 Nm saturation, and the command did not return for 183 s of wall time. Astra pressed Ctrl-C, `./robot` died in
`recv`, the server raised `BrokenPipeError` in `conn.sendall` (server.py ~line 418) and exited. The episode ended
with the socket gone. Astra had done nothing wrong. See the `_server.log`, `_astra.log` and `episode.jsonl`.

## Fixes
1. **Server never dies on a client problem.** A disconnect, a broken pipe, a reset or malformed input is logged
   in `episode.jsonl`, and the server keeps serving. A command that was executing completes (or stops as in 2),
   and its result is kept: add `./robot last` to print the previous command's result.
2. **Moves stop when they stall.** If the tool point stops progressing toward the target (for example under 1 mm
   of progress in 1 s of simulated time), or the arm saturates for more than about 0.5 s, stop the motion, hold
   the current pose (joint references set to the measured joints) and return `stalled: true` with the reached pose,
   the remaining error and which joints were saturated. Apply the same to `nudge` and `turn`.
3. **No command runs longer than 90 s wall time.** On the limit, stop and hold as in 2 and return
   `timed_out: true`. Print a progress line to the client every ~10 s so an operator can see the command is alive.
4. **README_OPERATOR.md**: say that commands normally return within 90 s, show the progress lines, describe
   `stalled` / `timed_out`, say not to interrupt `./robot` (it is safe to wait), and add `./robot last`.
5. If a stall comes from a target the arm cannot reach in that orientation, the `move` response should say so
   plainly (IK residual and limit flags are already there; make the wording clear).

## Validate
1. Replay Astra's first 29 commands from `episode.jsonl` against a fresh `--profile astra` server: command 29 now
   returns `stalled` within 90 s and the server stays up; `look` works afterwards.
2. Kill a client mid-move (SIGINT): the server stays up, and `./robot last` and `./robot look` work.
3. The scripted G3 validation under `--profile astra` still passes (>= 2 turns, ideally seated).
4. Update `runs/astra_flange/harness_report.md` (short section at the top). Leave no server running.
