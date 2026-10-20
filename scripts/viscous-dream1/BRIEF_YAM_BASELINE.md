# Task: Dream-1 baseline on Dream's own simulated YAM (station mode vs vision-only mode)

Work in `~/dreamtrial/viscous_sim` on this machine (gavin-linux). Python: `~/dreamtrial/.venv/bin/python` (has
`dream`, `dream_ik`, `dream_examples`, mujoco). Render offscreen with `MUJOCO_GL=egl`. The Dream client source is
installed in `~/dreamtrial/.venv/lib/python3.12/site-packages/{dream,dream_ik,dream_examples}`; read it whenever you
need to know how something works.

## Why
We drove our own robot arm (the "Viscous" arm) in MuJoCo with Dream Labs' hosted model `dreamlabs/dream-1-beta`
and it produced meaningless motion (see `runs/live_yamrig_1`). We now know (from `dream/client_ik.py` and
`dream/__init__.py`) that for any arm other than YAM the client sends `robot: other_embodiment`, and the server then
WITHHOLDS the whole robot state (joints, hand pose, grip): the model sees only the 3 cameras and the prompt, and
answers hand poses "composed on the hand pose the request carries". We checked offline that our arm's hand frame
matches Dream's convention. So the open question is: is Dream-1 any good in that vision-only mode even on its own
YAM arm? This task answers it with two live runs in Dream's own shipped YAM scene.

## What to do
`yam_baseline.py` (already in this directory, written by another agent, NOT yet run) has two modes:
- `--mode station`: exactly what Dream's `dream_examples/yam_mujoco/dream_control.py` does for beta (YAM station
  keys; the model reads the arm's state).
- `--mode vision`: client IK `{"arm": "yam", "head": "eef"}` with `robot: "other_embodiment"` (state withheld),
  the same path our Viscous arm uses.

1. Read `yam_baseline.py` and `dream_examples/yam_mujoco/dream_control.py`. Run both dry runs:
   `python yam_baseline.py --mode station --out runs/yam_station_dry` and the same with `--mode vision`.
   Fix any bugs (for example the hand-height field in `objects()` guesses a body name `left_link_6` that may not
   exist; use the `left_tcp` site instead). Keep the two modes as described; do not change what each one sends.
   Also make sure the vision mode's config is accepted by the client (it compiles IK on open; that takes a while).
2. Live run 1: `python yam_baseline.py --mode station --live --max-calls 20 --out runs/yam_station_live1`
3. Live run 2: `python yam_baseline.py --mode vision --live --max-calls 20 --out runs/yam_vision_live1`
4. For each live run, look at the saved `frame_*.png` images (third-person left, the model's top camera right) and
   the log, and judge plainly: did the left or right arm reach toward the apple, close on it, lift it, carry it to
   the bowl, release it? Record the closest apple-to-bowl distance from the log. Also note anything odd (arm
   flailing, rising away from the table, joint-limit notices, gripper never opening/closing).

## Hard rules
- Live calls are billed. At most 20 calls per live run, and at most 50 live calls in total for this whole task
  (if a run crashes midway and you must rerun, count the calls already made). Never loop live calls otherwise.
- A DA-2000 `pool_warming` refusal is not billed: waiting and retrying it is fine.
- Do not touch hardware: no serial ports, no CAN, no /dev/video*, no systemd services. A real robot is connected to
  this machine; leave it alone.
- Only write inside `~/dreamtrial/viscous_sim`. Do not edit the installed `dream*` packages. Never print or copy the
  API key in `~/.dreamlabs/credentials`.

## Report (final message)
For each run: the command, number of billed calls, what the arms did (your plain judgement from the frames), the
closest apple-to-bowl distance, and any errors. Then any changes you made to `yam_baseline.py` and why. Keep it short.
