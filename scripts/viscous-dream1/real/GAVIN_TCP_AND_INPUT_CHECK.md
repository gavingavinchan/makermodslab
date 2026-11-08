# Gavin: set the tool point and check what Dream-1 sees (real Viscous arm)

DreamLabs' rule is that a person does both of these steps, never an agent. An agent got
everything ready, but it has not run either tool, has not put in a tool point value, and
has not recorded a check.

- **(a) Tool point:** `dreamcli tcp`. Clears the sim warning "no tcp_calibration record
  says a person set its tool center point".
- **(b) Inputs check:** `python -m dream.input_check <id>`. Clears the warning "no one has
  checked what the model sees".

Everything below runs on this Mac (`gavin-mac`), where the arm and cameras are plugged in.

## What the two tools touch (read from the dreamlabs 1.0.37 source)

| | `dreamcli tcp` | `python -m dream.input_check <id>` |
|---|---|---|
| Robot / CAN / serial | **none** (`TcpServer`: "opens no arm or camera") | none |
| Cameras | none | none (it only writes the id you give it) |
| Network | the page loads three.js from a CDN; no API key, no billed call | none |
| What you do | in a browser page: load the URDF, confirm the arm's end, put a 3D marker where the closed fingertips meet, press **Set tool center point** | look at the camera frames first, then type one command |
| What it writes | `<home>/arms/viscous-arm.json` + `<home>/arms/viscous-arm.urdf` | appends `<id> <time>` to `~/.dreamlabs/inputs_checked` |

Neither tool reads live joint angles. The tool point is set on a 3D drawing of the URDF at
its zero pose, gripper closed. "On the real gripper" means **you** check that the marker
sits where the real closed fingertips meet, for example by measuring the real gripper
with a ruler or calipers.

## Before you start (both steps)

1. **No Lab session driving the arm.** In the Lab, stop any teleop, recording, replay,
   inference or tank measuring. Check: `curl -s localhost:8000/api/v1/sessions/current`
   should print `"session":null`.
2. **Arm safe and supported.** Neither tool moves the arm. Still, leave it torque-off,
   resting on its supports or folded in the tank, and nobody's hand in the tank. To be
   extra safe, switch off the arm's motor power. Neither step needs it on.
3. **Stopping.** Ctrl-C in the terminal ends `dreamcli tcp`. `dream.input_check` finishes
   in under a second. If anything ever moves, cut the arm's motor power. That is the
   hard stop.
4. **Wrist camera flip.** The `viscous_waterproof_01` record already has
   `rotation: 180` on `wrist` (uniqueID `0x1300000c45636b`, 640x480). Leave it on. Only
   step (b) needs the cameras.

## (a) Set the tool center point

1. Open a terminal and start the tool:

   ```bash
   cd ~/viscous-dream-real
   mkdir -p station
   .venv/bin/dreamcli --home ~/viscous-dream-real/station tcp --host 127.0.0.1
   ```

   `--home` keeps the output in its own folder. `--host 127.0.0.1` keeps the page off the
   LAN. The terminal prints `dreamcli tcp: serving the tool point page; writes arm files to
   .../station/arms` and opens a browser at `http://127.0.0.1:8094/?token=...`. If no
   browser opens, open the address it printed. Leave the terminal open.
2. **Load the URDF.** Press **Choose folder** and pick
   `~/Documents/viscous-arm-model/model` (the folder is better than the bare file: the
   meshes make it look like your arm). It must be this URDF (SHA-256 `c1e6c387...6ff7`,
   the same file on gavin-linux), the one the sim uses.
3. **"Is this the end of your arm?"** The page should show base `base_link` (blue) and end
   `wrist_roll_link` (green), 6 joints. That matches `viscous_arm.json`. If it shows
   another end link, click `wrist_roll_link`.
4. **"Does your robot move this joint?"** With no station file, the page asks this for all
   six joints. All six are motorised, so press **Moves** on each, unless you really mean
   to hold one still. Then press **Yes, it ends there**.
5. **Place the marker.** You will see your arm beside a YAM whose marker shows the
   convention. The legend gives the colours: **x = the way the fingers close, y = up, z =
   out of the gripper** (along the fingers). The marker starts at the `wrist_roll_link`
   origin and may not point along the fingers yet. Drag its arrows and rings, or type
   into the six fields (mm and degrees in `wrist_roll_link`'s frame), until:
   - its centre is **the point between the closed fingertips** (Dream's definition), and
   - z points out along the fingers, x along the closing direction, y up, turned like the
     YAM's.

   Check it against the real waterproof gripper (fingers closed, power off): measure from
   a feature you can see on both, such as the roll flange face, out to the fingertip
   midpoint. Use the waterproof unit's own fingers, since they may not match the bench
   unit's. **Flip up and down** turns the marker half a turn about z if y ends up
   pointing down.
6. Press **Set tool center point**. Do not use "Skip calibration": it records `skipped`,
   and every start keeps warning. The page shows the arm file's path, and the terminal
   logs `dreamcli tcp: wrote .../station/arms/viscous-arm.json (tool point calibrated)`.
   To change it, move the marker and press it again: the file is overwritten.
7. Ctrl-C in the terminal to finish.

**Check the result** (prints no secrets):

```bash
cd ~/viscous-dream-real
cat station/arms/viscous-arm.json
.venv/bin/python -I -c "import json,pathlib;from dream import arm_file as a;p=pathlib.Path('station/arms/viscous-arm.json');d=json.loads(p.read_text());print(a.tool_point_problem(d,(p.parent/d['urdf']).read_bytes()) or 'OK: tool point set by a person')"
```

The JSON should have `"base": "base_link"`, `"flange": "wrist_roll_link"`, your `tcp`,
`approach` and `jaw`, and `"tcp_calibration": {"status": "calibrated", "at": ..., "digest": ...}`.
The one-liner should print `OK`. For comparison only: the sim's hand-written file has never
been confirmed by a person. It put the point 132 mm along `wrist_roll_link` x (the finger-pad
midpoint), and the fingertip variant put it 3.4 cm further out. If yours is several cm from
both, look again before sending it.

## (b) Check what the model sees, then record it

`dream.input_check` does no checking itself. It records that **you looked**. The id is
12 hex digits computed from what the runner sends: each camera's key and JPEG size, each
state key's width, and the robot name (`other_embodiment`). Pixels and joint values are
not part of it. So:

1. **Get the frames and the id from the real-arm runner's no-motion preview.** The runner
   agent is building it in `scripts/viscous-dream1/real/`, and its README gives the exact
   command. That mode opens the cameras as a live run would, makes no API call, sends no
   motor command and opens no CAN bus. It then shows each frame as sent and prints the
   reminder line, which ends in `python -m dream.input_check <id>`.
2. **Look**, with the arm parked in its working view:
   - each camera is on the right key (`video.overhead` = the top view, `video.wrist_l` =
     the working arm's wrist; whatever stands in for `wrist_r` is as the runner's README
     says),
   - **upright**: the wrist image should look like the real Maker frames in this folder
     (`makerwrist_maker_*.jpg`), with the jaws coming up from the **bottom** of the
     image. If they come down from the top, the flip is wrong or the camera is mounted
     differently. Stop and tell me,
   - **natural colours** (skin is not blue: a blue tint means RGB/BGR are swapped),
   - the state is in **radians** (about -3.2 to 4.4, not hundreds of degrees),
   - the prompt is the task you mean.
3. **Only if all of that is right**, record it with the id the preview printed:

   ```bash
   ~/viscous-dream-real/.venv/bin/python -m dream.input_check <the 12-hex id>
   ```

   It prints `recorded in /Users/gavin/.dreamlabs/inputs_checked: no reminder for these
   inputs ...`. A malformed id prints an error and exits 2 without recording.
4. **Check:** `cat ~/.dreamlabs/inputs_checked` shows your id and the time.

When you need to do it again: the id changes if a camera is added, renamed or resized, or
a state part changes. The record belongs to this computer, so a run from gavin-linux
needs its own check there. Today the record has **only the wrist camera**, and the sim
needs a top camera (the sim results also depend on a second, parked arm). Do this check
with the final camera set.

## What to send back

- (a) the path and contents of `station/arms/viscous-arm.json` (no secrets in it), the
  terminal's `wrote ...` line, and a screenshot of the marker on the page.
- (b) the id you recorded, the `recorded in ...` line, and the frames you looked at (or a
  screenshot of the preview).
- Anything that looked wrong: the camera upside down, the end link, the marker convention.

Never paste `~/.dreamlabs/credentials` or the API key anywhere. Neither step needs it.

## For the runner agent (how to consume these)

- **No hardware adapter is needed for either tool.** `dreamcli tcp` works from the URDF
  only, and `dream.input_check` only appends an id. Dream's own station drivers
  (`robots/makers.py`, RobStride over slcan) and `python -m dream.check` are **not** in
  the path. `dream.check` drives the arm through the loop (8 cm hand circles, 10 degree
  joint moves, about 15 s), so do not use it in the tank without Gavin deciding to.
- **Arm file:** use `ik: {"arm": "<path>/viscous-arm.json", "head": "eef"}`. Keep the
  JSON **with the `viscous-arm.urdf` that `dreamcli tcp` wrote beside it**: the digest is
  computed over that copy's bytes, which are the source URDF plus a trailing newline.
  Pointing `urdf` back at `~/Documents/viscous-arm-model/...` breaks the digest, and the
  warning comes back. To keep the repo self-contained, copy both files together,
  byte-for-byte. The digest covers only `base`, `flange`, `tcp`, `approach`, `jaw` and the
  URDF bytes. `dreamcli tcp` writes `rest` as the URDF zero clamped to limits (all zeros
  here, not `viscous_arm.json`'s work pose). `rest` is outside the digest, so the runner
  may set it, but **never** edit the tool keys. Before a live run, gate on
  `dream.arm_file.tool_point_problem(arm, urdf_bytes) is None`.
- **No-motion inputs preview** (needed for step b): open the record's cameras the same
  way the live loop does (lerobot `OpenCVCameraConfig` with `rotation: 180`, so frames
  arrive upright). Encode JPEGs from RGB correctly (`cv2.imencode` expects BGR). Build
  the observation through the real `DreamClient.policy(config)` with the live config,
  with `policy._transport` replaced by a local stub, as `viscous_dream_sim.py`'s dry run
  does. The client then fires the reminder on the first request, from the observation
  as sent (with the `ik` hand-pose keys and the `embodiment` key added), and prints the
  id. Or compute `dream.input_check.inputs_id(obs_as_sent)` yourself. Show the frames
  (save the JPEGs, or `dream.DebugView` on 127.0.0.1). No motor enable, no CAN open, no
  billed call. Before a live run, gate on
  `dream.input_check.checked(inputs_id(obs_as_sent))`. Never call
  `dream.input_check.record()` from code.
- Venv: `~/viscous-dream-real/.venv` (Python 3.12.13, dreamlabs 1.0.37,
  dreamlabs-client 1.1.3, mujoco 3.13.0, jax/jaxlib 0.7.1, numpy 2.2.6; Grace's pins).
  `MUJOCO_GL=cgl` on macOS. The Mac has no `~/.dreamlabs/credentials` yet. A live run
  needs Gavin to create it (0600), ideally with a fresh key, since the old one was pasted
  into a chat.
