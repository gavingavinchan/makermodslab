# Copyright 2026 MakerMods. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Motor and camera adapter for the real-time tank child. The child alone imports MuJoCo."""

import base64
import io
import json
import math
import os
import select
import subprocess
import time
from pathlib import Path

from model import ROOT, core, real
from PIL import Image

from makermodslab.maker_rest_pose import MAKER_RETURN_CEILING_S, MAKER_RETURN_FPS, return_maker_to_pose

SIM_PYTHON = Path("~/dreamtrial/.venv/bin/python").expanduser()
SIM_CHILD = ROOT / "scripts/viscous-sim/child.py"


class SimArm:
    backend = "sim"

    def __init__(self, variant, task, folder, arm_file, start):
        self.folder = folder.resolve()
        self.init_fields = {
            "variant": variant,
            "task": task,
            "run_dir": str(self.folder),
            "arm_file": str(arm_file.resolve()),
            "start": start,
        }
        self.process = None
        self.target = dict(start)
        self.cameras = True
        self.last_frames = {}

    def connect(self):
        with (self.folder / "sim-child.log").open("ab") as errors:
            self.process = subprocess.Popen(
                [SIM_PYTHON, SIM_CHILD],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=errors,
                env={**os.environ, "MUJOCO_GL": "egl"},
                start_new_session=True,
            )
        self.request("init", **self.init_fields)
        self.frames()  # Verify both simulated views before allowing engage.

    def request(self, op, **fields):
        # Single owner, one outstanding request. Never match a late reply to the next request.
        process = self.process
        if process is None or process.poll() is not None:
            raise RuntimeError(f"sim child unavailable during {op}; see {self.folder / 'sim-child.log'}")
        process.stdin.write(json.dumps({"op": op, **fields}, allow_nan=False).encode() + b"\n")
        process.stdin.flush()
        deadline = time.monotonic() + (60 if op in ("init", "close") else 5)
        line = bytearray()
        while not line.endswith(b"\n"):
            if not select.select([process.stdout], [], [], max(0, deadline - time.monotonic()))[0]:
                process.kill()
                process.wait()
                raise TimeoutError(f"sim child timed out during {op}")
            part = os.read(process.stdout.fileno(), 65536)
            if not part:
                raise ConnectionError(
                    f"sim child disconnected during {op}; see {self.folder / 'sim-child.log'}"
                )
            line.extend(part)
            if len(line) > 16 * 1024 * 1024:
                process.kill()
                process.wait()
                raise ValueError(f"oversized sim {op} reply")
        reply = json.loads(line)
        if reply.get("ok") is not True:
            raise RuntimeError(f"sim {op}: {reply.get('error', 'invalid response')}")
        return reply

    def close(self):
        if (process := self.process) is None:
            return
        try:
            if process.poll() is None:
                self.request("close")
                process.wait(timeout=10)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            process.stdin.close()
            process.stdout.close()
            self.process = None

    def frames(self):
        if self.cameras:
            reply = self.request("frames")
            pair = {k: base64.b64decode(reply[k], validate=True) for k in ("head", "left")}
            for data in pair.values():
                with Image.open(io.BytesIO(data), formats=("JPEG",)) as image:
                    image.load()
            self.last_frames = pair
        return self.last_frames

    def drop_cameras(self):
        self.cameras = False

    def read(self):
        reply = self.request("read")
        return real.Reading(reply["pos"], reply["torque"], reply["temp"], {})

    def engage(self):
        self.target = self.request("engage")["pos"]
        return dict(self.target)

    def send(self, target, previous):
        self.request("goal", target=target)
        self.target = dict(target)

    def reanchor(self, offsets):
        if any(not math.isfinite(v) or abs(v) > real.REANCHOR_MAX_OFFSET_DEG for v in offsets.values()):
            raise ValueError("re-anchor offset too large")
        pos = self.read().pos
        goal = {**self.target, **{m: pos[m] + offsets[m] for m in core.ARM}}
        self.send(goal, self.target)
        return {m: pos[m] for m in core.ARM}

    def release_gripper(self):
        self.request("gripper_off")

    def return_to_rest(self, pose, abort, speed, watch=None):
        # The same feedback/settling helper used by LabArm. After gripper_off the
        # child's jaw is limp; retain its last setpoint in full seven-motor packets.
        arm = self
        tripped = []
        speed = min(speed, real.HARD_ARM_SPEED_DEG_S)
        step = speed / MAKER_RETURN_FPS

        class Feedback:
            def get_observation(self):
                r = arm.read()
                if watch and not tripped:
                    tripped.extend(watch(r.pos, r.torque, r.temp, arm.target))
                if tripped:
                    raise real.TripStopError(tripped)
                return {f"{m}.pos": r.pos[m] for m in core.ARM}

            def send_action(self, action):
                self.get_observation()
                if abort.is_set():
                    return
                # Match the bus slew cap even when a sagging joint starts a leg
                # several degrees from the re-anchored target.
                target = dict(arm.target)
                for m in core.ARM:
                    delta = action[f"{m}.pos"] - target[m]
                    target[m] += max(-step, min(step, delta))
                arm.send(target, arm.target)

        # Budget for the ramp, standing error at its start, and settling, as on LabArm.
        span = max(abs(pose[m] - self.target[m]) for m in core.ARM) + real.REANCHOR_MAX_OFFSET_DEG
        return return_maker_to_pose(
            Feedback(),
            pose,
            abort_event=abort,
            speed_deg_s=speed,
            ceiling_s=max(MAKER_RETURN_CEILING_S, 2 * span / speed + 3),
        )

    def release(self):
        self.request("release")

    def truth(self):
        return self.request("truth")

    def mark(self, label):
        return self.request("mark", label=label)
