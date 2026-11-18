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
"""Line-delimited JSON MuJoCo tank process. Run with the dreamtrial Python."""

import json
import os
import queue
import sys
import threading
import time
import traceback
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

from cameras import Cameras
from physics import Tank
from scene import DT


def dump(value):
    return json.dumps(value, allow_nan=False, separators=(",", ":"))


class Session:
    def __init__(self, req):
        directory = Path(req["run_dir"])
        if not directory.is_absolute() or not Path(req["arm_file"]).is_absolute():
            raise ValueError("init requires absolute run_dir and arm_file")
        self.tank = Tank(req, visuals=True)
        self.directory = directory
        directory.mkdir(exist_ok=True, parents=True)
        self.log = (directory / "sim-telemetry.jsonl").open("x", buffering=1)
        self.contacts = (directory / "sim-contacts.jsonl").open("x", buffering=1)
        self.marks = (directory / "sim-marks.jsonl").open("x", buffering=1)
        self.camera = Cameras(self.tank, directory)
        self.max_backlog = 0.0
        self.clock = time.monotonic()
        try:
            self.camera.offer(self.tank)
            self.camera.get(wait=True)
            self.clock = time.monotonic()
            self.log_sample()
            (directory / "sim-model.json").write_text(
                dump(
                    self.tank.metadata
                    | {"task": req["task"], "variant": req["variant"], "start": req["start"]}
                )
                + "\n"
            )
        except BaseException:
            self.close()
            raise

    def log_sample(self):
        self.log.write(
            dump(self.tank.truth() | self.tank.read() | {"wall_s": time.monotonic() - self.clock}) + "\n"
        )

    def catch_up(self):
        # Snapshot the deadline once. Every elapsed step is executed, including
        # time spent processing previous requests, camera work and OS preemption.
        due = int((time.monotonic() - self.clock) / DT)
        self.max_backlog = max(self.max_backlog, (due - self.tank.step_count) * DT)
        while self.tank.step_count < due:
            self.tank.step()
            for event in (*self.tank.events, *self.tank.object_events):
                self.contacts.write(dump(event) + "\n")
            self.tank.events.clear()
            self.tank.object_events.clear()
            if self.tank.step_count % 25 == 0:
                self.log_sample()
            if self.tank.step_count % 100 == 0:
                self.camera.offer(self.tank)
        if self.camera.error:
            raise RuntimeError("camera worker failed") from self.camera.error

    def request(self, req):
        tank = self.tank
        op = req["op"]
        if op == "read":
            return tank.read()
        if op == "truth":
            return tank.truth()
        if op == "frames":
            return self.camera.get()
        if op == "engage":
            return tank.engage()
        if op == "goal":
            tank.goal(req["target"])
        elif op in ("release", "gripper_off"):
            tank.release(gripper_only=op == "gripper_off")
        elif op == "mark":
            tank.label = req["label"]
            self.marks.write(dump({"t_s": tank.data.time, "label": tank.label}) + "\n")
        else:
            raise ValueError(f"unknown op {op!r}")
        return {}

    def close(self):
        try:
            self.log_sample()
            self.camera.close()
            summary = self.tank.truth() | {
                "wall_s": time.monotonic() - self.clock,
                "max_backlog_s": self.max_backlog,
                "camera_frames": self.camera.rendered,
                "missed_camera_snapshots": self.camera.dropped,
            }
            (self.directory / "sim-summary.json").write_text(dump(summary) + "\n")
        finally:
            self.log.close()
            self.contacts.close()
            self.marks.close()


def main():
    incoming = queue.Queue()

    def reader():
        for line in sys.stdin:
            incoming.put(line)
        incoming.put(None)

    threading.Thread(target=reader, daemon=True).start()
    session = None
    try:
        while True:
            if session:
                session.catch_up()
            try:
                line = incoming.get(timeout=0.004 if session else None)
            except queue.Empty:
                continue
            if line is None:
                break
            close = False
            try:
                req = json.loads(line)
                op = req["op"]
                if session:
                    session.catch_up()
                if op == "close":
                    close = True
                    if session:
                        closing, session = session, None
                        closing.close()
                    reply = {}
                elif op == "init" and session is None:
                    session = Session(req)
                    reply = {}
                elif session is None:
                    raise ValueError("init required")
                else:
                    reply = session.request(req)
                output = dump({"ok": True, **reply})
            except Exception as exc:
                traceback.print_exc(file=sys.stderr)
                output = dump({"ok": False, "error": str(exc)})
            print(output, flush=True)
            if close:
                break
    finally:
        if session is not None:
            session.close()


if __name__ == "__main__":
    main()
