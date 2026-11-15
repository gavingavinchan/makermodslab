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
"""Read-only observations for Lab. Never participates in a safety decision."""

from __future__ import annotations

import base64
import contextlib
import json
import os
import threading
import time
from pathlib import Path

SLOTS = ("overhead", "wrist_l", "wrist_r")


class DreamView:
    """Atomically publish the newest JPEG from each request, without re-encoding.

    The control loop and Dream worker share this object. Failures are reported
    once and swallowed: a full disk must never interrupt a return to rest.
    """

    def __init__(self, directory, *, run, mode, prompt, max_calls, slots, wall=time.time):
        self.directory = Path(directory)
        self.wall = wall
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.problem_reported = False
        self.status = {
            "run": run,
            "mode": mode,
            "prompt": prompt,
            "max_calls": max_calls,
            "slots": slots,
            "pid": os.getpid(),
            "phase": "connect",
            "state": "starting",
            "call": 0,
            "last_reason": None,
            "measured": None,
            "commanded": None,
            "requested_tcp_path": [],
            "peak_torque_nm": 0.0,
            "peak_temp_c": 0.0,
            "inputs_id": None,
            "inputs_checked": None,
            "images_written": None,
        }
        self.update()

    def _atomic(self, name, data):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / name
        tmp = path.with_name(f".{name}.{os.getpid()}.{threading.get_ident()}.tmp")
        try:
            tmp.write_bytes(data)
            os.replace(tmp, path)
        finally:
            with contextlib.suppress(FileNotFoundError):
                tmp.unlink()

    def _write_status(self):
        # Caller owns write_lock. Never hold the state lock for JSON/JPEG or IO.
        with self.lock:
            self.status["written"] = self.wall()
            snapshot = dict(self.status)
        self._atomic("status.json", json.dumps(snapshot, allow_nan=False, default=str).encode())

    def _problem(self, error):
        if not self.problem_reported:
            self.problem_reported = True
            with contextlib.suppress(Exception):
                print(f"dream-view unavailable: {type(error).__name__}: {error}", flush=True)

    def update(self, **values):
        try:
            with self.lock:
                self.status.update(values)
            # The 20 Hz control loop must not wait behind request JPEG IO.
            # That writer publishes the latest state on completion; subsequent
            # ticks also flush any update arriving after its snapshot.
            if not self.write_lock.acquire(blocking=False):
                return
            try:
                self._write_status()
            finally:
                self.write_lock.release()
        except Exception as error:
            self._problem(error)

    def observation(self, observation, **values):
        try:
            images = {}
            for slot in SLOTS:
                value = observation[f"video.{slot}"]
                newest = value[-1] if isinstance(value, list) else value
                images[slot] = base64.b64decode(newest, validate=True)
            with self.write_lock:
                for slot, data in images.items():
                    self._atomic(f"{slot}.jpg", data)
                with self.lock:
                    self.status.update(values, images_written=self.wall())
                self._write_status()
        except Exception as error:
            self._problem(error)


class ViewTransport:
    """Observe the request before transport and Dream's path before the slide.

    Delegate the identical request, headers and reply; the safety transform
    stays in SlideTransport. The preflight gate separately publishes requests
    it refuses before this transport is reached.
    """

    def __init__(self, real, view):
        self.real, self.view = real, view

    def request(self, request, headers, **kwargs):
        observation = request.get("observation")
        if observation is not None:
            self.view.observation(observation)
        reply = self.real.request(request, headers, **kwargs)
        with contextlib.suppress(Exception):
            path = reply.get("actions", {}).get("per_key", {}).get("action.left_eef_xyz", [])
            self.view.update(requested_tcp_path=path.tolist() if hasattr(path, "tolist") else path)
        return reply

    def __getattr__(self, name):
        return getattr(self.real, name)
