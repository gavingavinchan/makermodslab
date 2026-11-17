#!/usr/bin/env python3
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
"""One-click 'record this pose' window: Gavin poses the arm BY HAND, torque off, and clicks (2026-11-16).

Used for the dry-air start pose ("give me a button to click once i moved the arm into position").
Turn a record into the arm file's rest with make_tank_arm.py (motor deg -> IK radians), then run
dream_real.py with --keep-arm-rest. Usage: record_pose_button.py [--robot NAME] (the Lab robot record).

Click: Lab's tank-measure (torque-off, never enables or commands a motor) opens the bus, takes a still
reading, closes the bus; the pose is saved and shown. Refuses while dream_real.py runs.
"""

import argparse
import datetime
import json
import subprocess
import time
import tkinter as tk
import urllib.request
from pathlib import Path

LAB = "http://127.0.0.1:8000/api/v1"
_args = argparse.ArgumentParser(description=__doc__)
_args.add_argument("--robot", default="viscous_waterproof_01_dry_air", help="Lab robot record name")
ROBOT = _args.parse_args().robot
OUT = Path(__file__).resolve().parent / "runs" / "start_pose"


def call(path, body=None):
    req = urllib.request.Request(
        LAB + path,
        data=None if body is None else json.dumps(body).encode(),
        headers={"content-type": "application/json"},
        method="GET" if body is None else "POST",
    )
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


def record():
    if subprocess.run(["pgrep", "-f", "[d]ream_real.py"], capture_output=True).returncode == 0:
        show("The Dream runner is still running.\nRelease it with the red button first.", "#b00")
        return
    show("Reading (torque stays off)...", "#555")
    root.update()
    try:
        call("/tank-measure/start", {"robot_name": ROBOT})
        status = None
        for _ in range(30):
            time.sleep(0.2)
            status = call("/tank-measure/status")
            if status.get("positions") and not status.get("stale") and status.get("still"):
                break
        call("/tank-measure/stop", {})
    except Exception as error:  # noqa: BLE001
        show(f"Could not read the arm: {error}", "#b00")
        return
    if not status or not status.get("positions") or status.get("stale"):
        show(f"No fresh reading: {status and status.get('error')}", "#b00")
        return
    pose = status["positions"]
    stamp = datetime.datetime.now(datetime.UTC).strftime("%Y%m%dT%H%M%SZ")
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{stamp}-dry-air-start.json"
    path.write_text(
        json.dumps(
            {
                "what": f"Dream start pose candidate for {ROBOT}, recorded by Gavin's click (torque off)",
                "still": status.get("still"),
                "motor_deg": pose,
                "fingertip_mm": status.get("probe_mm"),
                "link_origins_mm": status.get("link_origins_mm"),
            },
            indent=1,
        )
    )
    tip = status.get("probe_mm") or [0, 0, 0]
    lines = "\n".join(f"{k:14s} {v:8.1f}" for k, v in pose.items())
    show(
        f"Recorded{'' if status.get('still') else ' (arm was MOVING)'}:\n{lines}\n"
        f"fingertip {-tip[0]:.0f} mm out, {tip[2]:.0f} mm up\nsaved {path.name}",
        "#060",
    )


def show(text, colour):
    label.config(text=text, fg=colour)


root = tk.Tk()
root.title("Record start pose")
root.attributes("-topmost", True)
tk.Button(
    root, text="RECORD START POSE", font=("Helvetica", 22, "bold"), command=record, padx=20, pady=20
).pack(padx=16, pady=12)
label = tk.Label(
    root, text="Move the arm into position, hold it still, click.", font=("Menlo", 13), justify="left"
)
label.pack(padx=16, pady=(0, 14))
root.mainloop()
