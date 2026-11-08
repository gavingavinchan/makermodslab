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
"""Record the Dream-1 start pose of the real Viscous arm with one key press, during Lab teleop.

Gavin (2026-11-08): "ill need a button on the keyboard to press when i teleop the arm to the right
position". This never opens the CAN bus: it listens to the running Lab server's joint broadcast
(/api/v1/ws/joint-data, the follower's `joints_deg` from teleoperate.get_can_joint_data), so it runs
beside an ordinary Lab teleop session and cannot move anything.

Keys (in this terminal window):
  space  record now (median of the last 0.5 s of samples)
  c      record after a 5 s countdown, so both hands can go back on the leader
  q      quit

Each record is written to runs/start_pose/<utc>.json with the motor degrees, the hand and wrist
positions against the workspace box, and the make_tank_arm.py command that turns it into the IK
rest. The live line also shows J5 (wrist_yaw) and J6 (wrist_roll) motor readings: teleop is
motor-to-motor and never uses configs/viscous/model/motor_to_urdf.json, so watching which way these
numbers move is the check of those two signs.

Run (Mac, Lab server up on :8000, teleop running):
  .venv/bin/python scripts/viscous-dream1/real/record_start_pose.py
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import select
import statistics
import sys
import termios
import time
import tty
from collections import deque
from pathlib import Path

import dream_real_core as core
import websockets

HERE = Path(__file__).resolve().parent
OUT = HERE / "runs/start_pose"
BOX_FILE = HERE / "workspace_box_viscous_waterproof_01.json"
SRC_URDF = core.ROOT / "configs/viscous/model/viscous_arm.urdf"
TANK_ARM = HERE / "tank_model/viscous_waterproof_tank_arm.json"
WINDOW_S = 0.5
COUNTDOWN_S = 5


def snapshot(samples: deque) -> dict:
    """Median per motor over the last WINDOW_S of samples."""
    newest = samples[-1][0]
    recent = [s for t, s in samples if newest - t <= WINDOW_S]
    return {m: round(statistics.median(s[m] for s in recent), 2) for m in core.MOTORS if m in recent[0]}


def record(motor: dict, box: core.WorkspaceBox, jmap: core.JointMap) -> Path:
    utc = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    arm_only = {m: motor[m] for m in core.ARM}
    positions = {k: [round(float(v), 1) for v in xyz] for k, xyz in box.positions_mm(arm_only).items()}
    rec = {
        "what": "Dream-1 start pose, recorded from Lab teleop's joint broadcast",
        "recorded_utc": utc,
        "motor_deg": motor,
        "ik_rest_rad": [round(v, 6) for v in jmap.to_ik(arm_only)],
        "box_positions_mm": positions,
        "box_violations": box.violations(arm_only),
        "make_tank_arm": (
            ".venv/bin/python scripts/viscous-dream1/real/make_tank_arm.py --rest-motor-deg "
            f"'{json.dumps(arm_only)}'"
        ),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{utc}.json"
    core.atomic_json(path, rec)
    return path


def key_pressed() -> str | None:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


async def run(url: str) -> None:
    jmap = core.JointMap.load()
    tcp = json.loads(TANK_ARM.read_text())["tcp"]
    box = core.WorkspaceBox.load(BOX_FILE, SRC_URDF, jmap, tcp_m=tcp)
    samples: deque = deque(maxlen=400)
    countdown_until = None
    print(__doc__.split("\n\n")[2])
    print(f"connecting to {url} ...\r")
    async with websockets.connect(url, max_size=None) as ws:
        while True:
            try:
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.1))
            except TimeoutError:
                msg = None
            if msg and msg.get("type") == "joint_update" and msg.get("joints_deg"):
                deg = msg["joints_deg"]
                if all(m in deg for m in core.ARM):
                    samples.append((time.monotonic(), deg))
            now = time.monotonic()
            fresh = samples and now - samples[-1][0] < 1.0
            key = key_pressed()
            if key == "q":
                print("\r\nbye\r")
                return
            if key in (" ", "c"):
                if not fresh:
                    print("\r\nno fresh joint data: is Lab teleop running?\r")
                elif key == " ":
                    countdown_until = now
                else:
                    countdown_until = now + COUNTDOWN_S
            if countdown_until is not None and fresh and now >= countdown_until:
                countdown_until = None
                motor = snapshot(samples)
                path = record(motor, box, jmap)
                bad = box.violations({m: motor[m] for m in core.ARM})
                sys.stdout.write("\a")
                print(
                    f"\r\nRECORDED {path.name}: " + ", ".join(f"{m} {v:.1f}" for m, v in motor.items()) + "\r"
                )
                print(("  box: OK" if not bad else "  box: " + "; ".join(bad)) + "\r")
            if not fresh:
                line = "waiting for teleop joint data ..."
            else:
                d = samples[-1][1]
                hand = box.positions_mm({m: d[m] for m in core.ARM}).get("tcp")
                hand_s = "" if hand is None else f" | hand x{hand[0]:.0f} y{hand[1]:.0f} z{hand[2]:.0f} mm"
                line = (
                    " ".join(f"{m[:5]}{d[m]:7.1f}" for m in core.ARM)
                    + f" grip{d.get('gripper', math.nan):6.1f}"
                    + hand_s
                )
                if countdown_until is not None:
                    line = f"[{math.ceil(countdown_until - now)}] " + line
            sys.stdout.write("\r\x1b[2K" + line)
            sys.stdout.flush()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="ws://127.0.0.1:8000/api/v1/ws/joint-data")
    args = p.parse_args()
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        asyncio.run(run(args.url))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
