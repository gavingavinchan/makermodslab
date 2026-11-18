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
"""Record closed-tip contacts through Lab's torque-off tank-measure session on the Mac."""

import argparse
import json
import math
import os
import select
import subprocess
import sys
import time
import urllib.request
from contextlib import contextmanager
from datetime import UTC, datetime
from pathlib import Path

from targets import GRID, target_xy

MOTORS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper")


def call(action, body=None):
    request = urllib.request.Request(
        "http://127.0.0.1:8000/api/v1/tank-measure/" + action,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=10) as response:
        return json.load(response)


@contextmanager
def lab(robot):
    if subprocess.run(["pgrep", "-f", "[d]ream_real.py"], capture_output=True).returncode == 0:
        raise RuntimeError("Release the Dream runner before starting torque-off measurement")
    start = call("start", {"robot_name": robot})
    trace = start.get("trace_path")
    try:

        def status():
            state = call("status")
            if not state["active"] or state["robot_name"] != robot or state.get("trace_path") != trace:
                raise RuntimeError("Lab measurement ended or was replaced")
            return state

        yield status
    finally:
        state = call("status")
        if state["active"] and state["robot_name"] == robot and state.get("trace_path") == trace:
            call("stop", {})


def fresh(state):
    p = state.get("positions")
    return (
        p
        if not state.get("stale", True)
        and not state.get("error")
        and p
        and set(p) == set(MOTORS)
        and all(math.isfinite(v) for v in p.values())
        else None
    )


def record_line(file, record):
    file.write(json.dumps(record, allow_nan=False) + "\n")
    file.flush()
    os.fsync(file.fileno())


def now():
    return datetime.now(UTC).isoformat()


def distance(a, b):
    return max(abs(a[j] - b[j]) for j in MOTORS[:-1])


class Hold:
    """Require a move away from the last contact, then contiguous fresh stationary poses."""

    def __init__(self, start):
        self.reference = start
        self.moved = False
        self.anchor = None
        self.last = None

    def update(self, p, still, t):
        if p is None:
            self.anchor = None
            return False
        if self.reference is None:
            self.reference = p
        self.moved |= distance(p, self.reference) >= 3
        if not self.moved or not still or abs(p["gripper"]) > 5:
            self.anchor = None
        elif self.anchor is None or t - self.last > 0.65 or distance(p, self.anchor) > 0.5:
            self.anchor, self.since = p, t
        self.last = t
        return self.anchor is not None and t - self.since >= 3


def parse_label(text, sheet):
    if text == "floor":
        return "floor", 0.0
    number, _, height = text.partition("@")
    n, h = int(number), float(height or 0)
    target_xy(sheet, n)
    if not math.isfinite(h) or h < 0:
        raise ValueError("Height must be finite and nonnegative")
    return n, h


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    plan = []
    levels = []
    if args.watch:
        ids = [int(n) for n in args.watch.split(",")]
        if (
            len(set(ids)) != len(ids)
            or len(ids) < 15
            or not math.isfinite(args.block_mm)
            or args.block_mm <= 0
        ):
            raise ValueError(
                "Supply at least 15 distinct reachable targets and a positive measured block height"
            )
        for n in ids:
            target_xy(args.sheet, n)
        plan = (
            [(n, 0) for n in ids] * 2
            + [(n, args.block_mm) for n in (ids[0], ids[len(ids) // 2], ids[-1])]
            + [("floor", 0)] * 3
        )
    meta = {
        "schema": "viscous-touch-v1",
        "utc": now(),
        "robot": args.robot,
        "sheet": args.sheet,
        "origin_guess_mm": args.origin_mm,
        "floor_z_mm": args.floor_z_mm,
        "sheet_z_mm": args.sheet_z_mm,
    }
    if args.survey_yaw_deg is not None:
        meta["survey_yaw_deg"] = args.survey_yaw_deg
    if args.roll_datum_deg is not None:
        meta["roll_datum_deg"] = args.roll_datum_deg
    if args.level_targets:
        levels = [int(n) for n in args.level_targets.split(",")]
        for n in levels:
            target_xy(args.sheet, n)
        if not plan:
            raise ValueError("--level-targets requires --watch")
        plan.extend([(n, 0) for n in levels])
    with lab(args.robot) as status, args.output.open("x") as out:
        record_line(out, meta)
        print("Manual: enter N, N@height_mm, level:N, floor, or done.", flush=True)
        index, hold = 0, Hold(None)
        while True:
            if plan:
                target, h = plan[index]
                level = index >= len(plan) - len(levels)
                if level:
                    print("LEVEL the jaw rail horizontally at this target.", flush=True)
                print(
                    f"{index + 1}/{len(plan)}  target {target}, height {h:g} mm. Move then hold.", flush=True
                )
                while True:
                    state = status()
                    p = fresh(state)
                    if hold.update(p, state["still"], time.monotonic()):
                        break
                    time.sleep(0.15)
            else:
                if not select.select([sys.stdin], [], [], 2)[0]:
                    status()
                    continue
                line = sys.stdin.readline()
                if not line or line.strip() == "done":
                    break
                try:
                    label = line.strip()
                    level = label.startswith("level:")
                    target, h = parse_label(label.removeprefix("level:"), args.sheet)
                except ValueError as exc:
                    print(exc)
                    continue
                state = status()
                p = fresh(state)
                if p is None or not state["still"] or abs(p["gripper"]) > 5:
                    print("Wait for fresh, still readings and fully closed jaws; enter the target again.")
                    continue
            record_line(
                out, {"target": target, "height_mm": h, "jaw_level": level, "motor_deg": p, "utc": now()}
            )
            print(f"Saved {target} at {h:g} mm.\a", flush=True)
            if plan:
                index += 1
                if index == len(plan):
                    break
                hold = Hold(p)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--robot", default="viscous_waterproof_01")
    parser.add_argument("--sheet", choices=GRID, required=True)
    parser.add_argument("--origin-mm", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--floor-z-mm", type=float, required=True)
    parser.add_argument("--sheet-z-mm", type=float, required=True)
    parser.add_argument(
        "--survey-yaw-deg", type=float, help="independently measured signed base_link heading of sheet +X"
    )
    parser.add_argument(
        "--roll-datum-deg", type=float, help="independently measured additive J6 zero correction"
    )
    parser.add_argument(
        "--watch", metavar="N,N,...", help="ordered target IDs, two passes, three block/floor touches"
    )
    parser.add_argument(
        "--level-targets", metavar="N,N,N", help="additional numbered touches with jaw rail level"
    )
    parser.add_argument("--block-mm", type=float, default=0)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
