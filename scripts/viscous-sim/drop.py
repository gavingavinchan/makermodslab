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
"""Faster-than-wall, torque-off drop. Reads an init request on stdin, emits JSON."""

import argparse
import json
import math
import sys
from contextlib import nullcontext
from pathlib import Path

from physics import Tank
from scene import DT


def simulate(req, seconds, drag_scale=1.0, added_scale=1.0, *, telemetry=None):
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("duration must be finite and positive")
    tank = Tank(req, drag_scale=drag_scale, added_scale=added_scale)
    initial = tank.truth() | tank.read()
    tank.release()
    with telemetry.open("x") if telemetry else nullcontext() as stream:
        if stream:
            print(json.dumps(initial, allow_nan=False), file=stream)
        for _ in range(math.ceil(seconds / DT)):
            tank.step()
            if stream and tank.step_count % 25 == 0:
                print(json.dumps(tank.truth() | tank.read(), allow_nan=False), file=stream)
    return {
        "initial": initial,
        "final": tank.truth() | tank.read(),
        "contacts": tank.events,
        "object_contacts": tank.object_events,
        "model": tank.metadata,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seconds", type=float, default=5)
    p.add_argument("--drag-scale", type=float, default=1)
    p.add_argument("--added-scale", type=float, default=1)
    p.add_argument("--telemetry", type=Path, help="new output JSONL for 20 Hz truth and motor data")
    args = p.parse_args()
    print(
        json.dumps(
            simulate(
                json.load(sys.stdin),
                args.seconds,
                args.drag_scale,
                args.added_scale,
                telemetry=args.telemetry,
            ),
            allow_nan=False,
        )
    )
