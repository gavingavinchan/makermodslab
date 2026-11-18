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
"""Replay archived wet rehearsal commands in articulated physics, with optional sustained end hold."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from physics import Tank
from scene import DT


def replay(run, end_s, comparison_s, variant):
    with (run / "ticks.jsonl").open() as f:
        rows = [row for line in f if (row := json.loads(line)).get("cmd") and row.get("torque")]
    t0 = rows[0]["t"]
    end_s = rows[-1]["t"] - t0 if end_s is None else end_s
    if not math.isfinite(end_s) or end_s <= 0:
        raise ValueError("end time must be finite and positive")
    arm_file = run / "arm/viscous-waterproof-tank.json"
    tank = Tank(
        {
            "variant": variant,
            "task": "cube",
            "object_xy": [-0.55, 0.16],
            "arm_file": str(arm_file),
            "start": rows[0]["pos"],
        }
    )
    tank.engage()
    tank.goal(rows[0]["cmd"])
    i = 0
    load = []
    first_trip = None
    for tick in range(math.ceil(end_s / DT)):
        elapsed = tick * DT
        while i + 1 < len(rows) and rows[i + 1]["t"] - t0 <= elapsed:
            i += 1
            tank.goal(rows[i]["cmd"])
        tank.step()
        if comparison_s[0] <= elapsed < comparison_s[1] and elapsed <= rows[-1]["t"] - t0:
            load.append((rows[i]["torque"]["shoulder_lift"], tank.data.actuator_force[1]))
        if first_trip is None and tank.thermal.limp[1]:
            first_trip = tank.data.time
    if not load:
        raise ValueError("comparison window contains no recorded samples")
    measured, simulated = np.mean(load, axis=0)
    return {
        "rehearsal": run.name,
        "variant": variant,
        "command_evidence_s": rows[-1]["t"] - t0,
        "total_sim_s": tank.data.time,
        "comparison_s": comparison_s,
        "measured_mean_j2_nm": measured,
        "simulated_mean_j2_nm": simulated,
        "simulated_j2_trip_s": first_trip,
        "final": tank.truth(),
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("--variant", choices=("current", "foamed"), default="current")
    p.add_argument("--end-s", type=float, help="extend the final command to this elapsed time")
    p.add_argument("--comparison-s", type=float, nargs=2, default=[100, 115], metavar=("START", "END"))
    args = p.parse_args()
    print(json.dumps(replay(args.run, args.end_s, args.comparison_s, args.variant), allow_nan=False))
