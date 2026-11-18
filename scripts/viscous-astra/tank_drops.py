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
"""Torque-off first-floor impacts from rest across the sampled tank workspace.

Run with the dreamtrial Python and OPENBLAS_NUM_THREADS=1. Each drop starts
in still water with zero velocity and no actuator torque. Report impact speed
before the solver applies the contact impulse. A sampled maximum is a lower
bound on the continuous-workspace maximum for this particular model.
"""

import argparse
import hashlib
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from itertools import product
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SCRIPTS / "viscous-sim"), str(SCRIPTS / "viscous-buoyancy")]
from build import ORDER, geo, workspace  # noqa: E402
from physics import Tank  # noqa: E402
from scene import DT, MODEL, MOTORS  # noqa: E402

ARM_FILE = SCRIPTS / "viscous-dream1/real/runs/live-20261108T205156Z/arm/viscous-waterproof-tank.json"


def drop(case):
    idx, variant, start, drag, added, displaced, settle = case
    path = MODEL / "buoyancy" / (variant + ".json")
    raw = path.read_bytes()
    side = json.loads(raw)
    for link in side["links"].values():
        link["volume_m3"] *= displaced
    tank = Tank(
        {
            "arm_file": str(ARM_FILE),
            "task": "cube",
            "object_xy": [-0.55, 0.16],
            "variant": variant,
            "start": start,
        },
        side,
        drag_scale=drag,
        added_scale=added,
    )
    height = tank.clearance["floor"]
    tip = tank.data.site_xpos[tank.tipid].copy()
    for _ in range(round((8 if settle else 4) / DT)):
        tank.step()
        if tank.first_floor and not settle:
            break
    truth = tank.truth()
    return {
        "index": idx,
        "variant": variant,
        "motor_deg": start,
        "drag_scale": drag,
        "added_scale": added,
        "displacement_scale": displaced,
        "settle": settle,
        "start_floor_clearance_m": height,
        "first_floor": tank.first_floor,
        "end_floor_clearance_m": truth["clearance_m"]["floor"],
        "tip_displacement_m": (np.array(truth["tip"]) - tip).tolist(),
        "end_velocity_rad_s": tank.data.qvel[:6].tolist(),
        "end_motor_deg": tank.pos(),
        "t_s": truth["t_s"],
        "first_wall_or_post": next((e for e in tank.events if e["surface"] != "glass_floor"), None),
        "sidecar_sha256": hashlib.sha256(raw).hexdigest(),
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.add_argument("--jobs", type=int, default=4)
    p.add_argument("--exponent", type=int, default=16, choices=range(10, 18))
    args = p.parse_args()
    if set(ORDER) != set(MOTORS):
        raise ValueError("workspace and simulator motor names differ")
    poses, _ = workspace(geo.load_model(geo.load_tank()), exponent=args.exponent)
    starts = [dict(zip(ORDER, q.tolist(), strict=True)) for q in poses]
    nominal = [(i, v, q, 1.0, 1.0, 1.0, False) for i, q in enumerate(starts) for v in ("current", "foamed")]
    rows = []
    with args.output.open("x", buffering=1) as f, ProcessPoolExecutor(args.jobs) as pool:
        for row in pool.map(drop, nominal, chunksize=4):
            print(json.dumps(row, allow_nan=False), file=f)
            if row["first_floor"]:
                rows.append(row)
        # Re-evaluate the most severe poses for either arm with hydrodynamic
        # and displacement perturbations. These are scenarios, not calibrated CIs.
        severe = set()
        for v in ("current", "foamed"):
            ranked = sorted(
                (r for r in rows if r["variant"] == v),
                key=lambda r: r["first_floor"]["normal_m_s"],
                reverse=True,
            )
            severe.update(r["index"] for r in ranked[:4])
        for row in pool.map(
            drop,
            (
                (i, v, starts[i], d, a, b, False)
                for i, v, d, a, b in product(
                    sorted(severe), ("current", "foamed"), (0.5, 2.0), (0.5, 2.0), (0.85, 1.15)
                )
            ),
        ):
            print(json.dumps(row, allow_nan=False), file=f)
        for row in pool.map(
            drop,
            (
                (i, v, starts[i], 1.0, 1.0, 1.0, True)
                for i, v in product(sorted(severe), ("current", "foamed"))
            ),
        ):
            print(json.dumps(row, allow_nan=False), file=f)


if __name__ == "__main__":
    main()
