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
"""Extract a self-contained, pose-balanced wet holding data set from original logs."""

import argparse
import hashlib
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from hydro import BODIES, JOINTS, ROOT, geo, write_json

REAL = ROOT / "scripts/viscous-dream1/real"
sys.path.insert(0, str(REAL))
from gravity_fit import select_ticks  # noqa: E402


def collect(archive):
    tank = geo.load_model(geo.load_tank())
    sources = {}
    records = []
    paths = {
        p.parent.name: p
        for root in (REAL / "runs", archive)
        for p in sorted(root.glob("*202611*/ticks.jsonl"))
        if p.parent.name.split("2026")[1][:4] in ("1107", "1108", "1114")
    }
    if not any("20261114" in n for n in paths):
        raise ValueError("11-14 tank logs required to refresh the stored wet observations")
    for group, path in sorted(paths.items()):
        buckets = defaultdict(list)
        free = 0
        with path.open() as stream:
            raw = [json.loads(line) for line in stream]
        windows = select_ticks(raw)
        times = np.array([r["t"] for r in raw])
        for row in windows:
            i = np.searchsorted(times, row["t"])
            pos = row["pos"] | {"gripper": raw[i]["pos"]["gripper"]}
            transforms = geo.forward_kinematics(tank, pos)
            points = np.vstack([tank.vertices[n] @ transforms[n][0].T + transforms[n][1] for n in BODIES[2:]])
            lo, hi = points.min(axis=0) * 1000, points.max(axis=0) * 1000
            # The old mount used near +80, sides +210/-224 and far -654 mm.
            near, left, right, far = (106, 227, -207, -616) if "1114T" in group else (80, 210, -224, -654)
            if min(lo[2] + 28, near - hi[0], left - hi[1], lo[1] - right, lo[0] - far) < 15:
                continue
            free += 1
            pose = [pos[n] for n in (*JOINTS, "gripper")]
            torque = [row["torque"][n] for n in JOINTS]
            key = tuple(np.rint(np.array(pose[1:6]) / 3).astype(int))
            buckets[key].append((pose, torque))
        for rows in buckets.values():
            records.append(
                {
                    "run": group,
                    "windows": len(rows),
                    "pos_deg": np.median([p for p, _ in rows], axis=0).tolist(),
                    "torque_nm": np.median([t for _, t in rows], axis=0).tolist(),
                }
            )
        sources[group] = {
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "static_windows": len(windows),
            "free_windows": free,
            "bins": len(buckets),
        }
    write_json(
        "wet_observations.json", {"joint_order": [*JOINTS, "gripper"], "sources": sources, "records": records}
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=Path.home() / "sim-tank-evidence/mac-tank-runs")
    collect(parser.parse_args().archive)
