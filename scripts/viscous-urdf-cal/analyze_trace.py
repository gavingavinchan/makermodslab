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
"""Offline plane fits for the old, unlabelled floor then side-wall sweeps."""

import json

# Prefer this worktree's Lab geometry even when using a sibling editable venv.
import sys

import numpy as np
from kinematics import ROOT, digest
from scipy.optimize import least_squares

sys.path.insert(0, str(ROOT))
from makermodslab import tank_geometry as geo  # noqa: E402

TRACE = ROOT / "configs/viscous/viscous_waterproof_01/evidence/tank-trace-20261108T044404Z.jsonl"


def main():
    tank = geo.load_tank()
    model = geo.load_model(tank)
    records = [json.loads(line) for line in TRACE.read_text().splitlines()]
    labels = [r for r in records if r.get("event") == "capture"]
    print("Lab fit on recorded capture labels:", geo.fit_tank(labels, tank))
    samples = [r for r in records if "event" not in r]
    local = geo.probe_local_points(model)
    print(
        "SHA256",
        digest(TRACE),
        "samples",
        len(samples),
        "flange fingertip mm",
        np.mean(list(local.values()), axis=0) * 1000,
    )
    bounds, tips, pad, time = [], [], [], []
    for row in samples:
        frames = geo.forward_kinematics(model, row["positions_deg"])
        vertices = np.vstack([model.vertices[j] @ frames[j][0].T + frames[j][1] for j in local]) * 1000
        bounds.append([vertices[:, 2].min(), vertices[:, 1].max(), vertices[:, 1].min()])
        tips.append(geo.probe_point(model, frames))
        pad.append(frames["tool0"][1] * 1000)
        time.append(row["t"])
    bounds, tips, pad, time = map(np.asarray, (bounds, tips, pad, time))
    print("time bin / median finger min-z, max-y, min-y (mm)")
    for t in range(0, 60, 3):
        use = (time >= t) & (time < t + 3)
        print(t, np.round(np.median(bounds[use], axis=0), 1))
    # Fixed time windows from operator-described order and visible plateaus.
    # Fit each signed plane offset independently, assuming axis alignment over that pass.
    for face, (lo, hi), axis in [
        ("floor-low plateau", (18.85, 19.6), 0),
        ("+y", (27, 42), 1),
        ("-y", (45, 57), 2),
    ]:
        use = (time >= lo) & (time < hi)
        v = bounds[use, axis]
        level = least_squares(
            lambda p, observed: observed - p[0], [np.median(v)], args=(v,), loss="soft_l1", f_scale=1.5
        ).x[0]
        print(
            f"{face} [{lo},{hi}) s, n={len(v)}, plane {level:.2f} mm, RMS {np.mean((v - level) ** 2) ** 0.5:.2f} mm, "
            f"range {v.min():.2f}..{v.max():.2f} mm"
        )
        if axis == 0:
            print("  median z (closed fingertip, pad TCP)", np.median(tips[use, 2]), np.median(pad[use, 2]))
    print("All-sample lowest finger z min/p05:", np.percentile(bounds[:, 0], [0, 5]))


if __name__ == "__main__":
    main()
