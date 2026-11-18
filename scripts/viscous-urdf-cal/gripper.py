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
"""Measure full inside jaw gap through Lab, or fit an existing five-opening JSONL."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from capture import fresh, lab, now, record_line
from fit import save
from kinematics import ROOT, digest

sys.path.insert(0, str(ROOT))
from makermodslab import tank_geometry as geo  # noqa: E402


def capture(path, robot):
    if path.exists():
        raise FileExistsError(path)
    with lab(robot) as status, path.open("x") as file:
        for label in ["closed", "one quarter", "half", "three quarters", "fully open"]:
            while True:
                text = input(f"Hold {label} still. Enter measured inside pad gap in mm: ")
                try:
                    gap = float(text)
                except ValueError:
                    continue
                state = status()
                p = fresh(state)
                if not p or not state["still"] or not math.isfinite(gap) or gap < 0:
                    print("Wait for a fresh still pose and a nonnegative measurement.")
                    continue
                record_line(
                    file, {"motor_deg": p["gripper"], "opening_mm": gap, "utc": now(), "robot": robot}
                )
                break


def opening_mm(curve, motor_deg):
    x, y = np.array(curve, dtype=float).T
    if not x[0] <= motor_deg <= x[-1]:
        raise ValueError("Outside measured J7 range")
    return float(np.interp(motor_deg, x, y))


def fit(path, out):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    curve = sorted((r["motor_deg"], r["opening_mm"]) for r in rows)
    if len(curve) < 5 or not np.isfinite(curve).all():
        raise ValueError("Need five finite caliper measurements")
    x, y = np.array(curve, dtype=float).T
    if np.any(np.diff(x) <= 0) or np.any(np.diff(y) >= 0) or np.min(y) < 0:
        raise ValueError("Check measurements: opening should increase as motor degrees decrease")
    slope, offset = np.polyfit(x, y, 1)
    model = geo.load_model(geo.load_tank())
    pos = dict.fromkeys(geo.JOINTS, 0.0)
    lab_mm = [geo.gripper_opening_mm(model, pos | {"gripper": float(v)}) for v in x]
    delta = y - (slope * x + offset)
    save(
        out,
        {
            "status": "pending Gavin's approval",
            "generated_utc": now(),
            "input_sha256": digest(path),
            "tank_sha256": digest(geo.DEFAULT_TANK),
            "linear_mm_per_deg": slope,
            "intercept_mm": offset,
            "linear_residuals_mm": delta.tolist(),
            "linear_rms_mm": float(np.mean(delta**2) ** 0.5),
            "piecewise_linear_deg_full_gap_mm": curve,
            "domain_deg": [x[0], x[-1]],
            "lab_one_finger_slide_mm": lab_mm,
            "full_gap_minus_twice_lab_slide_mm": (y - 2 * np.array(lab_mm)).tolist(),
            "measured_closed_gap_mm": y[-1],
            "interior_interpolation_holdout_mm": [
                y[i] - opening_mm(curve[:i] + curve[i + 1 :], x[i]) for i in range(1, len(x) - 1)
            ],
        },
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    get = sub.add_parser("capture")
    get.add_argument("samples", type=Path)
    get.add_argument("--robot", default="viscous_waterproof_01")
    solve = sub.add_parser("fit")
    solve.add_argument("samples", type=Path)
    solve.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.action == "capture":
        capture(args.samples, args.robot)
    else:
        fit(args.samples, args.output)
