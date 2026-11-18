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
"""Reproducible known-truth sheet contacts, made by inverse FK with real profile limits."""

import argparse
import json
from pathlib import Path

import numpy as np
from fit import Problem, Session, rms
from kinematics import JOINTS, ROOT, Arm, read
from scipy.optimize import least_squares
from targets import target_xy

IDS = [1, 29, 5, 25, 15, 9, 21, 7, 23, 3, 27, 13, 17, 11, 19, 2, 28, 20, 10, 16, 14, 22, 8, 4, 26]
TRUTH = np.array([3, -2.5, 3, -2, 2, 2.5, 8, 0, 6, 265, -65, 8, 0, 0], dtype=float)


def solve_contact(arm, motor, signs, adjustable, bounds, desired, level, level_noise):
    initial = (np.array([0, -110, 110, 0, 0, 0]) * arm.sign / signs)[adjustable]

    def evaluate(q, derivative):
        motor[adjustable] = q
        tip, jac = arm.tip(motor[None, :], TRUTH, signs)
        value = jac[0][:, adjustable] * signs[adjustable] if derivative else tip[0] - desired
        if level:
            axis, j = arm.tip(motor[None, :], TRUTH, signs, direction=arm.jaw * 60)
            extra = j[0, 2, adjustable] * signs[adjustable] if derivative else axis[0, 2] - level_noise
            value = np.vstack((value, extra)) if derivative else np.append(value, extra)
        return value

    # A reversed encoder reflects the injected unit's numerical limits too.
    limits = np.sort(bounds[adjustable] * (signs[adjustable] / arm.sign[adjustable])[:, None], axis=1)
    result = least_squares(
        evaluate, initial, args=(False,), jac=lambda q, _: evaluate(q, True), bounds=limits.T, max_nfev=50
    )
    motor[adjustable] = result.x
    return result


def generate(seed=18, n=25, noise=1.5, reverse=()):
    arm, rng = Arm(), np.random.default_rng(seed)
    signs = arm.sign.copy()
    signs[list(reverse)] *= -1
    profile = read(ROOT / "configs/viscous/viscous_waterproof_01/evidence/lab-hardware-profile.json")
    bounds = np.array([profile["follower"]["joint_limits"][j] for j in JOINTS])
    meta = {
        "schema": "viscous-touch-v1",
        "sheet": "A4",
        "floor_z_mm": -10,
        "sheet_z_mm": -9,
        "origin_guess_mm": [262, -61],
        "survey_yaw_deg": 8,
        "robot": "synthetic",
    }
    rows = []
    c, s = np.cos(np.deg2rad(8)), np.sin(np.deg2rad(8))
    for n_target in IDS[:n]:
        x, y = target_xy("A4", n_target)
        for pass_no in range(2 + 2 * (n_target in IDS[:3])):
            h = 35 if pass_no == 2 else 0
            level = pass_no == 3
            adjustable = [0, 1, 2, 5] if level else [0, 1, 2]
            level_noise = rng.normal(0, 0.3) if level and noise else 0
            xw, yw = np.array([[c, -s], [s, c]]) @ [x, y] + TRUTH[9:11]
            desired = np.array([xw, yw, meta["sheet_z_mm"] + h]) + rng.normal(0, noise, 3)
            for _ in range(160):
                motor = rng.uniform(bounds[:, 0], bounds[:, 1])
                # Choose distal angles freely, then solve three positioning joints.
                motor[3] = rng.uniform(-115, 75)
                result = solve_contact(arm, motor, signs, adjustable, bounds, desired, level, level_noise)
                if np.linalg.norm(result.fun) < 0.01 and (
                    pass_no != 1 or max(abs(motor[i] - rows[-1]["motor_deg"][JOINTS[i]]) for i in (2, 3)) > 12
                ):
                    break
            else:
                raise RuntimeError(f"Could not generate target {n_target}")
            rows.append(
                {
                    "target": n_target,
                    "height_mm": h,
                    "jaw_level": level,
                    "motor_deg": dict(zip(JOINTS, motor.tolist(), strict=True)) | {"gripper": 0},
                    "utc": "2026-11-18T00:00:00+00:00",
                }
            )
    # Pure floor points have no XY observations. Solve their measured z with a small lift adjustment.
    for i in (0, 3, 6):
        row = rows[i].copy()
        m = np.array([row["motor_deg"][j] for j in JOINTS])
        z = meta["floor_z_mm"] + rng.normal(0, noise)

        def vertical(q, m=m, z=z):
            m[1] = q[0]
            return arm.tip(m[None, :], TRUTH, signs)[0][0, 2] - z

        result = least_squares(vertical, [m[1]], bounds=sorted(bounds[1] * (signs[1] / arm.sign[1])))
        if abs(result.fun[0]) > 0.01:
            raise RuntimeError("Unreachable simulated floor")
        m[1] = result.x[0]
        rows.append(
            row
            | {
                "target": "floor",
                "jaw_level": False,
                "height_mm": 0,
                "motor_deg": dict(zip(JOINTS, m.tolist(), strict=True)) | {"gripper": 0},
            }
        )
    return meta, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, help="write a new synthetic session for fit.py")
    parser.add_argument("--seeds", type=int, default=5)
    args = parser.parse_args()
    trials = {n: [] for n in (5, 10, 15, 20, 25)}
    for seed in range(18, 18 + args.seeds):
        meta, rows = generate(seed)
        if seed == 18 and args.session:
            with args.session.open("x") as file:
                for row in [meta, *rows]:
                    print(json.dumps(row), file=file)
        for n, stats in trials.items():
            data = Session(meta, [r for r in rows if r["target"] in IDS[:n] or r["target"] == "floor"])
            arm = Arm()
            problem = Problem(arm, data, arm.sign)
            p, err, _ = problem.solve(np.ones(len(data.rows), bool))
            held = []
            for label in set(data.labels):
                mask = data.labels == label
                _, e, _ = problem.solve(~mask)
                held.extend(e[mask])
            diff = p - TRUTH
            stats.append(
                [
                    len(err),
                    max(abs(diff[:6])),
                    np.linalg.norm(diff[6:9]),
                    np.linalg.norm(diff[9:11]),
                    rms(held),
                ]
            )
    print("targets touches max_J1-J6_error_deg TCP_error_mm sheet_XY_error_mm heldout_RMS_mm (seed medians)")
    for n, stats in trials.items():
        print(n, *np.median(stats, axis=0).round(3))


if __name__ == "__main__":
    main()
