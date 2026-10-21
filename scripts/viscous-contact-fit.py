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
"""Identify and cross-validate contact friction from saved contact-check runs ONLY.

    .venv/bin/python scripts/viscous-contact-fit.py --fit <v2.json> --out <v2.json> <complete1.json> <complete2.json> [aborted.json]

Gravity is fixed to the supplied all-hold fit in the friction run folds. No push
or hold ticks enter friction identification. The optional aborted run enters
only the final fit; it never enters the two complete-run validation folds.
"""

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402
from viscous_contact_analysis import (  # noqa: E402
    ACCEL_LIMIT_DEG_S2,
    ARM,
    SMOOTH_SPEEDS,
    TRANSIENT_S,
    compensate,
    contacts,
    fit_friction,
    motion_arrays,
)

from makermodslab.arm_gravity import stored_path  # noqa: E402
from makermodslab.contact_torque import ContactEstimator  # noqa: E402


def stats(values):
    a = np.abs(values)
    return np.sqrt(np.mean(values**2)), *np.percentile(a, [95, 99])


def evaluate(data, old, new, friction):
    phases, col, velocity, motion = motion_arrays(data)
    current, gravity = contacts(data, old), contacts(data, new)
    corrected = compensate(gravity, velocity, friction)
    variants = (current, gravity, corrected)
    print(f"\n=== Evaluate {data['started']} ({motion.sum()} selected free-motion ticks) ===")
    print("Free motion, N m: each triple = rms / p95 / p99 (absolute percentiles)")
    print(f"{'joint':>14} {'current':>22} {'v2 gravity':>22} {'v2 + friction':>22}")
    for k in ARM:
        print(
            f"{k:>14}" + "".join("  " + "/".join(f"{s:6.3f}" for s in stats(c[k][motion])) for c in variants)
        )
    raw_motion = np.array([p.startswith(("move:", "approach:")) for p in phases])
    away = (col("shoulder_lift.pos") < -15) & (col("elbow_flex.pos") > 15)
    print(
        f"\nMotion selection audit: {raw_motion.sum()} move/approach ticks, {(raw_motion & away).sum()} away from fold, {motion.sum()} after transient cuts."
    )
    print("All away-from-fold motion INCLUDING transients (N m; rms / p95 / p99):")
    for k in ARM:
        print(
            f"{k:>14}"
            + "".join("  " + "/".join(f"{s:6.3f}" for s in stats(c[k][raw_motion & away])) for c in variants)
        )
    # Holds only, not settling transients or hands-on push data.
    still = np.array([p.startswith("hold:") for p in phases])
    print("\nStill-hold contact, N m: each pair = mean / rms (hold:* only)")
    print(f"{'joint':>14} {'current':>17} {'v2 gravity':>17} {'v2 + friction':>17}")
    for k in ARM:
        print(
            f"{k:>14}"
            + "".join(
                f"  {np.mean(c[k][still]):+7.3f}/{np.sqrt(np.mean(c[k][still] ** 2)):7.3f}" for c in variants
            )
        )
    print("\nWrist rest bias, N m (hold:* only, current -> v2):")
    for k in ("wrist_yaw", "wrist_roll"):
        print(f"  {k:>13} {np.mean(current[k][still]):+7.3f} -> {np.mean(gravity[k][still]):+7.3f}")
    push = phases == "push"
    print(
        "\nPush amplitude, N m: min / max; amplitude = baseline-relative p01 / p99 (first 2 s baseline mean)"
    )
    print(
        f"{'joint':>14} {'current range; amplitude':>34} {'v2 gravity range; amplitude':>34} {'v2 + friction range; amplitude':>34} {'max friction delta':>19}"
    )
    base = push & (col("t") < col("t")[push][0] + 2.0)
    for k in ARM:
        texts = []
        for c in variants:
            p = c[k][push]
            lo, hi = np.percentile(p - np.mean(c[k][base]), [1, 99])
            texts.append(f"{p.min():+6.3f}/{p.max():+6.3f}; {lo:+6.3f}/{hi:+6.3f}")
        print(
            f"{k:>14}"
            + "".join(f"  {v:>32}" for v in texts)
            + f"  {np.max(np.abs(gravity[k][push] - corrected[k][push])):19.3e}"
        )
    print(
        f"Maximum absolute commanded speed in push: {max(np.max(np.abs(velocity[k][push])) for k in ARM):.3e} deg/s"
    )
    interior = np.flatnonzero(push)[1:-1]
    delta = max(np.max(np.abs(gravity[k][interior] - corrected[k][interior])) for k in ARM)
    print(f"Maximum friction delta excluding push-window boundary ticks: {delta:.3e} N m")


def print_friction(friction, diagnostics):
    print(
        f"{'joint':>14} {'Coulomb Nm':>11} {'viscous Nm/(deg/s)':>19} {'v_s deg/s':>10} {'n (+/-)':>19}  status"
    )
    for k, terms in friction.items():
        d = diagnostics[k]
        print(
            f"{k:>14} {terms['coulomb_nm']:11.6f} {terms['viscous_nm_per_deg_s']:19.6f} "
            f"{terms['v_smooth_deg_s']:10.1f} {d['n']:5d} ({d['positive']:4d}/{d['negative']:4d})  {d['status']}"
        )


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("data", nargs="+", type=Path)
    p.add_argument("--fit", type=Path, required=True)
    p.add_argument("--current", type=Path, default=Path("configs/viscous/viscous_01/gravity-air-fit.json"))
    p.add_argument(
        "--out", type=Path, help="write a fit with friction (input is untouched unless paths are equal)"
    )
    args = p.parse_args()
    datasets = [json.loads(path.read_text()) for path in args.data]
    complete = [d for d in datasets if d["outcome"] == "complete"]
    if len(complete) != 2:
        p.error("Supply exactly two complete runs (and optionally aborted free-motion evidence)")
    old, new = ContactEstimator.from_fit(args.current), ContactEstimator.from_fit(args.fit)
    print("Friction validation by run; fixed v2 gravity uses all four sources' holds.")
    print("Smooth speed is chosen on TRAINING data by leave-one-contiguous-leg-out CV.")
    print("Joints below 2 deg/s use only a local viscous slope; no motion => zero terms.")
    print("No push/hold ticks used for friction fitting. Coefficients constrained nonnegative.")
    print("Selection: move/approach, measured lift < -15, measured elbow > 15 deg;")
    print(
        f"exclude |command acceleration| > {ACCEL_LIMIT_DEG_S2:g} deg/s^2 plus +/-{TRANSIENT_S:g} s, and +/-{TRANSIENT_S:g} s at phase edges."
    )
    for train, test in ((complete[0], complete[1]), (complete[1], complete[0])):
        print(f"\nTRAIN {train['started']} -> TEST {test['started']}")
        friction, diagnostics = fit_friction([train], new)
        print_friction(friction, diagnostics)
        evaluate(test, old, new, friction)
    print("\n=== Final friction fit (all supplied free-motion runs; not a validation score) ===")
    friction, diagnostics = fit_friction(datasets, new)
    print_friction(friction, diagnostics)
    if args.out:
        fit = json.loads(args.fit.read_text())
        fit["friction"] = friction
        fit["friction_identification"] = {
            "runs": [stored_path(path) for path in args.data],
            "velocity": "np.gradient(command_deg, t)",
            "phases": ["move:*", "approach:*"],
            "position_cut_deg": {"shoulder_lift_lt": -15, "elbow_flex_gt": 15},
            "max_acceleration_deg_s2": ACCEL_LIMIT_DEG_S2,
            "transient_exclusion_s": TRANSIENT_S,
            "smooth_speed_candidates_deg_s": SMOOTH_SPEEDS,
            "selection": "leave-one-contiguous-leg-out MSE on active ticks; nonnegative least squares",
            "diagnostics": diagnostics,
        }
        args.out.write_text(json.dumps(fit, indent=1) + "\n")
        print(f"\nWrote {stored_path(args.out)}")


if __name__ == "__main__":
    main()
