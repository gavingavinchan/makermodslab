#!/usr/bin/env python3
"""Fit the Viscous gravity model to holds recorded by viscous-gravity-collect.py.

    viscous-gravity-fit.py configs/viscous/viscous_01/evidence/gravity-air-*.json

Prints, per joint: whether the RS00 torque telemetry agrees with the
stiffness x error estimate and with the CAD model (sign and scale), how well
the CAD and the fitted model predict the holds (in-sample and held-out by
pose), the friction the two approach directions reveal, and the fitted body
masses against CAD. Writes the fit next to the first data file.
"""

import argparse
import datetime
import json
import math
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
os.environ.setdefault("MAKERMODSLAB_HOME", str(Path.home() / ".makermods/makermodslab-viscous"))
sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402

from makermodslab.arm_gravity import (  # noqa: E402
    VISCOUS_URDF,
    GravityModel,
    Sample,
    cross_validate,
    fit_gravity,
    stored_path,
)

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
LOADED = ("shoulder_lift", "elbow_flex", "wrist_flex")


def stiffness_torque(sample, gains):
    """kp * (command - position): what MIT position control applies at rest."""
    return {k: gains[k][0] * math.radians(sample["command_deg"][k] - sample["position_deg"][k]) for k in ARM}


def line(x, y):
    x, y = np.asarray(x), np.asarray(y)
    slope = float(np.dot(x, y) / np.dot(x, x)) if np.dot(x, x) > 0 else float("nan")
    r = float(np.corrcoef(x, y)[0, 1]) if np.std(x) > 0 and np.std(y) > 0 else float("nan")
    return slope, r


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("data", nargs="+", type=Path)
    p.add_argument(
        "--urdf",
        type=Path,
        default=VISCOUS_URDF,
        help="model to fit; must be the file the runs were recorded against (checked by hash)",
    )
    p.add_argument("--source", choices=("telemetry", "stiffness"), default="telemetry")
    p.add_argument("--regularization", type=float, default=1e-2)
    p.add_argument("--out", type=Path)
    args = p.parse_args()

    runs = [json.loads(path.read_text()) for path in args.data]
    urdfs = {run["urdf_sha256"] for run in runs}
    if len(urdfs) != 1 or len({run["condition"] for run in runs}) != 1:
        raise SystemExit("All runs must share one URDF and one condition")
    model = GravityModel(args.urdf, ARM)
    if model.urdf_sha256 not in urdfs:
        raise SystemExit(f"{args.urdf} is not the URDF these runs were recorded against")

    rows = []
    for index, run in enumerate(runs):
        for s in run["samples"]:
            rows.append((index * 1000 + s["pose"], s, stiffness_torque(s, run["gains"])))
    if len({group for group, _, _ in rows}) < 4:
        raise SystemExit("Too few poses to fit")
    print(f"{len(rows)} holds from {len(runs)} run(s), condition {runs[0]['condition']}\n")

    print("Torque sensing check (slope ~1 and r ~1 mean they agree):")
    print(f"{'joint':14s} {'telemetry vs kp*err':>22s} {'telemetry vs CAD':>20s} {'kp*err vs CAD':>18s}")
    for k in LOADED:
        tel = [s["torque_nm"][k] for _, s, _ in rows]
        stiff = [st[k] for _, _, st in rows]
        cad = [model.torque(s["position_deg"])[k] for _, s, _ in rows]
        a, b, c = line(stiff, tel), line(cad, tel), line(cad, stiff)
        print(
            f"{k:14s} slope {a[0]:5.2f} r {a[1]:5.2f}     slope {b[0]:5.2f} r {b[1]:5.2f}"
            f"   slope {c[0]:5.2f} r {c[1]:5.2f}"
        )

    samples = [
        Sample(s["position_deg"], s["torque_nm"] if args.source == "telemetry" else st, group=group)
        for group, s, st in rows
    ]
    fit = fit_gravity(model, samples, regularization=args.regularization)
    held_out = cross_validate(model, samples, regularization=args.regularization)
    cad_out = {}
    for k in ARM:
        err = [s.torques_nm[k] - model.torque(s.positions_deg, model.prior)[k] for s in samples]
        cad_out[k] = float(np.sqrt(np.mean(np.square(err))))

    print(f"\nPrediction error, RMS N m (fit on {args.source}):")
    print(f"{'joint':14s} {'CAD':>7s} {'fit':>7s} {'fit held-out':>13s} {'offset':>8s}")
    for i, k in enumerate(ARM):
        print(f"{k:14s} {cad_out[k]:7.3f} {fit.rms_nm[k]:7.3f} {held_out[k]:13.3f} {fit.offsets[i]:+8.3f}")

    pairs = {}
    for group, s, st in rows:
        source = s["torque_nm"] if args.source == "telemetry" else st
        pairs.setdefault(group, {})[s["approach"]] = source
    friction = {
        k: [abs(v["above"][k] - v["below"][k]) / 2 for v in pairs.values() if len(v) == 2] for k in LOADED
    }
    print("\nFriction (half the above/below difference), mean N m:")
    for k in LOADED:
        print(f"{k:14s} {np.mean(friction[k]) if friction[k] else float('nan'):6.3f}")

    print("\nBodies (mass kg; centre of mass in joint frame, mm):")
    for row in model.body_summary(fit.params):
        com = row["com_m"]
        com_text = " ".join(f"{1000 * v:7.1f}" for v in com) if com else "   n/a"
        cad_com = " ".join(f"{1000 * v:7.1f}" for v in row["cad_com_m"])
        print(
            f"{row['joint']:14s} fit {row['mass_kg']:6.3f} [{com_text}]   CAD {row['cad_mass_kg']:6.3f} [{cad_com}]"
        )

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = args.out or args.data[0].with_name(f"gravity-{runs[0]['condition']}-fit-{stamp}.json")
    out.write_text(
        json.dumps(
            {
                "robot": runs[0]["robot"],
                "condition": runs[0]["condition"],
                "fitted": stamp,
                "source": args.source,
                "regularization": args.regularization,
                "runs": [stored_path(path) for path in args.data],
                "urdf": stored_path(args.urdf),
                "urdf_sha256": model.urdf_sha256,
                "joints": list(ARM),
                "params": fit.params.tolist(),
                "offsets_nm": fit.offsets.tolist(),
                "rms_nm": fit.rms_nm,
                "held_out_rms_nm": held_out,
                "cad_rms_nm": cad_out,
            },
            indent=1,
        )
    )
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
