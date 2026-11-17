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
"""Offline, zero-prior empirical gravity identification for the waterproof arm.

Accept run folders, ticks.jsonl or samples.json. Raw inputs require dry-air
meta.json; sweep files carry their environment too. Never opens a device or Dream.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import deque
from pathlib import Path

import numpy as np

from makermodslab.arm_gravity import VISCOUS_URDF, GravityModel, stored_path

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
MAP_FILE = VISCOUS_URDF.parent / "motor_to_urdf.json"
DEFAULT_OUT = VISCOUS_URDF.parents[1] / "viscous_waterproof_01/gravity-dry-air-fit.json"
# ViscousFollowerConfig defaults + dream_real.FOLLOWER_GAINS. No robot constructed.
KP = dict(zip(ARM, (60.0, 90.0, 90.0, 30.0, 30.0, 30.0), strict=True))


# The feedforward's measured-pose gate (gravity_feedforward.support_weight): full strength within
# 4 deg of a pose the fit was measured at, zero beyond 12 deg.
SUPPORT_FULL_DEG = 4.0
SUPPORT_FADE_DEG = 8.0


def joint_map():
    return {
        n: {k: v[k] for k in ("sign", "offset_deg")}
        for n, v in json.loads(MAP_FILE.read_text())["joints"].items()
    }


def dry_air(meta):
    env = meta.get("environment")
    if (env.get("name") if isinstance(env, dict) else env) != "dry_air":
        raise ValueError("Input must have dry_air environment metadata")


def median_rows(rows):
    return {
        key: {n: float(np.median([r[key][n] for r in rows])) for n in ARM}
        for key in ("pos", "cmd", "torque", "temp")
    }


def select_ticks(ticks):
    """Nonoverlapping >=0.5 s windows; all joints span <1 deg/s, no gaps/trips.

    Direction is each joint's last commanded motion, retained through holds.
    The entire window is >=1 s after the phase transition. Stop readings are
    excluded: contact/support during return would teach gravity the wrong load.
    """
    window, selected = deque(), []
    phase, since, prev, last_selected = None, 0.0, None, -math.inf
    direction = dict.fromkeys(ARM, 0)
    for row in ticks:
        t = row["t"]
        if row["phase"] != phase:
            phase, since = row["phase"], t
            window.clear()
        cmd = row.get("cmd")
        if cmd and prev and prev.get("cmd"):
            for n in ARM:
                delta = cmd[n] - prev["cmd"][n]
                if abs(delta) > 0.03:
                    direction[n] = int(np.sign(delta))
        if prev and (t <= prev["t"] or t - prev["t"] > 0.15):
            window.clear()
        prev = row
        energized = phase in ("approach", "dream", "hold", "pause") or phase.startswith("sweep:")
        if not energized or not cmd or row.get("trips") or t - since < 1.0:
            window.clear()
            continue
        window.append(row)
        while len(window) > 1 and t - window[1]["t"] >= 0.5:
            window.popleft()
        dt = t - window[0]["t"]
        if dt < 0.5 or t - last_selected < 0.5:
            continue
        if any(np.ptp([r["pos"][n] for r in window]) / dt >= 1.0 for n in ARM):
            continue
        values = median_rows(window)
        if not all(math.isfinite(v) for d in values.values() for v in d.values()):
            continue
        selected.append(values | {"direction": dict(direction), "t": t, "ticks": len(window)})
        last_selected = t
    return selected


def load_inputs(paths):
    rows, sources = [], []
    seen = set()
    for path in paths:
        path = Path(path).expanduser()
        if path.is_dir():
            path = path / ("samples.json" if (path / "samples.json").exists() else "ticks.jsonl")
        path = path.resolve()
        if not path.exists():
            meta = json.loads((path.parent / "meta.json").read_text())
            dry_air(meta)
            sources.append(
                {
                    "run": path.parent.name,
                    "file": path.name,
                    "selected": 0,
                    "kind": "logs",
                    "reason": "no ticks (refused before execution)",
                }
            )
            continue
        if path in seen:
            continue
        seen.add(path)
        if path.name == "ticks.jsonl":
            meta = json.loads((path.parent / "meta.json").read_text())
            dry_air(meta)
            if meta.get("kind") not in ("live", "rehearse-live"):
                raise ValueError("Fake ticks are not empirical arm data")
            selected = select_ticks(json.loads(line) for line in path.read_text().splitlines())
            for r in selected:
                r["group"] = path.parent.name
            kind = "logs"
        else:
            data = json.loads(path.read_text())
            metadata = next(
                (
                    p / "meta.json"
                    for p in (path.parent, *list(path.parents)[:3])
                    if (p / "meta.json").is_file()
                ),
                None,
            )
            if metadata is None:
                raise ValueError("samples.json requires dry-air meta.json")
            dry_air(json.loads(metadata.read_text()))
            dry_air(data)
            if data.get("fake"):
                raise ValueError("Fake holds are not empirical arm data")
            selected = data["samples"]
            for r in selected:
                r["group"] = f"{path.parent.name}:pose:{r['pose']}"
            kind = "sweep"
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        source = {
            "run": path.parent.name,
            "file": path.name,
            "sha256": digest,
            "selected": len(selected),
            "kind": kind,
        }
        sources.append(source)
        for r in selected:
            r["source"] = path.parent.name
        rows.extend(selected)
    if not rows:
        raise ValueError("No quasi-static samples")
    return rows, sources


def design(model, rows, fit_offsets=True, fit_friction=True):
    """Static regressor, then per-joint offset and Coulomb columns. A term that is not fitted keeps
    its columns at zero, so theta always has 24 + 6 + 6 entries and that term comes out exactly 0."""
    n = len(ARM)
    y = np.vstack([model.regressor(r["pos"]) for r in rows])
    direction = np.array([[r["direction"][n] for n in ARM] for r in rows])
    coulomb = np.vstack([np.diag(d) for d in direction]) * float(fit_friction)
    offsets = np.tile(np.eye(n), (len(rows), 1)) * float(fit_offsets)
    return y, np.hstack([y, offsets, coulomb])


def solve(model, rows, regularization=0.01, fit_offsets=True, fit_friction=True):
    y, a = design(model, rows, fit_offsets, fit_friction)
    # SVD of scaled static regressor only: coefficients in the right-singular
    # basis are identifiable combinations, not individual masses or COMs.
    scale = np.tile([0.3, 0.03, 0.03, 0.03], len(ARM))
    _, singular, vt = np.linalg.svd(y * scale, full_matrices=False)
    rank = int(np.sum(singular > singular[0] * 1e-6))
    basis = vt[:rank].T
    reduced = np.hstack([(y * scale) @ basis, a[:, len(scale) :]])
    tau = np.array([[r["torque"][n] for n in ARM] for r in rows]).ravel()
    weights = np.full(reduced.shape[1], math.sqrt(regularization))
    beta = np.linalg.lstsq(
        np.vstack([reduced, np.diag(weights)]), np.r_[tau, np.zeros(len(weights))], rcond=None
    )[0]
    theta = np.r_[scale * (basis @ beta[:rank]), beta[rank:]]
    return theta, {
        "rank": rank,
        "static_columns": len(scale),
        "column_names": [f"{n}.{p}" for n in ARM for p in ("m", "m_cx", "m_cy", "m_cz")],
        "relative_svd_cutoff": 1e-6,
        "singular_values": singular.tolist(),
        "parameter_scales": scale.tolist(),
        "identified_combinations": vt[:rank].tolist(),
        "definition": "rows dotted with theta/static_parameter_scales; unobserved directions set to zero",
        "augmented_rank": int(np.linalg.matrix_rank(reduced, tol=singular[0] * 1e-6)),
    }


def fit_rows(rows, sources, regularization=0.01, fit_offsets=True, fit_friction=True):
    model = GravityModel(VISCOUS_URDF, ARM, joint_map=joint_map())
    terms = {"fit_offsets": fit_offsets, "fit_friction": fit_friction}
    theta, identification = solve(model, rows, regularization, **terms)
    y, a = design(model, rows, **terms)
    measured = np.array([[r["torque"][n] for n in ARM] for r in rows])
    residual = measured - (a @ theta).reshape(-1, len(ARM))
    cad = measured - (y @ model.prior).reshape(-1, len(ARM))
    groups = sorted({r["group"] for r in rows})
    if len(groups) < 2:
        raise ValueError("Cross-validation needs at least two runs/poses")
    held = []
    for group in groups:
        train, test = [r for r in rows if r["group"] != group], [r for r in rows if r["group"] == group]
        beta, _ = solve(model, train, regularization, **terms)
        _, at = design(model, test, **terms)
        held.extend(
            np.array([[r["torque"][n] for n in ARM] for r in test]) - (at @ beta).reshape(-1, len(ARM))
        )

    def rms(a):
        return dict(zip(ARM, np.sqrt(np.mean(np.array(a) ** 2, axis=0)).tolist(), strict=True))

    r2, telemetry = {}, {}
    for i, n in enumerate(ARM):
        denom = np.sum((measured[:, i] - measured[:, i].mean()) ** 2)
        r2[n] = float(1 - np.sum(residual[:, i] ** 2) / denom) if denom > 1e-12 else None
        stiff = np.array([KP[n] * math.radians(r["cmd"][n] - r["pos"][n]) for r in rows])
        tel = measured[:, i]
        telemetry[n] = {
            "kp": KP[n],
            "rms_difference_nm": float(np.sqrt(np.mean((tel - stiff) ** 2))),
            "slope": float(stiff @ tel / (stiff @ stiff)) if stiff @ stiff > 1e-12 else None,
            "correlation": float(np.corrcoef(stiff, tel)[0, 1])
            if min(np.std(stiff), np.std(tel)) > 1e-9
            else None,
        }
    coverage = {n: [min(r["pos"][n] for r in rows), max(r["pos"][n] for r in rows)] for n in ARM}
    return {
        "_copyright": "Copyright 2026 MakerMods. All rights reserved.",
        "_license": json.loads(MAP_FILE.read_text())["_license"],
        "robot": "viscous_waterproof_01",
        "condition": "dry_air",
        "robot_aliases": ["viscous_waterproof_01_dry_air"],
        "region_deg": {n: coverage[n] for n in ARM[1:]},
        "support": {
            "joints": list(ARM[1:]),
            "poses": [[round(r["pos"][n], 2) for n in ARM[1:]] for r in rows],
            "full_deg": SUPPORT_FULL_DEG,
            "fade_deg": SUPPORT_FADE_DEG,
        },
        "fade_deg": 10.0,
        "urdf": stored_path(VISCOUS_URDF),
        "urdf_sha256": model.urdf_sha256,
        "joints": list(ARM),
        "params": theta[:24].tolist(),
        "offsets_nm": theta[24:30].tolist(),
        "joint_map": model.joint_map,
        "friction": dict(zip(ARM, theta[30:].tolist(), strict=True)),
        "rms_nm": rms(residual),
        "held_out_rms_nm": rms(held),
        "cad_rms_nm": rms(cad),
        "r_squared": r2,
        "max_residual_nm": dict(zip(ARM, np.max(abs(residual), axis=0).tolist(), strict=True)),
        "sources": sources,
        "provisional": all(s["kind"] == "logs" for s in sources),
        "regularization": regularization,
        "fitted_terms": {"gravity": True, "offsets": fit_offsets, "friction": fit_friction},
        "prior": "ZERO, no CAD inertial prior",
        "identification": identification,
        "coverage_deg": coverage,
        "telemetry_agreement": telemetry,
        "sample_count": len(rows),
        "cv_groups": len(groups),
        "gripper": "excluded from torques; locked at Pinocchio neutral",
        "validation": "leave-one-run-out for logs; leave-one-pose-out for sweeps",
    }


def report(fit):
    lines = [
        "<!-- Copyright 2026 MakerMods. All rights reserved. Licensed under the Apache License, Version 2.0. -->",
        "# Waterproof arm empirical dry-air gravity fit",
        "",
        f"{fit['sample_count']} static windows/holds, {fit['cv_groups']} CV groups. Provisional: {fit['provisional']}.",
        "Provisional feedforward requires --allow-provisional-fit; environment installer refuses it. Gripper locked at Pinocchio neutral.",
        "Zero-centered ridge; CAD inertials appear only in the reference error. Directions are signed last motor motion.",
        "",
        "| Joint | R² | RMS Nm | Held-out Nm | CAD Nm | Max residual Nm | Coulomb Nm |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for n in ARM:
        values = [
            fit[k][n]
            for k in ("r_squared", "rms_nm", "held_out_rms_nm", "cad_rms_nm", "max_residual_nm", "friction")
        ]
        lines.append(f"| {n} | " + " | ".join("n/a" if v is None else f"{v:.3f}" for v in values) + " |")
    ident = fit["identification"]
    lines += [
        "",
        f"Static rank {ident['rank']}/{ident['static_columns']}; augmented rank {ident['augmented_rank']}.",
        "The JSON records every identified right-singular combination and scale. Individual masses/COMs are not identifiable.",
        "Basis is recomputed using training data only in each CV fold. Coulomb signs/offsets can be confounded in one-direction logs; signed estimates are not physical friction certificates.",
        "",
        "Coverage (motor degrees):",
    ]
    lines += [f"- {n}: {v[0]:.2f} to {v[1]:.2f}" for n, v in fit["coverage_deg"].items()]
    lines += [
        "",
        "Everything outside these ranges is unsampled, especially wrist yaw/roll and combinations of wide shoulder/elbow/flex angles. Narrow logs do not validate extrapolation.",
        "",
        "RS00 telemetry versus kp*(command-position), using Viscous defaults with runner overrides:",
        "",
    ]
    lines += [
        f"- {n}: kp {v['kp']:g}, slope {v['slope']}, r {v['correlation']}, RMS difference {v['rms_difference_nm']:.3f} Nm"
        for n, v in fit["telemetry_agreement"].items()
    ]
    lines += ["", "Sources (hashes and selected counts in JSON):"]
    lines += [f"- {s['run']}/{s['file']}: {s['selected']}" for s in fit["sources"]]
    return "\n".join(lines) + "\n"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("inputs", nargs="+", type=Path)
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    p.add_argument("--regularization", type=float, default=0.01)
    p.add_argument(
        "--no-offsets",
        action="store_true",
        help="pin the per-joint torque offsets to 0: on narrow coverage a constant offset cannot be told "
        "from gravity, and the feedforward commands gravity only",
    )
    p.add_argument(
        "--no-friction", action="store_true", help="pin the Coulomb terms to 0 (one-direction data)"
    )
    p.add_argument("--save-selected", type=Path, help="portable evidence of selected windows")
    args = p.parse_args(argv)
    if not math.isfinite(args.regularization) or args.regularization <= 0:
        p.error("regularization must be positive and finite")
    rows, sources = load_inputs(args.inputs)
    fit = fit_rows(
        rows, sources, args.regularization, fit_offsets=not args.no_offsets, fit_friction=not args.no_friction
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(fit, indent=2, allow_nan=False) + "\n")
    args.out.with_suffix(".md").write_text(report(fit))
    if args.save_selected:
        args.save_selected.write_text(
            json.dumps(
                {
                    "_copyright": fit["_copyright"],
                    "_license": fit["_license"],
                    "sources": sources,
                    "rows": rows,
                },
                indent=1,
            )
            + "\n"
        )
    print(report(fit))


if __name__ == "__main__":
    main()
