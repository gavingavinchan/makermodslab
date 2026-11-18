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
"""Fit the waterproof arm from a torque-off sheet session. Outputs are exclusive creates."""

import argparse
import hashlib
import itertools
import json
import shlex
import sys
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
from kinematics import ARM_FILE, JOINTS, MAP, PARAMS, URDF, Arm, digest, read
from scipy.optimize import least_squares
from targets import target_xy


def save(path, data):
    encoded = json.dumps(data, indent=2, allow_nan=False) + "\n"
    with path.open("x") as f:
        f.write(encoded)


class Session:
    def __init__(self, meta, rows):
        self.meta, self.rows = meta, rows
        self.floor = np.array([r["target"] == "floor" for r in rows], dtype=bool)
        self.labels = np.array([str(r["target"]) for r in rows])
        self.motors = np.array([[r["motor_deg"][j] for j in JOINTS] for r in rows], dtype=float)
        self.level = np.array([r.get("jaw_level", False) for r in rows], dtype=bool)
        self.xyz = []
        for r in rows:
            if set(r["motor_deg"]) != {*JOINTS, "gripper"} or abs(r["motor_deg"]["gripper"]) > 5:
                raise ValueError(
                    "Need all seven follower angles and jaws physically closed (J7 within 5 deg)"
                )
            height = float(r["height_mm"])
            if height < 0 or (r["target"] == "floor" and height != 0):
                raise ValueError("Invalid contact height")
            if r["target"] == "floor":
                self.xyz.append([0, 0, meta["floor_z_mm"]])
            else:
                x, y = target_xy(meta["sheet"], r["target"])
                self.xyz.append([x, y, meta["sheet_z_mm"] + height])
        self.xyz = np.array(self.xyz, dtype=float)
        if not rows or not all(
            np.isfinite(v).all()
            for v in (
                self.xyz,
                self.motors,
                meta["origin_guess_mm"],
                [r["motor_deg"]["gripper"] for r in rows],
            )
        ):
            raise ValueError("Empty session or nonfinite measurement")

    def coverage(self):
        reasons = []
        plane = {r["target"] for r in self.rows if r["target"] != "floor" and r["height_mm"] == 0}
        if len(plane) < 15:
            reasons.append("Need at least 15 numbered targets at sheet height")
        for label in sorted(plane):
            m = self.motors[[r["target"] == label and r["height_mm"] == 0 for r in self.rows]]
            if np.ptp(m[:, [2, 3]], axis=0).max() < 8:
                reasons.append(f"Target {label} needs another elbow/wrist pitch (at least 8 deg change)")
        if len({r["target"] for r in self.rows if r["height_mm"] > 0}) < 3 or self.floor.sum() < 3:
            reasons.append("Need three distinct block targets and three pure floor touches")
        return reasons


class Problem:
    def __init__(self, arm, data, sign, links=False):
        self.arm, self.data, self.sign = arm, data, sign
        # With free sheet yaw fix J1. With an independently surveyed yaw fit J1.
        self.free = [1, 2, 3, 4, 6, 7, 8, 9, 10]
        self.free.append(11 if data.meta.get("survey_yaw_deg") is None else 0)
        if data.level.any() and "roll_datum_deg" not in data.meta:
            self.free.append(5)
        if links:
            self.free.extend([12, 13])
        self.initial = np.zeros(len(PARAMS))
        self.initial[5] = float(data.meta.get("roll_datum_deg", 0))
        self.initial[9:11] = data.meta["origin_guess_mm"]
        if not np.isfinite(self.initial).all():
            raise ValueError("Invalid independent datum")
        if data.meta.get("survey_yaw_deg") is not None:
            self.initial[11] = float(data.meta["survey_yaw_deg"])

    def residual(self, p):
        tip, jac = self.arm.tip(self.data.motors, p, self.sign)
        yaw = np.deg2rad(p[11])
        c, s = np.cos(yaw), np.sin(yaw)
        # Express errors in sheet coordinates, so rotating J1, sheet yaw and
        # sheet translation together is an exact gauge even with noisy data.
        base_to_sheet = np.array([[c, s], [-s, c]])
        local = (tip[:, :2] - p[9:11]) @ base_to_sheet.T
        residual = tip - self.data.xyz
        residual[:, :2] = local - self.data.xyz[:, :2]
        jac[:, 0, 9] = jac[:, 1, 10] = -1
        jac[:, :2, :] = base_to_sheet @ jac[:, :2, :]
        jac[:, 0, 11] = local[:, 1] * np.pi / 180
        jac[:, 1, 11] = -local[:, 0] * np.pi / 180
        return residual, jac

    def solve(self, subset, sheet_only=False):
        free = [i for i in self.free if 9 <= i <= 11] if sheet_only else self.free
        point = self.initial.copy()
        if sheet_only:
            point[5] = 0
        use = subset & ~self.data.floor
        if use.sum() < 2:
            raise ValueError("Need multiple numbered touches")
        # Register using training rows only, including for held-out-target folds.
        xy = self.data.xyz[use, :2]
        tip = self.arm.tip(self.data.motors[use], point, self.sign)[0][:, :2]
        a, b = xy - xy.mean(axis=0), tip - tip.mean(axis=0)
        if 11 in free:
            point[11] = np.rad2deg(np.arctan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a * b).sum()))
        yaw = np.deg2rad(point[11])
        c, s = np.cos(yaw), np.sin(yaw)
        point[9:11] = tip.mean(axis=0) - xy.mean(axis=0) @ np.array([[c, s], [-s, c]])
        mask = np.broadcast_to(subset[:, None], (len(subset), 3)).copy()
        mask[self.data.floor, :2] = False
        # Smooth robust loss in mm. Rotational corrections small about the source zero.
        limits = np.array([20] * 6 + [65] * 3 + [np.inf] * 3 + [5] * 2)[free]

        def evaluate(v, derivative):
            point[free] = v
            r, j = self.residual(point)
            values = j[mask][:, free] if derivative else r[mask]
            level = subset & self.data.level
            if level.any():
                height, dj = self.arm.tip(
                    self.data.motors[level], point, self.sign, direction=60 * self.arm.jaw
                )
                extra = dj[:, 2, free] if derivative else height[:, 2]
                values = np.concatenate((values, extra))
            return values

        result = least_squares(
            evaluate,
            point[free],
            jac=lambda v, _: evaluate(v, True),
            args=(False,),
            bounds=(-limits, limits),
            loss="soft_l1",
            f_scale=1.5,
            x_scale="jac",
        )
        point[free] = result.x
        r, jac = self.residual(point)
        norms = np.linalg.norm(r, axis=1)
        norms[self.data.floor] = np.abs(r[self.data.floor, 2])
        # Robust solver rescales its Jacobian. Use physical, unweighted mm per degree/mm here.
        values = np.linalg.svd(evaluate(result.x, True), compute_uv=False)
        full = np.linalg.svd(jac[mask][:, : 14 if 12 in free else 12], compute_uv=False)
        weak = len(values) < len(free) or values[-1] < 0.5
        problems = []
        if weak:
            problems.append("Weak Jacobian: least constrained direction below 0.5 mm per degree/mm")
        if not result.success or result.active_mask.any():
            problems.append("Solver failed or a correction reached its search bound")
        return (
            point,
            norms,
            {
                "singular_values": values.tolist(),
                "touch_only_free_all_singular_values": full.tolist(),
                "columns": [PARAMS[i] for i in free],
                "issues": problems,
            },
        )


def rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def analyze(data, links=False):
    arm = Arm()
    all_rows = np.ones(len(data.rows), dtype=bool)
    problem = Problem(arm, data, arm.sign, links)
    p, after, ident = problem.solve(all_rows)
    nominal, before, _ = problem.solve(all_rows, sheet_only=True)
    signed_before = problem.residual(nominal)[0]
    signed_after = problem.residual(p)[0]
    issues = data.coverage() + ident["issues"]
    # Score all joint-sign combinations supported only by a photograph, holding documented signs fixed in outputs.
    suspect = [i for i, j in enumerate(JOINTS) if "photo-consistent" in read(MAP)["joints"][j]["status"]]
    signs = []
    for count in range(1, len(suspect) + 1):
        for flipped in itertools.combinations(suspect, count):
            trial = arm.sign.copy()
            trial[list(flipped)] *= -1
            _, errors, _ = Problem(arm, data, trial, links).solve(all_rows)
            signs.append({"reversed": [JOINTS[i] for i in flipped], "rms_mm": rms(errors)})
    if any(s["rms_mm"] <= rms(after) + 1 for s in signs):
        issues.append("Reversed-sign hypothesis better or within 1 mm RMS; verify signs by hand")
    holdout = np.empty(len(after))
    fold_checks = {}
    held_level = {}
    for label in sorted(set(data.labels) - {"floor"}):
        test = data.labels == label
        fold, err, check = problem.solve(~test)
        level = test & data.level
        if level.any():
            direction, _ = arm.tip(data.motors[level], fold, direction=60 * arm.jaw)
            held_level[label] = direction[:, 2].tolist()
            if np.abs(direction[:, 2]).max() > 3:
                issues.append(f"Held-out level witness misses by over 3 mm at target {label}")
        holdout[test] = err[test]
        fold_checks[label] = check
        if check["issues"]:
            issues.append(f"Unconstrained or bounded fold without target {label}")
    # Unlabelled floor observations measure z only; leave all of them out together.
    if data.floor.any():
        _, err, check = problem.solve(~data.floor)
        fold_checks["floor"] = check
        holdout[data.floor] = err[data.floor]
        issues += check["issues"]
    if max(rms(after), rms(holdout)) > 3:
        issues.append("Fitted or leave-one-target-out RMS exceeds 3 mm")
    level_mm = []
    if data.level.any():
        direction, _ = arm.tip(data.motors[data.level], p, direction=60 * arm.jaw)
        level_mm = direction[:, 2].tolist()
        if max(abs(v) for v in level_mm) > 3:
            issues.append("Jaw level observation misses by over 3 mm across 60 mm")
    return p, {
        "level_height_difference_mm_per_60mm": level_mm,
        "before_rms_mm": rms(before),
        "after_rms_mm": rms(after),
        "loto_rms_mm": rms(holdout),
        "worst_heldout_mm": float(max(holdout)),
        "jacobian": ident,
        "folds": fold_checks,
        "heldout_level_height_mm_per_60mm": held_level,
        "sign_trials": signs,
        "issues": issues,
        "parameters": dict(zip(PARAMS, p.tolist(), strict=True)),
        "gauges": {
            "shoulder_pan": "existing zero held" if 11 in problem.free else "surveyed sheet yaw",
            "wrist_roll": "independent roll datum"
            if "roll_datum_deg" in data.meta
            else ("level jaw witnesses" if 5 in problem.free else "existing zero held"),
        },
        "points": [
            {
                "row": i + 1,
                "target": row["target"],
                "height_mm": row["height_mm"],
                "utc": row["utc"],
                "signed_before_mm": signed_before[i, 2:].tolist()
                if data.floor[i]
                else signed_before[i].tolist(),
                "signed_after_mm": signed_after[i, 2:].tolist()
                if data.floor[i]
                else signed_after[i].tolist(),
                "before_mm": float(before[i]),
                "after_mm": float(after[i]),
                "heldout_mm": float(holdout[i]),
            }
            for i, row in enumerate(data.rows)
        ],
    }


def fit_file(path, out, links=False):
    if out.exists():
        raise FileExistsError(out)
    captured = path.read_bytes()
    lines = [json.loads(line) for line in captured.splitlines()]
    meta = lines[0]
    if meta.get("schema") != "viscous-touch-v1":
        raise ValueError("Expected capture metadata on the first line")
    data = Session(meta, lines[1:])
    p, report = analyze(data)
    if links:
        if report["loto_rms_mm"] <= 3:
            report["issues"].append("Link corrections requested but baseline already meets 3 mm")
        else:
            baseline = report
            p, report = analyze(data, links=True)
            report["baseline_loto_rms_mm"] = baseline["loto_rms_mm"]
            if report["loto_rms_mm"] >= baseline["loto_rms_mm"] - 0.5:
                report["issues"].append("Link corrections improve held-out RMS by less than 0.5 mm")
    out.mkdir(parents=True, exist_ok=False)
    provenance = {
        "status": "pending Gavin's approval",
        "generated_utc": datetime.now(UTC).isoformat(),
        "sha256": {
            "session": hashlib.sha256(captured).hexdigest(),
            "urdf": digest(URDF),
            "motor_map": digest(MAP),
            "arm": digest(ARM_FILE),
        },
        "session": meta,
        "before_rms_mm": report["before_rms_mm"],
        "after_rms_mm": report["after_rms_mm"],
        "loto_rms_mm": report["loto_rms_mm"],
        "gauges": report["gauges"],
    }
    report["provenance"] = provenance
    save(out / "residuals.json", report)
    if report["issues"]:
        return report
    mapping = read(MAP)
    for i, j in enumerate(JOINTS):
        mapping["joints"][j]["offset_deg"] += float(p[i])
        mapping["joints"][j]["offset_evidence"] = report["gauges"].get(j, "touch fit")
    mapping["provenance"] = provenance
    save(out / "motor_to_urdf.pending.json", mapping)
    save(
        out / "tcp.pending.json",
        {
            "flange": read(ARM_FILE)["flange"],
            "tcp": (Arm().tcp / 1000 + p[6:9] / 1000).tolist(),
            "units": "m",
            "link_length_delta_mm": p[12:].tolist(),
            "provenance": provenance,
        },
    )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("new_directory", type=Path)
    parser.add_argument("--links", action="store_true", help="test upper-arm and forearm length corrections")
    args = parser.parse_args()
    r = fit_file(args.session, args.new_directory, args.links)
    print(
        f"RMS mm: before {r['before_rms_mm']:.2f}, after {r['after_rms_mm']:.2f}, held-out {r['loto_rms_mm']:.2f}"
    )
    if r["issues"]:
        print("\n".join(r["issues"]), file=sys.stderr)
        return 2
    print(
        "Pending review. Regenerate in a new folder:\n.venv/bin/python scripts/viscous-urdf-cal/make_tank_arm.py",
        shlex.quote(str(args.new_directory)),
        shlex.quote(str(args.new_directory / "tank_model")),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
