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
"""Touches to candidate files, through the existing makermodslab.tank_geometry.fit_tank only.

fit_tank (the hand-guided tank measurement's fit) does the plane offsets, the shared yaw, the
held-out check touches and its own quality gates; this module adds the synthetic back-glass touch,
a leave-worst-out pass for a touch that is not on the glass, and the two candidate files. Both are
written into the run folder and marked pending Gavin's approval; nothing here writes tank.json,
the keep-out config or the Lab's tank measurement.
"""

from __future__ import annotations

import copy
import math
import time

import numpy as np
from geometry import POST_THICKNESS_MM, Estimate, back_touch, geo, tank_guard

MAX_DROPS = 2
OFF_CENTRE_FLAG_MM = 15.0  # half the 30 mm wall margin
YAW_FLAG_DEG = 1.5  # 1.5 deg is ~16 mm at the arm's ~610 mm reach, half the wall margin
PLANE_CHANGE_FLAG_MM = 10.0
PENDING = "pending Gavin's approval"
MIN_COUNTS = {"left": 2, "right": 2, "floor": 1}


def _fit(touches, tank):
    return geo.fit_tank(touches, tank)


def width_fallback(touches: list[dict], tank: dict) -> tuple[list[dict], dict | None]:
    """When the +y wall's real touches cannot meet fit_tank's own gate (2 touches 150 mm apart), the
    +y plane is DERIVED: the fitted -y wall moved by the nominal inner width (fit_tank needs left
    touches, so two synthetic ones are placed along the -y touches' span). Every real +y touch is
    kept as a held-out check of that width, so its residual measures the width error. Returns the
    touches for the fit and a note (None when not needed)."""
    left = [t for t in touches if t["label"] == "left"]
    xs = [t["point_mm"][0] for t in left]
    if len(left) >= 2 and max(xs) - min(xs) >= geo.MIN_WALL_SPAN_MM:
        return touches, None
    right = [t for t in touches if t["label"] == "right"]
    if len(right) < 2:
        return touches, None
    width = tank["outer_dimensions_mm"]["value"]["width"] - 2 * tank["glass_thickness_mm"]["value"]
    p = np.array([t["point_mm"] for t in right], dtype=float)
    dx, dy = p[:, 0] - p[:, 0].mean(), p[:, 1] - p[:, 1].mean()
    yaw = math.degrees(math.atan2(float(dx @ dy), float(dx @ dx))) if float(dx @ dx) > 1.0 else 0.0
    axes = geo.tank_axes(yaw)
    synthetic = []
    for t in sorted(right, key=lambda t: t["point_mm"][0])[:: max(1, len(right) - 1)]:
        p = axes @ np.asarray(t["point_mm"], dtype=float)
        p[1] += width
        synthetic.append(
            {
                "label": "left",
                "point_mm": (axes.T @ p).tolist(),
                "positions": None,
                "source": "DERIVED: the -y wall + the nominal inner width; not measured",
            }
        )
    checks = [{**t, "label": "check:left", "was": "left"} for t in left]
    rest = [t for t in touches if t["label"] != "left"]
    note = {
        "mode": "left_from_width",
        "inner_width_mm": width,
        "real_left_touches_as_checks": len(checks),
        "why": "the +y touches could not span fit_tank's 150 mm (J1's +187.5 deg soft limit)",
    }
    return [*rest, *checks, *synthetic], note


def robust_fit(touches: list[dict], tank: dict, estimate: Estimate) -> dict:
    """fit_tank, then while it fails only on the residual gate, drop the worst probed touch (a
    rigid object on the floor, say, passes the press check but not the plane) as long as the face
    keeps its minimum. Returns {fit, kept, dropped, dropped_labels}."""
    back = back_touch(estimate)
    kept, fallback = width_fallback(list(touches), tank)
    dropped = []
    fit = _fit([*kept, back], tank)
    while not fit["valid"] and "residuals" in fit and len(dropped) < MAX_DROPS:
        if not any("residual" in f for f in fit["failures"]):
            break
        candidates = []
        for r in fit["residuals"]:
            if r["index"] >= len(kept):
                continue  # the synthetic back touch
            touch = kept[r["index"]]
            face = touch["label"].removeprefix("check:")
            same = [t for t in kept if t["label"] == touch["label"]]
            if not touch["label"].startswith("check:") and len(same) - 1 < MIN_COUNTS.get(face, 1):
                continue
            candidates.append((abs(r["residual_mm"]), r["index"]))
        if not candidates:
            break
        _, index = max(candidates)
        dropped.append({**kept[index], "dropped_residual_mm": max(candidates)[0]})
        kept = kept[:index] + kept[index + 1 :]
        fit = _fit([*kept, back], tank)
    if fallback is not None:
        fit["left_derived"] = fallback
    return {
        "fit": fit,
        "kept": kept,
        "dropped": dropped,
        "dropped_labels": [d["label"] for d in dropped],
        "left_derived": fallback,
    }


def placement_report(fit: dict, keepout: dict) -> dict:
    """Where J1 sits in the tank, and how far that is from the nominal mount."""
    if "offsets_mm" not in fit:
        return {"valid": False, "failures": fit.get("failures")}
    o = fit["offsets_mm"]
    planes = {f: float(tank_guard._value(keepout["planes_mm"][f])) for f in ("left", "right", "floor")}
    off_centre = -(o["left"] + o["right"]) / 2  # J1's tank-y from the inner midline, + toward left
    flags = []
    if abs(off_centre) > OFF_CENTRE_FLAG_MM:
        flags.append(
            f"J1 is {off_centre:+.1f} mm off the tank's centre line (flag at {OFF_CENTRE_FLAG_MM:g})"
        )
    if abs(fit["yaw_deg"]) > YAW_FLAG_DEG:
        flags.append(f"base yawed {fit['yaw_deg']:+.2f} deg in the tank (flag at {YAW_FLAG_DEG:g})")
    change = {f: o[f] - planes[f] for f in planes}
    for f, d in change.items():
        if abs(d) > PLANE_CHANGE_FLAG_MM:
            flags.append(f"{f} plane {d:+.1f} mm from the current keep-out ({planes[f]:g})")
    return {
        "valid": fit["valid"],
        "yaw_deg": fit["yaw_deg"],
        "planes_mm": {f: o[f] for f in ("near", "left", "right", "floor", "far", "rim")},
        "inner_width_mm": fit["width_mm"],
        "j1_off_centre_mm": off_centre,
        "j1_to_back_glass_mm": o["near"],
        "floor_below_j1_mm": -o["floor"],
        "change_from_keepout_mm": change,
        "rms_mm": fit.get("rms_mm"),
        "max_residual_mm": fit.get("max_mm"),
        "flags": flags,
    }


def candidate_files(fit: dict, touches: list[dict], tank: dict, keepout: dict, run: str) -> tuple[dict, dict]:
    """(candidate tank geometry in the tank-measure export format, proposed keep-out). Only for a
    valid fit; both say PENDING at the top and in every changed value's status."""
    stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    serial = [{k: v for k, v in t.items() if k != "positions" or v is not None} for t in touches]
    exported = geo.export_tank(tank, fit, serial, None, {}, stamp)
    note = {
        "status": PENDING,
        "produced_by": "scripts/viscous-tank-autocal (compliance presses, simulation-validated only)",
        "run": run,
        "do_not_use_until": "Gavin approves it; copy values by hand, this file replaces nothing",
    }
    exported = {"approval": note, **exported}
    o = fit["offsets_mm"]
    proposal = copy.deepcopy(keepout)
    source = f"tank autocal {run} {stamp}; {PENDING}"
    for face in ("left", "right", "floor"):
        proposal["planes_mm"][face] = {
            "value": round(o[face], 1),
            "status": "autocal_pending",
            "source": source,
        }
    proposal["planes_mm"]["far"] = {
        "value": round(o["far"], 1),
        "status": "autocal_pending",
        "source": source + "; fitted back glass minus the nominal inner length (far is never reached)",
    }
    proposal["planes_mm"]["rim"] = {
        "value": round(o["rim"], 1),
        "status": "autocal_pending",
        "source": source + "; fitted floor plus the nominal inner height",
    }
    near = o["near"] - POST_THICKNESS_MM
    proposal["planes_mm"]["near"] = {
        "value": round(near, 1),
        "status": "autocal_pending",
        "source": source + "; the post face: the post is flush on the back glass (never probed), "
        f"{POST_THICKNESS_MM:g} mm in front of it",
    }
    proposal["yaw_deg"] = {"value": round(fit["yaw_deg"], 3), "status": "autocal_pending", "source": source}
    proposal = {"approval": note, **proposal}
    if not math.isfinite(near):
        raise ValueError("non-finite near plane")
    return exported, proposal
