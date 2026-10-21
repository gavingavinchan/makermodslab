"""Summarize recorded requests and physical contacts, with no inference."""
import yam_baseline
import json
import sys
from pathlib import Path
import numpy as np


def analyse(path):
    result = json.loads((path / "summary.json").read_text())
    rows = json.loads((path / "trajectory.json").read_text())
    task = json.loads((path / "configuration.json").read_text())["task"]
    ledger = [json.loads(line) for line in (path / "requests.jsonl").read_text().splitlines()]
    result["billed_calls"] = sum(x["event"] == "success" for x in ledger)
    result["warming_refusals"] = sum(x.get("code") == "DA-2000" and x.get("reason") == "pool_warming" for x in ledger)
    result["minimum_requested_hand_to_object_m"] = {
        side: min(float(np.linalg.norm(np.asarray(r["tracking"][side]["requested_xyz"]) -
                                      r[f"{task}_base_before_step"][side])) for r in rows)
        for side in ("left", "right")}
    initial = rows[0][f"{task}_xyz"][2]
    result["maximum_object_rise_m"] = max(r[f"{task}_xyz"][2] for r in rows) - initial
    result["object_contact_rows"] = sum(bool(r["object_robot_contacts"]) for r in rows)
    def bilateral(r, side):
        contacts = r["object_robot_contacts"]
        jaws = ("gripper_left", "gripper_right") if result["rig"] == "makers" else ("link7", "link8")
        return all(f"{side}_{j}" in contacts for j in jaws)
    result["bilateral_jaw_contact_rows"] = sum(bilateral(r, s) for r in rows for s in ("left", "right"))
    result["lifted_with_bilateral_contact"] = any(r[f"{task}_xyz"][2] > initial + .025 and
                                                 any(bilateral(r, s) for s in ("left", "right")) for r in rows)
    result["tracking"] = {s: {metric: np.percentile([r["tracking"][s][metric] for r in rows], [50, 95]).tolist()
                               for metric in ("raw_position_error_m", "raw_rotation_error_deg")}
                          for s in ("left", "right")}
    result["per_call"] = [{"call": c, "min_requested_distance_m": {
        s: min(float(np.linalg.norm(np.asarray(r["tracking"][s]["requested_xyz"]) - r[f"{task}_base_before_step"][s]))
               for r in rows if r["call"] == c) for s in ("left", "right")},
        "contacts": sorted({contact for r in rows if r["call"] == c for contact in r["object_robot_contacts"]}),
        "max_object_z": max(r[f"{task}_xyz"][2] for r in rows if r["call"] == c)}
        for c in sorted({r["call"] for r in rows})]
    (path / "analysis.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k not in ("per_call", "tracking")}, indent=2))
    return result


if __name__ == "__main__":
    for name in sys.argv[1:]:
        analyse(Path(name))
