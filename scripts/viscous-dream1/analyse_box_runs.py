"""Summarize raw requested hand distances and physical box motion without inference."""
import yam_baseline  # Keep all caches inside the workspace.
import json
from pathlib import Path

import numpy as np


def analyse(path):
    trajectory = json.loads((path / "trajectory.json").read_text())
    chunks = [json.loads(line) for line in (path / "chunks.jsonl").read_text().splitlines()]
    ledger = [json.loads(line) for line in (path / "requests.jsonl").read_text().splitlines()]
    result = dict(json.loads((path / "summary.json").read_text()))
    result["ledger_billed_calls"] = sum(row["event"] == "success" for row in ledger)
    result["warming_refusals"] = sum(row["event"] == "refusal" and row["code"] == "DA-2000"
                                     and row["reason"] == "pool_warming" for row in ledger)
    result["minimum_requested_hand_to_box_m"] = {}
    result["minimum_requested_hand_to_box_at_request_m"] = {}
    result["minimum_requested_hand_to_initial_box_m"] = {}
    for side in ("left", "right"):
        samples = []
        for row in trajectory:
            request = np.asarray(row["tracking"][side]["requested_xyz"])
            box = np.asarray(row["box_base_before_step"][side])
            samples.append((float(np.linalg.norm(request - box)), row["call"], row["row"]))
        distance, call, row = min(samples)
        result["minimum_requested_hand_to_box_m"][side] = {"distance_m": distance, "call": call, "row": row}
        result["minimum_requested_hand_to_box_at_request_m"][side] = min(
            float(np.linalg.norm(np.asarray(chunk["per_key"][f"action.{side}_eef_xyz"])
                                 - chunk["box_base_at_request"][side], axis=1).min()) for chunk in chunks)
        result["minimum_requested_hand_to_initial_box_m"][side] = min(
            float(np.linalg.norm(np.asarray(chunk["per_key"][f"action.{side}_eef_xyz"])
                                 - chunks[0]["box_base_at_request"][side], axis=1).min()) for chunk in chunks)
    result["box_height_range_above_table_m"] = [min(row["object_height_above_table_m"] for row in trajectory),
                                                max(row["object_height_above_table_m"] for row in trajectory)]
    result["box_contact_rows"] = sum(bool(row["object_robot_contacts"]) for row in trajectory)
    result["bilateral_finger_contact_rows"] = sum(
        all(f"{side}_tip_{finger}" in row["object_robot_contacts"] for finger in ("left", "right"))
        for row in trajectory for side in ("left", "right"))
    resting = [row for row in trajectory if row["success"] and not row["object_robot_contacts"]]
    result["successful_contact_free_rest_s"] = len(resting) / 20
    result["first_success"] = {k: resting[0][k] for k in ("call", "row", "box_xyz")} if resting else None
    result["final_box_xyz"] = trajectory[-1]["box_xyz"]
    result["per_call"] = []
    for chunk in chunks:
        rows = [row for row in trajectory if row["call"] == chunk["call"]]
        if not rows:
            continue
        result["per_call"].append({
            "call": chunk["call"],
            "box_robot_contact_rows": sum(bool(row["object_robot_contacts"]) for row in rows),
            "contacts": sorted({name for row in rows for name in row["object_robot_contacts"]}),
            "box_height_max_m": max(row["object_height_above_table_m"] for row in rows),
            "box_to_square_min_m": min(row["box_to_square_xy_m"] for row in rows),
            "commanded_grip_range": {side: [min(row[f"commanded_{side}"][6] for row in rows),
                                               max(row[f"commanded_{side}"][6] for row in rows)]
                                      for side in ("left", "right")},
            "closest_requested_hand_m": {side: min(float(np.linalg.norm(
                np.asarray(row["tracking"][side]["requested_xyz"]) - row["box_base_before_step"][side]))
                for row in rows) for side in ("left", "right")},
        })
    (path / "box_analysis.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    import sys
    for name in sys.argv[1:]:
        print(json.dumps(analyse(Path(name)), indent=2))
