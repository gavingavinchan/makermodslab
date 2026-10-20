"""Offline statistics for raw model intent, client IK tracking, and joint-limit/rest occupancy."""
import yam_baseline  # workspace-only paths before Dream/JAX imports
import json
from pathlib import Path

import numpy as np
from dream import arm_file
from dream_ik.solver import ChunkIK
from dream_ik.specs import EMBODIMENTS, SERVE
from dream_ik.decode import rot6d_to_mat

RUNS = ("yam_vision_live2", "viscous_apple_vision_live2", "replay_yam_path_on_viscous")


def percentiles(x):
    return dict(zip(("median", "p95"), np.percentile(x, (50, 95)).tolist()))


def main():
    results = {}
    plot_data = {}
    for name in RUNS:
        root = Path("runs") / name
        if not (root / "trajectory.json").exists():
            continue
        trajectory = json.loads((root / "trajectory.json").read_text())
        chunks = [json.loads(line) for line in (root / "chunks.jsonl").read_text().splitlines()]
        summary = json.loads((root / "summary.json").read_text())
        arm = "yam" if name.startswith("yam_") else arm_file.register(str(Path("viscous_arm.json").resolve()))
        solver = ChunkIK(arm, SERVE.tool, 20)
        result = {"summary": summary, "sides": {}}
        ledger = root / "requests.jsonl"
        if ledger.exists():
            events = [json.loads(line) for line in ledger.read_text().splitlines()]
            result["billed_calls"] = sum(e["event"] == "success" for e in events)
            result["pool_warming_refusals"] = sum(e.get("reason") == "pool_warming" for e in events)
        for side in ("left", "right"):
            tracks = [t["tracking"][side] for t in trajectory]
            requested = np.array([t["requested_xyz"] for t in tracks])
            apples = np.array([t["apple_base_before_step"][side] for t in trajectory])
            fk = np.array([t["fk_xyz"] for t in tracks])
            distance = np.linalg.norm(requested - apples, axis=1)
            idx = int(distance.argmin())
            q = np.array([t[f"commanded_{side}"][:6] for t in trajectory])
            measured = np.array([t["arm_joints"][side] for t in trajectory])
            measured_fk = solver.fk(measured, side)
            rotations = np.array([chunks[t["call"] - 1]["per_key"][f"action.{side}_eef_rot6"][t["row"]] for t in trajectory])
            target_rot = rot6d_to_mat(rotations)
            relative = np.swapaxes(target_rot, -1, -2) @ measured_fk[:, :3, :3]
            measured_angles = np.degrees(np.arccos(np.clip((np.trace(relative, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))
            spec = EMBODIMENTS[arm].side_arm(side)
            i = solver.sides.index(side)
            rest = np.asarray(spec.rest_pose)
            misses = np.array([t["raw_position_error_m"] > .05 or t["raw_rotation_error_deg"] > 15 for t in tracks])
            side_result = {
                "tracking": {label: percentiles([t[label] for t in tracks]) for label in (
                    "raw_position_error_m", "raw_rotation_error_deg", "filtered_position_error_m", "filtered_rotation_error_deg")},
                "min_requested_to_moving_apple_m": float(distance.min()),
                "min_requested_to_initial_apple_m": float(np.linalg.norm(requested - apples[0], axis=1).min()),
                "closest_request": {"call": trajectory[idx]["call"], "row": trajectory[idx]["row"],
                                    "requested_xyz": requested[idx].tolist(), "apple_xyz": apples[idx].tolist(),
                                    "fk_xyz": fk[idx].tolist(), **tracks[idx]},
                "min_command_fk_to_apple_m": float(np.linalg.norm(fk - apples, axis=1).min()),
                "min_measured_fk_to_apple_m": float(np.linalg.norm(measured_fk[:, :3, 3] - apples, axis=1).min()),
                "measured_tracking": {"raw_position_error_m": percentiles(np.linalg.norm(measured_fk[:, :3, 3] - requested, axis=1)),
                                      "raw_rotation_error_deg": percentiles(measured_angles)},
                "joint_servo_error_rad": percentiles(np.linalg.norm(q - measured, axis=1)),
                "miss_rows": int(misses.sum()),
                "joint_diagnostics": [],
            }
            for j, joint in enumerate(spec.joints):
                side_result["joint_diagnostics"].append({
                    "joint": joint, "command_min_rad": float(q[:, j].min()), "command_max_rad": float(q[:, j].max()),
                    "solver_lo_rad": float(solver.side_lo[i, j]), "solver_hi_rad": float(solver.side_hi[i, j]),
                    "rest_rad": float(rest[j]),
                    "all_fraction_within_0.02rad_lower_limit": float(np.mean(np.abs(q[:, j] - solver.side_lo[i, j]) < .02)),
                    "all_fraction_within_0.02rad_upper_limit": float(np.mean(np.abs(q[:, j] - solver.side_hi[i, j]) < .02)),
                    "all_fraction_within_0.1rad_rest": float(np.mean(np.abs(q[:, j] - rest[j]) < .1)),
                    "miss_fraction_within_0.02rad_lower_limit": float(np.mean(np.abs(q[misses, j] - solver.side_lo[i, j]) < .02)) if misses.any() else None,
                    "miss_fraction_within_0.02rad_upper_limit": float(np.mean(np.abs(q[misses, j] - solver.side_hi[i, j]) < .02)) if misses.any() else None,
                    "miss_fraction_within_0.1rad_rest": float(np.mean(np.abs(q[misses, j] - rest[j]) < .1)) if misses.any() else None,
                    "miss_median_distance_to_rest_rad": float(np.median(np.abs(q[misses, j] - rest[j]))) if misses.any() else None,
                })
            result["sides"][side] = side_result
        result["combined_tracking"] = {label: percentiles([t["tracking"][s][label] for t in trajectory for s in ("left", "right")])
                                       for label in ("raw_position_error_m", "raw_rotation_error_deg")}
        result["object"] = {"max_z_m": max(t["apple_xyz"][2] for t in trajectory),
                            "min_bowl_xy_m": min(t["apple_to_bowl_xy_m"] for t in trajectory),
                            "contact_rows": sum(bool(t["object_robot_contacts"]) for t in trajectory),
                            "ever_success": any(t["success"] for t in trajectory)}
        results[name] = result
        measured_left = solver.fk(np.array([t["arm_joints"]["left"] for t in trajectory]), "left")[:, :3, 3]
        plot_data[name] = {"trajectory": trajectory, "measured_left_xyz": measured_left.tolist()}
    Path("runs/hand_path_analysis.json").write_text(json.dumps(results, indent=2))
    for name, result in results.items():
        print(name, "billed", result.get("billed_calls", 0), "object", result["object"])
        for side, d in result["sides"].items():
            print(side, "tracking", d["tracking"], "min request/apple", d["min_requested_to_moving_apple_m"])
    Path("runs/hand_path_plot_data.json").write_text(json.dumps(plot_data))


if __name__ == "__main__":
    main()
