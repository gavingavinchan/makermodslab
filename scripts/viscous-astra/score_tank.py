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
"""Score an individual simulation attempt from independent physics and operator logs."""

import argparse
import json
from pathlib import Path

import numpy as np


def rows(path):
    if path.is_file():
        with path.open() as f:
            for line in f:
                yield json.loads(line)


def handle_lift(samples):
    """A visible handle grasp held clear of the glass over consecutive truth samples."""
    since = None
    for row in samples:
        if row["bilateral_contact"] and not row["object_on_floor"] and row["object_lift_m"] >= 0.010:
            if since is None:
                since = row["t_s"]
            if row["t_s"] - since >= 0.200 - 1e-8:
                return True
        else:
            since = None
    return False


def score(slot):
    trial = json.loads((slot / "trial.json").read_text())
    run = Path(trial["run"])
    stopfile = run / "stop.json"
    stop = json.loads(stopfile.read_text()) if stopfile.is_file() else {}
    cutoff = trial["handback_epoch"]
    ticks = list(rows(run / "ticks.jsonl"))
    offset = trial["epoch_minus_monotonic"]
    cutoff = min([cutoff, *(r["t"] + offset for r in ticks if r["phase"] != "operator")])
    start = trial["ready_epoch"]
    # Bound to the operator's ownership. Return motion and passive release have
    # their own outcome in stop.json and the whole-session simulation summary.
    wall0 = trial["physics_epoch"]
    sim = [r for r in rows(run / "sim-telemetry.jsonl") if wall0 + r["t_s"] <= cutoff]
    if not sim:
        raise ValueError(f"no readable operator physics samples for {run}")
    end = sim[-1]
    events = list(rows(run / "commands.jsonl"))
    requests = [e for e in events if "request" in e]
    waiting_since = start
    windows = []
    for event in events:
        t = min(cutoff, event["time"])
        if "request" in event and waiting_since is not None:
            windows.append((waiting_since, t))
            waiting_since = None
        if "reply" in event:
            waiting_since = t
    durations = [b - a for a, b in windows if b >= a]
    if waiting_since is not None and waiting_since < cutoff:
        windows.append((waiting_since, cutoff))
    # Integrate measured actuator effort while the controller awaits an op,
    # including camera interpretation. Distinguish this from a low-speed joint.
    names = tuple(end["torque"])
    think_peak = np.zeros(len(names))
    think_integral = np.zeros(len(names))
    for a, b in zip(sim, sim[1:], strict=False):
        t0, t1 = wall0 + a["t_s"], wall0 + b["t_s"]
        dt = sum(max(0.0, min(t1, y) - max(t0, x)) for x, y in windows)
        if dt:
            v = np.abs([a["torque"][k] for k in names])
            think_peak = np.maximum(think_peak, v)
            think_integral += v * dt
    faults = [e["reply"]["guard"] for e in events if "reply" in e and not e["reply"].get("ok")]
    transcript = run / "astra-transcript.jsonl"
    calls = [
        e["item"]
        for e in rows(transcript)
        if e["type"] == "item.started" and e.get("item", {}).get("type") == "mcp_tool_call"
    ]
    summary_file = run / "sim-summary.json"
    full = json.loads(summary_file.read_text()) if summary_file.is_file() else {}
    return {
        "run": str(run),
        "task": trial["task"],
        "variant": trial["variant"],
        "mode": trial["mode"],
        "success": handle_lift(sim) if trial["task"] == "screwdriver" else end["success"],
        "simulator_benchmark_success": end["success"],
        "grasped_lift_mm": 1000 * end["grasped_lift_m"],
        "xy_error_mm": 1000 * end["target_error_xy_m"],
        "server_commands": len(requests),
        "op_calls": sum(c.get("tool") == "op" for c in calls),
        "frame_calls": sum(c.get("tool") == "frames" for c in calls),
        "wall_s": trial["handback_epoch"] - start,
        "active_s": cutoff - start,
        "think_to_command_s": durations,
        "think_median_s": float(np.median(durations)) if durations else None,
        "think_p95_s": float(np.percentile(durations, 95)) if durations else None,
        "final_handback_dwell_s": max(0.0, cutoff - waiting_since) if waiting_since is not None else 0.0,
        "torque_peak_nm": end["torque_peak_nm"],
        "stationary_hold_peak_nm": end.get("hold_peak_nm"),
        "stationary_hold_integral_nm_s": end["hold_abs_integral_nm_s"],
        "think_hold_peak_nm": dict(zip(names, think_peak.tolist(), strict=True)),
        "think_hold_integral_nm_s": dict(zip(names, think_integral.tolist(), strict=True)),
        "guard_rejections": faults,
        "trips": [e["trips"] for e in events if "trips" in e],
        "limp": end["limp"],
        "min_glass_mm": 1000 * end["min_glass_clearance_m"],
        "glass_contact_speed_m_s": end["peak_glass_contact_speed_m_s"],
        "glass_floor_normal_m_s": end["peak_floor_normal_m_s"],
        "object_glass_contact_speed_m_s": end.get("object_peak_glass_contact_speed_m_s"),
        "whole_session_glass_contact_speed_m_s": full.get("peak_glass_contact_speed_m_s"),
        "whole_session_object_glass_contact_speed_m_s": full.get("object_peak_glass_contact_speed_m_s"),
        "whole_session_min_glass_mm": 1000 * full["min_glass_clearance_m"] if full else None,
        "reason": stop.get("reason"),
        "returned": stop.get("returned"),
        "release_problems": stop.get("release_problems"),
        "red_button": trial.get("red_button", False),
        "stop_complete": bool(stop),
        "video": str(run / "sim-video.mp4"),
        "camera_frames": full.get("camera_frames"),
        "missed_camera_snapshots": full.get("missed_camera_snapshots"),
        "max_backlog_s": full.get("max_backlog_s"),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("slot", type=Path)
    args = parser.parse_args()
    print(json.dumps(score(args.slot), indent=2, allow_nan=False))
