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
"""Report the settled start-pose rehearsal hold using tank-model TCP FK."""

import argparse
import json
from pathlib import Path

import dream_real_core as core
import numpy as np

HERE = Path(__file__).resolve().parent


def report(path, arm_file=None):
    path = Path(path)
    if path.is_dir():
        direct = path / "ticks.jsonl"
        path = direct if direct.is_file() else path / "nominal/ticks.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    holds = [r for r in rows if r.get("phase") == "hold" and not r.get("trips")]
    if not holds:
        raise ValueError("no rehearsal hold at the start pose")
    start = holds[0]["t"]
    holds = [r for r in holds if r["t"] >= start + 1.0]
    if not holds:
        raise ValueError("hold has no settled readings after its first second")
    plan_path = next(
        (p / "plan.json" for p in (path.parent, path.parent.parent) if (p / "plan.json").is_file()), None
    )
    plan = json.loads(plan_path.read_text()) if plan_path else None
    if arm_file is None and plan:
        arm_file = Path(plan["arm_file"]["path"])
        if not arm_file.is_file():
            raise ValueError("recorded arm file is unavailable; pass --arm-file from the run bundle")
    arm_file = Path(arm_file or HERE / "tank_model/viscous_waterproof_tank_arm.json")
    arm = json.loads(arm_file.read_text())
    joints = core.parse_urdf((HERE / "tank_model/viscous_waterproof_tank.urdf").read_bytes())
    jmap = core.JointMap.load()
    tcp = np.array([*arm["tcp"], 1.0])

    def height(q):
        ik = dict(zip(core.ARM, jmap.to_ik(q), strict=True))
        return float((core.fk(joints, "dream_base", "wrist_roll_link", ik) @ tcp)[2] * 1000)

    from gravity_fit import KP

    table = {
        n: {
            "cmd_minus_pos_deg": float(np.median([r["cmd"][n] - r["pos"][n] for r in holds])),
            "torque_nm": float(np.median([r["torque"][n] for r in holds])),
            "ff_nm": float(np.median([r.get("ff", {}).get(n, 0.0) for r in holds])),
            # Telemetry is total motor effort, including ff. Means make the signed effort balance
            # additive; medians above retain the original sag report's settled summary.
            "mean_ff_nm": float(np.mean([r.get("ff", {}).get(n, 0.0) for r in holds])),
            "mean_stiffness_nm": float(
                np.mean([KP[n] * np.radians(r["cmd"][n] - r["pos"][n]) for r in holds])
            ),
            "mean_torque_minus_effort_nm": float(
                np.mean(
                    [
                        r["torque"][n]
                        - KP[n] * np.radians(r["cmd"][n] - r["pos"][n])
                        - r.get("ff", {}).get(n, 0.0)
                        for r in holds
                    ]
                )
            ),
        }
        for n in core.ARM
    }
    sag = float(np.median([height(r["cmd"]) - height(r["pos"]) for r in holds]))
    return {
        "ticks": len(holds),
        "duration_s": holds[-1]["t"] - holds[0]["t"],
        "joints": table,
        "vertical_sag_mm": sag,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--arm-file", type=Path)
    args = parser.parse_args(argv)
    try:
        data = report(args.run, args.arm_file)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    print(f"Start-pose hold: {data['ticks']} settled readings, {data['duration_s']:.2f} s")
    print(f"{'joint':14s} {'cmd-pos deg':>12s} {'torque Nm':>12s} {'ff Nm':>12s}")
    for n, v in data["joints"].items():
        print(f"{n:14s} {v['cmd_minus_pos_deg']:12.3f} {v['torque_nm']:12.3f} {v['ff_nm']:12.3f}")
    print("Signed effort balance (means, Nm; settled velocity term omitted):")
    for n, v in data["joints"].items():
        print(
            f"{n:14s} kp*err {v['mean_stiffness_nm']:+.3f} ff {v['mean_ff_nm']:+.3f} "
            f"torque - (kp*err + ff) {v['mean_torque_minus_effort_nm']:+.3f}"
        )
    print(f"Hand vertical sag: {data['vertical_sag_mm']:.3f} mm (command TCP z - measured TCP z)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
