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
"""Offline proof of the custom arm's slot/base mapping, using only recorded feedback.

No DreamClient, transport, credentials, camera or arm device is constructed.
The audit hook denies networking before importing Dream. The default measured
pose is in the committed 2026-11-08 recording; --ticks accepts another recording.
"""

import argparse
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import dream_real as runner
import dream_real_core as core
import numpy as np


def check(arm_path: Path, ticks: Path, solve: str) -> dict:
    runner.deny_network()
    from dream.client_ik import POSE_KEYS, ClientIK
    from dream.smoothing import SeamSmoother

    arm = json.loads(arm_path.read_text())
    with ticks.open() as source:
        reading = next(row for line in source if (row := json.loads(line)).get("phase") == "dream")
    measured = reading["pos"]
    jmap = core.JointMap.load()
    q = np.asarray(jmap.to_ik(measured))
    ik = ClientIK(
        {"arm": str(arm_path), "head": "eef", "solve": solve},
        20,
        runner.YAM_EMBODIMENT,
        SimpleNamespace(id=runner.CHECKPOINT, embodiment="yam", cameras=list(core.SLOTS)),
    )
    if solve == "traj" and not ik.traj_ready.wait(timeout=60):
        raise RuntimeError("trajectory solver did not become ready")
    urdf = core.parse_urdf((arm_path.parent / arm["urdf"]).read_bytes())
    t = core.fk(urdf, arm["base"], arm["flange"], dict(zip(core.ARM, q, strict=True)))
    expected_xyz = t[:3, 3] + t[:3, :3] @ arm["tcp"]
    expected_rotation = t[:3, :3] @ core.tool_rotation(arm["approach"], arm["jaw"])
    # Tie this arm-file base to the physical model used by the workspace/guard.
    physical_q = {m: np.deg2rad(v) for m, v in jmap.urdf_deg(measured).items()}
    physical = core.fk(core.parse_urdf(runner.SRC_URDF.read_bytes()), "base_link", arm["flange"], physical_q)
    physical_xyz = physical[:3, 3] + physical[:3, :3] @ arm["tcp"]
    rz = np.diag([-1.0, -1.0, 1.0])
    np.testing.assert_allclose(expected_xyz, rz @ physical_xyz, atol=1e-12)
    np.testing.assert_allclose(t[:3, :3], rz @ physical[:3, :3], atol=1e-12)
    results = {}
    for slot in ("left", "right"):
        state = core.dream_state(measured, jmap, arm["rest"], "offline round trip", slot)
        state.update({f"video.{camera}": [] for camera in core.SLOTS})
        sent = ik.observe(state)
        xyz = np.asarray(sent[f"state.{slot}_eef_xyz"])
        rotation = core.rot6_to_matrix([sent[f"state.{slot}_eef_rot6"]])[0]
        np.testing.assert_allclose(xyz, expected_xyz, atol=2e-7, rtol=0)
        np.testing.assert_allclose(rotation, expected_rotation, atol=5e-7, rtol=0)
        per_key = {
            f"action.{side}_{part}": np.repeat(np.asarray(sent[f"state.{side}_{part}"])[None], 48, 0)
            for side in core.SIDES
            for part in ("eef_xyz", "eef_rot6", "grip")
        }
        commanded = core.commanded_slots([*q, state[f"state.{slot}_grip"][0]], arm["rest"], slot)
        reply = ik.solve({"per_key": per_key}, state, SeamSmoother(20, pose_keys=POSE_KEYS), 0, commanded)
        rows = np.asarray(reply[core.chunk_key(slot)])
        # Dream's regularized numerical solve has small rest/limit/collision bias, not an exact inverse.
        error_deg = np.rad2deg(np.max(np.abs(rows[:, :6] - q), axis=0))
        assert max(error_deg) < 0.5, error_deg
        np.testing.assert_allclose(rows[:, 6], state[f"state.{slot}_grip"][0], atol=1e-10)
        poses = ik._solver.fk(rows[:, :6], slot)
        tcp_error_mm = float(np.max(np.linalg.norm(poses[:, :3, 3] - xyz, axis=1)) * 1000)
        assert tcp_error_mm < 1.0, tcp_error_mm
        results[slot] = {
            "state_eef_xyz_m": xyz.tolist(),
            "state_eef_rot6": np.asarray(sent[f"state.{slot}_eef_rot6"]).tolist(),
            "first_solved_row": rows[0].tolist(),
            "max_joint_error_deg": error_deg.tolist(),
            "max_tcp_error_mm": tcp_error_mm,
        }
    np.testing.assert_allclose(
        results["left"]["state_eef_xyz_m"], results["right"]["state_eef_xyz_m"], atol=1e-9
    )
    np.testing.assert_allclose(
        results["left"]["first_solved_row"], results["right"]["first_solved_row"], atol=1e-6
    )
    assert ik._solver.side_base is None  # no side-dependent placement in a custom-arm spec
    return {
        "arm_file": str(arm_path),
        "ticks": str(ticks),
        "reading_t": reading["t"],
        "measured_motor_deg": measured,
        "measured_ik_rad": q.tolist(),
        "arm_base": arm["base"],
        "independent_fk_xyz_m": expected_xyz.tolist(),
        "physical_base_link_xyz_m": physical_xyz.tolist(),
        "dream_base_from_physical_base_link_rotation": rz.tolist(),
        "solve": solve,
        "side_base": None,
        "slots": results,
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--arm-file", type=Path, default=runner.DEFAULT_ARM_FILE)
    p.add_argument("--ticks", type=Path, default=runner.HERE / "runs/live-20261108T213138Z/ticks.jsonl")
    p.add_argument("--dream-venv", type=Path, default=Path("~/viscous-dream-real/.venv"))
    p.add_argument("--solve", choices=("tick", "traj"), default="traj")
    p.add_argument("--out", type=Path)
    args = p.parse_args()
    runner.deny_network()
    # Only add the package path: no client, model lookup, input check or API key.
    site = next(args.dream_venv.expanduser().glob("lib/python3.*/site-packages"), None)
    if site is not None:
        sys.path.append(str(site))
    os.environ.setdefault(
        "JAX_COMPILATION_CACHE_DIR", str(Path("~/viscous-dream-real/.cache/jax").expanduser())
    )
    result = check(args.arm_file.expanduser().resolve(), args.ticks.expanduser().resolve(), args.solve)
    if args.out:
        core.atomic_json(args.out, result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
