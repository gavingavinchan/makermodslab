#!/usr/bin/env python3
"""Reproduce the offline frame audit from committed joint/plate snapshots."""

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
OUT = REPO / "configs/viscous/viscous_01/evidence/left-mount-2026-11-05"
ARM = REPO / "configs/viscous/model/viscous_arm.urdf"
SOURCE = json.loads((OUT / "source.json").read_text())
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]


def vector(element, key, default="0 0 0"):
    return np.fromstring(element.get(key, default), sep=" ")


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def fk(root, q):
    # Determine root by topology, independent of the source robot's name.
    children = {j.find("child").get("link") for j in root.findall("joint")}
    base = next(link.get("name") for link in root.findall("link") if link.get("name") not in children)
    frames = {base: np.eye(4)}
    pending = list(root.findall("joint"))
    while pending:
        advanced = False
        for joint in pending.copy():
            parent = joint.find("parent").get("link")
            if parent not in frames:
                continue
            origin = joint.find("origin")
            transform = np.eye(4)
            if origin is not None:
                roll, pitch, yaw = vector(origin, "rpy")
                transform[:3, :3] = (
                    rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
                )
                transform[:3, 3] = vector(origin, "xyz")
            value = q.get(joint.get("name"), 0)
            motion = np.eye(4)
            axis = joint.find("axis")
            if joint.get("type") in ("revolute", "continuous"):
                motion[:3, :3] = rotation(vector(axis, "xyz"), value)
            elif joint.get("type") == "prismatic":
                motion[:3, 3] = vector(axis, "xyz") * value
            frames[joint.find("child").get("link")] = frames[parent] @ transform @ motion
            pending.remove(joint)
            advanced = True
        if not advanced:
            raise ValueError("Unrooted URDF")
    return frames


def check_full_plates():
    """Optional mesh check: needs trimesh, but no external CAD checkout/rtree."""
    import trimesh

    data = np.load(OUT / "mount-plates.npz", allow_pickle=False)
    left, right = [
        trimesh.Trimesh(vertices=data[side + "_vertices"], faces=data[side + "_faces"], process=False)
        for side in ("left", "right")
    ]
    distances = {}
    transforms = {"identity": np.eye(3), "reflect_Y": np.diag([1, -1, 1])}
    transforms.update(
        {f"rotate_Z_{angle}": rotation([0, 0, 1], np.radians(angle)) for angle in (90, -90, 180)}
    )
    for name, transform in transforms.items():
        moved = left.copy()
        matrix = np.eye(4)
        matrix[:3, :3] = transform
        moved.apply_transform(matrix)
        maximum = 0.0
        for source, target in ((moved, right), (right, moved)):
            for chunk in np.array_split(source.vertices, 50):
                maximum = max(maximum, float(trimesh.proximity.closest_point_naive(target, chunk)[1].max()))
        distances[name] = maximum * 1000
    recorded = json.loads((OUT / "full-plate-check.json").read_text())
    assert all(abs(value - recorded["distance_mm"][key]) < 1e-9 for key, value in distances.items())
    print(json.dumps({"offline": True, "full_plate_distances_mm": distances}, indent=2))


def main():
    arm, rov = ET.parse(ARM).getroot(), ET.parse(OUT / "rov-kinematics.urdf").getroot()
    refs = np.array(SOURCE["reference_arm_pose_rad"])
    zero = fk(rov, {})
    rng = np.random.default_rng(20261105)
    maximum = 0.0
    mirror_maximum = 0.0
    reflection = np.diag([1, -1, 1, 1])
    for _ in range(100):
        delta = rng.uniform(-0.7, 0.7, 6)
        table = fk(arm, dict(zip(JOINTS, refs + delta, strict=True)))
        native = fk(
            rov, {f"{side}_{j}": v for side in ("left", "right") for j, v in zip(JOINTS, delta, strict=True)}
        )
        for link in table:
            maximum = max(
                maximum, float(np.max(np.abs(zero["left_base_link"] @ table[link] - native[f"left_{link}"])))
            )
            mirror_maximum = max(
                mirror_maximum,
                float(
                    np.max(np.abs(reflection @ native[f"left_{link}"] @ reflection - native[f"right_{link}"]))
                ),
            )
    hulls = json.loads((OUT / "mount-hulls.json").read_text())
    left, right = [np.array(hulls[s]["vertices"]) for s in ("left", "right")]
    distances = {}
    transforms = {"identity": np.eye(3), "reflect_local_Y": np.diag([1, -1, 1])}
    transforms.update(
        {f"rotate_Z_{angle}": rotation([0, 0, 1], np.radians(angle)) for angle in (-90, 90, 180)}
    )
    for name, transform in transforms.items():
        a = left @ transform.T
        norms = np.linalg.norm(a[:, None, :] - right[None, :, :], axis=2)
        distances[name] = float(max(norms.min(axis=0).max(), norms.min(axis=1).max()) * 1000)

    # Pure Pinocchio computation, no drivers/ports: test the fitted model's
    # invariance to J1 about a vertical axis, rather than assuming fit symmetry.
    from makermodslab.gravity_feedforward import load_model

    _, model = load_model("configs/viscous/viscous_01/gravity-air-fit-v2.json", telemetry_offsets=True)
    gravity_error = 0.0
    for _ in range(100):
        pose = dict(zip(JOINTS, rng.uniform(-100, 100, 6), strict=True))
        shifted = {**pose, "shoulder_pan": pose["shoulder_pan"] + 90}
        gravity_error = max(
            gravity_error, max(abs(model.torque(pose)[j] - model.torque(shifted)[j]) for j in JOINTS)
        )
    mounts = {}
    for side in ("left", "right"):
        joint = rov.find(f"joint[@name='{side}_shoulder_pan']")
        mounts[side] = {
            "transform": zero[f"{side}_base_link"].tolist(),
            "shoulder_pan_axis_local": vector(joint.find("axis"), "xyz").tolist(),
        }
    report = {
        "offline_only": True,
        "samples": 100,
        "arm_urdf_sha256": hashlib.sha256(ARM.read_bytes()).hexdigest(),
        "source_rov_urdf_sha256": SOURCE["source_urdf_sha256"],
        "mounts": mounts,
        "cad_folded_in_single_arm_reference_deg": dict(zip(JOINTS, np.degrees(refs).tolist(), strict=True)),
        "left_FK_max_matrix_error": maximum,
        "right_is_Y_reflection_max_matrix_error": mirror_maximum,
        "plate_hull_bidirectional_vertex_distance_mm": distances,
        "level_base_fitted_gravity_J1_plus_90_max_difference_Nm": gravity_error,
        "right_recorded_old_motor_deg": -94.48,
        "right_world_CCW_deg": 94.48,
        "left_candidate_old_motor_deg": 90.0,
        "left_candidate_world_CCW_deg": -90.0,
        "candidate_status": "Conditional: physical base indexing is absent from CAD/motor evidence",
        "left_candidate_soft_limits_deg": [-275.4, 37.6],
        "left_candidate_estimated_hard_stops_deg": [-278.2647889756541, 40.46478897565412],
    }
    assert maximum < 1e-7 and mirror_maximum < 1e-7 and gravity_error < 1e-9
    (OUT / "analysis.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    if sys.argv[1:] == ["--plates-only"]:
        check_full_plates()
    elif sys.argv[1:]:
        raise SystemExit("Use no arguments for FK/gravity, or --plates-only for the optional trimesh check")
    else:
        main()
