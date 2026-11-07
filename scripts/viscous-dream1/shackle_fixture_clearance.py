# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.
# /// script
# requires-python = ">=3.12"
# dependencies = ["manifold3d==3.5.4", "trimesh==5.1.1", "numpy==2.2.6", "networkx==3.5"]
# ///
"""Screen selected fixture placement against the supplied gripper dimensions.

uv run --script scripts/viscous-dream1/shackle_fixture_clearance.py
This is a static geometry screen, not IK, dynamics or continuous path validation.
Only the new printed fixtures and tie envelopes are checked against the hand;
contacts with the held pin and the shackle need the full simulation test.
"""

import argparse
import hashlib
import json
from pathlib import Path

import manifold3d as md
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
DEFAULT_ASSETS = HERE / "runs/shackle_tied_holder/tilt_85deg"
PADDING = 0.5  # Expand every hand dimension by this amount for the static screen.


def solid(mesh):
    return md.Manifold(
        md.Mesh64(
            np.array(mesh.vertices, dtype=np.float64, order="C"),
            np.array(mesh.faces, dtype=np.uint64, order="C"),
        )
    )


def mesh_of(shape):
    mesh = shape.to_mesh64()
    return trimesh.Trimesh(mesh.vert_properties[:, :3], mesh.tri_verts, process=True)


def transform(mesh, rotation, translation):
    result = mesh.copy()
    result.vertices = result.vertices @ rotation.T + translation
    return result


def yaw(degrees):
    a = np.radians(degrees)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def frame(axis, across, roll):
    angle = np.radians(roll)
    jaw = np.cos(angle) * across + np.sin(angle) * np.cross(-axis, across)
    normal = np.cross(jaw, axis)
    return np.column_stack([jaw, normal, axis])


def hand_parts(tcp, axis, across, roll, opening):
    """Original conservative boxes from the orientation session's dimensions.

    Local X is jaw direction, Y is finger width, Z is headward along pin axis.
    The published dimensions bound the fingers; real CAD may have cutouts.
    """
    rotation = frame(axis, across, roll)
    # Both fingers: forward tip34.3mm, rear extent at palm start53.4mm.
    for sign in [-1, 1]:
        center = [sign * (opening + 14.5), 0, (53.4 - 34.3) / 2]
        size = np.array([29, 42, 53.4 + 34.3]) + 2 * PADDING
        part = md.Manifold.cube(size, center=True).translate(center)
        yield f"finger_{sign}", part.transform(np.c_[rotation, tcp])
    center = [0, 0, (53.4 + 110) / 2]
    size = np.array([160, 8, 110 - 53.4]) + 2 * PADDING
    part = md.Manifold.cube(size, center=True).translate(center)
    yield "palm", part.transform(np.c_[rotation, tcp])


def overlaps(a, b):
    aa = np.array(a.bounding_box()).reshape(2, 3)
    bb = np.array(b.bounding_box()).reshape(2, 3)
    return bool(np.all(aa[0] < bb[1]) and np.all(bb[0] < aa[1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, default=DEFAULT_ASSETS)
    args = parser.parse_args()
    root = args.assets
    holder_info = json.loads((root / "holder_checks.json").read_text())
    cup_info = json.loads((root / "cup_checks.json").read_text())
    assert holder_info["tilt_from_vertical_deg"] == 85
    cup_tip_local = np.array(cup_info["pin_tip_floor_frame_mm"])
    holder_rotation = np.array(holder_info["body_to_print_rotation"])
    bore = np.array(holder_info["shackle_bore_center_print_mm"]) + [575, 0, 0]
    selected_tip = yaw(15) @ np.array([500.0, 0.0, 0.0]) + [0, 0, 80]
    cup_rotation = yaw(15)
    cup_translation = selected_tip - cup_rotation @ cup_tip_local
    paths = [
        root / "shackle_tied_holder_body_frame.obj",
        root / "zip_tie_envelope_body_frame.obj",
        root / "pin_cup_floor_frame.obj",
    ]
    fixture_meshes = {}
    for name, path in zip(["holder", "ties_and_heads", "cup_pedestal"], paths, strict=True):
        mesh = trimesh.load(path, force="mesh")
        mesh.apply_scale(1000)  # Metre scene meshes -> mm audit coordinates.
        if name == "cup_pedestal":
            mesh = transform(mesh, cup_rotation, cup_translation)
        else:
            mesh = transform(mesh, holder_rotation, bore)
        assert mesh.is_watertight
        fixture_meshes[name] = mesh
    fixtures = {name: solid(mesh) for name, mesh in fixture_meshes.items()}
    fixture_intersections = []
    for name in ["holder", "ties_and_heads"]:
        volume = (fixtures[name] ^ fixtures["cup_pedestal"]).volume()
        fixture_intersections.append({"pair": [name, "cup_pedestal"], "volume_mm3": volume})
        assert volume < 1e-5
    axis = holder_rotation[:, 2]
    across = np.array([0.0, 1.0, 0.0])
    poses = [
        (name, bore + axis * along, axis, across)
        for name, along in [
            ("insert_start", 101.925),
            ("head_regrip", 112.2),
            ("thread_start", 77),
            ("seated", 57),
        ]
    ]
    cup_axis = cup_rotation @ np.array([-np.sin(np.radians(85)), 0, np.cos(np.radians(85))])
    poses.append(("cup_pickup", selected_tip + cup_axis * 68, cup_axis, cup_rotation @ across))
    hits = []
    box_tests = 0
    boolean_tests = 0
    for name, tcp, approach_axis, jaw_across in poses:
        # 121 static rolls, with closed, 12mm-per-side and fully open fingers.
        for roll in range(121):
            for opening in [0.0, 12.0, 52.4125]:
                for hand_name, hand in hand_parts(tcp, approach_axis, jaw_across, roll, opening):
                    for fixture_name, fixture in fixtures.items():
                        box_tests += 1
                        if not overlaps(hand, fixture):
                            continue
                        boolean_tests += 1
                        volume = (hand ^ fixture).volume()
                        if volume > 1e-5:
                            hits.append(
                                {
                                    "pose": name,
                                    "roll_deg": roll,
                                    "opening_per_side_mm": opening,
                                    "hand": hand_name,
                                    "fixture": fixture_name,
                                    "intersection_mm3": float(volume),
                                }
                            )
    data = {
        "status": "PASS" if not hits else "REVIEW_REQUIRED",
        "units": "mm",
        "frame": "s ahead of left J1, w toward sim +X, h above tank floor",
        "holder_tilt_deg": 85,
        "bore_midpoint_mm": bore.tolist(),
        "cup_pin_tip_mm": selected_tip.tolist(),
        "cup_azimuth_deg": 15,
        "cup_floor_rotation": cup_rotation.tolist(),
        "cup_floor_translation_mm": cup_translation.tolist(),
        "gripper_box_padding_mm": PADDING,
        "roll_samples_deg": [0, 120, 1],
        "finger_openings_per_side_mm": [0, 12, 52.4125],
        "poses": [p[0] for p in poses],
        "aabb_pair_tests": box_tests,
        "boolean_pair_tests": boolean_tests,
        "fixture_intersections": fixture_intersections,
        "hand_fixture_intersections": hits,
        "limitations": [
            "Static hand bounds only; no arm IK or wrist/forearm geometry",
            "No inter-pose travel or continuous roll proof",
            "Held-pin and shackle contacts excluded; seated fingertip/ear contact remains open",
        ],
        "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
    }
    out = root / "fixture_clearance_checks.json"
    out.write_text(json.dumps(data, indent=2) + "\n")
    for name, mesh in fixture_meshes.items():
        # Useful read-only placement artifact for the environment integration.
        mesh.apply_scale(0.001)
        (root / f"layout_{name}_plane_frame.obj").write_text(
            trimesh.exchange.obj.export_obj(mesh).rstrip() + "\n"
        )
    print(f"{data['status']}: {box_tests} pairs, {boolean_tests} booleans, {len(hits)} overlaps; {out}")
    if hits:
        print(json.dumps(hits[:10], indent=2))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
