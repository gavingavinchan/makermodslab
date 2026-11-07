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
# dependencies = [
#   "manifold3d==3.5.4", "trimesh==5.1.1", "numpy==2.2.6",
#   "shapely==2.2.0", "mapbox-earcut==2.1.0", "matplotlib==3.10.8",
#   "pillow==11.3.0", "rtree==1.4.1", "networkx==3.5", "lxml==6.1.3", "scipy==1.18.1",
# ]
# ///
"""Freestanding, two-tie shackle cradle, goal B4. All CAD dimensions are mm.

Run: uv run --script shackle_tied_holder.py --tilt 75
Angle is from vertical, with the head leaning toward the arm, bow toward/down.
75 degrees is a design example, NOT the environment's approved layout.
Only this design's assets and report directory are written. No hardware or API.
"""

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import manifold3d as md
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets/shackle"
BASE_LENGTH = 150.0
BASE_WIDTH = 130.0
BASE_HEIGHT = 6.0
SUPPORT_LENGTH = 17.0
SUPPORT_WIDTH = 44.0
FIT_CLEARANCE = 0.7  # Forged curve is provisional; allow a dry fit before use.
EDGE_RADIUS = 0.5
SLOT_WIDTH = 6.2  # Intended for ties up to 4.8 x 1.5 mm, user size unconfirmed.
SLOT_HEIGHT = 2.8
SLOT_ROOF = 3.0
TIE_WIDTH = 4.8
TIE_THICKNESS = 1.5
# Two separated points on the circular crown, measured from its lowest point.
# Keep ties away from the floor-contact apex so the shackle needs NO elevation.
STATION_OFFSETS_DEG = (-55.0, 55.0)


def mesh(solid):
    m = solid.to_mesh64()
    return trimesh.Trimesh(m.vert_properties[:, :3], m.tri_verts, process=True)


def solid(m):
    return md.Manifold(
        md.Mesh64(np.asarray(m.vertices, dtype=np.float64), np.asarray(m.faces, dtype=np.uint64))
    )


def box(size, center):
    return md.Manifold.cube(size, center=True).translate(center)


def chamfered_box(size, center, chamfer=EDGE_RADIUS):
    """Flat bed and 45-degree 0.5 mm edge breaks, including vertical edges."""
    sx, sy, sz = size
    c = chamfer
    ring = np.array(
        [
            [-sx / 2 + c, -sy / 2],
            [sx / 2 - c, -sy / 2],
            [sx / 2, -sy / 2 + c],
            [sx / 2, sy / 2 - c],
            [sx / 2 - c, sy / 2],
            [-sx / 2 + c, sy / 2],
            [-sx / 2, sy / 2 - c],
            [-sx / 2, -sy / 2 + c],
        ]
    )
    inner = np.array(
        [
            [-sx / 2 + c, -sy / 2 + c],
            [sx / 2 - c, -sy / 2 + c],
            [sx / 2 - c, sy / 2 - c],
            [-sx / 2 + c, sy / 2 - c],
        ]
    )
    pts = np.vstack(
        [
            np.c_[inner, np.full(4, -sz / 2)],
            np.c_[ring, np.full(8, -sz / 2 + c)],
            np.c_[ring, np.full(8, sz / 2 - c)],
            np.c_[inner, np.full(4, sz / 2)],
        ]
    )
    return md.Manifold.hull_points(pts).translate(center)


def build(tilt, out):
    # Importing this module defines the existing measured/provisional geometry;
    # generate() is never called, so the other session's meshes are untouched.
    import shackle_assets as source

    params = json.loads((ASSETS / "shackle_params.json").read_text())
    t = np.radians(tilt)
    rotation = np.array([[np.cos(t), 0, -np.sin(t)], [0, 1, 0], [np.sin(t), 0, np.cos(t)]])
    body = trimesh.load(ASSETS / "shackle_body.stl", force="mesh")
    body.apply_scale(1000)
    body.vertices = body.vertices @ rotation.T
    # Exactly the scene's zero-clearance pose: bow touches the tank floor.
    height = -body.bounds[0, 2]
    body.apply_translation([0, 0, height])
    pin = trimesh.load(ASSETS / "shackle_pin.stl", force="mesh")
    pin.apply_scale(1000)
    seated = np.array(params["frame"]["seated_pin_translation_m"]) * 1000
    pin.vertices = (pin.vertices + seated) @ rotation.T + [0, 0, height]

    radius = source.BOW_ROD_DIAMETER / 2
    crown_r = source.BOW_CROWN_RADIUS
    crown_x = source.BOW_CROWN_CENTER_X
    low_angle = 270 - tilt
    stations = []
    for delta in STATION_OFFSETS_DEG:
        a = np.radians(low_angle + delta)
        if not 90 <= low_angle + delta <= 270:
            raise ValueError("Tie station would leave the circular crown; choose tilt 55..85 deg")
        p = np.array([crown_x + crown_r * np.cos(a), 0, crown_r * np.sin(a)]) @ rotation.T
        p += [0, 0, height]
        stations.append(p)
    stations.sort(key=lambda p: p[0])
    base_s = float((stations[0][0] + stations[-1][0]) / 2)
    support = chamfered_box([BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT], [base_s, 0, BASE_HEIGHT / 2])
    for p in stations:
        top = p[2] + 1.0
        support += chamfered_box([SUPPORT_LENGTH, SUPPORT_WIDTH, top], [p[0], 0, top / 2])

    # Sweep each expanded bow ring vertically up. This opens every saddle to
    # drop-in assembly, with no clip undercuts and an opening through the plate
    # at the lowest bow point. Steel keeps its original floor height.
    path = source.bow_path()
    tangent = np.gradient(path, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    across = np.cross(tangent, [0, 1, 0])
    angle = np.arange(64) * 2 * np.pi / 64
    # Circumscribed rings: never shrink the nominal radial clearance on flats.
    r = (radius + FIT_CLEARANCE) / np.cos(np.pi / 64)
    rings = path[:, None, :] + r * (
        np.cos(angle)[None, :, None] * [0, 1, 0] + np.sin(angle)[None, :, None] * across[:, None, :]
    )
    rings = rings @ rotation.T + [0, 0, height]
    cutters = []
    for i in range(len(rings) - 1):
        pts = np.vstack([rings[i], rings[i + 1]])
        cutters.append(md.Manifold.hull_points(np.vstack([pts, pts + [0, 0, 160]])))
    cutter = md.Manifold.batch_boolean(cutters, md.OpType.Add)
    holder = support - cutter

    # Find the actual lowest steel surface in each entire tie-width slice.
    # Place the tunnel below it, retaining >=3 mm of plastic above the tunnel.
    channel_data = []
    tie_solids = []
    for index, p in enumerate(stations, 1):
        # Include all triangles crossing the slice, not only their vertices.
        tri = body.triangles
        intersects = (tri[:, :, 0].min(axis=1) <= p[0] + SLOT_WIDTH / 2) & (
            tri[:, :, 0].max(axis=1) >= p[0] - SLOT_WIDTH / 2
        )
        lower = tri[intersects, :, 2].min() - FIT_CLEARANCE
        available_floor = lower - SLOT_ROOF - SLOT_HEIGHT
        floor = min(20.0, available_floor - 1.0) if available_floor >= BASE_HEIGHT + 1.5 else 2.0
        if available_floor < floor:
            raise ValueError(f"Tie {index} needs more floor thickness: {floor:.2f} mm")
        z = floor + SLOT_HEIGHT / 2
        # Short U-shaped routing: two openings accessible from ABOVE, joined
        # under the saddle. No fishing a tie through 130 mm of buried base.
        # The loop closes over ONLY the lower bow stock, clear of ears and pin.
        y = 17.5
        opening_depth = 4.0
        # Lean into the open centre: a vertical loop at this crown station
        # would enclose both bow branches rather than only the seated stock.
        inward = -np.sign(p[0] - base_s) * 18.0
        top = float(p[2] + radius + 10)
        tunnel = box([SLOT_WIDTH, 2 * y + opening_depth, SLOT_HEIGHT], [p[0], 0, z])
        for side in [-1, 1]:
            tunnel += md.Manifold.batch_hull(
                [
                    box([SLOT_WIDTH, opening_depth, SLOT_HEIGHT], [p[0], side * y, z]),
                    box([SLOT_WIDTH, opening_depth, SLOT_HEIGHT], [p[0] + inward, side * y, top]),
                ]
            )
        holder -= tunnel
        # A complete illustrative tie loop, with air gaps around steel/plastic.
        # Loose routing intentionally overestimates required length.
        bottom = z
        loop = box([TIE_WIDTH, 2 * y + TIE_THICKNESS, TIE_THICKNESS], [p[0], 0, bottom])
        loop += box([TIE_WIDTH, 2 * y + TIE_THICKNESS, TIE_THICKNESS], [p[0] + inward, 0, top])
        for side in [-1, 1]:
            loop += md.Manifold.batch_hull(
                [
                    box([TIE_WIDTH, TIE_THICKNESS, TIE_THICKNESS], [p[0], side * y, bottom]),
                    box([TIE_WIDTH, TIE_THICKNESS, TIE_THICKNESS], [p[0] + inward, side * y, top]),
                ]
            )
        tie_solids.append(loop)
        channel_data.append(
            {
                "number": index,
                "center_mm": [float(p[0]), 0, float(z)],
                "clear_width_mm": SLOT_WIDTH,
                "clear_height_mm": SLOT_HEIGHT,
                "floor_mm": float(floor),
                "roof_to_cutter_mm": float(lower - floor - SLOT_HEIGHT),
                "illustrated_loop_length_mm": float(4 * y + 2 * np.hypot(top - bottom, inward)),
            }
        )
    print("Two tunnels cut; rounding exposed cradle/channel rims", flush=True)
    # All convex rims, including newly cut saddles/channel mouths, receive R0.5.
    # Coarse faceted rolling kernel circumscribes the requested radius.
    sphere = md.Manifold.sphere(1, 16)
    sm = mesh(sphere)
    inradius = np.min(np.einsum("ij,ij->i", sm.face_normals, sm.triangles_center))
    kernel = sphere.scale([EDGE_RADIUS / inradius] * 3)
    rounded = holder.minkowski_difference(kernel).minkowski_sum(kernel)
    # Keep the flat bed strip and its intentional 45-degree bottom chamfer.
    bed = box([BASE_LENGTH + 2, BASE_WIDTH + 2, 1.5], [base_s, 0, 0.75])
    holder = rounded + (holder ^ bed)
    holder = holder.as_original().simplify(0.002)
    raw = holder.to_mesh64()
    for grid in [0.0001, 0.00005, 0.0002, 0.0005]:
        healed = md.Manifold(
            md.Mesh64(
                np.array(
                    np.round(raw.vert_properties[:, :3] / grid) * grid, dtype=np.float64, order="C", copy=True
                ),
                np.array(raw.tri_verts, dtype=np.uint64, order="C", copy=True),
            )
        )
        hm = mesh(healed)
        if hm.is_watertight and hm.is_winding_consistent and len(hm.split()) == 1:
            holder = healed
            break
    print(
        f"Mesh: watertight={hm.is_watertight}, winding={hm.is_winding_consistent}, "
        f"components={[(len(m.faces), round(m.volume, 3)) for m in hm.split()]}",
        flush=True,
    )
    assert hm.is_watertight and hm.is_winding_consistent and len(hm.split()) == 1
    body_s, pin_s = solid(body), solid(pin)
    assert (holder ^ body_s).volume() < 1e-5
    assert (holder ^ pin_s).volume() < 1e-5
    raised_intersections = []
    for dz in [0, 0.5, 2, 5, 10, 20, 40, 80, 140]:
        v = (holder ^ body_s.translate([0, 0, dz])).volume()
        assert v < 1e-5
        raised_intersections.append(float(v))
    for tie in tie_solids:
        assert (holder ^ tie).volume() < 1e-5, "Illustrative tie crosses holder"
        assert (body_s ^ tie).volume() < 1e-5, "Illustrative tie crosses steel"
    # Check the pin's full insertion/extraction translation, independent of arm.
    for distance in np.linspace(0, 160, 33):
        shift = rotation @ np.array([0, 0, distance])
        assert (holder ^ pin_s.translate(shift)).volume() < 1e-5
    # Orientation-session handoff: keep a 60 mm diameter corridor clear from
    # the clear ear toward the arm/head side, for shank pickup and head regrip.
    clear_face = params["dimensions"]["EAR_OUTER_SPAN"]["value"] / 2
    corridor = md.Manifold.cylinder(300, 30 / np.cos(np.pi / 96), circular_segments=96)
    corridor = corridor.translate([0, 0, clear_face]).transform(np.c_[rotation, [0, 0, height]])
    assert (holder ^ corridor).volume() < 1e-5
    for tie in tie_solids:
        assert (tie ^ corridor).volume() < 1e-5

    out.mkdir(parents=True, exist_ok=True)
    hm.units = "mm"
    hm.export(out / "shackle_tied_holder.stl")
    hm.export(out / "shackle_tied_holder.3mf")
    with zipfile.ZipFile(out / "shackle_tied_holder.3mf") as archive:
        assert ET.fromstring(archive.read("3D/3dmodel.model")).attrib["unit"] == "millimeter"
    for filename in ["shackle_tied_holder.stl", "shackle_tied_holder.3mf"]:
        exported = trimesh.load(out / filename, force="mesh")
        assert exported.is_watertight and len(exported.split()) == 1
        assert np.max(np.abs(exported.bounds - hm.bounds)) < 1e-4
    # Sim OBJ is in the shackle BODY frame, in metres, like the old holder.
    sim = hm.copy()
    sim.vertices = ((sim.vertices - [0, 0, height]) @ rotation) / 1000
    (out / "shackle_tied_holder_body_frame.obj").write_text(
        trimesh.exchange.obj.export_obj(sim).rstrip() + "\n"
    )
    tie_envelope = mesh(md.Manifold.batch_boolean(tie_solids, md.OpType.Add))
    tie_envelope.vertices = ((tie_envelope.vertices - [0, 0, height]) @ rotation) / 1000
    (out / "zip_tie_envelope_body_frame.obj").write_text(
        trimesh.exchange.obj.export_obj(tie_envelope).rstrip() + "\n"
    )
    # Mass check, including water-filled print: no sealed-air buoyancy assumed.
    # Conservative holder mass assumes ONLY the two side bands are solid PETG.
    steel_density = 7.9e-6  # kg/mm^3
    petg_density = 1.27e-6
    masses = np.array(
        [body.volume * steel_density, pin.volume * steel_density, hm.volume * petg_density * 0.4]
    )
    coms = np.array([body.center_mass, pin.center_mass, hm.center_mass])
    stability = {}
    for wet in [False, True]:
        for with_pin in [False, True]:
            weights = masses.copy()
            if wet:
                weights *= [1 - 1 / 7.9, 1 - 1 / 7.9, 1 - 1 / 1.27]
            if not with_pin:
                weights[1] = 0
            com = np.average(coms, axis=0, weights=weights)
            # Inscribed support rectangle inside the bottom perimeter chamfer.
            margins = np.array(
                [BASE_LENGTH / 2 - 0.5 - abs(com[0] - base_s), BASE_WIDTH / 2 - 0.5 - abs(com[1])]
            )
            assert margins.min() > 0
            restoring = margins.min() / 1000 * weights.sum() * 9.81
            stability[f"{'wet' if wet else 'dry'}_{'pin_seated' if with_pin else 'body_only'}"] = {
                "combined_com_mm": com.tolist(),
                "support_margin_mm": margins.tolist(),
                "effective_weight_N": float(weights.sum() * 9.81),
                "minimum_gravity_restoring_moment_Nm": float(restoring),
            }
    down_faces = hm.face_normals[:, 2] < -0.01
    nonbed = hm.triangles_center[:, 2] > 0.05
    overhang = np.degrees(np.arcsin(np.clip(-hm.face_normals[:, 2], 0, 1)))
    steep = down_faces & nonbed & (overhang > 46)
    stats = {
        "tilt_from_vertical_deg": tilt,
        "layout_status": "EXAMPLE; await environment feasibility",
        "units": "mm",
        "dimensions_mm": hm.extents.tolist(),
        "holder_volume_mm3": hm.volume,
        "shackle_bore_center_print_mm": [0, 0, float(height)],
        "body_to_print_rotation": rotation.tolist(),
        "bow_floor_clearance_mm": float(body.bounds[0, 2]),
        "base_center_s_mm": base_s,
        "nominal_fit_clearance_mm": FIT_CLEARANCE,
        "edge_treatment_mm": EDGE_RADIUS,
        "zip_tie_channels": channel_data,
        "single_watertight_solid": True,
        "body_intersection_mm3": 0,
        "vertical_drop_in_intersections_mm3": raised_intersections,
        "pin_path_samples_clear": 33,
        "complete_tie_loops_clear": len(tie_solids),
        "head_side_corridor": {
            "diameter_mm": 60,
            "length_mm": 300,
            "starts_at_body_z_mm": clear_face,
            "holder_and_ties_intersection_mm3": 0,
        },
        "export_roundtrips_pass": True,
        "estimated_masses_kg_body_pin_40pct_petg": masses.tolist(),
        "static_stability": stability,
        "steep_downface_area_mm2": float(hm.area_faces[steep].sum()),
        "max_downface_bridge_span_mm": SLOT_WIDTH,
        "source_sha256": {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                ASSETS / "shackle_body.stl",
                ASSETS / "shackle_pin.stl",
                ASSETS / "shackle_params.json",
                HERE / "shackle_assets.py",
            ]
        },
    }
    (out / "holder_checks.json").write_text(json.dumps(stats, indent=2) + "\n")
    ties = [mesh(s) for s in tie_solids]
    return hm, body, pin, ties, stats


def render(out, hm, body, pin, ties, stats):
    import shackle_holder as renderer
    from shackle_holder import render as raster

    renderer.OUT = out
    center = np.array([stats["base_center_s_mm"], 0, hm.bounds[1, 2] / 2])
    scale = 245
    plastic = (hm, [0.10, 0.51, 0.74], False)
    tie_parts = [(m, [0.98, 0.45, 0.08], False) for m in ties]
    raster(
        "assembled",
        [plastic, (body, [0.74, 0.78, 0.82], True), (pin, [0.68, 0.72, 0.77], True), *tie_parts],
        center + [-170, -230, 160],
        center,
        scale,
        0,
        "Freestanding shackle cradle - two zip ties",
        f"{stats['tilt_from_vertical_deg']:g} deg example | broad 150 x 130 mm base | bow on tank floor",
    )
    raster(
        "holder_only",
        [plastic],
        center + [-170, -230, 160],
        center,
        210,
        0,
        "Printed part - flat base, two tie channels",
        "6.2 x 2.8 mm tie channels | 0.7 mm saddle allowance | 0.5 mm edge treatment",
    )
    raster(
        "side",
        [plastic, (body, [0.74, 0.78, 0.82], True), (pin, [0.68, 0.72, 0.77], True), *tie_parts],
        center + [0, -300, 0.01],
        center,
        scale,
        0,
        "Side view - pin and both ears stay accessible",
        "Zero added shackle height | ties away from floor-contact point | tilt remains provisional",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tilt", type=float, default=75)
    parser.add_argument("--output-dir", type=Path, help="Override the angle-specific output directory")
    parser.add_argument("--no-render", action="store_true", help="Build and verify without preview images")
    args = parser.parse_args()
    if not 55 <= args.tilt <= 85:
        parser.error("Supported tilt range: 55..85 degrees from vertical")
    out = args.output_dir or HERE / "runs/shackle_tied_holder" / f"tilt_{args.tilt:g}deg"
    hm, body, pin, ties, stats = build(args.tilt, out)
    print(
        json.dumps(
            {
                k: stats[k]
                for k in ["dimensions_mm", "bow_floor_clearance_mm", "zip_tie_channels", "static_stability"]
            },
            indent=2,
        ),
        flush=True,
    )
    if not args.no_render:
        render(out, hm, body, pin, ties, stats)
    print(f"Finished: {out}", flush=True)


if __name__ == "__main__":
    main()
