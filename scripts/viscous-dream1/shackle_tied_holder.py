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

Run: uv run --script shackle_tied_holder.py --tilt 85
Angle is from vertical, with the head leaning toward the arm, bow toward/down.
Selected CAD: holder 85 degrees; cup 85 degrees, tip 80 mm above floor.
Full motion and physical print acceptance remain pending.
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
BASE_LENGTH = 146.0
BASE_WIDTH = 112.0
FIT_CLEARANCE = 0.7  # Forged curve is provisional; allow a dry fit before use.
EDGE_RADIUS = 0.5
SLOT_WIDTH = 6.2  # Intended for ties up to 4.8 x 1.5 mm, user size unconfirmed.
SLOT_HEIGHT = 2.8
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

    # A low elliptical foot flows into a continuous crescent, not two towers.
    def ellipse_ring(rx, ry, z, x=base_s):
        a = np.linspace(0, 2 * np.pi, 96, endpoint=False)
        return np.c_[x + rx * np.cos(a), ry * np.sin(a), np.full(96, z)]

    support = md.Manifold.hull_points(
        np.vstack(
            [
                ellipse_ring(BASE_LENGTH / 2 - 0.5, BASE_WIDTH / 2 - 0.5, 0),
                ellipse_ring(BASE_LENGTH / 2, BASE_WIDTH / 2, 0.5),
                ellipse_ring(BASE_LENGTH / 2 - 3, BASE_WIDTH / 2 - 3, 3.5),
                ellipse_ring(BASE_LENGTH / 2 - 10, BASE_WIDTH / 2 - 12, 6),
            ]
        )
    )
    # Each contiguous section follows the actual circular crown. A broad skirt
    # blends the raised lips into the foot; a full drop-in cut opens the centre.
    sections = []
    for delta in np.linspace(-68, 68, 35):
        a = np.radians(low_angle + delta)
        q = np.array([crown_x + crown_r * np.cos(a), 0, crown_r * np.sin(a)]) @ rotation.T + [0, 0, height]
        f = max(0, 1 - (abs(delta) / 76) ** 4)
        z = q[2] + 1
        pts = []
        for side in [-1, 1]:
            pts.extend(
                [
                    [q[0], side * 14 * f, z],
                    [q[0], side * 20 * f, z - 4],
                    [q[0], side * 38 * f, 3.5],
                    [q[0], side * 40 * f, 0.5],
                ]
            )
        sections.append(np.array(pts))
    support += md.Manifold.batch_boolean(
        [
            md.Manifold.hull_points(np.vstack([a, b]))
            for a, b in zip(sections[:-1], sections[1:], strict=True)
        ],
        md.OpType.Add,
    )

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

    # Original annular routing around stock, following the Prusa principle.
    # Local axial width follows the bow tangent; the U return follows the stock.
    channel_data, tie_solids = [], []
    center = np.array([crown_x, 0, 0]) @ rotation.T + [0, 0, height]
    for index, p in enumerate(stations, 1):
        outward = (p - center) / crown_r
        tangent = np.cross([0, 1, 0], outward)

        def arc_sweep(
            width, thickness, start, end, routing_radius=14.5, p=p, outward=outward, tangent=tangent
        ):
            section = md.CrossSection.square([thickness, width], center=True).translate([routing_radius, 0])
            frame = np.column_stack([outward, [0, 1, 0], -tangent])
            return section.revolve(128, end - start).rotate([0, 0, start]).transform(np.c_[frame, p])

        # Local continuous saddle shell supplies a real roof and outer wall
        # all along the return, blended into the crescent skirt.
        housing = arc_sweep(12, 8, -95, 95)
        holder += housing
        tunnel = arc_sweep(SLOT_WIDTH, SLOT_HEIGHT, -105, 105)
        # Tangential mouths continue into open air; rim rounding eases feeding.
        for sign in [-1, 1]:
            a = np.radians(sign * 100)
            v = np.cos(a) * outward + np.sin(a) * np.array([0, 1, 0])
            direction = -np.sin(a) * outward + np.cos(a) * np.array([0, 1, 0])
            mouth = np.array(
                [
                    p + (14.5 + dr) * v + dt * tangent
                    for dr in [-SLOT_HEIGHT / 2, SLOT_HEIGHT / 2]
                    for dt in [-SLOT_WIDTH / 2, SLOT_WIDTH / 2]
                ]
            )
            tunnel += md.Manifold.hull_points(np.vstack([mouth, mouth + sign * direction * 30]))
        holder -= tunnel
        tie = arc_sweep(TIE_WIDTH, TIE_THICKNESS, -180, 180)
        # Explicit conservative locking head at the accessible +Y mouth.
        head = chamfered_box([7, 7, 5], p + [0, 17.5, 2.5])
        holder -= chamfered_box([8, 8, 6], p + [0, 17.5, 2.5])
        tie_solids.extend([tie, head])
        channel_data.append(
            {
                "number": index,
                "stock_center_mm": p.tolist(),
                "stock_tangent": tangent.tolist(),
                "outward_radial": outward.tolist(),
                "clear_width_mm": SLOT_WIDTH,
                "clear_height_mm": SLOT_HEIGHT,
                "routing_centerline_radius_mm": 14.5,
                "buried_return_angle_deg": 210,
                "loop_length_mm": float(2 * np.pi * 14.5),
                "locking_head_center_mm": (p + [0, 17.5, 2.5]).tolist(),
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
    holder = holder.as_original().simplify(0.02)
    raw = holder.to_mesh64()
    for grid in [0.001, 0.0005, 0.002, 0.0002]:
        healed = md.Manifold(
            md.Mesh64(
                np.array(
                    np.round(raw.vert_properties[:, :3] / grid) * grid, dtype=np.float64, order="C", copy=True
                ),
                np.array(raw.tri_verts, dtype=np.uint64, order="C", copy=True),
            )
        )
        hm = mesh(healed)
        # Boolean edge rounding can leave zero-volume facets after welding.
        # Keep only a closed positive-volume component if it accounts for the
        # entire CAD volume; never fill holes or accept an open main surface.
        components = hm.split(only_watertight=True)
        substantial = [part for part in components if part.volume > 0.01]
        if len(substantial) == 1 and abs(substantial[0].volume - healed.volume()) < 0.01:
            hm = substantial[0]
            healed = solid(hm)
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
    # Positive material checks: a clear loop alone cannot prove a buried
    # route. Verify inner and outer walls throughout each return after rounding.
    wall_probe_count = 0
    for station in channel_data:
        p = np.array(station["stock_center_mm"])
        outward = np.array(station["outward_radial"])
        for angle in np.linspace(-60, 60, 7):
            v = np.cos(np.radians(angle)) * outward + np.sin(np.radians(angle)) * np.array([0, 1, 0])
            for radius in [12.2, 16.8]:
                probe = md.Manifold.sphere(0.2, 16).translate(p + radius * v)
                assert (holder ^ probe).volume() > 0.99 * probe.volume(), "Missing tie-channel wall"
                wall_probe_count += 1
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
    # Effective PETG material fraction; flooded voids are assumed.
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
            # Actual convex support polygon from coplanar bed triangles.
            from scipy.spatial import ConvexHull

            bed_vertices = hm.vertices[hm.vertices[:, 2] < 0.001, :2]
            hull = ConvexHull(bed_vertices)
            margins = -(hull.equations[:, :2] @ com[:2] + hull.equations[:, 2])
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
        "layout_status": "85 deg selected; full motion acceptance pending"
        if tilt == 85
        else "Alternative CAD angle",
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
        "complete_tie_loops_clear": 2,
        "channel_inner_and_outer_wall_probes_pass": wall_probe_count,
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
        "channel_axial_width_mm": SLOT_WIDTH,
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
    plastic = (hm, [0.10, 0.51, 0.74], True)
    tie_parts = [(m, [0.98, 0.45, 0.08], False) for m in ties]
    raster(
        "assembled",
        [plastic, (body, [0.74, 0.78, 0.82], True), (pin, [0.68, 0.72, 0.77], True), *tie_parts],
        center + [-170, -230, 160],
        center,
        scale,
        0,
        "Freestanding shackle cradle - two zip ties",
        f"{stats['tilt_from_vertical_deg']:g} deg from vertical | sculpted 146 x 112 mm foot | bow on tank floor",
    )
    raster(
        "holder_only",
        [plastic],
        center + [-170, -230, 160],
        center,
        210,
        0,
        "Printed part - continuous crescent cradle",
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
        "Zero added shackle height | ties away from floor-contact point | motion acceptance pending",
    )

    # Local section documents the real passage and remaining saddle wall.
    station = stats["zip_tie_channels"][0]
    p = np.array(station["stock_center_mm"])
    tangent = np.array(station["stock_tangent"])
    outward = np.array(station["outward_radial"])
    frame = np.column_stack([tangent, [0, 1, 0], outward])
    slab = box([2, 46, 46], [0, 0, 0]).transform(np.c_[frame, p])
    raster(
        "tie_section",
        [
            (mesh(solid(hm) ^ slab), [0.10, 0.51, 0.74], False),
            (mesh(solid(body) ^ slab), [0.74, 0.78, 0.82], True),
            (mesh(solid(ties[0]) ^ slab), [0.98, 0.45, 0.08], False),
        ],
        p + 100 * tangent,
        p,
        58,
        0,
        "Section through one tie route",
        "Blue: print | silver: stock | orange: loose tie envelope",
    )


def build_cup(
    out,
    do_render,
    tilt=85,
    tip_height=80,
    foot_offset=-48,
    tip_forward=500,
    shackle_forward=575,
    assembly=None,
    azimuth=15,
):
    """30 mm axial cup on a tapered pedestal; floor origin below pin tip.

    +X is away from arm, +Z up. Pin local +Z points tip to head.
    Print and OBJ share orientation; only units differ.
    """
    import shackle_assets as source
    from scipy.spatial import ConvexHull

    t = np.radians(tilt)
    rotation = np.array([[np.cos(t), 0, -np.sin(t)], [0, 1, 0], [np.sin(t), 0, np.cos(t)]])
    tip = np.array([0.0, 0.0, tip_height])
    axis = rotation[:, 2]
    transform = np.c_[rotation, tip]
    # Tube outer length remains exactly 30 mm; pedestal height is additional.
    profile = [
        (0, -3),
        (13.5, -3),
        (14, -2.5),
        (14, 26.5),
        (13.5, 27),
        (11.75, 27),
        (10.75, 26),
        (10.75, 0),
        (0, 0),
        (0, -3),
    ]
    tube = solid(source.lathe(profile)).transform(transform)
    angles = np.linspace(0, 2 * np.pi, 96, endpoint=False)

    def ring(x, z, rx, ry):
        return np.c_[x + rx * np.cos(angles), ry * np.sin(angles), np.full(96, z)]

    foot = md.Manifold.hull_points(
        np.vstack(
            [
                ring(foot_offset, 0, 47.5, 37.5),
                ring(foot_offset, 0.5, 48, 38),
                ring(foot_offset, 3, 47, 37),
                ring(foot_offset, 5, 42, 32),
            ]
        )
    )
    # Smooth tapered oval mast, widening into the foot and into the tube saddle.
    saddle = tip + axis * 12
    pedestal = md.Manifold.hull_points(
        np.vstack(
            [
                ring(foot_offset + 8, 3, 32, 27),
                ring(foot_offset + 10, 8, 27, 22),
                ring(saddle[0], max(9, saddle[2] - 17), 13, 11),
                ring(saddle[0], max(10, saddle[2] - 6), 13, 10),
            ]
        )
    )
    cup = tube + foot + pedestal
    bore = md.Manifold.cylinder(160, 10.75, circular_segments=128).transform(transform)
    lead = md.Manifold.cylinder(1, 10.75, 11.75, circular_segments=128)
    lead = lead.translate([0, 0, 26]).transform(transform)
    cup -= bore + lead
    # Backstop central drain exits the rear tube; vertical drain bypasses the
    # threaded tip via a bottom-side aperture and exits beneath the pedestal.
    rear = md.Manifold.cylinder(16, 3, circular_segments=48)
    rear = rear.translate([0, 0, -12]).transform(transform)
    bottom_center = tip + axis * 2
    bottom = md.Manifold.cylinder(tip_height - 7 if tilt > 45 else tip_height + 2, 2.5, circular_segments=48)
    bottom = bottom.translate([bottom_center[0], 0, -1])
    # Floor-level transverse exits prevent the tank floor sealing the drain.
    outlet = md.Manifold.cylinder(90, 2.5, circular_segments=48)
    outlet = outlet.rotate([90, 0, 0]).translate([bottom_center[0], 45, 1.5])
    drains = rear + bottom + outlet
    drain_material = {
        name: float((cup ^ cutter).volume())
        for name, cutter in [("rear", rear), ("bottom", bottom), ("floor_outlet", outlet)]
    }
    assert all(v > 1 for v in drain_material.values())
    removed = (cup ^ drains).volume()
    assert removed > 10, "Drain cutter must remove actual material"
    cup -= drains
    # Blend a clearance cove into the mast below the approaching open fingers.
    relief = md.Manifold.cylinder(150, 31, circular_segments=96)
    relief = relief.translate([0, 0, 32.7]).transform(transform)
    finger_relief = box([44, 164, 86], [0, 0, 32.7 + 43]).transform(transform)
    cup -= relief + finger_relief
    # R0.5 convex edges, including drain mouths. Restore chamfered bed strip.
    kernel = md.Manifold.sphere(0.52, 16)
    bed = cup ^ box([220, 160, 0.5], [foot_offset, 0, 0.25])
    cup = cup.minkowski_difference(kernel).minkowski_sum(kernel) + bed
    cup = cup.simplify(0.002)
    # STL uses float32 vertices: weld numerical slivers before export and
    # verify the actual float32 round trip, not just the double CAD mesh.
    import io

    raw = cup.to_mesh64()
    for grid in [0.0001, 0.0005, 0.001, 0.002]:
        healed = md.Manifold(
            md.Mesh64(
                np.array(np.round(raw.vert_properties[:, :3] / grid) * grid, dtype=np.float64, order="C"),
                np.array(raw.tri_verts, dtype=np.uint64, order="C"),
            )
        )
        candidate = mesh(healed)
        parts = [m for m in candidate.split(only_watertight=True) if m.volume > 0.01]
        if len(parts) != 1 or abs(parts[0].volume - healed.volume()) > 0.01:
            continue
        candidate = parts[0]
        # Preserve the exact stop plane during vertex welding. Rounding an
        # oblique seated contact plane otherwise creates artificial penetration.
        axial = (candidate.vertices - tip) @ axis
        on_stop = np.abs(axial) < grid * 2
        candidate.vertices[on_stop] -= axial[on_stop, None] * axis
        exported = trimesh.load(io.BytesIO(candidate.export(file_type="stl")), file_type="stl", force="mesh")
        if exported.is_watertight and len(exported.split()) == 1:
            cup, cm = solid(candidate), candidate
            break
    else:
        raise AssertionError("Cup STL precision healing failed")
    assert cm.is_watertight and cm.is_winding_consistent and len(cm.split()) == 1
    assert abs(cm.bounds[0, 2]) < 1e-6
    pin = trimesh.load(ASSETS / "shackle_pin.stl", force="mesh")
    pin.apply_scale(1000)
    pin.apply_translation([0, 0, -pin.bounds[0, 2]])
    pin.vertices = pin.vertices @ rotation.T + tip
    ps = solid(pin)
    distances = np.linspace(0, 120, 61)
    intersections = [(cup ^ ps.translate(axis * d)).volume() for d in distances]
    assert max(intersections) < 1e-5, f"Pin intersections: {intersections[:3]}"
    # TCP/grip reference at tip+68; furthest-forward fingertip is 34.3 mm
    # toward tip. Conservative enclosing cylinder begins at 33.7, not 40.
    corridor = md.Manifold.cylinder(150, 30, circular_segments=96)
    corridor = corridor.translate([0, 0, 68 - 34.3]).transform(transform)
    corridor_overlap = (cup ^ corridor).volume()
    assert corridor_overlap < 1e-5
    drain_checks = {}
    for name, cutter in [("rear", rear), ("bottom", bottom), ("floor_outlet", outlet)]:
        v = (cup ^ cutter).volume()
        # Simplified triangle chords can cross the nominal tool by microns.
        # Prove a continuous 0.05 mm inset passage independently of that skin.
        core = cutter.minkowski_difference(md.Manifold.sphere(0.05, 8))
        core_overlap = (cup ^ core).volume()
        assert core_overlap < 1e-5
        assert v < cutter.surface_area() * 0.002, f"Drain {name}: nominal skin residual {v} mm3"
        drain_checks[name] = {
            "nominal_tool_residual_mm3": v,
            "inset_0_05mm_core_intersection_mm3": core_overlap,
        }
    fingers = box([42, 162, 84.3], [0, 0, 33.7 + 84.3 / 2]).transform(transform)
    palm = md.Manifold.cylinder(60, np.hypot(80, 4), circular_segments=128)
    palm = palm.translate([0, 0, 118]).transform(transform)
    wide_envelope = fingers + palm
    wide_overlap = (cup ^ wide_envelope).volume()
    assert wide_overlap < 1e-5, f"Wide pickup envelope intersects cup: {wide_overlap}"
    cm.units = "mm"
    for ext in ["stl", "3mf"]:
        path = out / f"pin_cup.{ext}"
        cm.export(path)
        m = trimesh.load(path, force="mesh")
        assert m.is_watertight and len(m.split()) == 1
        assert np.max(np.abs(m.bounds - cm.bounds)) < 1e-4
    with zipfile.ZipFile(out / "pin_cup.3mf") as archive:
        assert ET.fromstring(archive.read("3D/3dmodel.model")).attrib["unit"] == "millimeter"
    for name, original in [
        ("pin_cup_floor_frame", cm),
        ("pin_seated_in_cup_floor_frame", pin),
        ("pin_pickup_corridor_floor_frame", mesh(corridor)),
        ("pin_pickup_wide_envelope_floor_frame", mesh(wide_envelope)),
    ]:
        m = original.copy()
        m.apply_scale(0.001)
        path = out / f"{name}.obj"
        path.write_text(trimesh.exchange.obj.export_obj(m).rstrip() + "\n")
        loaded = trimesh.load(path, force="mesh")
        assert np.max(np.abs(loaded.bounds * 1000 - original.bounds)) < 1e-4
    hull = ConvexHull(cm.vertices[cm.vertices[:, 2] < 0.001, :2])
    stability = {}
    for wet in [False, True]:
        for with_pin in [False, True]:
            masses = np.array([pin.volume * 7.9e-6 if with_pin else 0, cm.volume * 1.27e-6 * 0.4])
            if wet:
                masses *= [1 - 1 / 7.9, 1 - 1 / 1.27]
            com = np.average([pin.center_mass, cm.center_mass], axis=0, weights=masses)
            margin = -(hull.equations[:, :2] @ com[:2] + hull.equations[:, 2]).max()
            assert margin > 0
            stability[f"{'wet' if wet else 'dry'}_{'with_pin' if with_pin else 'empty'}"] = {
                "combined_com_mm": com.tolist(),
                "support_margin_mm": float(margin),
                "effective_weight_N": float(masses.sum() * 9.81),
                "minimum_gravity_restoring_moment_Nm": float(margin * 0.001 * masses.sum() * 9.81),
            }
    yaw = np.radians(azimuth)
    floor_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    floor_origin = np.array([tip_forward * np.cos(yaw), tip_forward * np.sin(yaw), 0])
    layout = {
        "cup_tip_radial_mm": tip_forward,
        "cup_azimuth_deg": azimuth,
        "shackle_bore_forward_mm": shackle_forward,
        "placement_convention": "world forward/lateral/up; positive lateral is toward sim +X",
        "global_placement_exported": False,
        "suggested_floor_to_world_transform_mm": np.vstack(
            [np.c_[floor_rotation, floor_origin], [0, 0, 0, 1]]
        ).tolist(),
    }
    if assembly:
        hm, body, _, ties, stats = assembly
        shift = (np.array([shackle_forward, 0, 0]) - floor_origin) @ floor_rotation
        assembly = tuple(m.copy() for m in [hm, body])
        hm, body = assembly
        hm.vertices = hm.vertices @ floor_rotation
        body.vertices = body.vertices @ floor_rotation
        layout["shackle_tilt_deg"] = stats["tilt_from_vertical_deg"]
        layout["holder_overlap_mm3"] = float((cup ^ solid(hm).translate(shift)).volume())
        layout["shackle_overlap_mm3"] = float((cup ^ solid(body).translate(shift)).volume())
        layout["pin_holder_overlap_mm3"] = float((ps ^ solid(hm).translate(shift)).volume())
        layout["holder_bounds_cup_floor_frame_mm"] = (hm.bounds + shift).tolist()
        layout["cup_bounds_cup_floor_frame_mm"] = cm.bounds.tolist()
        layout["clash"] = any(
            layout[k] > 1e-5 for k in ["holder_overlap_mm3", "shackle_overlap_mm3", "pin_holder_overlap_mm3"]
        )
    data = {
        "units": "mm",
        "cup_axial_length_mm": 30,
        "insertion_depth_mm": 27,
        "bore_diameter_mm": 21.5,
        "lead_in_diameter_mm": 23.5,
        "floor_thickness_mm": 3,
        "cup_tilt_from_vertical_deg": tilt,
        "pin_tip_height_mm": tip_height,
        "pin_tip_floor_frame_mm": tip.tolist(),
        "extraction_axis_floor_frame": axis.tolist(),
        "foot_center_mm": [foot_offset, 0, 0],
        "foot_nominal_dimensions_mm": [96, 76],
        "dimensions_mm": cm.extents.tolist(),
        "pin_tip_to_floor_transform_mm": np.vstack([transform, [0, 0, 0, 1]]).tolist(),
        "floor_to_pin_tip_transform_mm": np.linalg.inv(np.vstack([transform, [0, 0, 0, 1]])).tolist(),
        "pin_tip_to_foot_center_mm": [foot_offset, 0, -tip_height],
        "pin_axis_tip_to_head": axis.tolist(),
        "pin_bounds_mm": pin.bounds.tolist(),
        "pickup_assumption": {
            "grip_center_from_tip_mm": 68,
            "grip_center_floor_mm": (tip + axis * 68).tolist(),
            "fingertip_forward_offset_mm": 34.3,
            "corridor_start_from_tip_mm": 33.7,
            "lip_clearance_mm": 6.7,
            "corridor_diameter_mm": 60,
            "corridor_length_mm": 150,
            "cup_intersection_mm3": corridor_overlap,
            "wide_fingers_and_swept_palm_intersection_mm3": wide_overlap,
            "wide_finger_box_mm": [42, 162, 84.3],
            "palm_sweep_radius_mm": float(np.hypot(80, 4)),
            "palm_axial_interval_from_tip_mm": [118, 178],
        },
        "drain_removed_material_mm3": removed,
        "drain_material_removed_by_tool_mm3": drain_material,
        "mesh_weld_grid_mm": grid,
        "estimated_cup_mass_kg_40pct_petg": float(cm.volume * 1.27e-6 * 0.4),
        "modeled_pin_mass_kg": float(pin.volume * 7.9e-6),
        "drain_clear_intersections_mm3": drain_checks,
        "extraction_samples_mm": distances.tolist(),
        "intersection_volumes_mm3": intersections,
        "single_watertight_solid": True,
        "export_roundtrips_pass": True,
        "static_stability": stability,
        "illustrative_layout": layout,
        "generator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scene_frame": "metres; origin floor below pin tip; +X away from arm, +Y across, +Z up",
        "status": "Selected 85/80 pose; full motion, wet sliding, print supports and drainage trial unverified",
    }
    (out / "cup_checks.json").write_text(json.dumps(data, indent=2) + "\n")
    if do_render:
        import shackle_holder as renderer

        renderer.OUT = out
        parts = [(cm, [0.10, 0.51, 0.74], True), (pin, [0.74, 0.78, 0.82], True)]
        center = np.array([-35, 0, tip_height / 2])
        renderer.render(
            "cup_with_pin",
            parts,
            center + [150, -240, 140],
            center,
            190,
            0,
            "Raised pin pickup cup - head toward arm",
            f"30 mm axial tube | {tilt:g} deg tilt | tip {tip_height:g} mm above floor",
        )
        renderer.render(
            "cup_side",
            parts,
            center + [0, -300, 0],
            center,
            190,
            0,
            "Pickup cup - side view",
            "96 x 76 mm rear-biased foot | 27 mm insertion | 6.7 mm finger gap",
        )
        if assembly:
            combined = parts + [
                (m.copy(), color, True) for m, color in [(hm, [0.10, 0.51, 0.74]), (body, [0.74, 0.78, 0.82])]
            ]
            for m, _, _ in combined[2:]:
                m.apply_translation(shift)
            for tie_mesh in ties:
                placed_tie = tie_mesh.copy()
                placed_tie.vertices = placed_tie.vertices @ floor_rotation + shift
                combined.append((placed_tie, [0.98, 0.45, 0.08], False))
            bounds = np.array([m.bounds for m, _, _ in combined])
            center = (bounds[:, 0].min(axis=0) + bounds[:, 1].max(axis=0)) / 2
            renderer.render(
                "cup_holder_layout",
                combined,
                center + [-140, -320, 180],
                center,
                360,
                0,
                "Selected off-line pickup and insertion station",
                f"Tip {tip_forward:g} mm | bore {shackle_forward:g} mm | clash: {layout['clash']}",
            )
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tilt", type=float, default=85)
    parser.add_argument("--output-dir", type=Path, help="Override the angle-specific output directory")
    parser.add_argument("--no-render", action="store_true", help="Build and verify without preview images")
    parser.add_argument("--cup-tilt", type=float, default=85)
    parser.add_argument("--pin-tip-height", type=float, default=80)
    parser.add_argument("--cup-foot-offset", type=float, default=-48)
    parser.add_argument("--cup-tip-forward", type=float, default=500)
    parser.add_argument("--cup-azimuth", type=float, default=15)
    parser.add_argument("--shackle-forward", type=float, default=575)
    args = parser.parse_args()
    if not 0 <= args.cup_tilt <= 90 or args.pin_tip_height < 30:
        parser.error("Cup tilt 0..90 degrees; raised tip height >=30 mm")
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
    build_cup(
        out,
        not args.no_render,
        args.cup_tilt,
        args.pin_tip_height,
        args.cup_foot_offset,
        args.cup_tip_forward,
        args.shackle_forward,
        (hm, body, pin, ties, stats),
        args.cup_azimuth,
    )
    if not args.no_render:
        render(out, hm, body, pin, ties, stats)
    print(f"Finished: {out}", flush=True)


if __name__ == "__main__":
    main()
