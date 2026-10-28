"""Gavin's glue-on triangular shackle cradle. Local geometry only; no sim/API calls.

Run with PYTHONDONTWRITEBYTECODE=1 ~/dreamtrial/.venv/bin/python shackle_holder.py
Reads existing meshes once, never imports/runs shackle_assets.py. All model work is
in mm. STL/3MF are translated to the print bed; the OBJ retains the body frame in m.
"""
from pathlib import Path
import hashlib
import io
import json
import os
import sys
import zipfile
import xml.etree.ElementTree as ET

# Optional render dependencies live inside the allowed output directory. Keep
# matplotlib's font/cache writes there too, rather than in the user's home.
_RUN = Path(__file__).resolve().parent / 'runs/shackle_holder'
sys.path.insert(0, str(_RUN/'_deps'))
os.environ['MPLCONFIGDIR'] = str(_RUN/'_mplconfig')
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import manifold3d as md
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import trimesh

# DESIGN INPUTS, mm unless stated. These are the blue sketch's untrimmed corners.
TRIANGLE_LEFT_X = -89.0
TRIANGLE_TOE_X = -60.5
TRIANGLE_TOP_Z = -28.0
TABLE_Z_OVERRIDE = None  # None: actual body-mesh minimum Z, approximately -49.2.
GLUE_GAP = 0.5
DEPTH_Y = 40.0
SADDLE_WRAP_HALF_ANGLE_DEG = 35.0  # outer radial direction, both sides of Y=0
TOP_TIP_TRUNCATION = 5.5  # avoids a sub-mm wall at the upper cavity mouth
TOE_TRUNCATION = 7.5      # avoids a sub-mm floor at the sketch's sharp toe
MIN_WALL = 2.5
CORNER_RADIUS = 1.0      # vertical outside corners, in the XY footprint
EDGE_CHAMFER = 0.5      # all handling edges: equivalent R0.5 rounds above the bed
BOTTOM_CHAMFER = EDGE_CHAMFER  # 45-degree anti-elephant's-foot edge treatment
EDGE_ROUND_SECTIONS = 48
BOTTOM_EDGE_ZONE = 3*EDGE_CHAMFER  # retain the printable chamfer, not a bed fillet
PRINT_LAYER_HEIGHT = 0.2
EDGE_AUDIT_MIN_FEATURE = PRINT_LAYER_HEIGHT/2  # half a print layer, below R0.5 features
EDGE_AUDIT_SHARP_ANGLE_DEG = 20.0  # exceeds smooth-surface tessellation turns
EDGE_AUDIT_DISTANCE_TOLERANCE = 0.005
OFFSET_SPHERE_SECTIONS = 48
BOOLEAN_CLEANUP_TOLERANCE = 0.001  # eliminate sub-micron boolean slivers before STL
EXPORT_VERTEX_GRID = 0.0001       # mm; robust to STL float32 and OBJ unit conversion
EXPORT_VERTEX_GRID_MAX = 0.001   # fallback for collapses of zero-width boolean fins
PROFILE_CIRCLE_SECTIONS = 256
STEEL_DENSITY_G_CM3 = 7.9
GRAVITY = 9.81
DOWNWARD_PIN_FORCE_N = 20.0
SCREW_TORQUE_NM = 2.0
FIT_SAMPLE_COUNT = 12000
RANDOM_SEED = 291294295

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets/shackle"
OUT = HERE / "runs/shackle_holder"


def log(s):
    print(s, flush=True)


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-15)


def solid(mesh):
    return md.Manifold(md.Mesh64(np.asarray(mesh.vertices, dtype=np.float64),
                                 np.asarray(mesh.faces, dtype=np.uint64)))


def mesh_of(manifold):
    m = manifold.to_mesh64()
    return trimesh.Trimesh(m.vert_properties[:, :3], m.tri_verts, process=True)


def clean_mesh(manifold, table):
    raw = manifold.as_original().simplify(BOOLEAN_CLEANUP_TOLERANCE).to_mesh64()
    # Quantise BEFORE merging vertices: merging submicron boolean slivers first
    # can destroy the combinatorial surface before degenerate faces are removed.
    for grid in [EXPORT_VERTEX_GRID, .00005, .00002, .00001, .0002, .0005, EXPORT_VERTEX_GRID_MAX]:
        verts = np.round(raw.vert_properties[:, :3]/grid)*grid
        verts[np.abs(verts[:, 2]-table) < grid, 2] = table
        # Let the manifold constructor collapse zero-length edges while retaining
        # their neighbouring triangles' connectivity. Dropping zero-area faces
        # in a triangle soup instead can leave a collinear triangular hole.
        healed = md.Manifold(md.Mesh64(np.array(verts, dtype=np.float64, order='C', copy=True),
            np.array(raw.tri_verts, dtype=np.uint64, order='C', copy=True))).as_original().set_tolerance(
            BOOLEAN_CLEANUP_TOLERANCE)
        if healed.is_empty():
            continue
        hm = mesh_of(healed)
        if hm.is_watertight and hm.is_winding_consistent and hm.euler_number == 2:
            hm.metadata['vertex_grid_mm'] = grid
            log(f'Export topology cleaned on {grid:.5f} mm grid')
            return hm
    np.savez_compressed(OUT/'edge_cleanup_debug.npz', vertices=raw.vert_properties[:, :3],
                        faces=raw.tri_verts)
    raise AssertionError('Edge cleanup changed topology; see edge_cleanup_debug.npz')


def edge_inventory(mesh):
    """Physical creases from topology + adjacent face normals, not triangle lines.

    Faceted curves have small normal turns. Quantisation can also produce tall
    skinny slivers or submicron edges with unreliable normals; require both edge
    length and adjacent triangle altitudes >0.1 mm for this crease audit (half
    the suggested print layer). Smaller facets cannot form a printed handling
    edge; they approximate the rolling-ball surface.
    The rounding construction itself processes ALL convex edges, without this
    filter; the filter only makes the subsequent diagnostic meaningful.
    """
    edges = mesh.face_adjacency_edges
    length = np.linalg.norm(mesh.vertices[edges[:, 1]]-mesh.vertices[edges[:, 0]], axis=1)
    altitude = 2*mesh.area_faces[mesh.face_adjacency]/np.maximum(length[:, None], 1e-15)
    resolved = (length > EDGE_AUDIT_MIN_FEATURE) & (altitude.min(axis=1) > EDGE_AUDIT_MIN_FEATURE)
    angles = np.rad2deg(mesh.face_adjacency_angles)
    centers = mesh.vertices[edges].mean(axis=1)
    return edges, angles, centers, resolved


def soften_edges(before, local, table, gap_upper):
    """Rolling-ball opening rounds every convex handling edge simultaneously.

    Erosion then dilation by the same R0.5 kernel leaves planes and the concave
    cradle interior in place, while rounding convex rims, top and side edges.
    Existing R1 outline corners already exceed the requested treatment. Restore
    only the lower strip, whose outside edges already have a 0.5 mm 45-degree
    chamfer; rounding a bed edge would make a support-unfriendly lower fillet.
    """
    sphere = md.Manifold.sphere(1, EDGE_ROUND_SECTIONS)
    sm = mesh_of(sphere)
    inradius = np.min(np.einsum('ij,ij->i', sm.face_normals, sm.triangles_center))
    kernel = sphere.scale([EDGE_CHAMFER/inradius]*3)
    base = solid(before)
    log('Rounding all upper edges and cradle rims with the 0.5 mm rolling-ball kernel')
    opened = base.minkowski_difference(kernel).minkowski_sum(kernel)
    low = md.Manifold.cube([before.extents[0]+2, before.extents[1]+2, BOTTOM_EDGE_ZONE]).translate(
        [before.bounds[0, 0]-1, before.bounds[0, 1]-1, table])
    after = clean_mesh(opened+(base ^ low), table)

    e, ang, center, valid = edge_inventory(before)
    upper = center[:, 2] > table+BOTTOM_EDGE_ZONE+1e-4
    sharp = valid & upper & (ang > EDGE_AUDIT_SHARP_ANGLE_DEG)
    # Check removal along every original feature segment, including its junctions.
    # Save the two-infinite-plane reference depth for comparison; finite faces
    # on a curved rim need not have that exact depth. Radius comes from the
    # rolling-ball construction, and the post-audit checks no crease survives.
    selected = before.vertices[e[sharp]]
    samples = np.concatenate([(1-t)*selected[:, 0]+t*selected[:, 1]
                              for t in [0, .25, .5, .75, 1]])
    clearance = point_distances(after, samples)
    target = np.tile(EDGE_CHAMFER*(1/np.cos(np.deg2rad(ang[sharp])/2)-1), 5)
    unbroken = clearance < EDGE_AUDIT_DISTANCE_TOLERANCE
    if unbroken.any():
        np.savez_compressed(OUT/'edge_audit_debug.npz', before_v=before.vertices, before_f=before.faces,
            after_v=after.vertices, after_f=after.faces, samples=samples, clearance=clearance,
            target=target, selected_edges=e[sharp], angles=ang[sharp],
            convex=before.face_adjacency_convex[sharp])
    assert not unbroken.any(), f'{unbroken.sum()} original edge samples were not rounded'
    ae, aa, ac, av = edge_inventory(after)
    au = ac[:, 2] > table+BOTTOM_EDGE_ZONE+1e-4
    remaining = av & au & (aa > EDGE_AUDIT_SHARP_ANGLE_DEG)
    if remaining.any():
        np.savez_compressed(OUT/'edge_post_audit_debug.npz', vertices=after.vertices,
            faces=after.faces, edges=ae[remaining], angles=aa[remaining])
    assert not remaining.any(), f'{remaining.sum()} sharp upper feature edges remain'

    # Preserve the actual mesh-derived cradle away from the intentionally broken
    # rim. Its boundary is found from the open contact-patch triangle topology.
    closest, d, _ = trimesh.proximity.closest_point(local, before.triangles_center)
    align = np.einsum('ij,ij->i', before.face_normals, unit(closest-before.triangles_center))
    ids = (d >= GLUE_GAP-.002) & (d < gap_upper+.003) & (align > .94)
    patch = before.submesh([np.flatnonzero(ids)], append=True, repair=False)
    unique, count = np.unique(patch.edges_sorted, axis=0, return_counts=True)
    borders = patch.vertices[unique[count == 1]]
    probes, _ = trimesh.sample.sample_surface(patch, FIT_SAMPLE_COUNT, seed=RANDOM_SEED)
    distance_to_rim = []
    start, vec = borders[:, 0], borders[:, 1]-borders[:, 0]
    length_sq = np.sum(vec*vec, axis=1)
    for block in np.array_split(probes, 24):
        delta = block[:, None]-start
        t = np.clip(np.sum(delta*vec, axis=2)/length_sq, 0, 1)
        distance_to_rim.append(np.linalg.norm(delta-t[:, :, None]*vec, axis=2).min(axis=1))
    protected = probes[np.concatenate(distance_to_rim) >= 2*EDGE_CHAMFER]
    deviation = point_distances(after, protected)
    assert len(protected) > 1000
    assert deviation.max() < EDGE_AUDIT_DISTANCE_TOLERANCE
    core_gap = point_distances(local, protected)
    final_core, _, _ = trimesh.proximity.closest_point(after, protected)
    final_core_gap = point_distances(local, final_core)
    bed_faces = np.all(after.triangles[:, :, 2] < table+1e-5, axis=1)
    bed = after.vertices[np.unique(after.faces[bed_faces])]
    insets = [bed[:, 0].min()-before.bounds[0, 0], before.bounds[1, 0]-bed[:, 0].max(),
              bed[:, 1].min()-before.bounds[0, 1], before.bounds[1, 1]-bed[:, 1].max()]
    assert min(insets) >= EDGE_CHAMFER-1e-4
    audit = {
        'method': 'R0.5 rolling-ball morphological opening of all convex edges; R1 outline rounds retained; lower strip retains 0.5 mm 45-degree chamfer.',
        'nominal_edge_round_radius_mm': EDGE_CHAMFER,
        'export_vertex_grid_mm': after.metadata['vertex_grid_mm'],
        'round_kernel_radius_bounds_mm': [EDGE_CHAMFER, EDGE_CHAMFER/inradius],
        'bottom_chamfer_mm': BOTTOM_CHAMFER,
        'bottom_face_insets_mm': insets,
        'upper_original_feature_segments': int(sharp.sum()),
        'original_feature_samples': len(samples),
        'original_feature_samples_left_unbroken': int(unbroken.sum()),
        'minimum_old_feature_clearance_mm': float(clearance.min()),
        'remaining_sharp_upper_feature_segments': int(remaining.sum()),
        'maximum_resolved_upper_normal_turn_deg': float(aa[av & au].max()),
        'maximum_resolved_chamfer_normal_turn_deg': float(aa[av & ~au].max()),
        'feature_detection_angle_deg': EDGE_AUDIT_SHARP_ANGLE_DEG,
        'feature_resolution_mm': EDGE_AUDIT_MIN_FEATURE,
        'cradle_core_samples': len(protected),
        'cradle_core_exclusion_distance_from_rim_mm': 2*EDGE_CHAMFER,
        'cradle_core_max_surface_deviation_mm': float(deviation.max()),
        'cradle_core_original_gap_range_mm': [float(core_gap.min()), float(core_gap.max())],
        'cradle_core_final_gap_range_mm': [float(final_core_gap.min()), float(final_core_gap.max())],
        'note': 'New chamfer boundaries are 135-degree included transitions; small facets approximate rounds. There are no untreated original handling creases. No chamfer is applied across the functional cradle core.',
    }
    np.savez_compressed(OUT/'edge_audit_samples.npz', original_edge_points_mm=samples,
        original_edge_removed_depth_mm=clearance, expected_round_depth_mm=target,
        protected_cradle_points_mm=protected, protected_cradle_surface_deviation_mm=deviation)
    return after, audit


def rounded_rect(x0, x1, y0, y1, radius, sections=12):
    pts = []
    for cx, cy, start in [(x1-radius, y1-radius, 0),
                           (x0+radius, y1-radius, 90),
                           (x0+radius, y0+radius, 180),
                           (x1-radius, y0+radius, 270)]:
        a = np.deg2rad(np.linspace(start, start+90, sections+1))
        pts.extend(np.c_[cx+radius*np.cos(a), cy+radius*np.sin(a)])
    return np.asarray(pts)


def build(body, dims, table):
    """Boolean Euclidean dilation, vertical sweep, and subtraction.

    The circumscribed polyhedral sphere contains the exact glue-gap sphere.
    Its small excess is measured and reported. The vertical Minkowski sweep
    clears every higher pose during insertion, rather than leaving an undercut.
    Only a generously padded local piece is dilated; clip faces are outside the
    holder and cannot create the seat. No shackle curve is reconstructed here.
    """
    x0, x1 = TRIANGLE_LEFT_X, TRIANGLE_TOE_X-TOE_TRUNCATION
    top = TRIANGLE_TOP_Z-TOP_TIP_TRUNCATION
    footprint = rounded_rect(x0, x1, -DEPTH_Y/2, DEPTH_Y/2, CORNER_RADIUS)
    lower = rounded_rect(x0+BOTTOM_CHAMFER, x1-BOTTOM_CHAMFER,
                         -DEPTH_Y/2+BOTTOM_CHAMFER, DEPTH_Y/2-BOTTOM_CHAMFER,
                         CORNER_RADIUS-BOTTOM_CHAMFER)
    blank = md.Manifold.hull_points(np.vstack([
        np.c_[lower, np.full(len(lower), table)],
        np.c_[footprint, np.full(len(footprint), table+BOTTOM_CHAMFER)],
        np.c_[footprint, np.full(len(footprint), top)]]))
    triangle = md.CrossSection([np.array([[TRIANGLE_LEFT_X, table],
        [TRIANGLE_TOE_X, table], [TRIANGLE_LEFT_X, TRIANGLE_TOP_Z]])])
    wedge = md.Manifold.extrude(triangle, DEPTH_Y+4).rotate([90, 0, 0]).translate([0, DEPTH_Y/2+2, 0])
    blank = blank ^ wedge

    # Limit wrap independently of subtraction. This cap only removes shoulders;
    # the seating surface itself is always derived from the actual body mesh.
    crown_cx = dims['BOW_CROWN_CENTER_X']['value']
    crown_r = dims['BOW_CROWN_RADIUS']['value']
    rod_r = dims['BOW_ROD_DIAMETER']['value']/2
    cap_r = crown_r+(rod_r+GLUE_GAP)*np.cos(np.deg2rad(SADDLE_WRAP_HALF_ANGLE_DEG))
    cap = md.Manifold.cylinder(DEPTH_Y+8, cap_r, cap_r,
        PROFILE_CIRCLE_SECTIONS, True).rotate([90, 0, 0]).translate([crown_cx, 0, 0])
    blank = blank-cap

    pad = 6.0
    clip_low = [x0-pad, -DEPTH_Y/2-pad, table-pad]
    clip_size = [x1-x0+2*pad, DEPTH_Y+2*pad, top-table+pad+14]
    local = solid(body) ^ md.Manifold.cube(clip_size).translate(clip_low)
    sphere = md.Manifold.sphere(1.0, OFFSET_SPHERE_SECTIONS)
    sphere_mesh = mesh_of(sphere)
    inradius = np.min(np.einsum('ij,ij->i', sphere_mesh.face_normals,
                                sphere_mesh.triangles_center))
    dilation_radius = (GLUE_GAP+BOOLEAN_CLEANUP_TOLERANCE)/inradius
    log(f'Dilating {local.num_tri()} local bow triangles; sphere excess {dilation_radius-GLUE_GAP:.5f} mm')
    offset = local.minkowski_sum(sphere.scale([dilation_radius]*3))
    swept = offset.minkowski_sum(md.Manifold.cube([1e-5, 1e-5, 100.0]))
    hm = clean_mesh(blank-swept, table)
    gap_upper = dilation_radius+BOOLEAN_CLEANUP_TOLERANCE+EXPORT_VERTEX_GRID_MAX+2e-5
    hm, edge_audit = soften_edges(hm, mesh_of(local), table, gap_upper)
    holder = solid(hm)
    return holder, hm, mesh_of(local), footprint, {
        'offset_sphere_inradius': float(inradius),
        'offset_bound_mm': [GLUE_GAP, gap_upper],
        'wrap_cap_radius_mm': cap_r,
        'tip_top_z_mm': top,
        'toe_x_mm': x1,
        'edge_treatment': edge_audit,
    }


def point_distances(mesh, pts):
    distances = []
    for start in range(0, len(pts), 1000):
        distances.append(trimesh.proximity.closest_point(mesh, pts[start:start+1000])[1])
    return np.concatenate(distances)


def checks(holder, hm, local, body, pin, footprint, table, dims, settings):
    bm = solid(body)
    assert hm.is_watertight and hm.is_winding_consistent and hm.volume > 0
    assert len(holder.decompose()) == 1
    intersection = (holder ^ bm).volume()
    minimum_gap = holder.min_gap(bm, 2.0)
    # Identify the mesh-derived cavity, separating it from the surrounding flat
    # triangle/rails. Use proximity AND orientation, not proximity alone.
    q, d, _ = trimesh.proximity.closest_point(local, hm.triangles_center)
    align = np.einsum('ij,ij->i', hm.face_normals, unit(q-hm.triangles_center))
    select = ((d >= GLUE_GAP-0.002) & (d < settings['offset_bound_mm'][1]+0.003)
              & (align > 0.94))
    vertex_gap = point_distances(local, hm.vertices)
    # Do not count a rounded transition triangle as a full-fit seat merely
    # because its centroid is near the seat: all its vertices must also fit.
    select &= vertex_gap[hm.faces].max(axis=1) <= settings['offset_bound_mm'][1]
    patch = hm.submesh([np.flatnonzero(select)], append=True, repair=False)
    rng = np.random.default_rng(RANDOM_SEED)
    samples, _ = trimesh.sample.sample_surface(patch, FIT_SAMPLE_COUNT, seed=rng)
    samples = np.vstack([samples, patch.vertices, patch.triangles_center])
    gap_samples = point_distances(local, samples)
    contact_area = patch.area
    contact_center = np.average(patch.triangles_center, weights=patch.area_faces, axis=0)
    # Area integrals use uniformly sampled triangle interiors; enough for glue
    # stress estimates, NOT material allowables or a validated adhesive model.
    glue_pts, _ = trimesh.sample.sample_surface(patch, 60000, seed=RANDOM_SEED)
    xy = glue_pts[:, :2]-contact_center[:2]
    patch_j = contact_area*np.mean(np.sum(xy**2, axis=1))
    patch_rmax = np.linalg.norm(patch.vertices[:, :2]-contact_center[:2], axis=1).max()
    wall_lengths = {}
    ray_start = EXPORT_VERTEX_GRID_MAX
    for name, direction in [('floor', [0, 0, -1]), ('rear', [-1, 0, 0]), ('side', None)]:
        directions = np.tile(direction if direction is not None else [0, 1, 0],
                             (len(patch.vertices), 1)).astype(float)
        if direction is None:
            directions[:, 1] = np.where(patch.vertices[:, 1] < 0, -1, 1)
        origins = patch.vertices+ray_start*directions
        hits, ids, _ = hm.ray.intersects_location(origins, directions, multiple_hits=False)
        assert len(ids) == len(origins)
        wall_lengths[name] = float((np.linalg.norm(hits-origins[ids], axis=1)+ray_start).min())
    floor, rear_wall, side_wall = (wall_lengths[n] for n in ['floor', 'rear', 'side'])
    # Floor, rear wall and lateral rails still meet the wall requirement after edge rounding.
    assert min(floor, rear_wall, side_wall) >= MIN_WALL-0.02
    lowest = np.all(hm.triangles[:, :, 2] < table+1e-5, axis=1)
    down = (hm.face_normals[:, 2] < -1e-6) & ~lowest
    overhang = np.rad2deg(np.arcsin(np.clip(-hm.face_normals[down, 2], 0, 1)))
    assert overhang.max(initial=0) <= 45.1
    drop_checks = {str(dz): float((holder ^ bm.translate([0, 0, dz])).volume())
                   for dz in [0.25, 0.5, 1, 2, 5, 10, 20, 50, 100]}
    bore_r = dims['CLEAR_BORE_DIAMETER']['value']/2
    # Entire infinite vertical bore cylinder is clear, a stronger test than
    # checking only the portions above the top/below the bottom ear.
    keepout = md.Manifold.cylinder(300, bore_r, bore_r, 128, True)
    keepout_volume = (holder ^ keepout).volume()
    min_axis_radius = float(-hm.bounds[1, 0])  # Y=0 is in the forward footprint edge

    mass_body = body.volume*STEEL_DENSITY_G_CM3/1e6
    mass_pin = pin.volume*STEEL_DENSITY_G_CM3/1e6
    # Body mesh has polygonal lowest vertices: use their convex contact set.
    bow = body.vertices[body.vertices[:, 2] <= table+1e-5]
    bow_center = bow.mean(axis=0)
    pivot_x = bow[:, 0].max()
    seated_com = (body.center_mass*mass_body+pin.center_mass*mass_pin)/(mass_body+mass_pin)
    base = md.CrossSection([rounded_rect(TRIANGLE_LEFT_X+BOTTOM_CHAMFER,
        TRIANGLE_TOE_X-TOE_TRUNCATION-BOTTOM_CHAMFER,
        -DEPTH_Y/2+BOTTOM_CHAMFER, DEPTH_Y/2-BOTTOM_CHAMFER,
        CORNER_RADIUS-BOTTOM_CHAMFER)])
    base_area = base.area()
    # Midpoint is exact by symmetry, including corner rounds/chamfer.
    base_center_x = (TRIANGLE_LEFT_X+TRIANGLE_TOE_X-TOE_TRUNCATION)/2
    base_lever = (pivot_x-base_center_x)/1000
    holder_lever = (pivot_x-contact_center[0])/1000
    # Uniform area samples on the actual bed face give its polar second moment.
    base_faces = hm.submesh([np.flatnonzero(lowest)], append=True, repair=False)
    bpts, _ = trimesh.sample.sample_surface(base_faces, 60000, seed=RANDOM_SEED)
    bxy = bpts[:, :2]-[base_center_x, 0]
    base_j = base_area*np.mean(np.sum(bxy**2, axis=1))
    base_rmax = np.linalg.norm(base_faces.vertices[:, :2]-[base_center_x, 0], axis=1).max()
    cases = {}
    for name, mass, com in [('body_only', mass_body, body.center_mass),
                             ('pin_seated', mass_body+mass_pin, seated_com)]:
        weight = mass*GRAVITY
        rest_moment = weight*(com[0]-pivot_x)/1000
        push_moment = DOWNWARD_PIN_FORCE_N*(-pivot_x)/1000
        resultant_x = weight*com[0]/(weight+DOWNWARD_PIN_FORCE_N)
        cases[name] = {
            'mass_kg': mass, 'center_of_mass_mm': com.tolist(), 'weight_N': weight,
            'gravity_overturning_moment_Nm': rest_moment,
            'with_20N_overturning_moment_Nm': rest_moment+push_moment,
            'with_20N_resultant_x_mm': resultant_x,
            'shackle_holder_vertical_tie_N': (rest_moment+push_moment)/holder_lever,
            'holder_table_vertical_tie_N': (rest_moment+push_moment)/base_lever,
            'shackle_holder_average_tie_stress_MPa': (rest_moment+push_moment)/holder_lever/contact_area,
            'table_average_tie_stress_MPa': (rest_moment+push_moment)/base_lever/base_area,
        }
    stats = {
        'edge_treatment': settings['edge_treatment'],
        'geometry': {'bounds_body_frame_mm': hm.bounds.tolist(), 'dimensions_mm': hm.extents.tolist(),
            'volume_mm3': hm.volume, 'triangles': len(hm.faces), 'watertight': hm.is_watertight,
            'single_solid': len(holder.decompose()) == 1,
            'minimum_seat_floor_mm': floor, 'minimum_rear_wall_mm': rear_wall,
            'minimum_lateral_rail_mm': side_wall,
            'max_non_bed_overhang_deg_from_vertical': float(overhang.max(initial=0)),
            'bed_face_area_mm2': base_area},
        'fit': {'exact_mesh_minimum_gap_mm': minimum_gap,
            'contact_sample_gap_range_mm': [float(gap_samples.min()), float(gap_samples.max())],
            'contact_samples': len(samples), 'contact_area_mm2': contact_area,
            'contact_center_mm': contact_center.tolist(),
            'contact_bounds_mm': patch.bounds.tolist(),
            'intersection_volume_mm3': intersection, 'vertical_insertion_intersection_volumes_mm3': drop_checks,
            'bore_keepout_intersection_mm3': keepout_volume,
            'minimum_axis_radius_mm': min_axis_radius,
            'bore_circle_radial_margin_mm': min_axis_radius-bore_r,
            'offset_conservative_bound_mm': settings['offset_bound_mm']},
        'statics': {'density_g_cm3': STEEL_DENSITY_G_CM3,
            'body_mass_kg': mass_body, 'pin_mass_kg': mass_pin,
            'pin_center_of_mass_seated_mm': pin.center_mass.tolist(),
            'bow_contact_vertices_mm': bow.tolist(), 'bow_contact_center_mm': bow_center.tolist(),
            'forward_support_edge_x_mm': pivot_x,
            'holder_table_contact_center_mm': [base_center_x, 0, table],
            'body_cases': cases,
            'saddle_torsion_Jz_mm4': patch_j, 'base_torsion_Jz_mm4': base_j,
            'saddle_torsion_peak_shear_estimate_MPa': SCREW_TORQUE_NM*1000*patch_rmax/patch_j,
            'table_torsion_peak_shear_estimate_MPa': SCREW_TORQUE_NM*1000*base_rmax/base_j,
            'saddle_20mm_force_couple_N': SCREW_TORQUE_NM/0.020,
            'table_40mm_force_couple_N': SCREW_TORQUE_NM/(DEPTH_Y/1000),
            'torsion_notes': 'T*r/J about bond centroid, uniformly thick elastic adhesive approximation; curved saddle uses projected X,Y lever arms and actual surface area. No allowance for peel, voids, creep or stress concentration.'},
    }
    assert intersection < 1e-6 and keepout_volume < 1e-6
    assert minimum_gap >= GLUE_GAP-0.002
    assert max(drop_checks.values()) < 1e-6
    patch.export(OUT/'contact_patch_mm.ply')
    return stats


def side_view(body, pin, holder, table):
    # Same physical page size, grid, orthographic projection, and triangle
    # shading as shackle_side_view.py. Load its functions without executing main.
    import importlib.util
    spec = importlib.util.spec_from_file_location('holder_side_source', HERE/'shackle_side_view.py')
    side = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(side)
    xmin, xmax = body.bounds[0, 0], max(body.bounds[1, 0], pin.bounds[1, 0])
    zmin, zmax = body.bounds[0, 2], pin.bounds[1, 2]
    ear_bottom = body.vertices[np.abs(body.vertices[:, 0]) < 20.5, 2].min()
    x0, x1 = np.floor(xmin/10)*10-40, np.ceil(xmax/10)*10+50
    z0, z1 = np.floor(zmin/10)*10-70, np.ceil(zmax/10)*10+30
    fig = plt.figure(figsize=((x1-x0)/25.4, (z1-z0)/25.4))
    ax = fig.add_axes([0, 0, 1, 1]); ax.set(xlim=(x0, x1), ylim=(z0, z1), aspect='equal'); ax.axis('off')
    for x in np.arange(x0, x1+1, 5):
        ax.axvline(x, color='#e4e8ee' if x % 10 else '#c9d1dc', lw=.3 if x % 10 else .6, zorder=0)
    for z in np.arange(z0, z1+1, 5):
        ax.axhline(z, color='#e4e8ee' if z % 10 else '#c9d1dc', lw=.3 if z % 10 else .6, zorder=0)
    for x in np.arange(x0, x1+1, 20): ax.text(x+.6, z0+1.5, f'{x:.0f}', fontsize=5, color='#8a96a8')
    for z in np.arange(z0+20, z1+1, 20): ax.text(x0+1, z+.6, f'{z:.0f}', fontsize=5, color='#8a96a8')
    for m, color, alpha, order in [(body, [.80, .82, .85], 1, 2),
                                  (holder, [.10, .49, .78], 1, 3), (pin, [.62, .74, .95], .45, 4)]:
        polys, colors = side.shaded_tris(m.vertices, m.faces, np.array(color))
        ax.add_collection(PolyCollection(polys, facecolors=colors, edgecolors='none', alpha=alpha,
                                        rasterized=True, zorder=order))
    ax.plot([0, 0], [zmin-25, zmax+15], ls=(0, (6, 3, 1, 3)), color='#c0392b', lw=.7, zorder=5)
    ax.annotate('pin goes in from above', xy=(0, zmax+8), xytext=(14, zmax+18), fontsize=7, color='#c0392b',
                arrowprops=dict(arrowstyle='->', color='#c0392b', lw=.7))
    ax.text(24, 30, 'clear ear (top)', fontsize=7, va='center')
    ax.text(24, -30, 'tapped ear (bottom)', fontsize=7, va='center')
    ax.text(24, ear_bottom-9, 'keep clear below the tapped hole:\nfull vertical bore cylinder is clear',
            fontsize=5.5, va='top', color='#c0392b')
    ax.axhline(table, xmin=(xmin-x0)/(x1-x0), xmax=(xmax-x0)/(x1-x0),
               color='#2c3e50', lw=.6, ls='--', zorder=1)
    ax.annotate(f'Gavin\'s cradle: 40 mm deep\n0.5 mm nominal glue gap\nflat base at Z = {table:.2f} mm',
        xy=(-81, -40), xytext=(-132, -70), fontsize=6, color='#165b8b',
        arrowprops=dict(arrowstyle='->', color='#165b8b', lw=.7))
    ax.text(x0+3, z1-4, 'Gavin\'s shackle holder. Scale 1:1, grid 5 / 10 mm; body frame, mm.\n'
            'Blue: printable cradle. Pale blue: seated pin. Print PDF at 100% / actual size.', fontsize=6, va='top')
    ax.text(x0+9, -93, 'All handling edges: 0.5 mm round/chamfer; outline corners R1.\n'
            'Shackle hot-glued to holder; holder hot-glued to desk.\n'
            'Hold the pin axis vertical while the hot glue cures.', fontsize=6, va='top')
    ax.plot([x1-60, x1-10], [z0+8, z0+8], color='black', lw=1.2)
    ax.text(x1-35, z0+10, '50 mm (check your print)', fontsize=6, ha='center')
    fig.savefig(OUT/'side_view_holder.pdf', dpi=12*25.4)
    fig.savefig(OUT/'side_view_holder.png', dpi=12*25.4)
    plt.close(fig)
    return {'page_mm': [x1-x0, z1-z0], 'extents_xz_mm': [x0, x1, z0, z1]}


def render(name, parts, eye, target, scale, ground, title, subtitle):
    """Deterministic CPU triangle z-buffer, matching the model report approach."""
    size = 1100
    forward = unit(np.asarray(eye)-target)
    right = unit(np.cross([0, 0, 1], forward)); up = unit(np.cross(forward, right))
    basis = np.array([right, up, forward]); target = np.asarray(target)
    depth = np.full((size, size), -np.inf)
    pixels = np.full((size, size, 3), .95)
    shadow = Image.new('L', (size, size)); sd = ImageDraw.Draw(shadow)
    def project(v):
        p = (v-target)@basis.T
        p[:, 0] = size/2+p[:, 0]*size/scale
        p[:, 1] = size/2-p[:, 1]*size/scale
        return p
    light = unit([-.5, -.7, 1.0])
    for mesh, rgb, smooth in parts:
        p = project(mesh.vertices)
        n = mesh.vertex_normals[mesh.faces] if smooth else np.repeat(mesh.face_normals[:, None], 3, axis=1)
        sv = mesh.vertices.copy(); sv[:, :2] += (sv[:, 2:3]-ground)*np.array([.25, .35]); sv[:, 2] = ground
        sp = project(sv)
        for f in mesh.faces: sd.polygon([tuple(q) for q in sp[f, :2]], fill=35)
        visible = np.flatnonzero(mesh.face_normals@forward > 0)
        for fi in visible:
            pp = p[mesh.faces[fi]]; nn = n[fi]
            xmin = max(0, int(np.floor(pp[:, 0].min()))); xmax = min(size-1, int(np.ceil(pp[:, 0].max())))
            ymin = max(0, int(np.floor(pp[:, 1].min()))); ymax = min(size-1, int(np.ceil(pp[:, 1].max())))
            if xmin > xmax or ymin > ymax: continue
            den = (pp[1, 1]-pp[2, 1])*(pp[0, 0]-pp[2, 0])+(pp[2, 0]-pp[1, 0])*(pp[0, 1]-pp[2, 1])
            if abs(den) < 1e-10: continue
            yy, xx = np.mgrid[ymin:ymax+1, xmin:xmax+1]; xx = xx+.5; yy = yy+.5
            a = ((pp[1, 1]-pp[2, 1])*(xx-pp[2, 0])+(pp[2, 0]-pp[1, 0])*(yy-pp[2, 1]))/den
            b = ((pp[2, 1]-pp[0, 1])*(xx-pp[2, 0])+(pp[0, 0]-pp[2, 0])*(yy-pp[2, 1]))/den
            c = 1-a-b
            z = a*pp[0, 2]+b*pp[1, 2]+c*pp[2, 2]
            crop = depth[ymin:ymax+1, xmin:xmax+1]
            mask = (a >= -1e-8) & (b >= -1e-8) & (c >= -1e-8) & (z > crop)
            if not mask.any(): continue
            crop[mask] = z[mask]
            normal = unit((a[..., None]*nn[0]+b[..., None]*nn[1]+c[..., None]*nn[2])[mask])
            shade = .35+.50*np.maximum(normal@light, 0)+.15*np.maximum(normal@unit([1, .5, .3]), 0)
            spec = (.16 if smooth else .035)*np.maximum(normal@unit(light+forward), 0)**24
            pixels[ymin:ymax+1, xmin:xmax+1][mask] = np.clip(shade[:, None]*rgb+spec[:, None], 0, 1)
    from PIL import ImageFilter
    mask = ~np.isfinite(depth)
    shadows = np.asarray(shadow.filter(ImageFilter.GaussianBlur(7)))/255
    pixels[mask] -= shadows[mask, None]
    im = Image.fromarray(np.uint8(np.clip(pixels, 0, 1)*255))
    draw = ImageDraw.Draw(im)
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font = ImageFont.truetype(font_path, 25); small = ImageFont.truetype(font_path, 17)
    draw.text((30, 22), title, font=font, fill='#17304b')
    draw.text((30, 58), subtitle, font=small, fill='#405569')
    bar = int(10*size/scale)
    draw.line([(40, size-50), (40+bar, size-50)], fill='#17304b', width=3)
    draw.text((40, size-80), '10 mm', font=small, fill='#17304b')
    im.save(OUT/(name+'.png'))
    log(f'Rendered {name}')


def report(stats, table, source_hashes):
    g, f, s, e = stats['geometry'], stats['fit'], stats['statics'], stats['edge_treatment']
    a, b = s['body_cases']['body_only'], s['body_cases']['pin_seated']
    txt = f"""# Gavin's shackle holder - edge update, 2026-10-28

Gavin's triangular holder is retained, with the shackle hot-glued to the holder and the holder hot-glued to the desk. Local geometry/rendering only: no billed calls, hardware or sim server. Protected source-file hashes are unchanged.

**Dimensions and edges.** Finished envelope **{g['dimensions_mm'][0]:.2f} x {g['dimensions_mm'][1]:.2f} x {g['dimensions_mm'][2]:.2f} mm**; body-frame X=-89..-68, Y=+/-20, bottom Z={table:.5f}, top Z=-33.5 mm. Named **`EDGE_CHAMFER = 0.5` mm** supplies equivalent R0.5 rounds on the top tip flat's perimeter, side/top transitions, vertical edges and every cradle rim. Existing **R1 outline corners** are retained. All bottom edges have a **0.5 mm, 45-degree chamfer**, including rounded footprint corners, to retain the flat bed face and avoid a lower fillet overhang.

**How edges were found and checked.** Rolling-ball erosion then dilation processes all convex edges together, so the treatment does not depend on manually selecting a few rims. The round kernel spans radius **{e['round_kernel_radius_bounds_mm'][0]:.3f}..{e['round_kernel_radius_bounds_mm'][1]:.3f} mm**. The functional cradle is a concave surface and stays in place except at its rounded boundary. Triangle adjacency and face normals identify physical creases: turns over {e['feature_detection_angle_deg']:.0f} degrees, with shared edge and both triangle altitudes over {e['feature_resolution_mm']:.2f} mm. The 0.1 mm resolution is half the suggested print layer. Smaller triangle lines are curve tessellation or numerical slivers, not design edges; the rounding operation itself is unfiltered.

The audit found **{e['upper_original_feature_segments']} original upper feature segments** and tested **{e['original_feature_samples']} points**, including endpoints, confirming that the original edges have been relieved. **{e['original_feature_samples_left_unbroken']} points left unbroken; {e['remaining_sharp_upper_feature_segments']} untreated upper creases remain.** Maximum resolved upper normal turn is **{e['maximum_resolved_upper_normal_turn_deg']:.2f} degrees**, across round tessellation; bottom transitions are **{e['maximum_resolved_chamfer_normal_turn_deg']:.2f} degrees**, the intentional blunt chamfer boundaries (135-degree included angle). Thus all handling edges have the requested round/chamfer or the larger existing R1 round. Bottom-face insets are **{min(e['bottom_face_insets_mm']):.3f} mm** or greater. Audit samples and removal depths are saved in `edge_audit_samples.npz`.

**Cradle and fit.** Same mesh-derived saddle, **0.5 mm nominal glue gap**, same +/-35-degree wrap limit and vertical drop-in cutter. At {e['cradle_core_samples']} protected points at least 1 mm from the rim, maximum surface movement is only **{e['cradle_core_max_surface_deviation_mm']:.5f} mm** (mesh tolerance); the final core gap is **{e['cradle_core_final_gap_range_mm'][0]:.5f}..{e['cradle_core_final_gap_range_mm'][1]:.5f} mm**. The round only relieves the rim; it does not tighten the seat. Global minimum gap **{f['exact_mesh_minimum_gap_mm']:.5f} mm**; {f['contact_samples']} samples on the full-fit patch give **{f['contact_sample_gap_range_mm'][0]:.5f}..{f['contact_sample_gap_range_mm'][1]:.5f} mm**. The relieved rim has a deliberately larger gap. Body intersection **0 mm^3**; nine raised-body checks are clear. Current full-fit bond area ~**{f['contact_area_mm2']:.0f} mm^2**. Minimum floor/rear/side thickness **{g['minimum_seat_floor_mm']:.2f}/{g['minimum_rear_wall_mm']:.2f}/{g['minimum_lateral_rail_mm']:.2f} mm**.

**Print verification.** Single connected, outward-wound, watertight solid; exported STL/3MF/OBJ roundtrips also pass. Flat face at **Z=0** in STL/3MF; maximum non-bed overhang **{g['max_non_bed_overhang_deg_from_vertical']:.2f} degrees from vertical**. Print **without supports**: 0.2 mm layers, 4 walls, 5 top/bottom layers, 35-45% infill, PLA or PETG. Import STL as **mm**; 3MF explicitly stores millimetres. Sim OBJ remains **metres in the shackle body frame**.

**Glue-joint loads, for information.** Stainless density 7.9 g/cm^3: body **{s['body_mass_kg']*1000:.1f} g**, pin **{s['pin_mass_kg']*1000:.1f} g**; seated total **{b['mass_kg']*1000:.1f} g**. COM without pin **({a['center_of_mass_mm'][0]:.2f}, {a['center_of_mass_mm'][1]:.3f}, {a['center_of_mass_mm'][2]:.2f}) mm**; seated **({b['center_of_mass_mm'][0]:.2f}, {b['center_of_mass_mm'][1]:.3f}, {b['center_of_mass_mm'][2]:.2f}) mm**. Bow/table contact near **({s['bow_contact_center_mm'][0]:.2f}, 0, {table:.2f}) mm**, saddle bond centroid **({f['contact_center_mm'][0]:.2f}, 0, {f['contact_center_mm'][2]:.2f}) mm**, desk bond centroid **(-78.5, 0, {table:.2f}) mm**, desk bond area **{g['bed_face_area_mm2']:.0f} mm^2**. With 20 N down at pin axis X=0:

| Load carried by the glued fixture | Body only | Pin seated |
|---|---:|---:|
| Gravity moment about bow contact, N m | {a['gravity_overturning_moment_Nm']:.3f} | {b['gravity_overturning_moment_Nm']:.3f} |
| Gravity + 20 N moment, N m | {a['with_20N_overturning_moment_Nm']:.3f} | {b['with_20N_overturning_moment_Nm']:.3f} |
| Equivalent saddle hold-down, N | {a['shackle_holder_vertical_tie_N']:.1f} | {b['shackle_holder_vertical_tie_N']:.1f} |
| Equivalent desk-bond hold-down, N | {a['holder_table_vertical_tie_N']:.1f} | {b['holder_table_vertical_tie_N']:.1f} |

These use a bow-contact pivot and bond-centroid lever arms, neglecting beneficial holder weight and friction. The **2 N m screw torque** passes through both glue joints: equivalent to 100 N opposing forces over 20 mm at the saddle, or 50 N over 40 mm at the base. Current-area elastic-layer estimates tau=T*r/J give peak shear **{s['saddle_torsion_peak_shear_estimate_MPa']:.2f} MPa saddle / {s['table_torsion_peak_shear_estimate_MPa']:.2f} MPa desk**; seated average hold-down force/area is **{b['shackle_holder_average_tie_stress_MPa']:.3f}/{b['table_average_tie_stress_MPa']:.3f} MPa**. These numbers describe the accepted glue load path; adhesive peel, voids and creep are not resolved by this simplified statics model.

**Real-part checks.** The m291/m294/m295 dimensions do not bound the exact forged crown curve. Retain the previous +/-2 mm local trial allowance (not a measured tolerance): hot glue fills an open gap, while interference needs fitting. Dry-fit to confirm lowest-bow contact, vertical pin axis and actual pin-tip/table clearance (~9.33 mm). Hold the shackle vertical through glue cure; the circular crown can slide along its arc before bonding. Avoid prolonged glue-gun heat on the plastic. The whole vertical bore cylinder remains clear: holder radius >=68 mm versus bore radius 10.3 mm.

**Files/rebuild.** STL, 3MF, sim OBJ and `holder_params.json` are under `assets/shackle/`. All requested images and the 1:1 PDF are under `runs/shackle_holder/`; print the PDF at 100% and check its 50 mm bar. Complete fit, edge, export and statics data are in `holder_checks.json`; full-fit contact mesh in `contact_patch_mm.ply`. Rebuild with `PYTHONDONTWRITEBYTECODE=1 ~/dreamtrial/.venv/bin/python shackle_holder.py`. Existing top/toe truncations and the sketch mapping are unchanged.
"""
    (OUT/'shackle_holder_report.md').write_text(txt)
    (OUT/'holder_analysis.md').write_text(txt)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    paths = [ASSETS/'shackle_body.stl', ASSETS/'shackle_pin.stl', ASSETS/'shackle_params.json',
             HERE/'shackle_side_view.py', HERE/'shackle/holder_sketch_gavin.png',
             HERE/'runs/shackle_side_view/shackle_side_view.png']
    blobs = {p: p.read_bytes() for p in paths}
    hashes = {str(p.relative_to(HERE)): hashlib.sha256(b).hexdigest() for p, b in blobs.items()}
    params = json.loads(blobs[ASSETS/'shackle_params.json'])
    body = trimesh.load(io.BytesIO(blobs[ASSETS/'shackle_body.stl']), file_type='stl', force='mesh')
    pin = trimesh.load(io.BytesIO(blobs[ASSETS/'shackle_pin.stl']), file_type='stl', force='mesh')
    body.apply_scale(1000); pin.apply_scale(1000)
    pin.apply_translation(np.array(params['frame']['seated_pin_translation_m'])*1000)
    table = body.bounds[0, 2] if TABLE_Z_OVERRIDE is None else TABLE_Z_OVERRIDE
    holder, hm, local, footprint, settings = build(body, params['dimensions'], table)
    log(f'Holder built: {len(hm.faces)} triangles, {hm.volume:.1f} mm^3; checking fit/statics')
    stats = checks(holder, hm, local, body, pin, footprint, table, params['dimensions'], settings)
    print_shift = [-hm.bounds[0, 0], -hm.bounds[0, 1], -table]
    printable = hm.copy(); printable.apply_translation(print_shift); printable.units = 'mm'
    printable.export(ASSETS/'shackle_holder.stl')
    printable.export(ASSETS/'shackle_holder.3mf')
    sim = hm.copy(); sim.apply_scale(.001)
    (ASSETS/'shackle_holder_sim.obj').write_text('# Units: metres; shackle body frame, clear ear +Z, bow -X\n'+trimesh.exchange.obj.export_obj(sim))
    # Verify actual export roundtrips, not just pre-export meshes.
    stl = trimesh.load(ASSETS/'shackle_holder.stl', force='mesh')
    mf = trimesh.load(ASSETS/'shackle_holder.3mf', force='mesh')
    obj = trimesh.load(ASSETS/'shackle_holder_sim.obj', force='mesh')
    for m in [stl, mf, obj]: assert m.is_watertight and len(m.split()) == 1
    assert np.max(np.abs(stl.bounds-printable.bounds)) < 1e-4
    assert np.max(np.abs(mf.bounds-printable.bounds)) < 1e-6
    assert np.max(np.abs(obj.bounds-sim.bounds)) < 1e-7
    assert abs(stl.bounds[0, 2]) < 1e-6
    with zipfile.ZipFile(ASSETS/'shackle_holder.3mf') as z:
        model = ET.fromstring(z.read('3D/3dmodel.model'))
        assert model.attrib['unit'] == 'millimeter'
    stats['exports'] = {'roundtrip_watertight_single_solid': True,
        'stl_3mf_numeric_units': 'mm', '3mf_embedded_unit': model.attrib['unit'],
        'sim_obj_numeric_units': 'm', 'body_mm_to_print_mm_translation': print_shift}
    log('Fit, wall, support, insertion and export checks passed')
    page = side_view(body, pin, hm, table)
    stats['side_view'] = page
    render('holder_3q', [(hm, [.12, .54, .82], False)], (-125, -120, 12), (-78.5, 0, -41), 64, table,
           'Gavin\'s curved shackle cradle', 'Body frame | 0.5 mm glue gap | 0.5 mm edge rounds/chamfers')
    render('holder_with_shackle_3q', [(hm, [.12, .54, .82], False), (body, [.80, .84, .88], True),
           (pin, [.56, .67, .82], True)], (-170, -250, 110), (-36, 0, 7), 175, table,
           'Shackle on Gavin\'s holder', 'Clear ear above, tapped ear below | hot-glued to holder and desk')
    render('holder_print_orientation', [(printable, [.12, .54, .82], False)], (80, -110, 70),
           (10.5, 20, 7), 68, 0, 'Print orientation: flat bottom on Z = 0',
           'Millimetres | no supports | 0.5 mm edge treatment | 0.2 mm layers')
    # Record whether concurrent source changes occurred, without overwriting them.
    changed = [str(p.relative_to(HERE)) for p in paths if hashlib.sha256(p.read_bytes()).hexdigest() != hashes[str(p.relative_to(HERE))]]
    stats['source_hashes_sha256'] = hashes
    stats['source_files_changed_during_build'] = changed
    if changed: raise RuntimeError(f'Source files changed during build; rerun for consistency: {changed}')
    (OUT/'holder_checks.json').write_text(json.dumps(stats, indent=2)+'\n')
    design = {k: v for k, v in globals().items() if k.isupper() and isinstance(v, (int, float, str, type(None)))}
    (ASSETS/'holder_params.json').write_text(json.dumps({'design': design, 'derived': settings,
        'table_z_body_frame_mm': table, 'checks': stats}, indent=2)+'\n')
    report(stats, table, hashes)
    log('Complete: assets/shackle/shackle_holder.* and runs/shackle_holder/')


if __name__ == '__main__':
    main()
