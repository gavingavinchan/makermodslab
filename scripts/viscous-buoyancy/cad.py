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
"""Measure occupied volume separately from the broad-phase collision shells."""

import hashlib
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from hydro import BODIES, CORNERS, ROOT, URDF, geo
from scipy.spatial import ConvexHull

VIS = ROOT / "scripts/viscous-dream1/assets/propeller_cardinal/arm"


def meshes(link, tag):
    for element in link.findall(tag):
        mesh = element.find("geometry/mesh")
        filename = mesh.get("filename")
        path = URDF.parent / filename if tag == "collision" else VIS / Path(filename).name
        m = trimesh.load_mesh(path)
        m.vertices *= geo.numbers(mesh.get("scale"), "1 1 1")
        r, t = geo.origin(element)
        m.vertices = m.vertices @ r.T + t
        yield path, m


def grid_union(parts, pitch):
    """Cast parallel rays across every closed part, union disjoint integer voxel centres."""
    occupied = set()
    for mesh in parts:
        lo, hi = np.floor(mesh.bounds[0] / pitch).astype(int), np.ceil(mesh.bounds[1] / pitch).astype(int)
        yz = np.array(list(itertools.product(range(lo[1], hi[1] + 1), range(lo[2], hi[2] + 1))))
        rays = np.column_stack((np.full(len(yz), lo[0] - 1), yz)) * pitch
        # Offsets avoid hitting symmetric triangulation edges or vertices.
        rays[:, 1:] += pitch * np.array([0.137, 0.263])
        loc, index, _ = mesh.ray.intersects_location(rays, np.tile([1.0, 0, 0], (len(yz), 1)))
        if not len(loc):
            continue
        hits = np.column_stack((index, loc[:, 0] / pitch))
        hits = hits[np.lexsort((hits[:, 1], hits[:, 0]))]
        for run in np.split(hits, np.flatnonzero(np.diff(hits[:, 0])) + 1):
            y, z = yz[int(run[0, 0])]
            for a, b in zip(run[::2], run[1::2], strict=True):
                occupied.update((x, y, z) for x in range(int(np.ceil(a[1])), int(np.ceil(b[1]))))
    return (np.array(sorted(occupied)) + np.array([0.0, 0.137, 0.263])) * pitch


def sample_centers(points, depth=5):
    """Equal-volume spatial quadrature. Replicate fractional boundary cells for exact first moment."""
    groups = [(points, np.ones(len(points)))]
    for _ in range(depth):
        children = []
        for cloud, weights in groups:
            order = cloud[:, np.argmax(np.ptp(cloud, axis=0))].argsort(kind="stable")
            cloud, weights = cloud[order], weights[order]
            left = np.clip(weights.sum() / 2 - (weights.cumsum() - weights), 0, weights)
            for w in (left, weights - left):
                pos = w > 1e-10
                children.append((cloud[pos], w[pos]))
        groups = children
    return np.array([np.average(c, axis=0, weights=w) for c, w in groups])


def measure():
    links, audit = {}, {}
    for link in ET.parse(URDF).findall("link"):
        name = link.get("name")
        if name not in BODIES:
            continue
        parts, hulls, provenance = [], [], {}
        for path, m in meshes(link, "visual"):
            if not m.is_volume:
                hulls.append(path.name)
                m = m.convex_hull
            parts.append(m)
            provenance[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        cells = grid_union(parts, 0.002)
        coarse = grid_union(parts, 0.003)
        envelope = grid_union([m.convex_hull for _, m in meshes(link, "collision")], 0.003)
        samples = sample_centers(cells)
        mass = float(link.find("inertial/mass").get("value"))
        v = len(cells) * 0.002**3
        links[name] = {
            "mass_kg": mass,
            "com_m": geo.numbers(link.find("inertial/origin").get("xyz")).tolist(),
            "volume_m3": v,
            "volume_cad_m3": v,
            "cob_m": cells.mean(axis=0).tolist(),
            "volume_samples": samples.tolist(),
            "volume_source": "2 mm centre-grid union of visual CAD parts",
            "trust": "geometric prior; actual flooding, potting, harness and cable displacement unmeasured",
            "notes": "Non-volume meshes replaced by individual sealed convex envelopes: " + ", ".join(hulls),
        }
        audit[name] = {
            "parts_sha256": provenance,
            "convex_replacements": hulls,
            "coarse_3mm_cm3": len(coarse) * 0.003**3 * 1e6,
            "guard_union_3mm_cm3": len(envelope) * 0.003**3 * 1e6,
        }
    return links, audit


def pockets():
    """Disjoint rectangular blanks, keeping at least 2 mm from visible metal and motor envelopes."""
    return [
        ("upper_core", "upper_arm_link", [-0.080, 0, 0], [0.096, 0.046, 0.036]),
        ("fore_center", "forearm_link", [0.079, 0, 0.05879], [0.040, 0.046, 0.036]),
        ("fore_proximal", "forearm_link", [0.048, 0, 0.05879], [0.022, 0.046, 0.020]),
        ("fore_distal", "forearm_link", [0.103, 0, 0.05879], [0.008, 0.046, 0.020]),
    ]


def pocket_checks(block):
    """AABB gaps conservatively screen parts; convex halfspaces screen containment in collision pieces."""
    link = ET.parse(URDF).find(f"link[@name='{block['link']}']")

    xyz = CORNERS * block["size_m"] + block["center_m"]
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    distances = {}
    for path, m in meshes(link, "visual"):
        gap = np.maximum(np.maximum(lo - m.bounds[1], m.bounds[0] - hi), 0)
        distances[path.stem] = float(np.linalg.norm(gap) * 1000)
    # The guard solids are convex pieces. Verify the entire box using a split at
    # each piece's x extrema (forearm CAD pieces meet in transverse planes).
    planes = [(m.bounds[:, 0], ConvexHull(m.vertices).equations) for _, m in meshes(link, "collision")]
    xs = sorted({lo[0], hi[0]} | {x for ends, _ in planes for x in ends if lo[0] < x < hi[0]})
    inside = True
    for a, b in itertools.pairwise(xs):
        corners = np.array(list(itertools.product((a, b), (lo[1], hi[1]), (lo[2], hi[2]))))
        inside &= any((corners @ eq[:, :3].T + eq[:, 3]).max() < 1e-6 for _, eq in planes)
    return {
        "part_aabb_gap_mm": min(distances.values()),
        "nearest_part": min(distances, key=distances.get),
        "inside_guard_union": bool(inside),
    }
