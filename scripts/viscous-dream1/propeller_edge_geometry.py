# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Geometry of the unchanged corrected blade and conservative swept clearance."""
import numpy as np
from propeller_corrected_scene import CENTER, ROOT_RADIUS, RADIUS, T_GRID, U_GRID, blade_point


def parameters(xyz):
    local = np.asarray(xyz)-CENTER
    r = np.hypot(local[..., 0], local[..., 2])
    t = (r-ROOT_RADIUS)/(RADIUS-ROOT_RADIUS)
    width = (.055+.270*t-.055*t*t)*np.sqrt(np.maximum(0, 1-t**4))
    u = (np.arctan2(local[..., 2], local[..., 0])-.12*np.sin(np.pi*t))*r/np.maximum(width, 1e-12)
    return np.stack((t, u), axis=-1)


def boundary():
    # Exact piecewise-linear rim of the existing 48 x 16 collision mesh,
    # not a newly smoothed outline and not only radial distance to the tip.
    tu = [(t, -1) for t in T_GRID] + [(t, 1) for t in T_GRID[-2::-1]] + [(0, u) for u in U_GRID[-2:0:-1]]
    return np.array([blade_point(0, t, u, 1)[[0, 2]] for t, u in tu]) + CENTER[[0, 2]]


def point_boundary_distance(xz):
    p = np.atleast_2d(xz); b = boundary(); a = b; v = np.roll(b, -1, axis=0)-b
    d = p[:, None, :]-a
    f = np.clip(np.einsum('nbi,bi->nb', d, v)/np.sum(v*v, axis=1), 0, 1)
    return np.sqrt(np.min(np.sum((d-f[..., None]*v)**2, axis=-1), axis=1))


def clearance_summary(samples):
    s = samples[samples[:, 1] == 1]
    # Exact distance of the 26 mm enclosing disk to every straight rim segment.
    # Positive clearance certifies the entire oriented ellipsoid stays in face
    # projection, irrespective of pad tilt. Always audit even during contact gaps.
    import shapely
    inside=shapely.contains_xy(shapely.Polygon(boundary()),s[:,2],s[:,4])
    dist = point_boundary_distance(s[:, [2, 4]])*np.where(inside,1.,-1.)-.026
    i = int(np.argmin(dist))
    # Lipschitz bounds for the piecewise-linear translation / shortest rotation
    # sweep between recorded substeps. Do not imply a hardware safety proof.
    pad_motion = np.linalg.norm(np.diff(s[:, [2,4]],axis=0),axis=1)
    swept_boundary = float(dist.min()-pad_motion.max()/2)
    left_swept = float(s[:,18].min()-(s[:,19].max()/2 if s.shape[1]>19 else 0))
    return dict(conservative_swept_pad_to_blade_boundary_m=swept_boundary,
                boundary_closest_time_s=float(s[i, 0]),
                sampled_boundary_clearance_m=float(dist[i]),
                maximum_pad_translation_per_ms_m=float(pad_motion.max()),
                interpolated_left_clearance_lower_bound_m=left_swept,
                left_gripper_clearance_m=float(s[:, 18].min()),
                clearance_evaluation='every 1 ms; enclosing 26 mm disk vs exact polygon rim; mj_geomDistance tool+right fingers vs all collidable left geoms',
                safety_passed=bool(swept_boundary>.001 and left_swept>.003),
                conservative_footprint_tip_radius_m=float(np.max(np.linalg.norm((s[:, 2:5]-CENTER)[:, [0, 2]], axis=1))+.026))
