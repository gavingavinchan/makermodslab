# Copyright 2026 MakerMods. All rights reserved.
# Licensed under the Apache License, Version 2.0; see repository LICENSE.
"""Smooth the observed stripe geometry and traverse it without reversal dwells.

This is an authored cinematic retarget, not a new observed physics trajectory.
Sub-millimetre sample jitter is removed before fitting a cubic surface-coordinate
spline. Arc-length timing keeps motion positive through rounded lane changes;
curvature reduces speed smoothly rather than holding at the stroke ends.
"""
import numpy as np
from scipy.interpolate import splprep, splev
from surface import surface


def continuous_path(source, duration_s):
    uv = np.array([[r['actual_t'], r['actual_u']] for r in source])
    xyz = np.array([surface(*p)[0] for p in uv])
    keep = [0]
    for i in range(1, len(uv) - 1):
        if np.linalg.norm(xyz[i] - xyz[keep[-1]]) >= .0008:
            keep.append(i)
    keep.append(len(uv) - 1)
    metric = np.array([.395, .20])
    nodes = uv[keep] * metric
    length = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xyz[keep], axis=0), axis=1))]
    length /= length[-1]
    spline, _ = splprep(nodes.T, u=length, s=len(nodes) * .00025**2, k=3)
    parameter = np.linspace(0, 1, 12001)
    coordinates = np.array(splev(parameter, spline)).T / metric
    # Smoothing must not shorten the edge reach or the radial coverage span.
    lo, hi = uv[:, 0].min(), uv[:, 0].max()
    coordinates[:, 0] = lo + (coordinates[:, 0]-coordinates[:, 0].min()) / np.ptp(coordinates[:, 0]) * (hi-lo)
    points = np.array([surface(*p)[0] for p in coordinates])
    ds = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.r_[0, np.cumsum(ds)]
    tangent = np.gradient(points, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    curvature = np.linalg.norm(np.gradient(tangent, axis=0), axis=1) / np.maximum(np.gradient(arc), 1e-10)
    # Positive speed everywhere: curved turns slow modestly, never dwell.
    weight = np.sqrt(1 + (curvature * .004)**2)
    weight = np.minimum(weight, 2.5)
    clock = np.r_[0, np.cumsum(ds * (weight[:-1] + weight[1:]) / 2)]
    clock *= duration_s / clock[-1]

    def sample(seconds):
        sec = np.clip(seconds, 0, duration_s)
        t, u = (float(np.interp(sec, clock, coordinates[:, j])) for j in range(2))
        original_parameter = float(np.interp(sec, clock, parameter))
        source_index = int(round(np.interp(original_parameter, length, keep)))
        return t, u, source_index

    return sample, dict(method='Cubic surface-coordinate smoothing and positive arc-length timing; curvature-weighted rounded turns; no intermediate holds',
                       observed_samples=len(source), spline_nodes=len(nodes), smoothing_rms_target_m=.00025,
                       path_length_m=float(arc[-1]), duration_s=duration_s,
                       authored_kinematic_path=True, physics_revalidated=False)
