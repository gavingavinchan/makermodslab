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
"""Offline contact-check arrays, motion selection and friction identification.

Command velocity is differentiated from the logged targets, not the noisy
encoder. Never use push ticks for identification. The fold can carry real
contact even during moves, so both position cuts apply to every motion table.
"""

import numpy as np

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
ACCEL_LIMIT_DEG_S2 = 50.0
TRANSIENT_S = 0.15
SMOOTH_SPEEDS = (1.0, 2.0, 3.0, 4.0, 6.0)


def tick_arrays(data):
    cols = data["ticks"]["columns"]
    rows = data["ticks"]["rows"]
    idx = {c: i for i, c in enumerate(cols)}
    phases = np.array([r[idx["phase"]] for r in rows])

    def col(name):
        return np.array([r[idx[name]] for r in rows], dtype=float)

    return phases, col


def motion_arrays(data):
    phases, col = tick_arrays(data)
    t = col("t")
    velocity = {k: np.gradient(col(f"{k}.cmd"), t) if len(t) > 1 else np.zeros(len(t)) for k in ARM}
    acceleration = {k: np.gradient(v, t) if len(t) > 1 else np.zeros(len(t)) for k, v in velocity.items()}
    moving = np.array([p.startswith(("move:", "approach:")) for p in phases], dtype=bool)
    away = (col("shoulder_lift.pos") < -15) & (col("elbow_flex.pos") > 15)
    transient = np.zeros(len(t), dtype=bool)
    for a in acceleration.values():
        for i in np.flatnonzero(np.abs(a) > ACCEL_LIMIT_DEG_S2):
            transient |= np.abs(t - t[i]) <= TRANSIENT_S
    # Also discard the boundary of each leg: acceleration before/after a leg
    # can otherwise masquerade as Coulomb torque at low commanded speeds.
    edges = np.flatnonzero(phases[1:] != phases[:-1]) + 1
    for i in edges:
        transient |= np.abs(t - t[i]) <= TRANSIENT_S
    return phases, col, velocity, moving & away & ~transient


def contacts(data, estimator):
    phases, col = tick_arrays(data)
    out = {k: np.empty(len(phases)) for k in ARM}
    # Cache columns: repeatedly materializing them for each row is quadratic.
    pos = {k: col(f"{k}.pos") for k in ARM}
    torque = {k: col(f"{k}.torque") for k in ARM}
    for i in range(len(phases)):
        c = estimator.estimate({k: pos[k][i] for k in ARM}, {k: torque[k][i] for k in ARM})
        for k in ARM:
            out[k][i] = c[k]
    return out


def compensate(contact, velocity, friction):
    return {
        k: contact[k]
        - terms["coulomb_nm"] * np.tanh(velocity[k] / terms["v_smooth_deg_s"])
        - terms["viscous_nm_per_deg_s"] * velocity[k]
        if k in friction
        else contact[k].copy()
        for k in contact
        for terms in [friction.get(k)]
    }


def nonnegative_fit(x, y):
    """Two-column NNLS without a scipy dependency; test the interior and faces."""
    beta = np.linalg.lstsq(x, y, rcond=None)[0]
    candidates = [np.zeros(2)]
    if np.all(beta >= 0):
        candidates.append(beta)
    for j in range(2):
        b = np.zeros(2)
        b[j] = max(0.0, np.dot(x[:, j], y) / np.dot(x[:, j], x[:, j]))
        candidates.append(b)
    return min(candidates, key=lambda b: np.sum((y - np.sum(x * b, axis=1)) ** 2))


def fit_friction(datasets, estimator):
    """NNLS per joint, smooth speed selected by leaving out contiguous move legs.

    A stationary joint is explicitly unidentifiable and receives zero terms.
    Low-speed yaw/roll are reported as weakly excited, rather than suggesting
    their Coulomb/viscous split has been separately validated.
    """
    records = []
    for index, data in enumerate(datasets):
        phases, _, velocity, mask = motion_arrays(data)
        segment = np.cumsum(np.r_[True, phases[1:] != phases[:-1]]) + index * 10000
        records.append((velocity, contacts(data, estimator), mask, segment))
    result, diagnostics = {}, {}
    for k in ARM:
        v = np.concatenate([r[0][k][r[2]] for r in records])
        y = np.concatenate([r[1][k][r[2]] for r in records])
        groups = np.concatenate([r[3][r[2]] for r in records])
        # Ignore floating point differentiation of a constant target. Include
        # resting ticks in evaluation, but they cannot identify friction.
        active = np.abs(v) > 0.1
        v, y, groups = v[active], y[active], groups[active]
        diagnostics[k] = {"n": len(v), "positive": int((v > 0).sum()), "negative": int((v < 0).sum())}
        if not len(v):
            result[k] = {"coulomb_nm": 0.0, "viscous_nm_per_deg_s": 0.0, "v_smooth_deg_s": 3.0}
            diagnostics[k]["status"] = "unidentifiable: no commanded motion"
            continue
        max_speed = float(np.max(np.abs(v)))
        if max_speed < 2.0:
            # All speeds are in tanh's nearly linear region for a few-deg/s
            # smoothing width, so a Coulomb/viscous split would be arbitrary.
            # The local slope is NOT stored as a term: applied at teleop speed
            # (wrist_roll's 0.31 N m per deg/s at 20 deg/s is 6 N m) it would
            # swamp the contact it exists to clean. Kept as a diagnostic only.
            slope = max(0.0, float(np.dot(v, y) / np.dot(v, v)))
            result[k] = {"coulomb_nm": 0.0, "viscous_nm_per_deg_s": 0.0, "v_smooth_deg_s": 3.0}
            diagnostics[k].update(
                {
                    "max_speed_deg_s": max_speed,
                    "local_slope_nm_per_deg_s": slope,
                    "status": "weak excitation: not compensated; local slope recorded only",
                }
            )
            continue
        scores = []
        for speed in SMOOTH_SPEEDS:
            x = np.column_stack([np.tanh(v / speed), v])
            errors = []
            for group in np.unique(groups):
                test = groups == group
                if (~test).sum() < 10:
                    continue
                beta = nonnegative_fit(x[~test], y[~test])
                errors.extend((y[test] - np.sum(x[test] * beta, axis=1)) ** 2)
            scores.append(float(np.mean(errors)) if errors else float("inf"))
        speed = SMOOTH_SPEEDS[int(np.argmin(scores))]
        x = np.column_stack([np.tanh(v / speed), v])
        beta = nonnegative_fit(x, y)
        if beta[0] == 0:
            speed = 3.0  # smoothing width has no effect without a Coulomb term
        result[k] = {
            "coulomb_nm": float(beta[0]),
            "viscous_nm_per_deg_s": float(beta[1]),
            "v_smooth_deg_s": speed,
            # The estimator holds the term at this speed beyond it: never extrapolate.
            "max_speed_deg_s": max_speed,
        }
        diagnostics[k].update(
            {
                "max_speed_deg_s": float(np.max(np.abs(v))),
                "leg_cv_mse": dict(zip(SMOOTH_SPEEDS, scores, strict=True)),
            }
        )
        diagnostics[k]["status"] = "fitted"
    return result, diagnostics
