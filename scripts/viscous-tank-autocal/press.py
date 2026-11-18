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
"""Press detection and the press verdict: pure functions over per-tick numbers, no arm.

The primary signal (Gavin, 2026-11-18) is the DIVERGENCE along the surface normal between where the
command puts the contact point and where the measured joints put it:

    div(s) = n . (p_cmd(s) - p_meas)

In free space it sits at a baseline (sag, drag and the MIT standing error, mostly compensated at the
standoff, the rest taken from the hold there and the first millimetres of the same approach). Against
rigid glass the measured point stops while the command keeps going, so div rises one-for-one with the
commanded depth. Contact = div over the baseline by a margin, for several ticks, AND rising with the
command. The moment of contact does not matter: any settled sample during the press is a point on
the plane, because the glass is rigid.

The joint torque residual against the standoff hold is the SECONDARY signal: an independent backstop
on the advance (TORQUE_BACKSTOP_NM, when the divergence detector misses), and a cross-check of each
press (the force the torques imply should agree with the stiffness times the divergence).
"""

from __future__ import annotations

import dataclasses
import math
from collections import deque

import numpy as np

DETECT_MM = 3.0  # divergence over the baseline that counts as contact (with SIGMA_K x the noise)
SIGMA_K = 6.0
PERSIST_TICKS = 5  # 0.25 s at 20 Hz
RISE_WINDOW_MM = 1.2  # over this much commanded travel...
RISE_RATIO = 0.5  # ...div must have risen by at least this fraction of it
BASELINE_MM = 6.0  # the first millimetres of the approach join the hold's baseline
NOISE_FLOOR_MM = 0.15
TORQUE_TICKS = 3

RIGID_MM = 1.5  # the two presses' plane points may differ along the normal by at most this
GROWTH_MIN = 0.6  # div must grow by >= this x the extra commanded depth between the presses
PRESS_MIN = 0.6  # and each press must show >= this x its commanded overshoot
SETTLED_SPREAD_MM = 1.0  # spread of the settled samples along the normal
SETTLED_SPEED_DEG_S = 0.6  # a sample is settled when no arm joint moved faster than this over 3 readings
FORCE_AGREE = (0.25, 4.0)  # torque-implied force / stiffness-predicted force


@dataclasses.dataclass
class Baseline:
    mean_mm: float
    sigma_mm: float
    torque_nm: dict

    @property
    def threshold_mm(self) -> float:
        return max(DETECT_MM, SIGMA_K * self.sigma_mm)


def baseline_from(divs, torques: list[dict]) -> Baseline:
    divs = np.asarray(divs, dtype=float)
    keys = torques[0].keys()
    return Baseline(
        float(np.median(divs)),
        max(NOISE_FLOOR_MM, float(np.std(divs))),
        {k: float(np.mean([t[k] for t in torques])) for k in keys},
    )


@dataclasses.dataclass
class Event:
    kind: str  # "contact" | "torque"
    s_mm: float  # commanded depth when it fired
    onset_mm: float | None = None  # estimated commanded depth at which the plane was met
    excess_mm: float = 0.0
    detail: str = ""


class PressDetector:
    """Fed every approach tick: commanded depth s (mm past the standoff), the divergence and the
    torques. Returns an Event when the advance must stop, else None."""

    def __init__(self, hold: Baseline, torque_backstop_nm: float, arm_joints):
        self.hold = hold
        self.backstop = float(torque_backstop_nm)
        self.joints = tuple(arm_joints)
        self.early: list[float] = []
        self.mean = hold.mean_mm
        self.history: deque = deque(maxlen=200)
        self.count = 0
        self.torque_count = 0
        self.peak_excess = 0.0

    def update(self, s_mm: float, div_mm: float, torque: dict) -> Event | None:
        if s_mm <= BASELINE_MM and not self.count:
            # Free space by construction (the standoff exceeds the plane uncertainty by 15 mm):
            # refine the baseline with the same approach's own first millimetres.
            self.early.append(div_mm)
            self.mean = float(np.median([self.hold.mean_mm, *self.early]))
        excess = div_mm - self.mean
        self.peak_excess = max(self.peak_excess, excess)
        self.history.append((s_mm, excess))
        over = excess > self.hold.threshold_mm
        self.count = self.count + 1 if over else 0
        if self.count >= PERSIST_TICKS and self._rising(s_mm, excess):
            return Event("contact", s_mm, s_mm - excess, excess)
        worst, joint = max((abs(torque[j] - self.hold.torque_nm[j]), j) for j in self.joints)
        self.torque_count = self.torque_count + 1 if worst > self.backstop else 0
        if self.torque_count >= TORQUE_TICKS:
            return Event(
                "torque",
                s_mm,
                None,
                excess,
                f"{joint} {worst:.2f} Nm over its free-space torque with divergence {excess:.1f} mm",
            )
        return None

    def _rising(self, s_mm: float, excess: float) -> bool:
        back = [e for s, e in self.history if s <= s_mm - RISE_WINDOW_MM]
        if not back:
            return False
        return excess - back[-1] >= RISE_RATIO * RISE_WINDOW_MM


def settled(samples: list[dict], gap: int = 3) -> list[dict]:
    """Samples whose measured joints moved (nearly) not at all over the previous `gap` readings:
    spacing the comparison keeps the 0.01 deg encoder noise from reading as motion."""
    out = []
    for a, b in zip(samples, samples[gap:], strict=False):
        dt = b["t"] - a["t"]
        if dt <= 0:
            continue
        speed = max(abs(b["pos"][m] - a["pos"][m]) / dt for m in b["arm"])
        if speed <= SETTLED_SPEED_DEG_S:
            out.append(b)
    return out


def judge(
    press1: list[dict],
    press2: list[dict],
    d1: float,
    d2: float,
    normal,
    stiffness: float,
    geometry_problems: list[str],
) -> dict:
    """The verdict on one probe's two presses; `pressN` are hold samples with keys
    t, pos, arm, point (contact point mm), div_excess, force_n (torque-implied, or None)."""
    n = np.asarray(normal, dtype=float)
    reasons = list(geometry_problems)
    s1, s2 = settled(press1), settled(press2)
    if len(s1) < 3 or len(s2) < 3:
        reasons.append(f"not settled: {len(s1)} / {len(s2)} still samples in the holds")
        return {"ok": False, "reasons": reasons}
    p1 = np.median([s["point"] for s in s1], axis=0)
    p2 = np.median([s["point"] for s in s2], axis=0)
    spread = max(float(np.ptp([s["point"] @ n for s in group])) for group in (s1, s2))
    e1 = float(np.median([s["div_excess"] for s in s1]))
    e2 = float(np.median([s["div_excess"] for s in s2]))
    shift = float((p2 - p1) @ n)
    slide = float(np.linalg.norm((p2 - p1) - shift * n))
    growth = (e2 - e1) / max(d2 - d1, 1e-9)
    if spread > SETTLED_SPREAD_MM:
        reasons.append(f"plane point wanders {spread:.1f} mm along the normal while held")
    if abs(shift) > RIGID_MM:
        reasons.append(
            f"plane point moved {shift:+.1f} mm with the deeper press: not rigid glass "
            "(a cable, the post, an object, or a link other than the jaw)"
        )
    if growth < GROWTH_MIN:
        reasons.append(f"the deeper press bent the arm only {growth:.2f} x the extra depth")
    if e1 < PRESS_MIN * d1 or e2 < PRESS_MIN * d2:
        reasons.append(
            f"presses show {e1:.1f} / {e2:.1f} mm of divergence for {d1:.1f} / {d2:.1f} mm "
            "commanded: the surface gave way"
        )
    forces = [s["force_n"] for s in s2 if s.get("force_n") is not None]
    force = float(np.median(forces)) if forces else None
    predicted = stiffness * e2 / 1000
    agree = None
    if force is not None and predicted > 0:
        agree = force / predicted
        if not FORCE_AGREE[0] <= agree <= FORCE_AGREE[1]:
            reasons.append(f"torques imply {force:.1f} N where the stiffness predicts {predicted:.1f} N")
    point = np.median([s["point"] for s in s1 + s2], axis=0)
    return {
        "ok": not reasons,
        "reasons": reasons,
        "point_mm": point.tolist(),
        "press1_point_mm": p1.tolist(),
        "press2_point_mm": p2.tolist(),
        "shift_mm": shift,
        "slide_mm": slide,
        "spread_mm": spread,
        "div_excess_mm": [e1, e2],
        "growth": growth,
        "torque_force_n": force,
        "predicted_force_n": predicted,
        "force_ratio": agree,
        "samples": [len(s1), len(s2)],
    }


def force_from_torques(dtau: np.ndarray, jac: np.ndarray, normal) -> float:
    """Least-squares push (N) along the outward normal explaining the torque change dtau
    (the glass pushes back along -normal; MIT reports the motor torque resisting it)."""
    g = jac.T @ np.asarray(normal, dtype=float)
    denom = float(g @ g)
    return math.nan if denom <= 1e-12 else float(g @ dtau) / denom
