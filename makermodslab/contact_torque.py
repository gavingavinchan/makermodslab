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
"""Contact torque on a CAN follower: what its motors push with beyond holding the arm up.

The haptic signal for the Viscous arm. RS00 torque telemetry is the motor's
whole output (gravity, friction, drag and contact), and the pilot should feel
only contact, so:

    contact = reported torque - (fitted gravity torque at the measured pose + telemetry bias)

This holds whether or not gravity feedforward is on: feedforward changes how
the motor's output splits between ``tau_ff`` and stiffness, not what it is.
What is left in free motion is friction (~0.1 N m static on the dry arm) and
model error (~0.2 N m), which sets the floor a contact must clear. Fits may
include a smooth Coulomb + viscous friction model. It is subtracted only when
callers supply commanded velocities in degrees/s; static callers are unchanged.
This compensation belongs only to the contact signal, never motor feedforward.
"""

from __future__ import annotations

import math
from pathlib import Path


class ContactEstimator:
    def __init__(self, model, friction=None):
        self.model = model
        self.joints = model.joints
        self.friction = friction or {}

    @classmethod
    def from_fit(cls, path: str | Path) -> ContactEstimator:
        from .gravity_feedforward import load_model

        fit, model = load_model(path, telemetry_offsets=True)
        return cls(model, fit.get("friction"))

    def friction_torque(self, velocities_deg_s: dict[str, float]) -> dict[str, float]:
        """Smooth, odd Coulomb + viscous torque; zero at zero commanded velocity.

        Missing joint velocities leave that joint uncompensated. A non-finite
        supplied velocity propagates NaN instead of reporting spurious contact.
        """
        out = dict.fromkeys(self.joints, 0.0)
        for name, terms in self.friction.items():
            velocity = velocities_deg_s.get(name, 0.0)
            if (
                isinstance(velocity, bool)
                or not isinstance(velocity, (int, float))
                or not math.isfinite(velocity)
            ):
                out[name] = math.nan
                continue
            # Hold the term at the fastest speed it was identified at: beyond
            # that it is extrapolation, and a wrong friction torque reads as contact.
            limit = terms.get("max_speed_deg_s")
            if limit is not None:
                velocity = max(-limit, min(limit, velocity))
            out[name] = terms["coulomb_nm"] * math.tanh(velocity / terms["v_smooth_deg_s"]) + (
                terms["viscous_nm_per_deg_s"] * velocity
            )
        return out

    def estimate(
        self,
        positions_deg: dict[str, float],
        torques_nm: dict[str, float],
        velocities_deg_s: dict[str, float] | None = None,
    ) -> dict[str, float]:
        """Per modelled joint, N m; NaN for a joint whose torque reading is missing."""
        expected = self.model.torque(positions_deg)
        if velocities_deg_s is not None:
            friction = self.friction_torque(velocities_deg_s)
            expected = {name: expected[name] + friction[name] for name in self.joints}
        out = {}
        for name in self.joints:
            measured = torques_nm.get(name)
            ok = isinstance(measured, (int, float)) and not isinstance(measured, bool)
            out[name] = measured - expected[name] if ok and math.isfinite(measured) else math.nan
        return out

    def from_observation(
        self, obs: dict, velocities_deg_s: dict[str, float] | None = None
    ) -> dict[str, float]:
        """From a follower observation (``<joint>.pos`` degrees, ``<joint>.torque`` N m)."""
        return self.estimate(
            {name: obs[f"{name}.pos"] for name in self.joints},
            {name: obs.get(f"{name}.torque") for name in self.joints},
            velocities_deg_s,
        )
