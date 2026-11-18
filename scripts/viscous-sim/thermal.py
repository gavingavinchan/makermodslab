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
"""RS00 overload proxy calibrated on the 2026-11-14 wet shoulder collapse."""

import argparse
import json
from pathlib import Path

import numpy as np

KNEE_NM = 4.5
COOLING_S = 300.0
BUDGET_NM2_S = 595.936
AMBIENT_C = 22.0


class Thermal:
    def __init__(self, dt, n=7):
        self.set_timestep(dt)
        self.temp = np.full(n, AMBIENT_C)
        self.dose = np.zeros(n)
        self.limp = np.zeros(n, dtype=bool)

    def set_timestep(self, dt):
        self.decay = np.exp(-dt / COOLING_S)
        self.gain = -COOLING_S * np.expm1(-dt / COOLING_S)
        self.temp_decay = np.exp(-dt / 63.506)
        self.temp_gain = 1.1573 * (1 - self.temp_decay)

    def advance(self, torque):
        power = np.square(torque)
        self.dose *= self.decay
        self.dose += self.gain * np.maximum(power - KNEE_NM**2, 0)
        self.temp[:] = AMBIENT_C + (self.temp - AMBIENT_C) * self.temp_decay + self.temp_gain * power
        self.limp |= self.dose >= BUDGET_NM2_S


def replay(directory):
    """Exact zero-order-hold integration at the recorded irregular sample times."""
    results = []
    for run in sorted(directory.iterdir()):
        ticks = run / "ticks.jsonl"
        if not ticks.exists():
            continue
        with ticks.open() as f:
            rows = [row for line in f if (row := json.loads(line)).get("torque")]
        if len(rows) < 2:
            continue
        names = tuple(rows[0]["torque"])
        state = Thermal(0, len(names))
        state.temp[:] = [rows[0]["temp"][n] for n in names]
        shoulder = names.index("shoulder_lift")
        temp_errors = []
        peak = 0
        trips = {}
        for prev, curr in zip(rows, rows[1:], strict=False):
            dt = curr["t"] - prev["t"]
            state.set_timestep(dt)
            state.advance(np.array([prev["torque"][name] for name in names]))
            temp_errors.append(state.temp[shoulder] - curr["temp"]["shoulder_lift"])
            peak = max(peak, state.dose.max())
            for i in np.flatnonzero(state.limp):
                trips.setdefault(names[i], round(curr["t"] - rows[0]["t"], 3))
        results.append(
            {
                "run": run.name,
                "peak_nm2_s": round(peak, 3),
                "trips": trips,
                "shoulder_temp_rmse_c": round(float(np.sqrt(np.mean(np.square(temp_errors)))), 3),
                "final_shoulder_nm2_s": round(state.dose[names.index("shoulder_lift")], 3),
            }
        )
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    print(json.dumps(replay(parser.parse_args().archive), indent=2))
