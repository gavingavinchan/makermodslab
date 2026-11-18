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
"""Offline survey of a staged wall probe over mount positions, x and height.

Run in the Lab environment, e.g. python survey.py --workers 8 --out survey.json.
The post remains fixed. Side-to-side translations preserve the nominal pane width.
Each candidate uses the commissioned joint limits and complete transit/press checks.
"""

import argparse
import itertools
import json
import multiprocessing
import tempfile
from pathlib import Path

import context
import geometry
import planner


def check(args):
    global survey_planner
    left, u, v = args
    if survey_planner is None:
        folder = tempfile.TemporaryDirectory()
        try:
            model = context.offline_model(Path(folder.name))
        finally:
            folder.cleanup()
        tank, keepout = geometry.load_inputs()
        survey_planner = planner.Planner(model, keepout, model.limits, context.arm_file_rest(model))
    tank, keepout = geometry.load_inputs()
    prior = geometry.nominal_estimate(tank, keepout)
    right = left - prior.dims["inner_width"]
    touches = [{"label": "floor", "point_mm": [-380, 0, prior.floor]}]
    touches.extend({"label": "right", "point_mm": [x, right, 80]} for x in (-500, -330))
    estimate = planner.update_estimate(prior, touches)
    survey_planner.good.clear()
    planner.sync_guard(survey_planner.model.guard, estimate, survey_planner.nominal, True)
    spec = planner.ProbeSpec("left", "left", u, v)
    plan = survey_planner.plan(spec, estimate, touches, survey_planner.hub(estimate))
    return {
        "left_mm": left,
        "x_mm": u,
        "height_mm": v,
        "ok": plan.ok,
        "reasons": plan.reasons,
        "tilt_deg": plan.tilt_deg,
        "lead_mm": plan.lead_mm,
        "window_mm": plan.window_mm,
        "max_pan_deg": max((q["shoulder_pan"] for q in plan.nodes or []), default=None),
    }


survey_planner = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    cells = itertools.product((207, 217, 227), range(-480, -279, 40), (80, 120, 160))
    with multiprocessing.get_context("spawn").Pool(args.workers) as pool:
        rows = pool.map(check, cells, chunksize=1)
    args.out.write_text(json.dumps(rows, indent=2) + "\n")
    for left in (207, 217, 227):
        good = [r for r in rows if r["ok"] and r["left_mm"] == left]
        print(left, "mm:", len(good), "feasible", sorted({r["x_mm"] for r in good}))


if __name__ == "__main__":
    main()
