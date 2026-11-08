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
"""Find specific binding faces, then independently verify the granted ramps.

The guard is used to select adversarial requests. The verdict on each granted
segment comes only from validate.verify_segment's independently parsed planes.
"""

import json
import math
from collections import Counter
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import validate as v

SEARCH = None


def checked_search_limit(start, target):
    """Search calls also produce grants; independently verify each one immediately."""
    decision = v.G.limit(start, target)
    SEARCH["requests"] += 1
    SEARCH["counts"][decision.status] += 1
    if decision.status in ("ok", "clamped"):
        ns, worst, bad = v.verify_segment(start, decision.target, v.independent_setup(v.G.config, v.G.model))
        SEARCH["independent_samples"] += ns
        SEARCH["worst_independent_slack_mm"] = min(SEARCH["worst_independent_slack_mm"], worst[0])
        if bad:
            SEARCH["violations"].append(
                {"previous": start, "requested": target, "decision": v.asdict(decision), "violation": bad}
            )
    return decision


def search_face(face, pool):
    g = v.G
    candidates = sorted(pool, key=lambda q: min(x for (_link, f), x in g.clearances(q).items() if f == face))
    for start in candidates[:100]:
        for m in v.tg.ARM:
            for sign in (-1, 1):
                target = {**start, m: float(np.clip(start[m] + sign * 19.9, *g.limits[m]))}
                if target[m] == start[m]:
                    continue
                end = g.check_pose(target)
                if end.ok or end.face != face:
                    continue
                d = checked_search_limit(start, target)
                if d.reason == "keepout" and d.verdict.face == face:
                    n = math.ceil(max(abs(target[m] - start[m]) for m in v.tg.MOTORS) / 0.05)
                    return (
                        "directed_" + face,
                        v.SEED,
                        start,
                        [v.lerp(start, target, k / n) for k in range(1, n + 1)],
                    )
    return None


def main():
    global SEARCH
    SEARCH = {
        "category": "directed_search_commands",
        "requests": 0,
        "counts": Counter(),
        "independent_samples": 0,
        "worst_independent_slack_mm": math.inf,
        "violations": [],
    }
    v.init_worker()
    pool = json.loads((v.OUT / "pose_pool.json").read_text())["safe_poses"]
    jobs = []
    miss = []
    for face in v.tg.FACES:
        job = search_face(face, pool)
        if job:
            jobs.append(job)
            print("found", face, job[2], flush=True)
        else:
            miss.append(face)
            print("not found", face, flush=True)
    folded = []
    for q in pool:
        transforms = v.geo.forward_kinematics(v.G.model, q)
        # The forearm returns toward +x, the near glass, from its elbow origin.
        if transforms["wrist_pitch_link"][1][0] > transforms["forearm_link"][1][0] + 0.02:
            folded.append(q)
    job = search_face("near", folded)
    if job:
        jobs.append(("directed_folded_back_near", *job[1:]))
        print("found folded-back near", job[2], flush=True)
    else:
        miss.append("folded_back_near")
    folded_start = {**v.WORK, "shoulder_lift": -174.0, "elbow_flex": 250.0, "wrist_flex": -120.0}
    folded_target = {**folded_start, "shoulder_lift": -100.0}
    jobs.append(
        (
            "folded_back_near_attempt",
            v.SEED,
            folded_start,
            [v.lerp(folded_start, folded_target, k / 1480) for k in range(1, 1481)],
        )
    )
    # Seek a pose where opening fingers, rather than moving the padded wrist,
    # reaches a wall. Both endpoints share exactly the same six arm angles.
    for face in ("left", "right", "near"):
        closed = []
        for q in pool:
            q = {**q, "gripper": v.G.limits["gripper"][1]}
            if v.G.start_gate(q).ok:
                closed.append(q)
        candidates = sorted(
            closed,
            key=lambda q: min(
                x for (link, f), x in v.G.clearances(q).items() if f == face and link.startswith("gripper")
            ),
        )
        found = None
        for start in candidates[:150]:
            # First pan toward wall while the closed hand remains feasible.
            for offset in np.linspace(-8, 8, 33):
                q = {
                    **start,
                    "shoulder_pan": float(
                        np.clip(start["shoulder_pan"] + offset, *v.G.limits["shoulder_pan"])
                    ),
                }
                if not v.G.start_gate(q).ok:
                    continue
                target = {**q, "gripper": -115.0}
                d = checked_search_limit(q, target)
                if d.reason == "keepout" and d.verdict.face == face and d.verdict.link.startswith("gripper"):
                    n = math.ceil(abs(target["gripper"] - q["gripper"]) / 0.1)
                    found = (
                        "directed_gripper_open_" + face,
                        v.SEED,
                        q,
                        [v.lerp(q, target, k / n) for k in range(1, n + 1)],
                    )
                    break
            if found:
                break
        if found:
            jobs.append(found)
            print("found gripper", face, flush=True)
        else:
            miss.append("gripper_open_" + face)
            print("not found gripper", face, flush=True)
    result = [SEARCH]
    with ProcessPoolExecutor(8, initializer=v.init_worker) as executor:
        for r in executor.map(v.exercise, jobs):
            result.append(r)
            print(r["category"], r["counts"], r["binding_link_faces"], flush=True)
    v.dump(
        "directed_summary.json",
        {
            "campaigns": result,
            "not_found": miss,
            "requests": sum(r["requests"] for r in result),
            "independent_samples": sum(r["independent_samples"] for r in result),
            "violations": [x for r in result for x in r["violations"]],
        },
    )


if __name__ == "__main__":
    main()
