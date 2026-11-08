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
"""Independently inspect every granted sim policy AND synthetic approach segment."""

import json
import math
from collections import Counter
from concurrent.futures import ProcessPoolExecutor

import validate as v


def one(saved):
    rows, _, _, hashes = v.sim_rows(v.RUNS / saved["run"], saved["side"])
    assert hashes == saved["sha256"]
    requests = [dict(zip(v.tg.MOTORS, row.tolist(), strict=True)) for row in v.sim_map(rows)]
    accepted_start = v.G.start_gate(requests[0]).ok
    runner = v.tg.GuardedTargets(v.G, requests[0] if accepted_start else v.REST)
    setup = v.independent_setup(v.G.config, v.G.model)
    counts = Counter()
    verified = Counter()
    samples = Counter()
    violations = []
    worst = math.inf
    if not accepted_start:
        n = math.ceil(max(abs(requests[0][m] - v.REST[m]) for m in v.tg.MOTORS))
        ramp = [v.lerp(v.REST, requests[0], k / n) for k in range(1, n + 1)]
    else:
        ramp = []
    for stage, sequence in [("ramp", ramp), ("policy", requests)]:
        for i, q in enumerate(sequence):
            prev = dict(runner.target)
            d = runner.step(q)
            if stage == "policy":
                counts[d.status] += 1
            if d.status in ("ok", "clamped"):
                ns, w, bad = v.verify_segment(prev, d.target, setup)
                samples[stage] += ns
                verified[stage] += 1
                worst = min(worst, w[0])
                if bad:
                    violations.append(
                        {
                            "stage": stage,
                            "index": i,
                            "previous": prev,
                            "requested": q,
                            "decision": v.asdict(d),
                            "violation": bad,
                        }
                    )
    assert dict(counts) == saved["counts"], (saved["run"], saved["side"], counts, saved["counts"])
    return {
        "run": saved["run"],
        "side": saved["side"],
        "segments": dict(verified),
        "samples": dict(samples),
        "worst_slack_mm": None if math.isinf(worst) else worst,
        "violations": violations,
    }


def main():
    saved = json.loads((v.OUT / "sim_summary.json").read_text())["runs"]
    results = []
    with ProcessPoolExecutor(12, initializer=v.init_worker) as pool:
        for r in pool.map(one, saved):
            results.append(r)
            print(r["run"], r["side"], r["segments"], r["samples"], len(r["violations"]), flush=True)
    v.dump(
        "sim_independent_verification.json",
        {
            "runs": results,
            "verified_segments": sum(sum(r["segments"].values()) for r in results),
            "samples": sum(sum(r["samples"].values()) for r in results),
            "violations": [dict(run=r["run"], side=r["side"], **x) for r in results for x in r["violations"]],
        },
    )


if __name__ == "__main__":
    main()
