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
"""Count a proposed exact hull reduction. Never modify a guard model or run it."""

import sys
from pathlib import Path

import validate as v

if Path("/tmp/tankbox/analysis-deps").is_dir():
    sys.path.append("/tmp/tankbox/analysis-deps")
from scipy.spatial import ConvexHull


def main():
    v.init_worker()
    counts = {}
    for link in v.G.links:
        vertices = v.G.model.vertices[link]
        counts[link] = {
            "all_vertices": len(vertices),
            "hull_vertices": len(ConvexHull(vertices).vertices),
            "exempt_all_faces": all(x is None for x in v.G.required[link].values()),
        }
    v.dump(
        "performance_proposal.json",
        {
            "per_link": counts,
            "total_vertices": sum(x["all_vertices"] for x in counts.values()),
            "checked_hull_vertices": sum(
                x["hull_vertices"] for x in counts.values() if not x["exempt_all_faces"]
            ),
            "applied": False,
        },
    )


if __name__ == "__main__":
    main()
