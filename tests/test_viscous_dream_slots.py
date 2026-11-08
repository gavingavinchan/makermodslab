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
"""Dream camera slots for the waterproof arm's wrist-only setup (the record's one camera, `head`)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/viscous-dream1/real"))

import dream_real_core as core  # noqa: E402


def test_wrist_only_is_deliberate_not_missing():
    slots, missing = core.resolve_slots(["head"], "black", "head", "black")
    assert slots == {"overhead": "black", "wrist_l": "head", "wrist_r": "black"}
    assert missing == []


def test_an_unchosen_or_unknown_camera_is_still_missing():
    _, missing = core.resolve_slots(["head"], None, "head", "black")
    assert missing == ["overhead (none chosen)"]
    _, missing = core.resolve_slots(["head"], "black", "wrist", "black")
    assert missing == ["wrist_l (camera 'wrist' is not in the record)"]


def test_hand_targets_slide_onto_the_box():
    box = {"x": (-600.0, -120.0), "y": (-154.0, 140.0), "z": (-5.0, 284.0)}
    # dream_base metres: physical x = -x, y = -y. Row 0 inside; row 1 below the floor limit; row 2 too far left.
    rows = [[0.40, 0.00, 0.10], [0.40, 0.02, -0.03], [0.30, -0.20, 0.05]]
    out, changed = core.clamp_eef_rows(rows, box, margin_mm={"x": 2.0, "y": 2.0, "z": 2.0})
    assert changed == 2
    assert out[0].tolist() == rows[0]
    assert out[1][2] == pytest.approx(-0.003) and out[1][:2].tolist() == rows[1][:2]  # only z rides the limit
    assert -out[2][1] * 1000 == pytest.approx(138.0) and out[2][0] == rows[2][0]  # y slides, x kept
