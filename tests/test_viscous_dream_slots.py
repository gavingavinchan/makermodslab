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
"""Dream camera slots for the waterproof arm's wrist-only setup (the record's one camera, `head`), and the
pre-IK slide that keeps Dream's hand targets inside the box and off the tank floor."""

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

REAL = Path(__file__).resolve().parents[1] / "scripts/viscous-dream1/real"
sys.path.insert(0, str(REAL))

import dream_real  # noqa: E402
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


# --- floor slide (core.FloorSlide): the hand's lowest vertex rides above the glass ----------------------

FLOOR = -28.0


def _box(lo, hi, n=3):
    axes = [np.linspace(a, b, n) for a, b in zip(lo, hi, strict=True)]
    return np.stack(np.meshgrid(*axes), axis=-1).reshape(-1, 3)


def _hand():
    """A small synthetic hand in the flange frame (approach +x): a 20 mm half-height body, two 10 mm thick
    fingers reaching 30 mm past the tcp and opening 50 mm each along +-y, and a 15 mm radius roll housing."""
    finger = _box((0.08, 0.0, -0.005), (0.20, 0.01, 0.005))
    ring = np.array(
        [
            [x, 0.015 * math.cos(a), 0.015 * math.sin(a)]
            for x in (-0.05, -0.01)
            for a in np.linspace(0, 2 * math.pi, 9)
        ]
    )
    return core.HandMesh(
        rigid=_box((-0.01, -0.02, -0.02), (0.08, 0.02, 0.02)),
        fingers=((finger, np.array([0.0, 1.0, 0.0])), (finger * [1, -1, 1], np.array([0.0, -1.0, 0.0]))),
        slide_open_m=0.05,
        grip_closed_deg=0.0,
        grip_open_deg=-115.0,
        swept=ring,
        roll_axis=np.array([1.0, 0.0, 0.0]),
    )


TOOL = core.tool_rotation([1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
TCP = np.array([0.17, 0.0, 0.0])
# Flange rotations (columns: the flange's x, y, z in dream_base). DOWN: approach straight down. SIDE: approach
# along -x (forward, level), jaw across. ROLLED: the same, the jaw vertical (flange y up).
DOWN = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
SIDE = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
ROLLED = np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])


def _rot6(flange, rows=1):
    eef = flange @ TOOL
    return np.repeat(np.concatenate([eef[:, 0], eef[:, 1]])[None], rows, 0)


def _slide(sag=0.0):
    return core.FloorSlide(_hand(), TOOL, TCP, FLOOR, buffer_mm=10.0, sag_mm_per_m2=sag)


def test_dream_rot6_is_the_flange_rotation_times_the_tool_axes():
    # The relation the slide relies on, re-fit from a logged live run: Dream's rot6 replies against forward
    # kinematics of the very joint rows the client solved from them (rows the IK tracked within 5 mm).
    run = REAL / "runs/live-20261108T213138Z"
    arm = json.loads((run / "arm/viscous-waterproof-tank.json").read_text())
    urdf = core.parse_urdf((run / "arm/viscous-waterproof-tank.urdf").read_bytes())
    tool = core.tool_rotation(arm["approach"], arm["jaw"])
    angles = []
    for line in (run / "chunks.jsonl").read_text().splitlines():
        chunk = json.loads(line)
        xyz = np.asarray(chunk["per_key"]["action.left_eef_xyz"])
        eef = core.rot6_to_matrix(chunk["per_key"]["action.left_eef_rot6"])
        for i, row in enumerate(chunk["q_l_chunk"]):
            t = core.fk(urdf, "dream_base", arm["flange"], dict(zip(core.ARM, row[:6], strict=True)))
            if np.linalg.norm(t[:3, :3] @ arm["tcp"] + t[:3, 3] - xyz[i]) < 0.005:
                rel = (t[:3, :3] @ tool).T @ eef[i]
                angles.append(math.degrees(math.acos(min(1.0, (np.trace(rel) - 1) / 2))))
    assert len(angles) > 100
    assert np.median(angles) < 1.0 and max(angles) < 3.0


def test_a_downward_gripper_is_raised_so_its_fingertips_clear_the_floor():
    # Fingertips hang 30 mm below the tcp: the tcp must sit at floor + buffer + 30.
    rows = [[0.35, 0.02, -0.050], [0.35, 0.02, 0.100]]
    out, lift = _slide().lift_rows(rows, _rot6(DOWN, 2), [[0.0], [0.0]])
    assert out[0][2] * 1000 == pytest.approx(FLOOR + 10.0 + 30.0)
    assert lift[0] == pytest.approx(12.0 - -50.0) and lift[1] == 0.0
    assert out[1].tolist() == rows[1]  # high enough already: untouched


def test_a_sideways_gripper_needs_no_lift_where_a_downward_one_does():
    rows = [[0.35, 0.0, 0.005]]
    # Level, jaw across: the 20 mm body is the lowest part, 20 mm under the tcp; at z = 5 it clears -28 + 10.
    _, lift = _slide().lift_rows(rows, _rot6(SIDE), [[0.0]])
    assert lift[0] == 0.0
    assert _slide().lowest_mm(rows, _rot6(SIDE), [[0.0]])[0] == pytest.approx(-15.0)
    # The same tcp pointing down puts the fingertips (30 mm under it) at -25: raised 7 mm.
    _, lift = _slide().lift_rows(rows, _rot6(DOWN), [[0.0]])
    assert lift[0] == pytest.approx(7.0)


def test_opening_the_grip_lowers_a_vertical_jaw_and_raises_the_hand():
    # Jaw vertical: the lower finger opens straight down, 50 mm at grip 1. Closed, the body (20 mm) is lowest.
    rows = [[0.35, 0.0, 0.0], [0.35, 0.0, 0.0]]
    low = _slide().lowest_mm(rows, _rot6(ROLLED, 2), [[0.0], [1.0]])
    assert low[0] == pytest.approx(-20.0)
    assert low[1] == pytest.approx(-(10.0 + 50.0))
    _, lift = _slide().lift_rows(rows, _rot6(ROLLED, 2), [[0.0], [1.0]])
    assert lift.tolist() == pytest.approx([FLOOR + 10.0 + 20.0, FLOOR + 10.0 + 60.0])


def test_the_roll_housing_is_bounded_over_every_roll():
    hand = _hand()
    far_up = np.array([[-1.0, 0.0, 0.0]])  # a rigid point a metre behind the flange: never the lowest here
    hand = core.HandMesh(far_up, (), 0.0, 0.0, -115.0, hand.swept, hand.roll_axis)
    # Roll axis level: the 15 mm housing reaches 15 mm under the axis whatever the roll; pointing down, none.
    assert hand.hang_mm(SIDE[None], [0, 0, 0], [0.0])[0] == pytest.approx(15.0)
    assert hand.hang_mm(DOWN[None], [0, 0, 0], [0.0])[0] == pytest.approx(-10.0)


def test_the_predicted_sag_adds_with_reach_squared():
    rows = [[0.40, -0.10, -0.050]]
    _, flat = _slide().lift_rows(rows, _rot6(DOWN), [[0.0]])
    _, sagged = _slide(sag=224.0).lift_rows(rows, _rot6(DOWN), [[0.0]])
    assert sagged[0] - flat[0] == pytest.approx(224.0 * (0.40**2 + 0.10**2))


def test_the_slide_keeps_x_y_orientation_and_grip_exactly():
    rot6 = _rot6(DOWN, 3).tolist()
    grip = [[0.0], [0.4], [1.0]]
    xyz = [[0.35, 0.02, -0.030], [0.36, 0.03, -0.034], [0.37, 0.04, 0.20]]  # inside the box (z >= -35 mm)
    per_key = {"action.left_eef_xyz": xyz, "action.left_eef_rot6": rot6, "action.left_grip": grip}
    reply = {"actions": {"per_key": json.loads(json.dumps(per_key))}}
    box = {"x": (-600.0, -120.0), "y": (-154.0, 140.0), "z": (-35.0, 284.0)}
    slide = dream_real.SlideTransport(SimpleNamespace(request=lambda *a, **k: reply), box, _slide())
    out = slide.request({}, {})["actions"]["per_key"]
    assert out["action.left_eef_rot6"] == per_key["action.left_eef_rot6"]
    assert out["action.left_grip"] == per_key["action.left_grip"]
    for got, asked in zip(out["action.left_eef_xyz"], xyz, strict=True):
        assert got[:2] == asked[:2]
    assert [round(v, 6) for v in slide.lifts_mm[-1][:2]] == [42.0, 46.0] and slide.lifts_mm[-1][2] == 0.0
    assert slide.clamped_rows == [0]


def test_the_runner_builds_the_floor_slide_on_the_guard_mesh():
    arm = json.loads((REAL / "runs/live-20261108T213138Z/arm/viscous-waterproof-tank.json").read_text())
    floor, problem = dream_real.make_floor_slide(arm)
    assert problem is None
    assert floor.floor_mm == core.keepout_floor_mm()
    # Gavin 2026-11-08: the jaws may sag onto the glass and slide on it; the guard's floor press bounds it.
    assert floor.buffer_mm == core.FLOOR_BUFFER_MM == 0.0
    assert {len(f[0]) for f in floor.hand.fingers} and len(floor.hand.fingers) == 2
