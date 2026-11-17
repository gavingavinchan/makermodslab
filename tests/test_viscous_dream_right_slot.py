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
"""Offline slot routing and physical-frame proof; no hardware or Dream network."""

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from tests import (
    test_viscous_dream_overlap as overlap,
    test_viscous_dream_real as fixtures,
    test_viscous_dream_slots as slide_fixtures,
)

core, runner = fixtures.core, fixtures.dream_real


def test_left_default_state_and_pinned_plan_ids():
    jmap = core.JointMap.load()
    rest = [0, -1.2, 1.1, -0.6, -0.07, 0.09]
    expected = {
        "state.left_arm": jmap.to_ik(fixtures.POSE),
        "state.left_grip": [core.motor_to_grip(fixtures.POSE["gripper"])],
        "state.right_arm": rest,
        "state.right_grip": [1.0],
        "prompt": "offline",
    }
    assert core.dream_state(fixtures.POSE, jmap, rest, "offline") == expected
    assert core.dream_state(fixtures.POSE, jmap, rest, "offline", "left") == expected
    plan = fixtures.EnvironmentTest().plan
    assert plan()["plan_id"] == "c6c40272db"
    assert plan("dry_air")["plan_id"] == "468746b661"  # 2026-11-17: dry-air tcp held 20 cm out
    assert plan("dry_air") == plan("dry_air", arm_slot="left")
    right = plan("dry_air", arm_slot="right")
    assert right["plan_id"] != plan("dry_air")["plan_id"]
    assert right["arm_slot"] == "right"
    assert "answers are dropped" in right["left_arm"]
    assert "right hand target" in right["trips"]["slide"]


def test_right_state_and_commanded_slots_are_exact_side_swaps():
    jmap = core.JointMap.load()
    rest = [0, -1.2, 1.1, -0.6, -0.07, 0.09]
    left = core.dream_state(fixtures.POSE, jmap, rest, "offline")
    right = core.dream_state(fixtures.POSE, jmap, rest, "offline", "right")
    for side, other in (("left", "right"), ("right", "left")):
        for part in ("arm", "grip"):
            assert right[f"state.{side}_{part}"] == left[f"state.{other}_{part}"]
    row = core.motor_to_ik_row(fixtures.POSE, jmap)
    commanded = core.commanded_slots(row, rest, "right")
    np.testing.assert_array_equal(commanded["q_r_chunk"], row)
    assert commanded["q_l_chunk"] == [*rest, 1.0]


def test_tank_accepts_right_and_keeps_left_plan_ids():
    # Gavin 2026-11-17, after the right slot grasped the cube in dry air: run it in the water tank too.
    left = fixtures.EnvironmentTest().plan("tank")
    right = fixtures.EnvironmentTest().plan("tank", arm_slot="right")
    assert right["arm_slot"] == "right"
    assert "arm_slot" not in left
    assert left["plan_id"] != right["plan_id"]


def test_right_camera_flag_names_are_literal():
    slots, missing = core.resolve_slots(["head", "left"], "head", "black", "left")
    assert slots == {"overhead": "head", "wrist_l": "black", "wrist_r": "left"}
    assert not missing
    assert core.resolve_slots([], "black", "black", "left")[1] == [
        "wrist_r (camera 'left' is not in the record)"
    ]
    assert core.resolve_slots(["head", "left"], "head", "black", "camera:left")[0] == slots
    frames = core.slot_frames(
        {"head": np.full((480, 640, 3), 200, np.uint8), "left": np.full((480, 640, 3), 100, np.uint8)}, slots
    )
    assert frames["video.wrist_r"] == core.jpeg_b64(
        core.to_model_frame(np.full((480, 640, 3), 100, np.uint8))
    )
    assert frames["video.wrist_l"] == core.jpeg_b64(core.black_frame())


@pytest.mark.parametrize("slot", ["left", "right"])
def test_slide_clips_and_lifts_only_our_side(slot):
    other = "right" if slot == "left" else "left"
    per_key = {
        f"action.{side}_{part}": value
        for side in ("left", "right")
        for part, value in (
            ("eef_xyz", [[0.35, 0.02, -0.03], [0.35, -0.3, 0.1]]),
            ("eef_rot6", slide_fixtures._rot6(slide_fixtures.DOWN, 2).tolist()),
            ("grip", [[0], [1]]),
        )
    }
    # Avoid aliasing between the sides' arrays.
    original = json.loads(json.dumps(per_key))
    reply = {"actions": {"per_key": json.loads(json.dumps(per_key))}}
    transport = runner.SlideTransport(
        SimpleNamespace(request=lambda *a, **k: reply),
        {"x": (-600.0, -120.0), "y": (-154.0, 140.0), "z": (-35.0, 284.0)},
        slide_fixtures._slide(),
        slot,
    )
    out = transport.request({}, {})["actions"]["per_key"]
    assert out[f"action.{slot}_eef_xyz"] != original[f"action.{slot}_eef_xyz"]
    assert transport.clamped_rows == [1]
    assert transport.lifts_mm[0][0] > 0
    for part in ("eef_xyz", "eef_rot6", "grip"):
        assert out[f"action.{other}_{part}"] == original[f"action.{other}_{part}"]
    for part in ("eef_rot6", "grip"):
        assert out[f"action.{slot}_{part}"] == original[f"action.{slot}_{part}"]


def test_real_adapter_keeps_custom_arm_for_both_sides_and_selects_right_slide(tmp_path):
    client = mock.MagicMock()
    dream = SimpleNamespace(
        DreamClient=mock.Mock(return_value=client), DreamSlowNetworkWarning=RuntimeWarning
    )
    with mock.patch.dict(sys.modules, dream=dream), mock.patch.dict(os.environ):
        adapter = runner.DreamReal(
            runner.DEFAULT_ARM_FILE,
            tmp_path,
            offline=False,
            environment=runner.ENVIRONMENTS["dry_air"],
            arm_slot="right",
        )
    assert adapter.config["ik"]["arm"] == str(runner.DEFAULT_ARM_FILE)
    assert adapter.slide.KEY == "action.right_eef_xyz"
    assert adapter.slide.ROT == "action.right_eef_rot6"
    assert adapter.slide.GRIP == "action.right_grip"


class RightRecordingStub(fixtures.RecordingStub):
    def __init__(self, *args):
        super().__init__(*args)
        self.arm_slot = "right"
        self.replies = []

    def request(self, commanded):
        reply = super().request(commanded)
        # A malicious phantom reply would instantly trip if selected for execution.
        reply["q_l_chunk"] = np.full((48, 7), 100.0)
        reply["per_key"] = {"action.left_eef_xyz": [[100.0, 200.0, 300.0]] * 48}
        self.replies.append(reply)
        return reply


@pytest.mark.parametrize("async_mode", [False, True])
def test_right_episode_executes_right_drops_and_logs_raw_left_including_overlap(async_mode):
    logged = []
    write = runner.Episode._write

    def capture(ep, path, row):
        if path == ep.chunks:
            logged.append(row)
        return write(ep, path, row)

    with (
        mock.patch.object(fixtures, "RecordingStub", RightRecordingStub),
        mock.patch.object(runner.Episode, "_write", capture),
    ):
        fixture, summary, arm, seam = overlap.run_episode(overlap=async_mode, changes={"arm_slot": "right"})
    assert summary["reason"] == "call cap 2 reached"
    fixture.assert_stopped_safely(arm)
    assert summary["substeps_sent"] > 0
    assert len(logged) == 2
    for state in fixture.dream.states:
        assert state["state.left_arm"] == fixture.ep.right_rest
        assert state["state.left_grip"] == [1.0]
    for commanded in fixture.dream.commanded:
        assert commanded["q_l_chunk"] == [*fixture.ep.right_rest, 1.0]
    for chunk, reply in zip(logged, fixture.dream.replies, strict=True):
        assert chunk["arm_slot"] == "right"
        np.testing.assert_array_equal(chunk["raw_reply"]["q_l_chunk"], reply["q_l_chunk"])
        assert chunk["raw_reply"]["per_key"] == reply["per_key"]
        assert chunk["q_l_chunk"] == chunk["raw_reply"]["q_l_chunk"]
        assert all(max(abs(v) for v in step.values()) < 360 for step in chunk["substeps"])
    if async_mode:
        assert len(logged[1]["q_r_chunk"]) == 36
        assert len(logged[1]["raw_reply"]["q_r_chunk"]) == 48
        assert len(seam.windows) == 1
    else:
        assert not seam.windows


def test_right_ignores_phantom_but_refuses_a_jump_in_our_chunk():
    with mock.patch.object(fixtures, "RecordingStub", RightRecordingStub):
        fixture = fixtures.EpisodeOnFakesTest()
        summary, arm = fixture.episode(
            environment=runner.ENVIRONMENTS["dry_air"],
            settings_changes={"arm_slot": "right"},
            mutate=runner.jump_mutation,
        )
    assert [trip["name"] for trip in summary["trips"]] == ["jump"]
    fixture.assert_stopped_safely(arm)


@pytest.mark.parametrize("solve", ["tick", "traj"])
def test_installed_client_custom_arm_fk_right_eef_ik_roundtrip(solve, tmp_path):
    site = next(
        Path("~/viscous-dream-real/.venv")
        .expanduser()
        .glob("lib/python3.*/site-packages/dream/client_ik.py"),
        None,
    )
    if site is None:
        pytest.skip("optional installed Dream client unavailable")
    out = tmp_path / "proof.json"
    env = {key: value for key, value in os.environ.items() if key != "DREAM_API_KEY"}
    result = subprocess.run(
        [sys.executable, str(runner.HERE / "check_slot_roundtrip.py"), "--solve", solve, "--out", str(out)],
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    proof = json.loads(out.read_text())
    assert proof["side_base"] is None
    assert max(proof["slots"]["right"]["max_joint_error_deg"]) < 0.5
