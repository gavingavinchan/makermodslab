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
"""The stop button reopens where it was left, and a bad saved geometry can never hide or shrink it."""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "viscous-dream1" / "real"))

import stop_button  # noqa: E402

from makermodslab.utils import config  # noqa: E402


def test_round_trip_restores_size_and_place(tmp_path):
    path = tmp_path / "dream" / "stop_button_geometry.json"
    stop_button.save_geometry(path, "700x520+1210+-35")
    assert stop_button.load_geometry(path) == "700x520+1210+-35"
    assert not list(path.parent.glob("*.tmp"))


@pytest.mark.parametrize(
    "content",
    [
        None,  # no file
        "not json",
        json.dumps({"geometry": "100x80+40+40"}),  # too small to hit
        json.dumps({"geometry": "580x480+99999+40"}),  # off every screen
        json.dumps({"geometry": "580x480"}),  # no position
        json.dumps({"geometry": 5}),
        json.dumps(["580x480+40+40"]),
    ],
)
def test_bad_saved_geometry_opens_the_default(tmp_path, content):
    path = tmp_path / "g.json"
    if content is not None:
        path.write_text(content)
    assert stop_button.load_geometry(path) == stop_button.DEFAULT_GEOMETRY


def test_invalid_geometry_is_never_saved(tmp_path):
    path = tmp_path / "g.json"
    stop_button.save_geometry(path, "1x1+0+0")
    assert not path.exists()


def test_save_never_raises(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("")
    stop_button.save_geometry(blocker / "sub" / "g.json", "580x480+40+40")  # parent is a file


def test_path_comes_from_config():
    assert stop_button.geometry_file() == config.DREAM_STOP_BUTTON_GEOMETRY_FILE
    assert config.DREAM_STOP_BUTTON_GEOMETRY_FILE.parent == Path(config.MAKERMODSLAB_HOME) / "dream"
