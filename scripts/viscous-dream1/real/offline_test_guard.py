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
"""Opt-in pytest guard for the offline gravity build; load with -p offline_test_guard.

Real signal integration tests are skipped; fake handlers remain covered.
Audit hooks reject real signals, device opens and external socket connects.
Local sockets used by fake transport tests remain available.
"""

import sys

import pytest


def audit(event, args):
    if event in ("os.kill", "os.killpg") and args[1] != 0:
        raise ProcessLookupError("offline brief: real process signals forbidden")
    if (
        event == "open"
        and isinstance(args[0], str)
        and args[0].startswith(("/dev/tty", "/dev/cu.", "/dev/video", "/dev/serial", "/dev/can"))
    ):
        raise PermissionError("offline brief: hardware devices forbidden")
    if (
        event == "socket.connect"
        and isinstance(args[1], tuple)
        and args[1][0] not in ("127.0.0.1", "localhost", "::1")
    ):
        raise PermissionError("offline brief: external network disabled in tests")


sys.addaudithook(audit)


def pytest_collection_modifyitems(items):
    real_signals = (
        "test_terminate_tree_reaps_the_grandchild_the_runner_forked",
        "test_terminate_tree_escalates_to_sigkill_when_sigterm_is_ignored",
        "test_sigterm_unwinds_marker_in_spawned_fake_runner",
    )
    for item in items:
        if any(name in item.nodeid for name in real_signals):
            item.add_marker(pytest.mark.skip(reason="offline brief forbids sending any real process signal"))
