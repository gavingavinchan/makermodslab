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
"""No hardware: torque-off bus contract, reciprocal ownership, clock and disk."""

import json
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from makermodslab import calibrate, sessions, tank_measure as tm
from makermodslab.api_errors import ApiError
from makermodslab.arms.viscous import VISCOUS
from makermodslab.utils import config as cfg
from tests.test_tank_geometry import synthetic


class FakeBus:
    def __init__(self, positions):
        self.positions = positions
        self.calls = []
        self.fail = False

    def connect(self, *, handshake):
        assert handshake is False, "Never handshake"
        self.calls.append("open_torque_off")

    def disable_torque(self):
        self.calls.append("disable")

    def enable_torque(self, *args):
        raise AssertionError("Never enable torque")

    def write(self, *args):
        raise AssertionError("Never write Goal_Position or any register")

    def handshake(self):
        raise AssertionError("Never handshake")

    def disconnect(self, disable):
        assert disable is False
        self.calls.append("disconnect")

    def read(self, register, name):
        assert register == "Present_Position"
        if self.fail:
            raise ConnectionError("missed joint")
        self.calls.append("read:" + name)
        return self.positions[name]


@pytest.fixture
def rig(tmp_lerobot_home, tmp_path, monkeypatch):
    sessions.tracker.reset()
    monkeypatch.setattr("makermodslab.arms.can_common.time", SimpleNamespace(sleep=lambda _: None))
    monkeypatch.setattr(cfg, "TANK_MEASURE_PATH", str(tmp_path / "tank"))
    library = tmp_path / "calibration"
    library.mkdir()
    (library / "viscous_waterproof_01.json").write_text("{}")
    monkeypatch.setattr(VISCOUS, "follower_calibration_dir", lambda: str(library))
    cfg.save_robot_record(
        "aquarium",
        {
            "arm_type": "viscous",
            "mode": "single",
            "follower_port": "/dev/FAKE",
            "follower_config": "viscous_waterproof_01",
        },
        allow_create=True,
    )
    profiles = Path(cfg.MAKERMODSLAB_HOME) / "viscous_profiles"
    profiles.mkdir(parents=True, exist_ok=True)
    evidence = tm.geo.ROOT / "configs/viscous/viscous_waterproof_01/evidence/lab-hardware-profile.json"
    (profiles / "viscous_waterproof_01.json").write_text(evidence.read_text())
    now = [0.0]
    manager = tm.TankMeasurement(clock=lambda: now[0], utcnow=lambda: datetime(2026, 11, 7, tzinfo=UTC))
    monkeypatch.setattr(tm, "manager", manager)
    monkeypatch.setattr("makermodslab.monitored_can.bridge_metadata", lambda *args: None)
    pose = json.loads(tm.geo.DEFAULT_TANK.read_text())["reference_poses_motor_deg"]["read"]["value"]
    # In the tank lift reports one turn above teleop's -201.83 degrees.
    raw = {**pose, "shoulder_lift": pose["shoulder_lift"] + 360, "gripper": 0.0}
    bus = FakeBus(raw)
    config = VISCOUS.single_follower_config("/dev/FAKE", "viscous_waterproof_01")
    device = SimpleNamespace(
        bus=bus, config=config, connect=Mock(side_effect=AssertionError("Never follower.connect"))
    )
    monkeypatch.setattr("lerobot.robots.make_robot_from_config", lambda _: device)
    yield manager, now, bus, device
    manager.stop()
    sessions.tracker.reset()


def open_rig(rig):
    manager, now, bus, device = rig
    manager.start("aquarium", run_worker=False)
    manager.device = VISCOUS.open_follower_torque_off("/dev/FAKE", "viscous_waterproof_01")
    return manager, now, bus, device


def fill_still(manager, now):
    for i in range(12):
        now[0] = i * 0.1
        manager.poll_once()


def test_worker_never_energizes_and_cleans_up_every_exit(rig, monkeypatch):
    manager, now, bus, device = rig
    manager.start("aquarium", run_worker=False)
    monkeypatch.setattr(manager.stop_event, "wait", lambda _: manager.stop_event.set())
    manager._worker()
    assert manager.error is None, "The worker must not swallow a forbidden energizing call"
    assert not manager.active
    assert bus.calls[0:2] == ["open_torque_off", "disable"]
    assert {x for x in bus.calls if x.startswith("read:")} == {
        "read:" + n for n in device.config.motor_can_ids
    }
    assert bus.calls[-2:] == ["disable", "disconnect"]
    device.connect.assert_not_called()


def test_failed_poll_is_stale_requires_new_full_second_and_does_not_crash(rig):
    manager, now, bus, _ = open_rig(rig)
    fill_still(manager, now)
    assert manager.status()["still"]
    bus.fail = True
    manager.poll_once()
    assert manager.active and manager.status()["stale"]
    with pytest.raises(ApiError, match="fresh"):
        manager.capture("right")
    bus.fail = False
    manager.poll_once()
    assert not manager.status()["still"]


def test_capture_averages_unwrapped_joint_angles_and_delete(rig):
    manager, now, _, _ = open_rig(rig)
    fill_still(manager, now)
    status = manager.capture("near")
    assert status["positions"]["shoulder_lift"] == pytest.approx(-201.83)
    assert status["touches"][0]["positions"]["shoulder_lift"] == pytest.approx(-201.83)
    assert len(manager.delete(0)["touches"]) == 0
    with pytest.raises(ApiError):
        manager.delete(-1)
    now[0] += 0.5
    assert manager.status()["stale"]
    with pytest.raises(ApiError):
        manager.capture("near")


def test_every_reading_and_action_is_traced_to_disk_saved_or_not(rig):
    manager, now, bus, _ = open_rig(rig)
    fill_still(manager, now)
    manager.capture("near")
    manager.delete(0)
    bus.fail = True
    manager.poll_once()
    path = Path(manager.status()["trace_path"])
    manager.stop()
    assert path.parent == Path(cfg.TANK_MEASURE_PATH) / "aquarium" / cfg.TANK_TRACE_DIR
    lines = [json.loads(x) for x in path.read_text().splitlines()]
    events = [x.get("event") for x in lines]
    assert events[:2] == ["start", "offsets"]
    assert events[-4:] == ["capture", "delete", "read_error", "stop"]
    samples = [x for x in lines if "event" not in x]
    assert len(samples) == 12 == lines[-1]["samples"]
    assert samples[0]["positions_deg"]["shoulder_lift"] == pytest.approx(
        samples[0]["raw_deg"]["shoulder_lift"] - 360
    )
    assert len(samples[0]["probe_mm"]) == 3 and "yaw_link" in samples[0]["link_origins_mm"]
    assert not (Path(cfg.TANK_MEASURE_PATH) / "aquarium" / cfg.TANK_LATEST_FILE).exists()


def test_moving_probe_open_gripper_and_invalid_label_refused(rig):
    manager, now, bus, _ = open_rig(rig)
    for i in range(12):
        now[0] = i * 0.1
        bus.positions["shoulder_pan"] += 1
        manager.poll_once()
    with pytest.raises(ApiError, match="still"):
        manager.capture("left")
    bus.positions["gripper"] = -30
    manager.samples.clear()
    fill_still(manager, now)
    with pytest.raises(ApiError, match="Close"):
        manager.capture("left")
    with pytest.raises(ApiError, match="Unknown"):
        manager.capture("garbage")
    with pytest.raises(ApiError, match="Fit the walls"):
        manager.capture("envelope:yaw_link:near")


def test_idle_timeout_without_sleep_and_sessions_stop(rig):
    manager, now, bus, _ = open_rig(rig)
    assert calibrate.calibration_is_active()
    assert sessions.held_by() == "calibration"
    now[0] += tm.IDLE_SECONDS + 1
    assert not manager.status()["active"]
    assert bus.calls[-2:] == ["disable", "disconnect"]
    manager.start("aquarium", run_worker=False)
    assert sessions._dispatch_stop("calibration")["success"]
    assert not calibrate.calibration_is_active()


@pytest.mark.parametrize(
    "holder",
    [
        "teleoperation",
        "recording",
        "inference",
        "remote_inference",
        "replay",
        "calibration",
        "auto_calibration",
        "wiggle",
        "hosting",
        "remote_teleoperation",
    ],
)
def test_all_other_holders_refuse_without_bus_open(rig, monkeypatch, holder):
    manager, _, bus, _ = rig
    monkeypatch.setattr(sessions, "held_by", lambda: holder)
    with pytest.raises(ApiError) as exc:
        manager.start("aquarium", run_worker=False)
    assert exc.value.code == f"robot.busy.{holder}"
    assert not bus.calls


def test_training_and_monitor_bridge_refused(rig, monkeypatch):
    manager, _, bus, _ = rig
    monkeypatch.setattr("makermodslab.jobs.training_is_active", lambda: "run")
    with pytest.raises(ApiError) as exc:
        manager.start("aquarium", run_worker=False)
    assert exc.value.code == "robot.busy.training"
    monkeypatch.setattr("makermodslab.jobs.training_is_active", lambda: None)
    monkeypatch.setattr("makermodslab.monitored_can.bridge_metadata", lambda *args: {"pid": 1})
    with pytest.raises(ApiError, match="monitor bridge"):
        manager.start("aquarium", run_worker=False)
    assert not bus.calls


def test_save_roundtrip_latest_without_hardware_and_history(rig, client, monkeypatch):
    manager, _, bus, _ = rig
    assert client.get("/api/v1/tank-measure/latest?robot_name=aquarium").json()["measurement"] is None
    manager.start("aquarium", run_worker=False)
    with pytest.raises(ApiError, match="Cannot save"):
        manager.save()
    manager.touches = synthetic()
    # Exact axis-aligned fit for today's offline reader (nonzero is rejected).
    original = manager._geometry

    def axis_aligned():
        fit, tower, pads = original()
        fit["yaw_deg"] = 0.0
        return fit, tower, pads

    monkeypatch.setattr(manager, "_geometry", axis_aligned)
    exported = manager.save()["measurement"]
    folder = tm.storage_dir("aquarium")
    assert len(list((folder / cfg.TANK_HISTORY_DIR).glob("*.json"))) == 1
    assert (folder / cfg.TANK_ARM_EXTRAS_FILE).exists()
    manager.stop()
    assert client.get("/api/v1/tank-measure/latest?robot_name=aquarium").json()["measurement"] == exported
    assert not bus.calls
    from tests.test_tank_geometry import offline

    module = offline.__wrapped__()
    assert module.load_tank(folder / cfg.TANK_LATEST_FILE)["schema_version"] == 2
    manager.start("aquarium", run_worker=False)
    assert manager.previous == exported
    assert manager.extras["paddings"] == exported["measured"]["paddings"]


def test_api_typed_status_coded_refusal_and_validation(rig, client):
    status = client.get("/api/v1/tank-measure/status")
    assert status.status_code == 200 and status.json()["active"] is False
    assert (
        client.post("/api/v1/tank-measure/capture", json={"label": "near"}).json()["code"]
        == "robot.not_ready"
    )
    assert client.post("/api/v1/tank-measure/start", json={}).json()["code"] == "request.validation"
    assert client.get("/api/v1/tank-measure/latest?robot_name=..").status_code == 400
    assert client.get("/tank-measure/status").status_code == 404


def test_worker_idle_watchdog_releases_without_status_poll(rig, monkeypatch):
    manager, now, bus, _ = rig
    manager.start("aquarium", run_worker=False)
    monkeypatch.setattr(manager.stop_event, "wait", lambda _: now.__setitem__(0, tm.IDLE_SECONDS + 1))
    manager._worker()
    assert not manager.active
    assert bus.calls[-2:] == ["disable", "disconnect"]


def test_disable_failure_still_disconnects(rig, monkeypatch):
    manager, _, bus, _ = open_rig(rig)
    monkeypatch.setattr(bus, "disable_torque", Mock(side_effect=OSError("USB lost")))
    manager.stop()
    assert not manager.active
    assert bus.calls[-1] == "disconnect"


def test_partial_tower_measurement_cannot_silently_keep_previous_box(rig):
    manager, _, _, _ = rig
    manager.start("aquarium", run_worker=False)
    manager.touches = synthetic() + [
        {"label": "tower_front", "point_mm": [10, 0, 80], "positions": {}, "timestamp": "now"}
    ]
    assert not manager.status()["fit"]["valid"]
    with pytest.raises(ApiError, match="four tower planes"):
        manager.save()


@pytest.mark.parametrize("case", ["unsupported", "unknown", "bimanual", "missing_calibration"])
def test_record_readiness_refusals_do_not_open_hardware(rig, case):
    manager, _, bus, _ = rig
    path = Path(cfg.ROBOTS_PATH) / "aquarium.json"
    record = json.loads(path.read_text())
    if case == "unsupported":
        record["arm_type"] = "so101"
    elif case == "unknown":
        record["arm_type"] = "not-installed"
    elif case == "bimanual":
        record["mode"] = "bimanual"
    else:
        record["follower_config"] = "missing"
    path.write_text(json.dumps(record))
    with pytest.raises(ApiError):
        manager.start("aquarium", run_worker=False)
    assert not bus.calls


def test_existing_reciprocal_calibration_gate_sees_tank_measurement(rig):
    from makermodslab.can_wiggle import _busy_refusal

    manager, _, bus, _ = rig
    manager.start("aquarium", run_worker=False)
    assert _busy_refusal()["code"] == "robot.busy.calibration"
    assert not bus.calls


def test_both_calibration_managers_refuse_while_tank_holds_bus(rig):
    from makermodslab.step_calibrate import StepCalibrationManager, StepCalibrationRequest

    manager, _, bus, _ = rig
    manager.start("aquarium", run_worker=False)
    assert calibrate.CalibrationManager().start_calibration(Mock())["code"] == "robot.busy.calibration"
    request = StepCalibrationRequest(
        device_type="teleop", port="/dev/FAKE", config_file="new", arm_type="maker"
    )
    assert StepCalibrationManager().start(request)["code"] == "robot.busy.calibration"
    assert not bus.calls
