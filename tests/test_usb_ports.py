import json
from types import SimpleNamespace

import pytest

from makermodslab import usb_ports
from makermodslab.utils import config as cfg


def port(device, serial="adapter-A", vid=5840, pid=4478):
    return SimpleNamespace(device=device, vid=vid, pid=pid, serial_number=serial)


def test_saved_adapter_follows_serial_even_when_old_path_is_reused():
    record = {"follower_port": "/dev/ttyUSB0"}
    identities = {"follower_port": usb_ports.identity(port("/dev/ttyUSB0"))}
    live = [port("/dev/ttyUSB0", "other"), port("/dev/ttyUSB2")]
    assert usb_ports.resolve_ports(record, identities, live)["follower_port"] == "/dev/ttyUSB2"
    assert usb_ports.resolve_ports(record, identities, list(reversed(live))) == usb_ports.resolve_ports(
        record, identities, live
    )


def test_missing_or_duplicate_serial_never_uses_old_path():
    record = {"follower_port": "/dev/ttyUSB0"}
    identities = {"follower_port": usb_ports.identity(port("/dev/ttyUSB0"))}
    for live in ([], [port("/dev/ttyUSB0", "other")], [port("/dev/ttyUSB0"), port("/dev/ttyUSB1")]):
        assert usb_ports.resolve_ports(record, identities, live)["follower_port"] == ""


def test_no_serial_rebinds_only_one_model_in_rig_and_on_host():
    record = {"leader_port": "/dev/cu.wchusbserial110"}
    leader = port("/dev/cu.wchusbserial1110", "", 6790, 29987)
    identities = {"leader_port": usb_ports.identity(leader)}
    assert usb_ports.resolve_ports(record, identities, [leader])["leader_port"] == leader.device
    assert (
        usb_ports.resolve_ports(
            record, identities, [leader, port("/dev/cu.wchusbserial1120", "", 6790, 29987)]
        )["leader_port"]
        == ""
    )
    record["right_leader_port"] = "/dev/cu.wchusbserial120"
    identities["right_leader_port"] = usb_ports.identity(leader)
    assert usb_ports.resolve_ports(record, identities, [leader])["leader_port"] == ""


def test_record_preserves_identity_through_unplug_settings_save_and_rename(tmp_path, monkeypatch):
    monkeypatch.setattr(cfg, "ROBOTS_PATH", str(tmp_path))
    live = [port("/dev/cu.usbmodem1")]
    monkeypatch.setattr(usb_ports, "serial_ports", lambda: live)
    cfg.save_robot_record("demo", {"follower_port": "/dev/tty.usbmodem1"})
    live.clear()
    cfg.save_robot_record("demo", {"motor_power": 50})
    assert cfg.get_robot_record("demo")["follower_port"] == ""
    assert cfg.rename_robot_record("demo", "renamed") == (True, "")
    live.append(port("/dev/cu.usbmodem2"))
    assert cfg.get_robot_record("renamed")["follower_port"] == "/dev/cu.usbmodem2"
    assert "port_identities" not in cfg.get_robot_record("renamed")
    cfg.save_robot_record("renamed", {"follower_port": ""})
    assert cfg.get_robot_record("renamed")["follower_port"] == ""
    assert "follower_port" not in json.loads((tmp_path / "renamed.json").read_text()).get(
        "port_identities", {}
    )


def test_port_reassignment_replaces_previous_adapter_identity(tmp_path, monkeypatch):
    monkeypatch.setattr(cfg, "ROBOTS_PATH", str(tmp_path))
    first = port("/dev/cu.usbmodem1", "adapter-A")
    second = port("/dev/cu.usbmodem2", "adapter-B")
    monkeypatch.setattr(usb_ports, "serial_ports", lambda: [first, second])
    cfg.save_robot_record("demo", {"follower_port": first.device})
    cfg.save_robot_record("demo", {"follower_port": second.device})
    assert cfg.get_robot_record("demo")["follower_port"] == second.device
    saved = json.loads((tmp_path / "demo.json").read_text())
    assert saved["port_identities"]["follower_port"]["serial"] == "adapter-B"


def test_explicit_same_path_reassignment_replaces_adapter_but_offline_save_keeps_identity(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(cfg, "ROBOTS_PATH", str(tmp_path))
    path = "/dev/cu.usbmodem1"
    live = [port(path, "adapter-A")]
    monkeypatch.setattr(usb_ports, "serial_ports", lambda: live)
    cfg.save_robot_record("demo", {"follower_port": path})
    live.clear()
    cfg.save_robot_record("demo", {"follower_port": path, "motor_power": 50})
    saved = json.loads((tmp_path / "demo.json").read_text())
    assert saved["port_identities"]["follower_port"]["serial"] == "adapter-A"
    live.append(port(path, "adapter-B"))
    assert cfg.get_robot_record("demo")["follower_port"] == ""
    cfg.save_robot_record("demo", {"follower_port": path})
    assert cfg.get_robot_record("demo")["follower_port"] == path
    saved = json.loads((tmp_path / "demo.json").read_text())
    assert saved["port_identities"]["follower_port"]["serial"] == "adapter-B"


def test_enumeration_error_refuses_stale_paths_then_recovers(tmp_path, monkeypatch):
    monkeypatch.setattr(cfg, "ROBOTS_PATH", str(tmp_path))
    first = port("/dev/cu.usbmodem1")
    monkeypatch.setattr(usb_ports, "serial_ports", lambda: [first])
    cfg.save_robot_record("demo", {"follower_port": first.device})
    original = (tmp_path / "demo.json").read_text()
    monkeypatch.setattr(usb_ports, "serial_ports", lambda: None)
    assert cfg.get_robot_record("demo")["follower_port"] == ""
    assert (tmp_path / "demo.json").read_text() == original
    monkeypatch.setattr(usb_ports, "serial_ports", lambda: [port("/dev/cu.usbmodem2")])
    assert cfg.get_robot_record("demo")["follower_port"] == "/dev/cu.usbmodem2"


def test_discovery_includes_wch_callout_and_excludes_bluetooth(monkeypatch):
    from serial.tools import list_ports

    monkeypatch.setattr(cfg.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(
        list_ports,
        "comports",
        lambda: [port("/dev/cu.wchusbserial1110", ""), port("/dev/cu.Bluetooth", None, None, None)],
    )
    monkeypatch.setattr("makermodslab.gs_usb_transport.available_gs_usb_ports", lambda: [])
    assert cfg.find_available_ports() == ["/dev/cu.wchusbserial1110"]


@pytest.mark.parametrize("field", ["arm_type", "leader_kind"])
def test_hardware_switch_discards_old_identity(tmp_path, monkeypatch, field):
    monkeypatch.setattr(cfg, "ROBOTS_PATH", str(tmp_path))
    monkeypatch.setattr(usb_ports, "serial_ports", lambda: [port("/dev/cu.usbmodem1")])
    cfg.save_robot_record("demo", {"arm_type": "metal", "leader_port": "/dev/cu.usbmodem1"})
    cfg.save_robot_record("demo", {field: "maker" if field == "arm_type" else "metal"})
    assert cfg.get_robot_record("demo")["leader_port"] == ""
