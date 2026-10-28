"""Commissioned Viscous Arm v1 with its matched Star Arm 102 leader.

The existing CAN session machinery owns recording, replay and stopping. The
LeRobot preset owns the all-RS00 motor fit, measured limits, leader mapping,
startup check and thermal stop. Never substitute the Maker arm's presets.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import draccus

from .can_common import CanArmFamily


class ViscousFamily(CanArmFamily):
    id = "viscous"
    label = "Viscous Arm v1"
    short_label = "Viscous"
    indefinite_label = "a Viscous arm"
    supports_bimanual = False
    supports_teleop_speed_limit = True
    teleop_usb_discovery = True
    teleop_requires_adapter_enrollment = True
    supports_joint_posture_mapping = True
    supports_gripper_haptics = True
    single_robot_type = "viscous_follower"
    # No bimanual Viscous driver exists. All public starts check the capability;
    # the builder below also refuses direct callers before touching hardware.
    bimanual_robot_type = ""
    robot_type_markers = ("viscous",)
    follower_probe_protocol = "robstride"
    motion_identify_energizes_follower = False
    follower_zero_pose = (
        "Use the existing commissioned Viscous follower calibration (viscous_01). "
        "Import or select that file in the calibration library. "
        "Re-zeroing the follower is disabled because it invalidates its measured limits."
    )

    def follower_calibration_dir(self) -> str:
        # Registration can run while utils.config is still importing. Its base
        # constants are already defined then, but its helper functions are not.
        from ..utils.config import CALIBRATION_BASE_PATH_ROBOTS

        return os.path.join(CALIBRATION_BASE_PATH_ROBOTS, "viscous_follower")

    def teleop_adapter_units(self) -> dict:
        """Committed adapter containment, not a physical motor fingerprint."""
        bundles = Path(__file__).resolve().parents[2] / "configs" / "viscous"
        return {
            manifest["arm"]: manifest["adapter_enrollment"]
            for path in sorted(bundles.glob("*/manifest.json"))
            if "adapter_enrollment" in (manifest := json.loads(path.read_text()))
        }

    @staticmethod
    def _commissioned_config(config_class, port, config_id, section):
        """Load a per-calibration hardware profile without changing the unit-1 preset."""
        from ..utils.config import MAKERMODSLAB_HOME

        if not config_id:
            # One-sided builders discard the unselected side.
            return config_class(port=port, id=config_id)
        if Path(config_id).name != config_id:
            raise ValueError("Invalid Viscous calibration id")
        path = Path(MAKERMODSLAB_HOME) / "viscous_profiles" / f"{config_id}.json"
        if not path.exists():
            # Only unit 1 has a known built-in hardware profile.
            original_id = "viscous_01" if section == "follower" else "star_viscous"
            if config_id != original_id:
                raise FileNotFoundError(f"Missing commissioned Viscous hardware profile: {path}")
            return config_class(port=port, id=config_id)
        profile = json.loads(path.read_text())
        fields = profile.get(section)
        allowed = (
            {"joint_limits"}
            if section == "follower"
            else {"joint_directions", "joint_ranges", "joint_offsets", "max_start_offset_deg"}
        )
        if not isinstance(fields, dict) or set(fields) != allowed:
            raise ValueError(f"Incomplete or unsupported Viscous {section} profile: {path}")
        config = draccus.decode(config_class, {"port": port, "id": config_id, **fields})
        names = set(config.motor_can_ids if section == "follower" else config.joint_ids)
        ranges = config.joint_limits if section == "follower" else config.joint_ranges
        if set(ranges) != names or any(
            len(bounds) != 2 or not all(math.isfinite(v) for v in bounds) or bounds[0] >= bounds[1]
            for bounds in ranges.values()
        ):
            raise ValueError(f"Invalid seven-joint Viscous ranges: {path}")
        if section == "leader" and (
            set(config.joint_directions) != names
            or set(config.joint_offsets) != names
            or any(not math.isfinite(v) or v == 0 for v in config.joint_directions.values())
            or any(not math.isfinite(v) for v in config.joint_offsets.values())
            or config.max_start_offset_deg is None
            or not 0 < config.max_start_offset_deg <= 30
        ):
            raise ValueError(f"Invalid Viscous leader mapping: {path}")
        return config

    def single_follower_config(self, port: str, config_id: str):
        from lerobot.robots.viscous_follower import ViscousFollowerConfig

        return self._commissioned_config(ViscousFollowerConfig, port, config_id, "follower")

    def single_leader_config(self, port: str, config_id: str, leader_kind: str | None = None):
        from lerobot.teleoperators.rebot_102_leader import (
            RebotArm102LeaderViscousTeleopConfig,
        )

        return self._commissioned_config(RebotArm102LeaderViscousTeleopConfig, port, config_id, "leader")

    def build_single_configs(self, request, cameras, leader_id, follower_id):
        follower = self.single_follower_config(request.follower_port, follower_id)
        if cameras is not None:
            follower.cameras = cameras
        leader = self.single_leader_config(request.leader_port, leader_id)
        expected = {name: [math.ceil(lo), math.floor(hi)] for name, (lo, hi) in follower.joint_limits.items()}
        # Star ranges use integer degrees. Allow the gripper's targets
        # to reach fractional endpoints; the follower still clamps the action
        # to its exact measured limit before sending it to the motor.
        gripper_min = leader.joint_ranges["gripper"][0]
        gripper_max = leader.joint_ranges["gripper"][1]
        if gripper_min == math.floor(follower.joint_limits["gripper"][0]):
            expected["gripper"][0] = gripper_min
        if gripper_max == math.ceil(follower.joint_limits["gripper"][1]):
            expected["gripper"][1] = gripper_max
        if leader_id and follower_id and leader.joint_ranges != expected:
            raise ValueError(
                "Viscous follower limits and leader mapping do not match; select the commissioned pair"
            )
        return follower, leader

    def build_bimanual_configs(self, request, cameras, base, leader_staging, follower_staging):
        raise ValueError("Viscous currently supports a single follower and Star leader only.")

    def open_for_calibration(self, device_type, port, config_id, leader_kind=None):
        if device_type == "robot":
            raise ValueError(self.follower_zero_pose)
        return super().open_for_calibration(device_type, port, config_id, leader_kind)

    def calibrate(self, device, device_type, ui):
        if device_type == "robot":
            raise ValueError(self.follower_zero_pose)
        return super().calibrate(device, device_type, ui)


VISCOUS = ViscousFamily()
