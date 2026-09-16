"""Commissioned Viscous Arm v1 with its matched Star Arm 102 leader.

The existing CAN session machinery owns recording, replay and stopping. The
LeRobot preset owns the all-RS00 motor fit, measured limits, leader mapping,
startup check and thermal stop. Never substitute the Maker arm's presets.
"""

from __future__ import annotations

import os

from .can_common import CanArmFamily


class ViscousFamily(CanArmFamily):
    id = "viscous"
    label = "Viscous Arm v1"
    short_label = "Viscous"
    indefinite_label = "a Viscous arm"
    supports_bimanual = False
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

    def single_follower_config(self, port: str, config_id: str):
        from lerobot.robots.viscous_follower import ViscousFollowerConfig

        return ViscousFollowerConfig(port=port, id=config_id)

    def single_leader_config(self, port: str, config_id: str, leader_kind: str | None = None):
        from lerobot.teleoperators.rebot_102_leader import RebotArm102LeaderViscousTeleopConfig

        return RebotArm102LeaderViscousTeleopConfig(port=port, id=config_id)

    def build_single_configs(self, request, cameras, leader_id, follower_id):
        follower = self.single_follower_config(request.follower_port, follower_id)
        if cameras is not None:
            follower.cameras = cameras
        return follower, self.single_leader_config(request.leader_port, leader_id)

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
