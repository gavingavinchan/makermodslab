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
"""Torque-off hand-guided aquarium probing; measurement/display ONLY.

The base moves whenever Gavin removes the arm from the tank. Closed-gripper
wall touches replace the old estimated placement, including the near/back
wall that J3 can hit. Cabling and the base tower are optional persistent arm
extras. Nothing here enables torque, sets zero, or sends a position command.
"""

from __future__ import annotations

import contextlib
import json
import threading
import time
from collections import deque
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from . import tank_geometry as geo
from .api_errors import ApiError, ErrorCode
from .arm_capabilities import require_known_arm_type
from .arms import registry
from .session_events import notify_session_changed
from .utils import config as cfg

POLL_SECONDS = 0.1
WINDOW_SECONDS = 1.0
STILLNESS_MM = 1.5
STALE_SECONDS = 0.35
IDLE_SECONDS = 15 * 60


def refuse(message, code=ErrorCode.ROBOT_NOT_READY, status=409):
    raise ApiError(status_code=status, detail=message, code=code)


def storage_dir(robot_name: str) -> Path:
    # Names are also path components. Never let a latest lookup escape tank/.
    if (
        not robot_name
        or robot_name in (".", "..")
        or Path(robot_name).name != robot_name
        or "\\" in robot_name
    ):
        refuse("Invalid robot name", ErrorCode.REQUEST_INVALID_NAME, 400)
    return Path(cfg.TANK_MEASURE_PATH) / robot_name


def read_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text()) if path.exists() else None
    except (OSError, ValueError) as error:
        refuse(f"Cannot read saved tank measurement: {error}", status=500)


def latest(robot_name: str) -> dict:
    return {
        "robot_name": robot_name,
        "measurement": read_json(storage_dir(robot_name) / cfg.TANK_LATEST_FILE),
    }


def full_turn_offsets(raw: dict, config) -> dict:
    """Run LeRobot's exact teleop unwrap on a FRESH family read.

    Invoke the installed MakerFollower implementation on a read-only context
    instead of calling device.connect (which would energize), or reimplementing
    its grace window, shift order, or commissioned joint-limit semantics.
    Subsequent samples retain these per-session offsets, just like teleop.
    """
    from lerobot.robots.maker_follower.maker_follower import MakerFollower

    state = SimpleNamespace(config=config, _turn_offset={}, _read_raw_positions=lambda: raw)
    MakerFollower._detect_full_turn_offsets(state)
    return state._turn_offset


class TankMeasurement:
    def __init__(self, *, clock=time.monotonic, utcnow=lambda: datetime.now(UTC)):
        self.clock, self.utcnow = clock, utcnow
        self.lock = threading.RLock()
        self.active = False
        self.stop_event = threading.Event()
        self.thread = None
        self.device = None
        self.samples = deque(maxlen=32)
        self.touches = []
        self.robot_name = None
        self.error = None
        self.stale = True
        self.last_poll = 0.0
        self.offsets = None
        self.previous = None
        self.extras = {}
        self.model = None
        self.tank = None

    def start(self, robot_name: str, *, run_worker=True) -> dict:
        from . import jobs, monitored_can, sessions, wiggle
        from .can_wiggle import _BUSY_CODES

        storage_dir(robot_name)
        record = cfg.get_robot_record(robot_name, resolve_usb=False)
        if record is None:
            refuse("Robot record not found", ErrorCode.ROBOT_NOT_FOUND, 404)
        require_known_arm_type(record.get("arm_type"))
        family = registry.get(cfg.normalize_arm_type(record.get("arm_type")))
        if not family.supports_tank_measurement or record.get("mode", "single") != "single":
            refuse("Tank measurement requires a supported single follower", status=400)
        port, profile = record.get("follower_port"), record.get("follower_config")
        if not port or not profile:
            refuse("Select the follower port and commissioned follower calibration first", status=400)
        if not (Path(family.follower_calibration_dir()) / f"{profile.removesuffix('.json')}.json").is_file():
            refuse("The selected follower calibration is missing", status=400)
        try:
            if monitored_can.bridge_metadata(port, profile) is not None:
                refuse("A CAN monitor bridge owns this adapter. Stop the monitor before measuring the tank.")
        except (ValueError, OSError) as error:
            refuse(f"Cannot claim the monitored CAN adapter: {error}")
        # Config and model loading are offline and fail before the bus claim.
        try:
            family.single_follower_config(port, profile)
        except (ValueError, OSError) as error:
            refuse(f"Cannot load the commissioned follower profile: {error}", status=400)
        tank = json.loads(geo.DEFAULT_TANK.read_text())
        model = geo.load_model(tank)
        with self.lock:
            if self.active:
                refuse("Tank measurement is already active", ErrorCode.ROBOT_BUSY_CALIBRATION)
            holder = sessions.held_by()
            if holder or wiggle.wiggle_active:
                holder = holder or "wiggle"
                refuse(
                    f"Stop {holder.replace('_', ' ')} before measuring the tank",
                    _BUSY_CODES.get(holder, f"robot.busy.{holder}"),
                )
            if jobs.training_is_active() is not None:
                refuse("Stop local training before measuring the tank", ErrorCode.ROBOT_BUSY_TRAINING)
            self.tank, self.model, self.family = tank, model, family
            self.port, self.profile, self.robot_name = port, profile, robot_name
            self.previous = latest(robot_name)["measurement"]
            self.extras = read_json(storage_dir(robot_name) / cfg.TANK_ARM_EXTRAS_FILE) or {}
            self.samples.clear()
            self.touches = []
            self.error, self.offsets, self.device = None, None, None
            self.stale = True
            self.last_poll = self.clock()
            self.stop_event = threading.Event()
            self.thread = None
            self.active = True
            notify_session_changed("calibration", True, phase="tank_measurement")
            if run_worker:
                self.thread = threading.Thread(target=self._worker, daemon=True, name="tank-measure")
                try:
                    self.thread.start()
                except Exception:
                    self.active = False
                    notify_session_changed("calibration", False, phase="error")
                    raise
        return self.status()

    def expired(self) -> bool:
        return self.clock() - self.last_poll >= IDLE_SECONDS

    def _worker(self):
        try:
            # EXACTLY the zero-calibration bus-only path: handshake=False,
            # disable_torque. Never device.connect(), even to obtain angles.
            self.device = self.family.open_follower_torque_off(self.port, self.profile)
            while not self.stop_event.is_set() and not self.expired():
                self.poll_once()
                self.stop_event.wait(POLL_SECONDS)
        except Exception as error:
            with self.lock:
                self.error, self.stale = str(error), True
        finally:
            self._release()

    def poll_once(self):
        """One fresh-all-joints poll; failed replies invalidate the whole window."""
        try:
            raw = self.family._read_fresh_positions(self.device)
            if set(raw) != {*geo.JOINTS, "gripper"} or not all(np.isfinite(v) for v in raw.values()):
                raise ConnectionError("Every joint must return a fresh finite position")
            if self.offsets is None:
                self.offsets = full_turn_offsets(raw, self.device.config)
            positions = {n: raw[n] + self.offsets[n] for n in raw}
            probe = geo.probe_point(self.model, geo.forward_kinematics(self.model, positions)).tolist()
            with self.lock:
                self.samples.append((self.clock(), positions, probe))
                self.stale, self.error = False, None
        except Exception as error:
            with self.lock:
                self.samples.clear()
                self.stale, self.error = True, str(error)

    def _release(self):
        device = self.device
        if device is not None:
            with contextlib.suppress(Exception):
                device.bus.disable_torque()
            with contextlib.suppress(Exception):
                device.bus.disconnect(False)
        with self.lock:
            self.device = None
            self.stale = True
            self.active = False
            notify_session_changed("calibration", False)

    def stop(self) -> dict:
        self.stop_event.set()
        thread = self.thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=5)
        elif self.active:
            self._release()
        # If a fresh-read timeout is still unwinding, keep the claim until
        # worker cleanup actually disconnects; another start must not race it.
        return self.status(refresh_idle=False)

    def window(self):
        if not self.samples or self.clock() - self.samples[-1][0] > STALE_SECONDS:
            return [], False
        end = self.samples[-1][0]
        samples = list(self.samples)
        preceding = [i for i, s in enumerate(samples) if s[0] <= end - WINDOW_SECONDS]
        if not preceding:
            return [], False
        window = samples[preceding[-1] :]
        if any(b[0] - a[0] > STALE_SECONDS for a, b in zip(window, window[1:], strict=False)):
            return [], False
        p = np.array([s[2] for s in window])
        span = np.linalg.norm(p[:, None, :] - p[None, :, :], axis=2).max()
        # The preceding sample proves a full second of stillness; average
        # only angles actually within the final one-second interval.
        return [s for s in window if s[0] >= end - WINDOW_SECONDS], bool(span < STILLNESS_MM)

    def _geometry(self):
        fit = geo.fit_tank(self.touches, self.tank, self.previous)
        tower = self.extras.get("tower_box")
        paddings = self.extras.get("paddings", {})
        if "offsets_mm" in fit:
            tower_touches = [t for t in self.touches if t["label"].startswith("tower_")]
            if tower_touches and {t["label"] for t in tower_touches} != {
                "tower_front",
                "tower_top",
                "tower_left",
                "tower_right",
            }:
                fit["valid"] = False
                fit["failures"].append("Complete all four tower planes before saving a re-measurement")
            try:
                tower = geo.tower_box(self.touches, fit) or tower
            except ValueError as error:
                fit["valid"] = False
                fit["failures"].append(str(error))
            paddings = geo.cable_paddings(self.model, self.touches, fit, paddings)
        return fit, tower, paddings

    def status(self, *, refresh_idle=True) -> dict:
        if self.active and self.expired():
            self.stop_event.set()
            if self.thread is None:
                self._release()
        with self.lock:
            if refresh_idle and self.active and not self.stop_event.is_set():
                self.last_poll = self.clock()
            _, still = self.window()
            stale = self.stale or not self.samples or self.clock() - self.samples[-1][0] > STALE_SECONDS
            positions = self.samples[-1][1] if self.samples else None
            transforms = geo.forward_kinematics(self.model, positions) if positions else None
            fit, tower, paddings = self._geometry() if self.model else (None, None, {})
            clearances = (
                geo.live_clearances(self.model, self.tank, positions, fit, paddings, tower)
                if positions and fit and "offsets_mm" in fit
                else None
            )
            return {
                "active": self.active,
                "robot_name": self.robot_name,
                "error": self.error,
                "positions": positions,
                "stale": bool(stale),
                "still": bool(still and not stale),
                "probe_mm": geo.probe_point(self.model, transforms).tolist() if transforms else None,
                "gripper_opening_mm": geo.gripper_opening_mm(self.model, positions) if positions else None,
                "link_origins_mm": {n: (t * 1000).tolist() for n, (_, t) in transforms.items()}
                if transforms
                else {},
                "touches": list(self.touches),
                "fit": fit,
                "clearances": clearances,
                "tower_box": tower,
                "paddings": paddings,
                "idle_deadline": self.last_poll + IDLE_SECONDS if self.active else None,
            }

    def capture(self, label: str) -> dict:
        with self.lock:
            if not self.active or self.stop_event.is_set() or self.expired():
                refuse("Start tank measurement first")
            allowed = {
                "left",
                "right",
                "near",
                "floor",
                "tower_front",
                "tower_top",
                "tower_left",
                "tower_right",
                *(f"check:{f}" for f in ("left", "right", "near", "far")),
            }
            if label.startswith("envelope:"):
                parts = label.split(":")
                if (
                    len(parts) != 3
                    or parts[1] not in self.model.vertices
                    or parts[1] not in self.model.moving_links
                    or parts[2] not in geo.FACES
                ):
                    refuse("Choose a moving link and tank face", ErrorCode.REQUEST_VALIDATION, 400)
                if not self._geometry()[0]["valid"]:
                    refuse("Fit the walls successfully before measuring cable padding")
            elif label not in allowed:
                refuse("Unknown touch label", ErrorCode.REQUEST_VALIDATION, 400)
            window, still = self.window()
            if self.stale or not still:
                refuse("Wait for fresh readings and hold the probe still for one second")
            positions = {n: float(np.mean([s[1][n] for s in window])) for n in window[0][1]}
            if not label.startswith("envelope:") and geo.gripper_opening_mm(self.model, positions) > 5:
                refuse("Close the gripper before touching with its tip (more than 5 mm open per finger)")
            point = geo.probe_point(self.model, geo.forward_kinematics(self.model, positions)).tolist()
            self.touches.append(
                {
                    "label": label,
                    "point_mm": point,
                    "positions": positions,
                    "timestamp": self.utcnow().isoformat(),
                }
            )
        return self.status()

    def delete(self, index: int) -> dict:
        with self.lock:
            if not self.active or self.stop_event.is_set():
                refuse("Start tank measurement first")
            if index < 0 or index >= len(self.touches):
                refuse("Touch index does not exist", ErrorCode.REQUEST_VALIDATION, 404)
            del self.touches[index]
        return self.status()

    def save(self) -> dict:
        with self.lock:
            if not self.active or self.stop_event.is_set() or self.expired():
                refuse("Start tank measurement first")
            fit, tower, paddings = self._geometry()
            if not fit["valid"]:
                refuse("Cannot save: " + "; ".join(fit["failures"]), status=400)
            now = self.utcnow()
            timestamp = now.isoformat()
            exported = geo.export_tank(self.tank, fit, self.touches, tower, paddings, timestamp)
            folder = storage_dir(self.robot_name)
            history = folder / cfg.TANK_HISTORY_DIR / (now.strftime("%Y%m%dT%H%M%S.%fZ") + ".json")
            # Each file is replaced atomically. History precedes latest so
            # the current measurement is never published without its evidence.
            cfg._atomic_write_text(str(history), json.dumps(exported, indent=2, allow_nan=False) + "\n")
            cfg._atomic_write_text(
                str(folder / cfg.TANK_ARM_EXTRAS_FILE),
                json.dumps(
                    {"tower_box": tower, "paddings": paddings, "timestamp": timestamp},
                    indent=2,
                    allow_nan=False,
                )
                + "\n",
            )
            cfg._atomic_write_text(
                str(folder / cfg.TANK_LATEST_FILE), json.dumps(exported, indent=2, allow_nan=False) + "\n"
            )
        return {"robot_name": self.robot_name, "measurement": exported}


manager = TankMeasurement()


def tank_measurement_is_active() -> bool:
    return manager.active
