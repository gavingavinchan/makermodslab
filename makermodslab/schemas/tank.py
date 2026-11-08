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
"""Typed read-only tank measurement wire contract (all distances in mm)."""

from typing import Any

from pydantic import BaseModel, Field


class TankStart(BaseModel):
    robot_name: str = Field(min_length=1)


class TankCapture(BaseModel):
    label: str = Field(min_length=1, max_length=160)


class TankTouch(BaseModel):
    label: str
    point_mm: list[float]
    positions: dict[str, float]
    timestamp: str


class TankResidual(BaseModel):
    index: int
    label: str
    residual_mm: float


class TankChange(BaseModel):
    translation_mm: float
    toward: str
    rotation_deg: float


class TankFit(BaseModel):
    valid: bool
    failures: list[str]
    yaw_deg: float | None = None
    offsets_mm: dict[str, float] | None = None
    width_mm: float | None = None
    rms_mm: float | None = None
    max_mm: float | None = None
    residuals: list[TankResidual] | None = None
    j1_distances_mm: dict[str, float] | None = None
    far_status: str | None = None
    change: TankChange | None = None


class TankTower(BaseModel):
    min_mm: list[float]
    max_mm: list[float]
    rear_status: str


class TankPadding(BaseModel):
    padding_mm: float
    status: str


class TankLinkClearance(BaseModel):
    padding_mm: float
    padding_status: str
    distances_mm: dict[str, float]


class TankClearances(BaseModel):
    links: dict[str, TankLinkClearance]
    probe_mm: dict[str, float]
    margins_mm: dict[str, float]
    floor_informational: bool
    tower_method: str


class TankStatus(BaseModel):
    active: bool
    robot_name: str | None
    error: str | None
    positions: dict[str, float] | None
    stale: bool
    still: bool
    probe_mm: list[float] | None
    gripper_opening_mm: float | None
    link_origins_mm: dict[str, list[float]]
    touches: list[TankTouch]
    fit: TankFit | None
    clearances: TankClearances | None
    tower_box: TankTower | None
    paddings: dict[str, TankPadding]
    idle_deadline: float | None


class TankLatest(BaseModel):
    robot_name: str
    # Schema-v2 tank file is extensible provenance-bearing JSON; preserve it
    # verbatim (including measured extras) for offline tools and future guards.
    measurement: dict[str, Any] | None
