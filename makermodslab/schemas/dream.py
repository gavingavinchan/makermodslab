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
"""Dream page requests and responses. Hardware settings stay in robot records."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class DreamSettings(BaseModel):
    model_config = ConfigDict(extra="forbid")

    prompt: str = Field("put apple into the bowl", min_length=1, max_length=4000)
    overhead: str | None = None
    wrist_l: str = "wrist"
    wrist_r: str = "black"
    max_calls: int = Field(10, ge=1, le=20)
    pause_s: float = Field(0.0, ge=0, le=600, allow_inf_nan=False)


class DreamPlanBody(DreamSettings):
    robot: str


class DreamOptions(BaseModel):
    model_config = ConfigDict(extra="forbid")

    plan_id: str = Field(pattern=r"^[a-f0-9]{10}$")


class DreamInputsApproval(BaseModel):
    model_config = ConfigDict(extra="forbid")

    preview_id: str = Field(pattern=r"^[a-f0-9]{32}$")
    inputs_id: str = Field(pattern=r"^[a-f0-9]{12}$")


class DreamPlanResponse(BaseModel):
    plan_id: str
    movement: str
    readiness: list[str]


class DreamPreviewResponse(BaseModel):
    preview_id: str
    inputs_id: str
    inputs_checked: bool
    run: str


class DreamApprovalResponse(BaseModel):
    inputs_id: str
    approved: bool


class DreamViewStatus(BaseModel):
    run: str
    mode: Literal["live", "preview", "dry", "replay"]
    prompt: str
    max_calls: int
    slots: dict[str, str]
    pid: int | None
    phase: str
    state: str
    call: int
    last_reason: str | None
    measured: dict[str, float] | None
    commanded: dict[str, float] | None
    requested_tcp_path: list[list[float]]
    peak_torque_nm: float
    peak_temp_c: float
    inputs_id: str | None
    inputs_checked: bool | None
    images_written: float | None
    written: float
    last_reading: float | None = None
    message: str = ""


class DreamStatusResponse(BaseModel):
    active: bool
    checking_inputs: bool
    run: str | None
    robot: str | None
    status: DreamViewStatus | None
    error: str | None
    stale: bool
    button_alive: bool


class DreamReplayResponse(BaseModel):
    run: str
    duration_s: float
    elapsed_s: float
    status: DreamViewStatus
    console: list[str]
