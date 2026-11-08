// Copyright 2026 MakerMods. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
import { apiRequest, type Fetcher } from "./apiClient";

export type TankFace = "near" | "far" | "left" | "right" | "floor" | "rim";
export interface TankTouch {
  label: string;
  point_mm: number[];
  positions: Record<string, number>;
  timestamp: string;
}
export interface TankFit {
  valid: boolean;
  failures: string[];
  yaw_deg?: number;
  offsets_mm?: Record<TankFace, number>;
  width_mm?: number;
  rms_mm?: number;
  max_mm?: number;
  residuals?: { index: number; label: string; residual_mm: number }[];
  j1_distances_mm?: Record<string, number>;
  far_status?: string;
  change?: {
    translation_mm: number;
    toward: TankFace;
    rotation_deg: number;
  } | null;
}
export interface TowerBox {
  min_mm: number[];
  max_mm: number[];
  rear_status: string;
}
export interface TankPadding {
  padding_mm: number;
  status: string;
}
export interface TankStatus {
  active: boolean;
  robot_name: string | null;
  error: string | null;
  stale: boolean;
  still: boolean;
  positions: Record<string, number> | null;
  probe_mm: number[] | null;
  gripper_opening_mm: number | null;
  link_origins_mm: Record<string, number[]>;
  touches: TankTouch[];
  fit: TankFit | null;
  tower_box: TowerBox | null;
  paddings: Record<string, TankPadding>;
  idle_deadline: number | null;
  clearances: {
    links: Record<
      string,
      {
        distances_mm: Record<string, number>;
        padding_mm: number;
        padding_status: string;
      }
    >;
    probe_mm: Record<TankFace, number>;
    margins_mm: Record<TankFace, number>;
    floor_informational: boolean;
    tower_method: string;
  } | null;
}
export interface TankLatest {
  robot_name: string;
  measurement: { measured: TankFit & { timestamp: string } } | null;
}
export const tankApi = (baseUrl: string, fetcher: Fetcher) => ({
  start: (robot_name: string) =>
    apiRequest<TankStatus>(baseUrl, fetcher, "/api/v1/tank-measure/start", {
      method: "POST",
      body: { robot_name },
    }),
  status: () =>
    apiRequest<TankStatus>(baseUrl, fetcher, "/api/v1/tank-measure/status"),
  capture: (label: string) =>
    apiRequest<TankStatus>(baseUrl, fetcher, "/api/v1/tank-measure/capture", {
      method: "POST",
      body: { label },
    }),
  delete: (index: number) =>
    apiRequest<TankStatus>(
      baseUrl,
      fetcher,
      `/api/v1/tank-measure/touches/${index}`,
      { method: "DELETE" },
    ),
  save: () =>
    apiRequest<TankLatest>(baseUrl, fetcher, "/api/v1/tank-measure/save", {
      method: "POST",
    }),
  stop: () =>
    apiRequest<TankStatus>(baseUrl, fetcher, "/api/v1/tank-measure/stop", {
      method: "POST",
    }),
  latest: (robot: string) =>
    apiRequest<TankLatest>(
      baseUrl,
      fetcher,
      `/api/v1/tank-measure/latest?robot_name=${encodeURIComponent(robot)}`,
    ),
});

/** Auto-capture must see a new placement, even if the probe stays still. */
export function movedSinceCapture(
  previous: number[] | null,
  current: number[] | null,
): boolean {
  return (
    !previous ||
    (!!current && Math.hypot(...current.map((v, i) => v - previous[i])) >= 20)
  );
}
