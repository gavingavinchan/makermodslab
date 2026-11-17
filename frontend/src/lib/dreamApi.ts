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

export type DreamEnvironment = "tank" | "dry_air";
export type DreamSlot = "overhead" | "wrist_l" | "wrist_r";
export interface DreamSettings {
  environment: DreamEnvironment;
  prompt: string;
  overhead: string | null;
  wrist_l: string;
  wrist_r: string;
  max_calls: number;
  pause_s: number;
}
export const DREAM_DEFAULTS: DreamSettings = {
  environment: "tank", prompt: "", overhead: null, wrist_l: "wrist", wrist_r: "black", max_calls: 10, pause_s: 0,
};
// Prompts are model input data, kept byte-identical in every language.
export const DREAM_EXAMPLES = ["put apple into the bowl", "pick up the screwdriver", "move the sponge to the left"];
export interface DreamPlan { plan_id: string; movement: string; readiness: string[] }
export interface DreamPreview { preview_id: string; inputs_id: string; inputs_checked: boolean; run: string }
export interface DreamView {
  environment: DreamEnvironment; run: string; mode: "live" | "preview" | "dry" | "replay"; prompt: string;
  max_calls: number; slots: Record<DreamSlot, string>; pid: number | null; phase: string; state: string;
  call: number; last_reason: string | null; measured: Record<string, number> | null;
  commanded: Record<string, number> | null; requested_tcp_path: number[][];
  peak_torque_nm: number; peak_temp_c: number; inputs_id: string | null; inputs_checked: boolean | null;
  images_written: number | null; written: number; last_reading: number | null; message: string;
}
export interface DreamStatus {
  active: boolean; checking_inputs: boolean; run: string | null; robot: string | null; status: DreamView | null;
  error: string | null; stale: boolean; button_alive: boolean;
}
export interface DreamReplay { run: string; duration_s: number; elapsed_s: number; status: DreamView; console: string[] }
export function dreamPlan(base: string, fetcher: Fetcher, robot: string, settings: DreamSettings) {
  return apiRequest<DreamPlan>(base, fetcher, "/api/v1/dream/plan", { method: "POST", body: { robot, ...settings } });
}
export function dreamPreview(base: string, fetcher: Fetcher, robot: string, settings: DreamSettings) {
  return apiRequest<DreamPreview>(base, fetcher, "/api/v1/dream/preview", { method: "POST", body: { robot, ...settings } });
}
export function dreamApprove(base: string, fetcher: Fetcher, preview: DreamPreview) {
  return apiRequest<{ inputs_id: string; approved: boolean }>(base, fetcher, "/api/v1/dream/inputs-approval", {
    method: "POST", body: { preview_id: preview.preview_id, inputs_id: preview.inputs_id },
  });
}

/** The runner centre-crops to 16:9 before resizing to 320x176. The idle
 * MJPEG stays raw; this band marks that exact integer-pixel crop inside an
 * object-contain tile, including pillarboxing and letterboxing. */
export function dreamCropBand(width: number, height: number) {
  const aspect = 320 / 176;
  const displayWidth = Math.min(1, width / height / aspect);
  const displayHeight = Math.min(1, aspect / (width / height));
  const wantHeight = Math.round(width * 9 / 16);
  const cropWidth = height < wantHeight ? Math.round(height * 16 / 9) : width;
  const cropHeight = height > wantHeight ? wantHeight : height;
  const w = displayWidth * cropWidth / width * 100;
  const h = displayHeight * cropHeight / height * 100;
  return { left: `${(100 - w) / 2}%`, top: `${(100 - h) / 2}%`, width: `${w}%`, height: `${h}%` };
}
