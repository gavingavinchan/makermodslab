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

import { afterEach, beforeEach, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import "@/i18n";
import TeleopDialog from "./TeleopDialog";

const mocks = vi.hoisted(() => ({ fetch: vi.fn(), heartbeat: vi.fn(), stop: vi.fn(), toast: vi.fn() }));
vi.mock("@/contexts/ApiContext", () => ({ useApi: () => ({ baseUrl: "http://test", fetchWithHeaders: mocks.fetch }) }));
vi.mock("@/hooks/use-toast", () => ({ useToast: () => ({ toast: mocks.toast }) }));
vi.mock("@/hooks/useRobots", () => ({ useRobots: () => ({ selectedRecord: null }) }));
vi.mock("@/hooks/useSessionHeartbeat", () => ({ useSessionHeartbeat: mocks.heartbeat }));
vi.mock("@/hooks/useUnloadWarning", () => ({ useUnloadWarning: () => {} }));
vi.mock("@/lib/sessionApi", () => ({ stopSession: mocks.stop }));
vi.mock("@/components/control/SessionLiveView", () => ({ default: () => null }));

beforeEach(() => { vi.clearAllMocks(); mocks.stop.mockResolvedValue({ result: { success: true } }); });
afterEach(cleanup);

it("keeps heartbeating during waiting and Stop cancels by session id", async () => {
  mocks.fetch.mockResolvedValue({ ok: true, json: async () => ({ teleoperation_active: true, status_sentence: "Waiting for the arm to power up..." }) });
  const close = vi.fn();
  render(<TeleopDialog open onOpenChange={close} sessionId="waiting-session" />);
  expect(await screen.findByRole("status")).toHaveTextContent("Waiting for the arm to power up...");
  expect(mocks.heartbeat).toHaveBeenLastCalledWith("waiting-session", expect.any(String), true);
  fireEvent.click(screen.getByRole("button", { name: "Stop" }));
  await waitFor(() => expect(mocks.stop).toHaveBeenCalledWith("http://test", mocks.fetch, "waiting-session"));
  expect(close).toHaveBeenCalledWith(false);
});

it("shows a failed start sentence with details behind a disclosure", async () => {
  mocks.fetch.mockResolvedValue({ ok: true, json: async () => ({ teleoperation_active: false, outcome: "failed", code: "robot.feedback_missing", error: "Reconnect the large arm’s power cable.", details: { missing: ["gripper"] } }) });
  render(<TeleopDialog open onOpenChange={() => {}} sessionId="failed-session" />);
  expect(await screen.findByText("Reconnect the large arm’s power cable.")).toBeVisible();
  expect(screen.getByText("Details").closest("details")).not.toHaveAttribute("open");
  await waitFor(() => expect(mocks.heartbeat).toHaveBeenLastCalledWith("failed-session", expect.any(String), false));
});
