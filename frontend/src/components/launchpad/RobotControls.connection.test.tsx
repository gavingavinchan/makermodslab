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
import { ApiError } from "@/lib/apiClient";
import { TooltipProvider } from "@/components/ui/tooltip";
import RobotControls from "./RobotControls";

const mocks = vi.hoisted(() => ({ start: vi.fn(), toast: vi.fn() }));
vi.mock("@/contexts/ApiContext", () => ({ useApi: () => ({ baseUrl: "http://test", fetchWithHeaders: vi.fn() }) }));
vi.mock("@/hooks/use-toast", () => ({ useToast: () => ({ toast: mocks.toast }) }));
vi.mock("@/hooks/useRobots", () => ({ useRobots: () => ({ selectedRecord: {
  name: "bench", arm_type: "viscous", mode: "single", arms: "both", is_clean: false,
  follower_config: "viscous_01", leader_config: "star_viscous", leader_ready: true,
} }) }));
vi.mock("@/hooks/useArms", () => ({ useArms: () => ({ byId: () => ({ capabilities: { supports_teleop_speed_limit: true } }) }) }));
vi.mock("@/hooks/useHostingStatus", () => ({ useHostingStatus: () => ({ status: null, refresh: vi.fn() }) }));
vi.mock("@/hooks/useStationStatus", () => ({ useStationStatus: () => ({ status: null }) }));
vi.mock("@/lib/sessionApi", () => ({ startSession: mocks.start, formatSessionHeld: () => null }));
vi.mock("@/components/dialogs/TeleopDialog", () => ({ default: ({ open }: { open: boolean }) => open ? <div data-testid="live-teleop" /> : null }));
vi.mock("@/components/dialogs/HostingDialog", () => ({ default: () => null }));
vi.mock("@/components/dialogs/StationRobotDialog", () => ({ default: () => null }));
vi.mock("@/components/dialogs/RemoteTeleopDialog", () => ({ default: () => null }));
vi.mock("@/components/dialogs/RemoteExtraInstallDialog", () => ({ default: () => null }));

beforeEach(() => { mocks.start.mockReset(); mocks.toast.mockReset(); });
afterEach(cleanup);

function mount() {
  render(<TooltipProvider><RobotControls onOpenSettings={() => {}} onCreateRobot={() => {}} /></TooltipProvider>);
}

it.each(["robot.unit_mismatch", "robot.identity_unverified"])("%s requires one explicit retry and never remembers it", async (code) => {
  const sentence = "Adapter 207E37A55445 belongs to viscous_01; continuing uses viscous_waterproof_01 settings.";
  mocks.start.mockRejectedValueOnce(new ApiError("technical wrapper", 409, sentence, code, { settings_unit: "viscous_waterproof_01", observed_unit: "viscous_01" }))
    .mockResolvedValueOnce({ session: { id: "one", phase: "starting" }, warnings: null });
  mount();
  fireEvent.click(screen.getByRole("button", { name: "Teleop" }));
  expect(await screen.findByRole("alert")).toHaveTextContent(sentence);
  expect(screen.getByText("Details").closest("details")).not.toHaveAttribute("open");
  expect(mocks.start.mock.calls[0][2].options).not.toHaveProperty("allow_unit_override");
  fireEvent.click(screen.getByRole("button", { name: "Continue anyway" }));
  await waitFor(() => expect(mocks.start).toHaveBeenCalledTimes(2));
  expect(mocks.start.mock.calls[1][2].options).toEqual({ speed_limit_deg_s: 15, allow_unit_override: true });
  await screen.findByTestId("live-teleop");
  // Another Teleop press starts with the normal options, even after an override.
  mocks.start.mockRejectedValueOnce(new ApiError("again", 409, sentence, code));
  fireEvent.click(screen.getByRole("button", { name: "Teleop" }));
  await waitFor(() => expect(mocks.start).toHaveBeenCalledTimes(3));
  expect(mocks.start.mock.calls[2][2].options).not.toHaveProperty("allow_unit_override");
});

it("physical faults show the plain sentence and secondary details without override", async () => {
  mocks.start.mockRejectedValueOnce(new ApiError("wrapper", 409, "Unplug the extra arm connection.", "robot.connection_ambiguous", { candidates: ["a", "b"] }));
  mount();
  fireEvent.click(screen.getByRole("button", { name: "Teleop" }));
  expect(await screen.findByRole("alert")).toHaveTextContent("Unplug the extra arm connection.");
  expect(screen.queryByRole("button", { name: "Continue anyway" })).toBeNull();
  expect(screen.getByText("Details").closest("details")).not.toHaveAttribute("open");
});
