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
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor, within } from "@testing-library/react";
import { MemoryRouter } from "react-router-dom";
import Dream from "./Dream";

const mocks = vi.hoisted(() => ({ fetcher: vi.fn() }));
vi.mock("@/contexts/ApiContext", () => ({ useApi: () => ({ baseUrl: "http://lab", fetchWithHeaders: mocks.fetcher }) }));
vi.mock("@/hooks/useArms", () => ({ useArms: () => ({ byId: (id: string) => ({ capabilities: { supports_dream: id === "viscous" } }) }) }));
vi.mock("@/hooks/useRobots", () => ({ useRobots: () => ({ records: Object.fromEntries([
  { name: "viscous_waterproof_01", arm_type: "viscous", mode: "single", cameras: [] },
  { name: "viscous_waterproof_01_dry_air", arm_type: "viscous", mode: "single", cameras: [{ id: "head", name: "new-head", camera_index: 9 }] },
  { name: "other", arm_type: "so101", mode: "single", cameras: [] },
].map((r) => [r.name, r])) }) }));
vi.mock("@/hooks/useSessionHeartbeat", () => ({ useSessionHeartbeat: () => {} }));
vi.mock("@/hooks/useUnloadWarning", () => ({ useUnloadWarning: () => {} }));
vi.mock("@/components/LanguageSwitcher", () => ({ default: () => null }));

beforeEach(() => {
  mocks.fetcher.mockReset();
  mocks.fetcher.mockImplementation(async (url: string, options?: RequestInit) => {
    const body = options?.body ? JSON.parse(options.body as string) : {};
    let data = {};
    if (url.endsWith("/dream/status")) data = { active: false, checking_inputs: false, status: null };
    if (url.endsWith("/sessions/current")) data = { session: null };
    if (url.endsWith("/dream/preview")) data = { preview_id: "a".repeat(32), inputs_id: "abcdef123456", inputs_checked: true, run: "preview-test" };
    if (url.endsWith("/dream/plan")) data = { plan_id: "0123456789", movement: `Environment: ${body.environment}\nJoints that may move (ALL SEVEN)`, readiness: [] };
    if (url.endsWith("/sessions")) data = { session: { id: "test", kind: "dream", robot: body.robot, owner: "test" } };
    return new Response(JSON.stringify(data), { status: 200 });
  });
});
afterEach(cleanup);

function bodyFor(path: string) {
  const call = mocks.fetcher.mock.calls.find(([url]) => url.endsWith(path));
  expect(call, `missing request to ${path}`).toBeDefined();
  return JSON.parse(call![1].body);
}

function open() {
  return render(<MemoryRouter><Dream /></MemoryRouter>);
}

describe("Dream environment", () => {
  it.each(["tank", "dry_air"])("keeps %s through preview, plan confirmation and start for the new Viscous record", async (environment) => {
    open();
    fireEvent.change(screen.getByRole("combobox", { name: "Robot" }), { target: { value: "viscous_waterproof_01_dry_air" } });
    expect(screen.queryByRole("option", { name: "other" })).not.toBeInTheDocument();
    fireEvent.click(screen.getByRole("button", { name: "Settings" }));
    const select = screen.getByRole("combobox", { name: "Environment" });
    expect(select).toHaveValue("tank");
    fireEvent.change(select, { target: { value: environment } });
    fireEvent.click(screen.getByRole("button", { name: "Done" }));
    expect(within(screen.getByRole("status")).getByText(`Environment: ${environment === "tank" ? "Tank" : "Dry air"}`)).toBeInTheDocument();
    fireEvent.click(screen.getByRole("button", { name: "Check cameras" }));
    await waitFor(() => expect(screen.getByRole("button", { name: "Run" })).toBeEnabled());
    fireEvent.click(screen.getByRole("button", { name: "Run" }));
    expect(await screen.findByText(`Environment: ${environment} Joints that may move (ALL SEVEN)`)).toBeInTheDocument();
    fireEvent.click(screen.getByRole("button", { name: "Confirm and run" }));
    await waitFor(() => expect(bodyFor("/sessions").options).toEqual({ plan_id: "0123456789", environment }));
    for (const path of ["/dream/preview", "/dream/plan"]) {
      expect(bodyFor(path)).toMatchObject({ robot: "viscous_waterproof_01_dry_air", environment });
    }
  });

  it("shows the running environment from status after a page reload", async () => {
    const fallback = mocks.fetcher.getMockImplementation()!;
    mocks.fetcher.mockImplementation(async (url: string, options?: RequestInit) => {
      if (!url.endsWith("/dream/status")) return fallback(url, options);
      return new Response(JSON.stringify({ active: true, run: "live-test", robot: "viscous_waterproof_01_dry_air", status: {
        run: "live-test", mode: "live", environment: "dry_air", phase: "dream", call: 1, max_calls: 10,
        slots: { overhead: "black", wrist_l: "black", wrist_r: "black" }, peak_torque_nm: 0, peak_temp_c: 30,
      } }), { status: 200 });
    });
    open();
    await waitFor(() => expect(within(screen.getByRole("status")).getByText("Environment: Dry air")).toBeInTheDocument());
  });

  it("invalidates camera review when the environment changes and keeps settings only in page state", async () => {
    const page = open();
    fireEvent.click(screen.getByRole("button", { name: "Check cameras" }));
    await waitFor(() => expect(screen.getByRole("button", { name: "Run" })).toBeEnabled());
    fireEvent.click(screen.getByRole("button", { name: "Settings" }));
    fireEvent.change(screen.getByRole("combobox", { name: "Environment" }), { target: { value: "dry_air" } });
    fireEvent.click(screen.getByRole("button", { name: "Done" }));
    expect(screen.getByRole("button", { name: "Run" })).toBeDisabled();
    page.unmount();
    open();
    fireEvent.click(screen.getByRole("button", { name: "Settings" }));
    expect(screen.getByRole("combobox", { name: "Environment" })).toHaveValue("tank");
  });
});
