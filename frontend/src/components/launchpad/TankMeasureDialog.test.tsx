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
import {
  act,
  cleanup,
  fireEvent,
  render,
  screen,
} from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import type { TankStatus } from "@/lib/tankApi";
import TankMeasureDialog from "./TankMeasureDialog";

const fixture = vi.hoisted(() => ({
  fetcher: vi.fn(),
  capture: vi.fn(),
  status: {} as TankStatus,
}));
vi.mock("@/contexts/ApiContext", () => ({
  useApi: () => ({ baseUrl: "http://fake", fetchWithHeaders: fixture.fetcher }),
}));

function fresh(): TankStatus {
  return {
    active: true,
    robot_name: "aquarium",
    error: null,
    stale: false,
    still: true,
    positions: {},
    probe_mm: [-200, -217, 80],
    gripper_opening_mm: 0,
    link_origins_mm: {
      base_link: [0, 0, 0],
      yaw_link: [0, 0, 57],
      tool0: [-200, -217, 80],
    },
    touches: [],
    fit: { valid: false, failures: [] },
    clearances: null,
    paddings: {},
    tower_box: null,
    idle_deadline: 900,
  };
}

beforeEach(() => {
  vi.useFakeTimers();
  fixture.status = fresh();
  fixture.capture.mockReset();
  fixture.fetcher.mockImplementation(
    async (url: string, options?: RequestInit) => {
      if (url.includes("/latest"))
        return new Response(
          JSON.stringify({ robot_name: "aquarium", measurement: null }),
        );
      if (url.endsWith("/capture")) {
        const label = JSON.parse(options?.body as string).label;
        fixture.capture(label);
        fixture.status = {
          ...fixture.status,
          touches: [
            ...fixture.status.touches,
            {
              label,
              point_mm: [...fixture.status.probe_mm!],
              positions: {},
              timestamp: "now",
            },
          ],
        };
      }
      if (url.endsWith("/stop"))
        fixture.status = { ...fixture.status, active: false };
      return new Response(JSON.stringify(fixture.status));
    },
  );
});
afterEach(() => {
  cleanup();
  vi.useRealTimers();
});
async function tick(ms = 120) {
  await act(async () => {
    await vi.advanceTimersByTimeAsync(ms);
  });
}

describe("tank measurement operator flow", () => {
  it("auto-captures once, advances, then requires movement before capturing again", async () => {
    render(
      <TankMeasureDialog open onOpenChange={vi.fn()} robotName="aquarium" />,
    );
    await tick();
    expect(fixture.capture).toHaveBeenCalledTimes(1);
    expect(fixture.capture).toHaveBeenLastCalledWith("right");
    expect(
      screen.getByText("Measurement only: teleop does not use this yet."),
    ).toBeInTheDocument();
    await tick(500);
    expect(fixture.capture).toHaveBeenCalledTimes(1);
    fixture.status = {
      ...fixture.status,
      probe_mm: [-230, -217, 80],
      still: false,
    };
    await tick();
    fixture.status = { ...fixture.status, still: true };
    await tick();
    expect(fixture.capture).toHaveBeenCalledTimes(2);
    expect(
      screen.getByRole("button", { name: /Capture Left/ }),
    ).toBeInTheDocument();
  });

  it.each(["stale", "open"])(
    "disables capture for %s readings",
    async (reason) => {
      fixture.status = {
        ...fixture.status,
        stale: reason === "stale",
        gripper_opening_mm: reason === "open" ? 20 : 0,
      };
      render(
        <TankMeasureDialog open onOpenChange={vi.fn()} robotName="aquarium" />,
      );
      await tick();
      expect(fixture.capture).not.toHaveBeenCalled();
      expect(
        screen.getByRole("button", { name: /Capture Right/ }),
      ).toBeDisabled();
      expect(
        screen.getByRole("button", { name: "Save measurement" }),
      ).toBeDisabled();
    },
  );

  it("stops its active session before closing", async () => {
    const close = vi.fn();
    render(
      <TankMeasureDialog open onOpenChange={close} robotName="aquarium" />,
    );
    await tick();
    fireEvent.click(screen.getByRole("button", { name: "Close" }));
    await tick();
    expect(
      fixture.fetcher.mock.calls.some(([url]) =>
        url.endsWith("/tank-measure/stop"),
      ),
    ).toBe(true);
    expect(close).toHaveBeenCalledWith(false);
  });
});
