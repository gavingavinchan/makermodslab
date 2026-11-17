// Copyright 2026 MakerMods. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import { afterEach, expect, it, vi } from "vitest";
import { act, cleanup, fireEvent, render, screen } from "@testing-library/react";
import "@/i18n";
import BackendCameraStream from "./BackendCameraStream";

const mocks = vi.hoisted(() => ({ fetch: vi.fn() }));
vi.mock("@/contexts/ApiContext", () => ({
  useApi: () => ({ baseUrl: "http://lab", fetchWithHeaders: mocks.fetch }),
}));
afterEach(() => { cleanup(); vi.useRealTimers(); mocks.fetch.mockReset(); });

it("shows the Dream 409 detail, pauses retries, and lets the user retry after the run", async () => {
  vi.useFakeTimers();
  mocks.fetch.mockResolvedValue({ ok: false, status: 409, json: async () => ({
    code: "robot.busy.dream", detail: "a Dream run is using the cameras",
  }) });
  render(<BackendCameraStream cameraIndex={0} />);
  fireEvent.error(screen.getByRole("img"));
  await act(async () => { await Promise.resolve(); });
  expect(screen.getByText("a Dream run is using the cameras")).toBeInTheDocument();
  expect(screen.queryByRole("img")).not.toBeInTheDocument();
  expect(screen.queryByText("retrying...")).not.toBeInTheDocument();
  await act(async () => { vi.advanceTimersByTime(60000); });
  expect(mocks.fetch).toHaveBeenCalledTimes(1);
  expect(screen.queryByRole("img")).not.toBeInTheDocument();
  fireEvent.click(screen.getByRole("button"));
  expect(screen.getByRole("img")).toHaveAttribute("src", "http://lab/api/v1/camera-preview/0?r=1");
});

it("keeps the existing retry behavior for recording and transient errors", async () => {
  vi.useFakeTimers();
  mocks.fetch.mockResolvedValue({ ok: false, status: 409, json: async () => ({
    detail: "Recording is active — the cameras are in use.",
  }) });
  render(<BackendCameraStream cameraIndex={0} />);
  fireEvent.error(screen.getByRole("img"));
  await act(async () => { await Promise.resolve(); });
  expect(screen.getByText("Recording is active — the cameras are in use.")).toBeInTheDocument();
  await act(async () => { vi.advanceTimersByTime(2000); });
  expect(screen.getByRole("img")).toBeInTheDocument();
});
