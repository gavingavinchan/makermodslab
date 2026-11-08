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
import { describe, expect, it, vi } from "vitest";
import { movedSinceCapture, tankApi } from "./tankApi";

describe("tank measurement contract", () => {
  it("uses only versioned routes and retains data labels", async () => {
    const fetcher = vi.fn(
      async (_url: string, _options?: RequestInit) =>
        new Response("{}", { status: 200 }),
    );
    const api = tankApi("http://lab", fetcher);
    await api.start("tank arm");
    await api.status();
    await api.capture("envelope:yaw_link:near");
    await api.delete(2);
    await api.save();
    await api.stop();
    await api.latest("tank arm");
    expect(fetcher.mock.calls.map((c) => c[0])).toEqual([
      "http://lab/api/v1/tank-measure/start",
      "http://lab/api/v1/tank-measure/status",
      "http://lab/api/v1/tank-measure/capture",
      "http://lab/api/v1/tank-measure/touches/2",
      "http://lab/api/v1/tank-measure/save",
      "http://lab/api/v1/tank-measure/stop",
      "http://lab/api/v1/tank-measure/latest?robot_name=tank%20arm",
    ]);
    expect(JSON.parse(fetcher.mock.calls[2][1]?.body as string)).toEqual({
      label: "envelope:yaw_link:near",
    });
  });

  it("requires twenty millimetres between automatic touches", () => {
    expect(movedSinceCapture(null, [0, 0, 0])).toBe(true);
    expect(movedSinceCapture([0, 0, 0], [19, 0, 0])).toBe(false);
    expect(movedSinceCapture([0, 0, 0], [0, 12, 16])).toBe(true);
    expect(movedSinceCapture([0, 0, 0], null)).toBe(false);
  });
});
