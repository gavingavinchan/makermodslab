import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, expect, it, vi } from "vitest";
import CreateRobotDialog from "./CreateRobotDialog";
import { SO101 } from "../dialogs/robotConfigFixtures";

vi.mock("@/hooks/useArms", () => ({
  useArms: () => ({
    arms: [SO101, { ...SO101, id: "viscous", label: "Viscous Arm v1", supports_bimanual: false }],
    loading: false,
  }),
}));
vi.mock("@/contexts/ApiContext", () => ({ useApi: () => ({ baseUrl: "http://test" }) }));
afterEach(cleanup);

it("creates a Viscous robot from the manifest and forces its supported single-arm layout", async () => {
  const create = vi.fn().mockResolvedValue(true);
  render(<CreateRobotDialog open onOpenChange={vi.fn()} availableNames={[]} defaultMode="bimanual" onCreateNew={create} />);
  fireEvent.change(screen.getByRole("textbox"), { target: { value: "viscous_01" } });
  fireEvent.click(screen.getByRole("radio", { name: /Viscous Arm v1/ }));
  expect(screen.getByRole("radio", { name: /Bimanual/ })).toBeDisabled();
  expect(screen.getByRole("radio", { name: /Single/ })).toHaveAttribute("aria-checked", "true");
  fireEvent.submit(screen.getByRole("textbox").closest("form")!);
  await waitFor(() => expect(create).toHaveBeenCalledWith("viscous_01", "single", "viscous"));
});
