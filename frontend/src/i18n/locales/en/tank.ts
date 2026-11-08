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
export default {
  button: "Measure tank",
  title: "Measure tank · {{robot}}",
  instruction:
    "Torque stays off. Move the arm by hand and touch the glass with the tip of the closed gripper.",
  measurementOnly: "Measurement only: teleop does not use this yet.",
  start: "Start measurement",
  stop: "Stop measurement",
  close: "Close",
  capture: "Capture {{label}}",
  auto: "Auto-capture when still",
  moveAgain: "Move at least 20 mm before the next auto-capture.",
  stale: "Stale · capture disabled",
  still: "Still · ready to capture",
  moving: "Hold still for 1 second",
  probe: "Probe xyz (mm): {{xyz}}",
  opening: "Gripper opening per finger: {{value}} mm",
  openWarning: "Close the gripper: more than 5 mm open per finger.",
  guide: "Glass touches",
  spacing:
    "On each long wall, separate the two touches by at least 150 mm. Back wall touches go either side of the tower.",
  rightFirst: "Right wall · near the base",
  rightSecond: "Right wall · farther along the wall",
  leftFirst: "Left wall · near the base",
  leftSecond: "Left wall · farther along the wall",
  nearFirst: "Back wall · one side of the tower",
  nearSecond: "Back wall · other side of the tower",
  floorStep: "Floor · one touch",
  checkStep: "Check touch · any wall",
  checkFace: "Check wall",
  done: "Glass sequence complete. Review the fit and save.",
  tower: "Base tower (once)",
  towerHint:
    "Re-measure all four faces only if the mount changes. Saved tower geometry stays in the base frame.",
  tower_front: "Tower · front toward the arm",
  tower_top: "Tower · top",
  tower_left: "Tower · left (+y)",
  tower_right: "Tower · right (−y)",
  envelope: "Cable envelope (after re-cabling)",
  envelopeHint:
    "Fit the walls first. Press the selected link’s cable side against its selected face, then capture. Unmeasured links use 40 mm default padding.",
  link: "Link",
  face: "Face",
  useEnvelope: "Measure selected cable side",
  useGuide: "Return to glass sequence",
  fit: "Tank fit",
  width: "Inner width: {{value}} mm",
  yaw: "Tank yaw: {{value}}°",
  rms: "Residual RMS / max: {{rms}} / {{max}} mm",
  j1Distance: "J1 → {{face}}: {{value}} mm",
  pass: "Quality gates pass",
  fail: "Quality gates pending or failed",
  save: "Save measurement",
  saved: "Measurement saved.",
  previous: "Last saved: {{time}}",
  change:
    "J1 moved {{distance}} mm toward {{face}}; rotated {{rotation}}° since last save.",
  clearances: "Live link clearances (mm)",
  clearanceHint:
    "Red means below the configured margin. Floor contact is informational. Rim is headroom. Tower uses mesh vertices against a box; this display does not enforce clearance.",
  padding: "Padding: {{value}} mm · {{source}}",
  defaultPadding: "default",
  measuredPadding: "measured",
  touches: "Captured touches",
  delete: "Delete touch {{index}}",
  residual: "Residual: {{value}} mm",
  error: "Measurement request failed",
  drawing: "Top view in the arm base frame",
  axes: "−x → arm reach; +y ↑ left",
  base: "J1 / base",
  estimated: "estimated",
  near: "Back / near (+x)",
  far: "Far (−x)",
  left: "Left (+y)",
  right: "Right (−y)",
  floor: "Floor",
  rim: "Rim",
  towerFace: "Tower",
  checkLabel: "Check · {{face}}",
  envelopeLabel: "Cable · {{link}} · {{face}}",
  optionalTarget: "Selected probe: {{label}}",
  retry: "Retry status",
  floorInfo: "floor contact: informational",
} as const;
