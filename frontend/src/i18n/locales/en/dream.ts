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
  open: "Dream-1 Playground", activity: "a Dream-1 run", title: "Dream-1", subtitle: "Viscous · Aquarium",
  robot: "Robot", chooseRobot: "Select a Viscous robot", noRobot: "Add a Viscous robot in Lab to use Dream-1.",
  settings: "Settings", cameras: "Cameras", front: "Front", left: "Left", right: "Right", swap: "Bring {{camera}} to the centre",
  imageUnavailable: "Camera image unavailable. Check the camera connection or the runner status.",
  black: "black", noCamera: "Camera off", liveCrop: "live camera, crop shown", exact: "Dream request · exact JPEG · 320 × 176",
  replayCrop: "Replay · reconstructed Dream crop", what: "What should it do?", task: "Task", model: "Model", beta: "Dream-1 Beta",
  betaOnly: "The runner supports Dream-1 Beta only.", run: "Run", working: "Working on: {{prompt}}", rest: "Return to rest",
  stopping: "Returning to rest…", idle: "Ready to check the camera inputs.", planning: "Running the plan on a fake arm…", checking: "Checking cameras…",
  maxCalls: "Maximum Dream calls", pause: "Pause before the first call (seconds)", mapping: "Dream camera slots", chooseCamera: "Select a camera",
  copyLeft: "Copy Left", mappingNote: "Rotations come from the robot record. The runner applies the 16:9 centre crop and resizes to 320 × 176.",
  check: "Check cameras", inputs: "Inputs id: {{id}}", approve: "Inputs look right", approved: "Inputs approved", review: "Check each tile: correct camera, upright, natural colours, and the arm in view.",
  confirm: "Run this plan?", confirmNote: "Review the runner’s movement plan. Confirming starts the standalone red button, then the real-arm runner.",
  readiness: "Live readiness from the runner", readinessNote: "Confirmation supplies the plan id. Other refusals still apply; the runner checks again before connecting.",
  cancel: "Cancel", confirmRun: "Confirm and run", close: "Done", failed: "Dream action failed.", stale: "No fresh status from the runner. Check the arm and the standalone red button.",
  peak: "Peak {{torque}} Nm · {{temperature}} °C", call: "Dream call {{call}} of {{cap}}", button: "Standalone stop button is open",
  buttonOnly: "Release all torque stays on the standalone red button.",
  demo: "Replay demo", exitDemo: "Your robot", demoLabel: "Saved aquarium run · 14 Nov 2026 · no hardware",
  demoNote: "Replay uses saved camera JPEGs through the runner’s crop. These are reconstructed views; new live runs show the exact request bytes.",
  play: "Play", pauseReplay: "Pause", timeline: "Replay timeline", telemetry: "Measured joints", joint: "Joint", measured: "Measured °", commanded: "Commanded °",
  no3d: "Viscous has no frontend URDF registered. Measured and commanded joints are shown below.",
  phases: "Recorded phases", previewFirst: "Check cameras before running.", previewSaved: "Camera inputs captured for review.", back: "Back to Lab",
} as const;
