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
  environment: "环境", tank: "水箱", dryAir: "干燥空气",
  noCodeEdits: "运行期间请勿编辑代码。",
  open: "Dream-1 Playground", activity: "Dream-1 运行", title: "Dream-1", subtitle: "Viscous · Dream-1",
  robot: "机器人", chooseRobot: "选择 Viscous 机器人", noRobot: "请先在 Lab 中添加 Viscous 机器人，以使用 Dream-1。",
  settings: "设置", cameras: "摄像头", front: "前方", left: "左侧", right: "右侧", swap: "将{{camera}}移到中央",
  imageUnavailable: "摄像头画面不可用。请检查摄像头连接或运行器状态。",
  black: "黑屏", noCamera: "摄像头已关闭", liveCrop: "实时摄像头，已显示裁剪范围", exact: "Dream 请求 · 原始 JPEG · 320 × 176",
  replayCrop: "回放 · 重建的 Dream 裁剪画面", what: "让它做什么？", task: "任务", model: "模型", beta: "Dream-1 Beta",
  betaOnly: "运行器仅支持 Dream-1 Beta。", run: "运行", working: "正在执行：{{prompt}}", rest: "返回静止姿态",
  stopping: "正在返回静止姿态…", idle: "可以检查摄像头输入。", planning: "正在用模拟机械臂验证计划…", checking: "正在检查摄像头…",
  maxCalls: "Dream 最大调用次数", pause: "首次调用前暂停（秒）", mapping: "Dream 摄像头槽位", chooseCamera: "选择摄像头",
  copyLeft: "复制左侧画面", mappingNote: "旋转角度来自机器人记录。运行器居中裁剪为 16:9，再缩放至 320 × 176。",
  check: "检查摄像头", inputs: "输入标识：{{id}}", approve: "输入画面正确", approved: "输入已确认", review: "请检查每个画面：摄像头正确、方向朝上、色彩自然，且机械臂在画面内。",
  confirm: "运行此计划？", confirmNote: "请审阅运行器的运动计划。确认后将启动独立红色停止按钮，再启动真实机械臂运行器。",
  readiness: "运行器的实时就绪检查", readinessNote: "确认时将提供计划标识。其他拒绝条件仍然生效；运行器会在连接前再次检查。",
  cancel: "取消", confirmRun: "确认并运行", close: "完成", failed: "Dream 操作失败。", stale: "未收到运行器的最新状态。请检查机械臂和独立红色停止按钮。",
  peak: "峰值 {{torque}} Nm · {{temperature}} °C", call: "Dream 调用 {{call}} / {{cap}}", button: "独立停止按钮已打开",
  buttonOnly: "释放全部扭矩只能通过独立红色停止按钮操作。",
  demo: "回放演示", exitDemo: "你的机器人", demoLabel: "水族箱录制 · 2026 年 11 月 14 日 · 不连接硬件",
  demoNote: "回放将保存的摄像头 JPEG 通过运行器裁剪重建画面；新的真实运行会显示实际请求的原始字节。",
  play: "播放", pauseReplay: "暂停", timeline: "回放时间轴", telemetry: "实测关节", joint: "关节", measured: "实测 °", commanded: "指令 °",
  no3d: "Viscous 尚未注册前端 URDF。下方显示实测与指令关节角度。",
  phases: "已记录阶段", previewFirst: "运行前请检查摄像头。", previewSaved: "摄像头输入已捕获，请审阅。", back: "返回 Lab",
} as const;
