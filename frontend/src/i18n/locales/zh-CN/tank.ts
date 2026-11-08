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
  button: "测量鱼缸",
  title: "测量鱼缸 · {{robot}}",
  instruction: "扭矩保持关闭。用手移动机械臂，以闭合夹爪的尖端触碰玻璃。",
  measurementOnly: "仅用于测量：遥操作尚未使用这些数据。",
  start: "开始测量",
  stop: "停止测量",
  close: "关闭",
  capture: "采集：{{label}}",
  auto: "静止后自动采集",
  moveAgain: "再次自动采集前，请移动至少 20 mm。",
  stale: "读数过期 · 无法采集",
  still: "已静止 · 可以采集",
  moving: "请保持静止 1 秒",
  probe: "探针 xyz (mm)：{{xyz}}",
  opening: "每侧夹指开度：{{value}} mm",
  openWarning: "请闭合夹爪：每侧开度已超过 5 mm。",
  guide: "玻璃触点",
  spacing:
    "每侧长壁的两个触点沿壁间距至少 150 mm。后壁触点分别位于支撑塔两侧。",
  rightFirst: "右壁 · 靠近底座",
  rightSecond: "右壁 · 远离底座",
  leftFirst: "左壁 · 靠近底座",
  leftSecond: "左壁 · 远离底座",
  nearFirst: "后壁 · 支撑塔一侧",
  nearSecond: "后壁 · 支撑塔另一侧",
  floorStep: "缸底 · 一个触点",
  checkStep: "检查触点 · 任意侧壁",
  checkFace: "检查侧壁",
  done: "玻璃触点已完成。请检查拟合结果并保存。",
  tower: "底座支撑塔（测量一次）",
  towerHint:
    "仅在安装发生变化后重新测量全部四个面。已保存的支撑塔几何保持在底座坐标系中。",
  tower_front: "支撑塔 · 朝向机械臂的正面",
  tower_top: "支撑塔 · 顶面",
  tower_left: "支撑塔 · 左侧 (+y)",
  tower_right: "支撑塔 · 右侧 (−y)",
  envelope: "线缆包络（重新布线后）",
  envelopeHint:
    "请先完成侧壁拟合。将所选连杆的线缆侧抵住所选面，然后采集。未测连杆默认使用 40 mm 余量。",
  link: "连杆",
  face: "表面",
  useEnvelope: "测量所选线缆侧",
  useGuide: "返回玻璃触点流程",
  fit: "鱼缸拟合",
  width: "内宽：{{value}} mm",
  yaw: "鱼缸偏航角：{{value}}°",
  rms: "残差均方根 / 最大值：{{rms}} / {{max}} mm",
  j1Distance: "J1 → {{face}}：{{value}} mm",
  pass: "质量检查通过",
  fail: "质量检查未完成或未通过",
  save: "保存测量",
  saved: "测量已保存。",
  previous: "上次保存：{{time}}",
  change:
    "与上次保存相比，J1 向{{face}}移动 {{distance}} mm；旋转 {{rotation}}°。",
  clearances: "实时连杆间隙 (mm)",
  clearanceHint:
    "红色表示低于配置余量。缸底接触仅供参考。缸沿表示上方空间。支撑塔间隙按网格顶点到箱体计算；此显示不限制运动。",
  padding: "余量：{{value}} mm · {{source}}",
  defaultPadding: "默认",
  measuredPadding: "已测量",
  touches: "已采集触点",
  delete: "删除触点 {{index}}",
  residual: "残差：{{value}} mm",
  error: "测量请求失败",
  drawing: "机械臂底座坐标系俯视图",
  axes: "−x → 机械臂伸展方向；+y ↑ 左侧",
  base: "J1 / 底座",
  estimated: "估计",
  near: "后壁 / 近壁 (+x)",
  far: "远壁 (−x)",
  left: "左壁 (+y)",
  right: "右壁 (−y)",
  floor: "缸底",
  rim: "缸沿",
  towerFace: "支撑塔",
  checkLabel: "检查 · {{face}}",
  envelopeLabel: "线缆 · {{link}} · {{face}}",
  optionalTarget: "当前探测项：{{label}}",
  retry: "重试状态读取",
  floorInfo: "缸底接触：仅供参考",
} as const;
