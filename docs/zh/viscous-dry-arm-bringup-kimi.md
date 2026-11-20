# Viscous 干臂（非防水）新机上电调试指南 —— 给 Kimi Code 执行

> 本文件是写给 **Kimi Code** 的操作手册。深圳的同学把这个文件交给 Kimi Code，Kimi 按步骤带着同学完成。
> 作者：Gavin（经 Claude Code 整理），2026-11-19。

---

## 0. Kimi，请先读这一节

### 0.1 目标（只有这一个）

两台新组装的 **Viscous Arm v1 干臂**（非防水版，结构与第一台干臂 `viscous_01` 完全相同），每台配一个 **Star Arm 102 主臂（leader）**。

**目标：让每台臂能在 MakerMods Lab 里遥操作，确认 7 个电机全部能动、方向正确，然后把结果打包成 zip，同学用微信发给 Gavin。**

不需要精确标定。零位"大概对"就可以。限位直接沿用第一台干臂的数据（已经替你算好，见第 5 节）。

### 0.2 你的角色

- 你负责：安装软件、运行命令、检查输出、生成文件、打包。
- 同学负责：所有**物理操作**（摆姿势、接线、断电/上电、按 Lab 里的按钮、拍照）。
- 你要用**中文**、**一步一句**地告诉同学要做什么，然后**等同学回复"好了"再继续**。不要一次给一长串物理操作。
- 每一步的命令输出都用 `tee` 存到日志文件里（见 1.3 的目录结构）。

### 0.3 硬性规则（必须遵守）

1. **只用 MacBook。** maker-arm SDK 只支持 macOS 和 Linux，不支持 Windows。如果同学用的是 Windows 电脑，停下来告诉他们换 MacBook。
2. **一次只处理一台臂。** 电脑上同一时间只插这一台臂的 CANable 和这一台的 Star。另一台的 USB 全部拔掉。
3. **不要让电机上力矩**，除了第 4.8 步同学在 Lab 里按"Teleop"按钮。具体来说，**禁止运行**：`maker-arm check`、`maker-arm teleop`、`examples/02_enable_hold.py`、`examples/03_sine_wave.py`、`tools/experimental/lerobot/mit_move.py`、`mit_spin.py`、`tools/diagnostics/jog_one_motor.py`，以及任何会 enable 电机的自写代码。
4. **写电机 flash 的命令**（`assign-id`、`zero`、`persist_can_timeout.py`、`switch_protocol.py`、Star 的 `set_origin_point`）运行前，必须先让同学确认对应的物理状态（例如"机械臂已折叠、手已离开"），得到"好了"才运行。
5. **不要修改** maker-arm SDK 或 MakerMods Lab 的代码。不要 `git commit` / `git push`。
6. **遇到任何异常就停下**：用中文向同学解释发生了什么，把日志保存好，不要自己想办法绕过（尤其是任何会让电机动起来的办法）。可以让同学把情况发给 Gavin。
7. 机械臂**没有刹车**。断电或 Lab 停止时力矩会释放，臂会在重力下落下。所以任何时候上电测试前，臂都要先折叠好放在桌上。

### 0.4 名称（ID）

| | 第一台新臂 | 第二台新臂 |
|---|---|---|
| 从臂（follower）ID | `viscous_02` | `viscous_03` |
| 主臂（leader）ID | `star_viscous_02` | `star_viscous_03` |

如果 Gavin 另外指定了名字，用 Gavin 的。下文用 `<ARM>` 表示从臂 ID、`<STAR>` 表示主臂 ID。

开始第一台之前，让同学用标签纸把 `viscous_02` 贴在：从臂底座、它的 Star 主臂、它的 CANable 转接器上。第二台同理贴 `viscous_03`。**CANable 会和这台臂绑定**（Lab 通过 CANable 的序列号识别是哪台臂），之后不要混用。

### 0.5 硬件背景

- 从臂：7 个 RobStride **RS00** 电机，走 CAN 总线，通过 **CANable 2**（slcan 固件）接 USB。在 Mac 上显示为 `/dev/cu.usbmodem*`。
- 主臂：Star Arm 102，FashionStar 舵机（ID 0–6），USB 串口（CH340），在 Mac 上显示为 `/dev/cu.wchusbserial*` 或 `/dev/cu.usbserial*`。没有出现的话需要装 CH340 驱动。
- 关节和电机 CAN ID 的对应：

| 关节 | 名称 | 从臂 CAN ID | Star 舵机 ID |
|---|---|---|---|
| J1 | 底座旋转 shoulder_pan | 1 | 0 |
| J2 | 肩 shoulder_lift | 2 | 1 |
| J3 | 肘 elbow_flex | 3 | 2 |
| J4 | 腕俯仰 wrist_flex | 4 | 3 |
| J5 | 腕偏航 wrist_yaw | 5 | 4 |
| J6 | 腕旋转 wrist_roll | 6 | 5 |
| J7 | 夹爪 gripper | 7 | 6 |

---

## 1. 一次性准备（这台 Mac 只做一次）

### 1.1 需要的东西

- MacBook（macOS），能上 GitHub（如果 GitHub 很慢，请同学先开代理）。
- Homebrew、git、git-lfs、Node.js（`brew install git git-lfs node`）、uv（`brew install uv`）。
- 每台臂：CANable 2 + 线、Star 主臂 + USB 线、两台臂各自的电源。
- 一个夹子或螺丝把从臂底座**固定在桌子上**（遥操作时底座旋转，不固定会翻倒）。

### 1.2 下载并安装

```bash
mkdir -p ~/viscous-bringup && cd ~/viscous-bringup

# maker-arm SDK（RobStride 电机工具），固定在验证过的版本
git clone https://github.com/makermods-robotics/maker-arm-sdk.git
git -C maker-arm-sdk checkout 8d3998d774650c239bc1b42f73c3f7d569e2c59d
uv venv --python 3.12 .venv-sdk
uv pip install --python .venv-sdk/bin/python -e ./maker-arm-sdk
.venv-sdk/bin/maker-arm --help

# MakerMods Lab（Viscous 分支）
git clone --branch feat/viscous-arm https://github.com/gavingavinchan/makermodslab.git
cd makermodslab
git lfs install && git lfs pull      # 失败的话可以继续，遥操作不需要这些文件
uv sync                               # 会从 GitHub 安装固定版本的 lerobot，可能比较慢
cd frontend && npm ci && npm run build && cd ..
```

说明：
- 如果 npm 下载太慢：`npm ci --registry=https://registry.npmmirror.com`。
- 如果 uv 下载 Python 或依赖太慢：优先让同学开代理。**不要**为了换镜像去改 `uv.lock` 或 `pyproject.toml`。
- `frontend/dist` 是本地构建产物，不要提交。

### 1.3 工作目录

每台臂的所有产出都放在 Lab 仓库里这台臂自己的文件夹下（最后整个文件夹打包）：

```
~/viscous-bringup/makermodslab/configs/viscous/<ARM>/
├── manifest.json
├── robot-record-template.json
├── calibration/
│   ├── robots/viscous_follower/<ARM>.json
│   └── teleoperators/rebot_102_leader/<STAR>.json
├── viscous_profiles/
│   ├── <ARM>.json
│   └── <STAR>.json
└── evidence/
    ├── logs/        # 每一步的命令输出
    ├── photos/      # 同学拍的照片
    ├── bringup-record.json
    └── README_zh.md
```

### 1.4 SDK 用的配置文件

把下面内容存为 `~/viscous-bringup/viscous_dry.yaml`（这是第一台干臂的 SDK 配置，单位是弧度；它只用来让 SDK 连接和写零位，**不会**让电机上力矩）：

```yaml
# Viscous Arm v1 干臂：7 个关节全部是 RS00。零位 = 折叠姿态，夹爪完全闭合。
# 只用于 assign-id / scan / zero / 读数。不要用它做 check 或 teleop。
control_rate_hz: 200
max_velocity: 5.0
limit_margin: 0.0
feedback_timeout: 0.2
motor_can_timeout_ms: 200
joints:
  - {motor_id: 1, model: RS00, direction: 1, offset: 0.0, lo: -3.235, hi: 2.227, kp: 60, kd: 4.0}
  - {motor_id: 2, model: RS00, direction: 1, offset: 0.0, lo: -3.078, hi: -0.042, kp: 30, kd: 2.0}
  - {motor_id: 3, model: RS00, direction: 1, offset: 0.0, lo: 0.05, hi: 4.383, kp: 30, kd: 2.0}
  - {motor_id: 4, model: RS00, direction: 1, offset: 0.0, lo: -2.109, hi: 1.606, kp: 30, kd: 2.0}
  - {motor_id: 5, model: RS00, direction: 1, offset: 0.0, lo: -1.556, hi: 1.505, kp: 30, kd: 2.0}
  - {motor_id: 6, model: RS00, direction: 1, offset: 0.0, lo: -2.641, hi: 2.68, kp: 30, kd: 2.0}
  - {motor_id: 7, model: RS00, direction: 1, offset: 0.0, lo: -2.094, hi: -0.048, kp: 20, kd: 0.5}
```

下文的简写：

```bash
SDK=~/viscous-bringup/.venv-sdk/bin
PY=$SDK/python
CFG=~/viscous-bringup/viscous_dry.yaml
SDKDIR=~/viscous-bringup/maker-arm-sdk
CAN=/dev/cu.usbmodemXXXX        # 4.1 步找到的 CANable 端口
STARPORT=/dev/cu.wchusbserialXXXX  # 4.1 步找到的 Star 端口
EV=~/viscous-bringup/makermodslab/configs/viscous/<ARM>/evidence
```

---

## 2. 整体流程（每台臂一遍，约 20–30 分钟）

| 步骤 | 内容 | 同学要做的物理操作 |
|---|---|---|
| 4.1 | 找端口、记录 CANable 序列号 | 插 USB、给从臂上电 |
| 4.2 | 检查电机协议和 ID | 可能需要逐个接电机 |
| 4.3 | 确认关节对应 | 按顺序掰动每个关节 |
| 4.4 | 摆零位并写零 | 把两台臂都折叠好、拍照 |
| 4.5 | 断电重启，检查零位 | 断电再上电 |
| 4.6 | 写看门狗、切换 MIT 协议 | 断电再上电 |
| 4.7 | 生成 Lab 配置文件 | 无 |
| 4.8 | Lab 遥操作测试 | 固定底座、按按钮、动主臂、录视频 |
| 4.9 | 同学确认，打包 zip，微信发送 | 发微信 |

---

## 3. 每次开始一台臂之前

1. 确认电脑上只插着这台臂的 CANable 和 Star（另一台的 USB 都拔掉）。
2. `mkdir -p $EV/logs $EV/photos`
3. 在 `$EV/bringup-record.json` 里开始记录（见第 6 节的格式），每完成一步就更新。

---

## 4. 步骤

### 4.1 找端口、记录 CANable 序列号

对同学说：**"请把这台臂的 CANable 和 Star 插到 Mac 上，然后打开从臂电源。"** 等"好了"。

```bash
$PY -c "from serial.tools import list_ports
for p in list_ports.comports(): print(p.device, '|', p.serial_number, '|', hex(p.vid or 0), hex(p.pid or 0), '|', p.description)" | tee $EV/logs/01-ports.txt
```

- CANable：`/dev/cu.usbmodem…`，VID/PID `0x16d0 0x117e`（CANable 2 slcan 固件，Lab 就是按这个识别的）。记下它的 **serial_number**（例如 `205038953846`），写进 bringup-record，后面 manifest 要用。如果 VID/PID 不是这个，停下报告：Lab 会认不出这个转接器。
- Star：`/dev/cu.wchusbserial…` 或 `/dev/cu.usbserial…`，VID/PID `0x1a86 0x7523`。
- 如果看到两个 CANable 或两个 Star，说明另一台没拔，让同学拔掉。

### 4.2 检查电机协议和 ID

```bash
$SDK/maker-arm scan --backend slcan --port $CAN --max-id 127 | tee $EV/logs/02-scan.txt
```

`scan` 只发停止帧，不会让电机动。看结果：

**情况 A：正好找到 ID 1–7，七个都在** → 跳到 4.3。

**情况 B：只找到 ID 127（出厂默认）或部分 ID** → 需要分配 ID。RobStride 按 ID 寻址，多个 127 同时在总线上会一起被改，所以**一次只能有一个 127 的电机在总线上**。用"从底座往上逐个接"的办法：

1. 对同学说：**"请关掉从臂电源，把 CAN 线在 J1（底座电机）后面断开，只让 J1 接在总线上。然后上电。"**
2. 运行（确认只有一个电机）：
   ```bash
   $SDK/maker-arm scan --backend slcan --port $CAN --max-id 127
   $SDK/maker-arm assign-id --backend slcan --port $CAN --current-id 127 --new-id 1 | tee -a $EV/logs/02-assign-id.txt
   ```
   （`assign-id` 会要求按回车确认，确认只有一个电机后可以用 `printf '\n' |` 传回车。）
3. 对同学说：**"请断电，把下一个电机（J2）接上，再上电。"** 这时总线上 J1 已经是 ID 1，只有 J2 是 127。重复，`--new-id 2`。
4. 依次到 J7（`--new-id 7`）。
5. 全部接好后断电再上电，再 `scan` 一次，必须正好是 1–7。

**情况 C：一个电机都找不到** →
1. 先让同学检查：电源是否打开、CAN 线是否接好、终端电阻。
2. 如果硬件没问题，电机可能已经被切到了 MIT 协议（这时 SDK 的 `scan` 看不到它们）。运行：
   ```bash
   $PY $SDKDIR/tools/experimental/lerobot/switch_protocol.py --backend slcan --port $CAN --ids 1,2,3,4,5,6,7 --to private | tee $EV/logs/02-switch-to-private.txt
   ```
   如果显示切换成功：让同学**断电再上电**，然后重新 `scan`。如果还是找不到，停下，报告给 Gavin。

### 4.3 确认关节对应（只读，不动电机）

对同学说：**"接下来 40 秒，请按顺序用手慢慢掰动从臂的 J1、J2、J3、J4、J5、J6、J7（夹爪），每个关节掰动 3 秒左右再换下一个。每换一个请说出来。"**

```bash
cd $SDKDIR && timeout 45 $PY examples/01_read_joints.py --config $CFG --backend slcan --port $CAN | tee $EV/logs/03-joint-identify.txt
```

检查：每个时间段里只有对应的 `J<n>` 读数在变。如果顺序对不上（例如掰 J2 时 J3 在变），说明 ID 分配错了，停下报告。

### 4.4 摆零位并写零（从臂 + 主臂）

**零位姿态**（两台臂摆成一样）：
- 从臂完全**折叠**，放在桌上：J2（肩）和 J3（肘）**靠在折叠方向的机械限位上**。
- **夹爪完全闭合**。
- J1、J4、J5、J6 **目测摆正**：手臂和底座在一条直线上，腕部不扭转，夹爪朝前。不需要精确，大概对就行。
- Star 主臂摆成同样的折叠姿态，**夹爪完全闭合**。

对同学说：**"请把从臂和 Star 主臂都摆成折叠姿态（说明如上），夹爪都完全闭合。摆好后，从侧面和上面各拍一张照片（两台臂都拍），然后手离开机械臂，告诉我好了。"**

把照片存到 `$EV/photos/`（例如 `zero-follower-side.jpg`、`zero-follower-top.jpg`、`zero-star.jpg`）。

确认"好了"之后，写从臂零位：

```bash
printf '\n' | $SDK/maker-arm zero --all --config $CFG --backend slcan --port $CAN | tee $EV/logs/04-follower-zero.txt
```

日志里应有 `current motor coordinates`（写零前）、7 行 `motor N: zeroed and saved`、`post-zero motor coordinates` 都接近 0。

再写 Star 主臂零位（和 MakerMods Lab 的 Star 标定做的事完全一样：逐个舵机 unlock 再 set_origin_point）。把下面的代码存为 `~/viscous-bringup/star_zero.py`：

```python
import json, sys, time
from motorbridge_smart_servo import FashionStarServo

port, out = sys.argv[1], sys.argv[2]
ids = list(range(7))
bus = FashionStarServo(port, baudrate=1_000_000)

def read():
    for _ in range(3):
        data = bus.sync_monitor(ids)
        time.sleep(0.05)
    return {i: (round(data[i].angle_deg, 2) if data[i] and data[i].reliable else None) for i in ids}

try:
    pre = read()
    for i in ids:
        bus.unlock(i)
        time.sleep(0.01)
        bus.set_origin_point(i)
    time.sleep(0.3)
    for i in ids:
        bus.reset_multi_turn(i)
    post = read()
finally:
    bus.close()
json.dump({"pre_zero_deg": pre, "post_zero_deg": post}, open(out, "w"), indent=2)
print(json.dumps({"pre_zero_deg": pre, "post_zero_deg": post}, indent=2))
```

```bash
$PY ~/viscous-bringup/star_zero.py $STARPORT $EV/logs/04-star-zero.json | tee $EV/logs/04-star-zero.txt
```

`post_zero_deg` 每个舵机应在 0 附近（±3°）。如果有 `None`，说明那个舵机没回应，检查 Star 的线和电源后重跑。

### 4.5 断电重启，检查零位是否保存

对同学说：**"请关掉从臂电源，等 5 秒再打开。Star 也拔掉 USB 再插上。臂保持折叠不要动。"**

```bash
$SDK/maker-arm scan --backend slcan --port $CAN --max-id 8 | tee $EV/logs/05-scan-after-powercycle.txt
```

每个电机的 `pos_rad` 应接近 0（±0.2 rad 以内；如果接近 ±6.28 也算正常，是整圈计数在重启后复位）。

Star 用下面的只读命令检查（不写任何东西）：

```bash
$PY -c "
import time
from motorbridge_smart_servo import FashionStarServo
b = FashionStarServo('$STARPORT', baudrate=1_000_000)
ids = list(range(7))
for i in ids: b.reset_multi_turn(i)
for _ in range(3): d = b.sync_monitor(ids); time.sleep(0.05)
print({i: (round(d[i].angle_deg, 1) if d[i] else None) for i in ids})
b.close()" | tee $EV/logs/05-star-after-powercycle.txt
```

每个读数应在 0 附近（±5°）。不对的话回到 4.4 重新摆姿势、重新写零。

### 4.6 写看门狗、切换到 MIT 协议

**为什么：** 看门狗是存在每个电机里的超时设置：如果电机 200 ms 没收到命令（电脑死机、USB 被拔、Lab 被强制关掉），电机自己释放力矩，不会一直卡在最后的命令上。MakerMods Lab 用 **MIT 协议**驱动电机，而 MIT 协议下读写不了这个设置，所以必须**先写看门狗，再切 MIT**。

对同学说：**"接下来会改电机的设置，臂保持折叠，手离开。"** 等"好了"。

```bash
$PY $SDKDIR/tools/experimental/lerobot/persist_can_timeout.py --backend slcan --port $CAN --ids 1,2,3,4,5,6,7 --timeout-ms 200 | tee $EV/logs/06-watchdog.txt
```

7 个电机都要显示 `written and saved to flash`，最后一行 `failed []`。有失败的就停下报告。

```bash
$PY $SDKDIR/tools/experimental/lerobot/switch_protocol.py --backend slcan --port $CAN --ids 1,2,3,4,5,6,7 --to mit | tee $EV/logs/06-switch-to-mit.txt
```

7 个都要显示 `private -> mit command acked ✅`（最后一行 `failed []`）。然后对同学说：**"请关掉从臂电源，等 5 秒再打开。"**

验证：

```bash
$SDK/maker-arm scan --backend slcan --port $CAN --max-id 8 | tee $EV/logs/06-scan-after-mit.txt
```

这次**应该找不到任何电机**（`no motors found`）——因为它们已经是 MIT 协议了，这是正确的。再确认它们在 MIT 下有回应（这条命令对已经是 MIT 的电机只会显示 "already mit, skipping"，不会改任何东西）：

```bash
$PY $SDKDIR/tools/experimental/lerobot/switch_protocol.py --backend slcan --port $CAN --ids 1,2,3,4,5,6,7 --to mit | tee $EV/logs/06-verify-mit.txt
```

7 个都必须显示 `already mit, skipping`。

> 从这里开始，SDK 的 `scan` / `zero` / `01_read_joints.py` 都看不到电机了。如果之后需要重新写零，要先用 `switch_protocol.py --to private` 切回去、断电重启，做完再切回 MIT。

### 4.7 生成 Lab 配置文件

在 `configs/viscous/<ARM>/` 下按第 5 节生成所有文件（把 `<ARM>`、`<STAR>`、CANable 序列号替换成真实值）。然后：

1. 计算 manifest 里的 sha256（第 5.6 节）。
2. 校验 Lab 能读懂这些配置（不连接硬件）：
   ```bash
   cd ~/viscous-bringup/makermodslab
   CHECK_HOME=$(mktemp -d) && mkdir -p $CHECK_HOME/viscous_profiles
   cp configs/viscous/<ARM>/viscous_profiles/*.json $CHECK_HOME/viscous_profiles/
   MAKERMODSLAB_HOME=$CHECK_HOME uv run python -c 'from makermodslab.arms.viscous import VISCOUS; f = VISCOUS.single_follower_config("/dev/null", "<ARM>"); l = VISCOUS.single_leader_config("/dev/null", "<STAR>"); print("OK", f.joint_limits, l.joint_ranges)' | tee configs/viscous/<ARM>/evidence/logs/07-profile-check.txt
   ```
   必须打印 `OK`。
3. 安装到这台 Mac 的 Lab 里（只写磁盘文件，不碰硬件）：
   ```bash
   python3 scripts/restore-viscous-waterproof.py --bundle configs/viscous/<ARM> \
     --follower-port $CAN --leader-port $STARPORT | tee configs/viscous/<ARM>/evidence/logs/07-restore.txt
   ```
   （脚本名字虽然叫 waterproof，但可以安装任何 Viscous 臂的配置。默认安装到 `~/.makermods/makermodslab-viscous`。）

### 4.8 Lab 遥操作测试

启动 Lab（在另一个终端窗口，保持运行）：

```bash
cd ~/viscous-bringup/makermodslab
MAKERMODSLAB_HOME=~/.makermods/makermodslab-viscous uv run makermodslab 2>&1 | tee configs/viscous/<ARM>/evidence/logs/08-lab-server.log
```

浏览器打开 `http://127.0.0.1:8000`。

**物理准备**，逐条对同学说，每条等"好了"：
1. **"请用夹子或螺丝把从臂底座牢牢固定在桌子上。"**
2. **"从臂保持折叠，Star 主臂也折叠、夹爪闭合，拿在手里。"**
3. **"从臂周围 1 米内不要放东西，人也不要靠近。另一位同学把手放在从臂电源开关旁边，有任何异常（乱动、异响、冒烟）就立刻关电源。"**
4. **"请用手机开始录像，拍到从臂和主臂。"**

**测试**，对同学说：
1. **"在 Lab 页面选择机器人 `<ARM>`，进入遥操作（Teleop），点开始。等页面显示已连接、正在跟随。"**
   - 如果页面提示 CANable/适配器不匹配或身份未验证，**不要点"继续"**，停下：说明 manifest 里的序列号不对，回到 4.7 检查。
   - 如果提示 Star 起始姿态偏差太大，让同学把 Star 再折叠好重试。
2. **"现在慢慢地，一次只动 Star 的一个关节，每个动 20–30 度，看从臂对应的关节是否跟着动、方向是否一样。顺序：J1 底座、J2 肩（只抬一点点）、J3 肘、J4 腕俯仰、J5 腕偏航、J6 腕旋转、J7 夹爪张开再闭合。每个关节动完都回到折叠位置。"**
3. **"每个关节告诉我：动了没有？方向对不对？"** 你把结果记进 bringup-record 的 `teleop_test`。
4. **"全部测完后，把 Star 折叠回去，在 Lab 里点停止（Stop）。从臂会自己慢慢回到起始姿态然后释放力矩。手扶着从臂。"**
5. **"停止录像。"** 让同学把视频存到 `evidence/photos/teleop-test.mp4`（太大的话只发微信，不放进 zip）。

**整个测试控制在 2 分钟以内**。干臂的肩部电机在受力时发热很快，不要让臂长时间伸展着保持不动。

判断标准：
- 7 个关节都跟着动、方向都对 → 通过。
- 某个关节不动或方向相反 → 记录下来，**不要改配置**，在 README_zh.md 里写清楚，照样打包发给 Gavin。
- 出现报错 → 把 Lab 页面的错误截图存到 `evidence/photos/`，日志在 `08-lab-server.log`，停下报告。

测试结束后可以在 Lab 终端按 Ctrl-C 关闭 Lab。

### 4.9 同学确认，打包

1. 问同学：**"这台臂测试都正常吗？我现在打包，你确认一下。"** 同学回复确认后再打包。
2. 写 `evidence/README_zh.md`（第 6.2 节）和最终的 `bringup-record.json`。
3. 重新计算 manifest 里所有文件的 sha256（因为 evidence 加了文件）。
4. 打包：
   ```bash
   cd ~/viscous-bringup/makermodslab/configs/viscous
   zip -r ~/Desktop/<ARM>-bringup-$(date +%Y%m%d).zip <ARM> -x '*.DS_Store'
   ls -lh ~/Desktop/<ARM>-bringup-*.zip
   ```
5. 告诉同学：**"打包好了，文件在桌面：`<ARM>-bringup-日期.zip`。请用微信发给 Gavin。录像视频单独发。"**

然后换第二台臂：拔掉这台的 USB，插上另一台的，从第 3 节重新开始。

---

## 5. 要生成的文件（内容已经算好）

### 5.1 为什么是这些数字

- 两台新干臂和第一台干臂 `viscous_01` 结构完全相同，所以**直接沿用 `viscous_01` 测出来的软限位**。
- 因为零位只是"大概对"，为了防止零位偏几度导致关节被命令顶到机械限位上（会堵转发热），**每个不是靠机械限位定零的端点都往里缩了 5°**。J2、J3 的折叠端和夹爪的闭合端是靠限位/闭合定零的，所以不缩。
- Star 到从臂的映射系数沿用 `viscous_01` 的预设。夹爪偏移量用 **0**（`viscous_01` 预设里的 −33° 是补偿它自己零位漂移的，新臂不需要）。
- 这些数字已经用 MakerMods Lab 自己的加载代码验证过。**不要改动。**

### 5.2 `viscous_profiles/<ARM>.json` 和 `viscous_profiles/<STAR>.json`

两个文件**内容完全相同**：

```json
{
  "follower": {
    "joint_limits": {
      "shoulder_pan": [-180.4, 122.6],
      "shoulder_lift": [-171.4, -2.4],
      "elbow_flex": [2.9, 246.1],
      "wrist_flex": [-115.8, 87.0],
      "wrist_yaw": [-84.2, 81.2],
      "wrist_roll": [-146.3, 148.6],
      "gripper": [-115.0, -2.8]
    }
  },
  "leader": {
    "joint_directions": {
      "shoulder_pan": 1.439742,
      "shoulder_lift": -0.868697,
      "elbow_flex": -0.915867,
      "wrist_flex": -1.082952,
      "wrist_yaw": -1.260963,
      "wrist_roll": 0.945426,
      "gripper": -1.778452
    },
    "joint_ranges": {
      "shoulder_pan": [-180, 122],
      "shoulder_lift": [-171, -3],
      "elbow_flex": [3, 246],
      "wrist_flex": [-115, 87],
      "wrist_yaw": [-84, 81],
      "wrist_roll": [-146, 148],
      "gripper": [-115, -3]
    },
    "joint_offsets": {
      "shoulder_pan": 0.0,
      "shoulder_lift": 0.0,
      "elbow_flex": 0.0,
      "wrist_flex": 0.0,
      "wrist_yaw": 0.0,
      "wrist_roll": 0.0,
      "gripper": 0.0
    },
    "max_start_offset_deg": 30.0
  }
}
```

### 5.3 `calibration/robots/viscous_follower/<ARM>.json`

零位保存在电机里，这个文件只是 lerobot 需要的记录（`homing_offset` 为 0 是正确的）：

```json
{
  "shoulder_pan":  {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": -180, "range_max": 122},
  "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": 0, "range_min": -171, "range_max": -3},
  "elbow_flex":    {"id": 3, "drive_mode": 0, "homing_offset": 0, "range_min": 3, "range_max": 246},
  "wrist_flex":    {"id": 4, "drive_mode": 0, "homing_offset": 0, "range_min": -115, "range_max": 87},
  "wrist_yaw":     {"id": 5, "drive_mode": 0, "homing_offset": 0, "range_min": -84, "range_max": 81},
  "wrist_roll":    {"id": 6, "drive_mode": 0, "homing_offset": 0, "range_min": -146, "range_max": 148},
  "gripper":       {"id": 7, "drive_mode": 0, "homing_offset": 0, "range_min": -115, "range_max": -3}
}
```

### 5.4 `calibration/teleoperators/rebot_102_leader/<STAR>.json`

同上，但 `id` 是 Star 舵机号 0–6：

```json
{
  "shoulder_pan":  {"id": 0, "drive_mode": 0, "homing_offset": 0, "range_min": -180, "range_max": 122},
  "shoulder_lift": {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": -171, "range_max": -3},
  "elbow_flex":    {"id": 2, "drive_mode": 0, "homing_offset": 0, "range_min": 3, "range_max": 246},
  "wrist_flex":    {"id": 3, "drive_mode": 0, "homing_offset": 0, "range_min": -115, "range_max": 87},
  "wrist_yaw":     {"id": 4, "drive_mode": 0, "homing_offset": 0, "range_min": -84, "range_max": 81},
  "wrist_roll":    {"id": 5, "drive_mode": 0, "homing_offset": 0, "range_min": -146, "range_max": 148},
  "gripper":       {"id": 6, "drive_mode": 0, "homing_offset": 0, "range_min": -115, "range_max": -3}
}
```

### 5.5 `robot-record-template.json`

端口留空（安装脚本会填），摄像头为空：

```json
{
  "name": "<ARM>",
  "mode": "single",
  "arms": "both",
  "arm_type": "viscous",
  "leader_kind": "star",
  "motor_power": 38,
  "gripper_current_limit_a": null,
  "gripper_hold_torque_nm": null,
  "leader_port": "",
  "follower_port": "",
  "leader_config": "<STAR>",
  "follower_config": "<ARM>",
  "right_leader_port": "",
  "right_follower_port": "",
  "right_leader_config": "",
  "right_follower_config": "",
  "cameras": []
}
```

### 5.6 `manifest.json`

```json
{
  "schema_version": 1,
  "arm": "<ARM>",
  "driver_commit": "2a5ea4e581c0d8956b6e58a8e1c6bf8e13486daa",
  "sdk_commit": "8d3998d774650c239bc1b42f73c3f7d569e2c59d",
  "hardware_profiles": "Dry Viscous Arm v1, built identical to viscous_01. Limits are viscous_01's built-in preset, shrunk 5 deg at every end not set by a stop (J2/J3 folded ends and gripper closed end kept). Star mapping is viscous_01's preset with gripper offset 0. Rough zero (folded, gripper closed); ranges NOT measured on this unit.",
  "physical_test": "<一句英文：日期、谁做的、遥操作结果，例如 'Bring-up in Shenzhen 2026-11-20 by <names>: IDs 1-7, rough zero, watchdog 200 ms, MIT verified after power cycle; Lab teleop moved all 7 joints in the correct direction.'>",
  "files": {
    "<相对路径>": "<sha256>"
  },
  "recovery_scope": "Bring-up bundle from Shenzhen: calibration, profiles, robot-record template and bring-up evidence.",
  "adapter_enrollment": {
    "follower_usb_serial": "<4.1 步记录的 CANable 序列号>",
    "follower_calibrations": ["<ARM>"],
    "identity_kind": "usb_adapter_only"
  }
}
```

`files` 必须包含这个文件夹里**除了 manifest.json 本身以外的每一个文件**（相对 `configs/viscous/<ARM>/` 的路径，用 `/` 分隔）。安装脚本会逐个校验 sha256，`calibration/` 和 `viscous_profiles/` 下的 json 不在列表里会被拒绝。生成方法：

```bash
cd ~/viscous-bringup/makermodslab/configs/viscous/<ARM>
python3 -c "
import hashlib, json, pathlib
root = pathlib.Path('.')
m = json.loads(pathlib.Path('manifest.json').read_text())
m['files'] = {p.as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(root.rglob('*')) if p.is_file() and p.name not in ('manifest.json', '.DS_Store')}
pathlib.Path('manifest.json').write_text(json.dumps(m, indent=2, ensure_ascii=False) + '\n')
print(len(m['files']), 'files')"
```

**注意：** 如果 `files` 里列了视频，zip 会很大；视频不要放进这个文件夹，单独微信发。

---

## 6. 记录

### 6.1 `evidence/bringup-record.json`

每完成一步就更新。格式：

```json
{
  "arm": "<ARM>",
  "leader": "<STAR>",
  "date": "YYYY-MM-DD",
  "operators": ["同学的名字"],
  "mac": "macOS 版本",
  "canable": {"port": "/dev/cu.usbmodem...", "serial": "..."},
  "star": {"port": "/dev/cu.wchusbserial..."},
  "steps": {
    "ids": {"done": true, "method": "already_1_to_7 | assigned_from_127", "log": "logs/02-scan.txt"},
    "joint_identify": {"done": true, "ok": true},
    "zero": {"done": true, "follower_pre_zero_rad": [], "star_pre_zero_deg": {}, "photos": []},
    "zero_after_powercycle": {"done": true, "follower_rad": [], "star_deg": {}},
    "watchdog_200ms": {"done": true, "ids_ok": [1,2,3,4,5,6,7]},
    "mit": {"done": true, "verified_after_powercycle": true},
    "profile_check": {"done": true},
    "teleop_test": {
      "done": true,
      "joints": {
        "shoulder_pan": {"moved": true, "direction_ok": true},
        "shoulder_lift": {"moved": true, "direction_ok": true},
        "elbow_flex": {"moved": true, "direction_ok": true},
        "wrist_flex": {"moved": true, "direction_ok": true},
        "wrist_yaw": {"moved": true, "direction_ok": true},
        "wrist_roll": {"moved": true, "direction_ok": true},
        "gripper": {"moved": true, "direction_ok": true}
      },
      "stop_returned_and_released": true,
      "notes": ""
    },
    "operator_confirmed": true
  },
  "problems": []
}
```

### 6.2 `evidence/README_zh.md`

简短写清楚：
- 哪天、谁做的、哪台 Mac。
- 每一步的结果（一行一步）。
- 遥操作测试结果：哪些关节正常，哪些不正常（不正常的描述现象）。
- 遇到的任何问题和怎么处理的，没处理的也写上。
- 照片和视频文件名。

---

## 7. 常见问题

| 现象 | 可能原因 | 处理 |
|---|---|---|
| Mac 上看不到 Star 的串口 | 缺 CH340 驱动 | 让同学安装 WCH 官方 CH34x macOS 驱动，重新插拔 |
| `scan` 只找到部分电机 | 接线/电源/终端电阻；或部分已是 MIT | 先查硬件；再按 4.2 情况 C 处理 |
| `scan` 读数跳动很大 | 线松了 | 让同学检查 CAN 线和插头 |
| 写零后断电重启读数不接近 0 | 写零时臂在动，或 flash 没保存 | 回到 4.4 重来 |
| `uv sync` 卡住或失败 | GitHub/PyPI 网络 | 开代理后重试；不要改 lock 文件 |
| Lab 提示适配器未登记/不匹配 | manifest 里序列号错，或插了别的 CANable | 不要点继续；核对 4.1 的序列号，重新生成 manifest 并重跑 4.7 |
| Lab 提示 Star 起始姿态偏差太大 | Star 没折叠好 | 折叠好、夹爪闭合后重试 |
| 遥操作时某关节方向相反 | 装配和第一台不同 | 不要改配置，记录下来报告 Gavin |
| 遥操作时某关节不动 | 电机/线/限位 | 记录，停下，报告 Gavin |
| 从臂发热、异响 | 堵转 | 立刻在 Lab 点停止；必要时关电源（先扶住臂） |

如果遇到表里没有的问题：停下，保存日志，用中文向同学说明情况，让同学联系 Gavin。
