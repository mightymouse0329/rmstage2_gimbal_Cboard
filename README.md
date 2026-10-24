@@ -3,88 +3,235 @@
RM 云台（2× GM6020 + 1× M2006）的轨迹跟随、自动流程与串口调试工程。
底盘与功率限制已拆分到另一个工程，本工程只负责云台。
---
## 一、硬件
| 器件 | 用途 | CAN 反馈 | 指令帧 | 备注 |
|---|---|---|---|---|
| GM6020 | **yaw** 轴 | `0x209`（ID 4） | `0x1FE` | 直驱，最大 2.22 N·m |
| GM6020 | **pitch** 轴 | `0x207`（ID 2） | `0x1FE` | 直驱，最大 2.22 N·m |
| M2006 + C610 | pitch 平移 | `0x201`（ID 1） | `0x200` | 36:1，输出轴最大约 1.0 N·m |
| 达妙 IMU | 姿态角 + 角速度 | — | UART | 角度用姿态角，速度用陀螺 |
- **CAN1 @ 1 Mbps**，控制周期 **1 ms**（`FOLLOW_PERIOD_MS`）
- 两个 GM6020 共用 `0x1FE` 一帧下发（yaw 在前，pitch 在后）
- 串口调试：**USART1 @ 921600 8N1**
---
## 二、工程结构
```
applications/
  control_constants.hpp      所有可调参数（唯一参数源，别在别处写死）
  trajectory_follow.cpp      轨迹跟随：位置环 + 速度环 + 四类前馈
  trajectory_task.cpp        轨迹表（681 点，yaw ±22.5°，pitch ±30°）★勿改
  pitch_start.cpp            启动流程：后靠 → 抬角 → 再转 → 锁定
  infrared_pitch_auto.cpp    红外触发的自动流程
  grading.cpp                打分（评委给定算法）★勿改
  plot_task.cpp              串口绘图（多模式）
  buzzer_task.cpp            蜂鸣器（开机提示 / 完成长音）
  led_task.cpp               状态灯
  rc_task.cpp                遥控器读取
  infrared_task.cpp          红外测距（UART3）
  shared_data.cpp            任务间共享数据
Src/                         生成代码（main / freertos / usart / can / tim ...）
sp_middleware/               PID、绘图、电机驱动、IMU 驱动
```
---
## 三、控制结构
```
轨迹表 ──按时间采样 + 时间缩放──┐
                                ▼
      ┌────────────────────────────────────────┐
      │ 位置环 P/I   角度误差 → 速度指令         │
      │   ＋ 加速度限制 ＋ 斜坡（slew）          │
      ├────────────────────────────────────────┤
      │ 速度环 P/I   速度误差 → 力矩             │
      │   前馈叠加：                              │
      │     · 速度前馈      VELOCITY_FF × ω_ref  │
      │     · 加速度前馈    ACCEL_FF_J × α_ref   │
      │     · 重力前馈      GRAVITY_NM × cos(θ)  │
      │     · 摩擦前馈      平滑符号 × FRICTION   │
      ├────────────────────────────────────────┤
      │ 限幅：速度 / 加速度 / 力矩 / 积分抗饱和   │
      └────────────────────────────────────────┘
                                ▼
                  CAN 0x1FE（yaw + pitch 同帧），1 ms
```
### 反馈来源
| 量 | 来源 | 原因 |
|---|---|---|
| 角度 | **IMU 姿态角** | 绝对、无累积漂移 |
| 速度 | **陀螺仪角速度** | 比角度差分干净得多 |
### 两条必须记住的规律
1. **`ANGLE_KD` 必须等于 `VELOCITY_FF`**（都 1.0 或都 0）
   不匹配时前馈会被削弱，表现为"加了前馈还是滞后"。例：KD=1.0 / FF=0.9 → 实际只有 0.81。
2. **内环决定稳定性，外环决定跟踪精度**
   - 内环过高 → **嗡嗡**（高频自激）→ 降 `SPEED_KP`
   - 外环过高 → **超调 / 摆头 / 发散** → 降 `ANGLE_KP`
   - 判据：`τ = J/SPEED_KP ≥ 4 × CAN 周期`
---
## 四、启动流程（`pitch_start`）
`PITCH_START_ENABLED = false` 时关闭。
| Stage | 值 | 动作 |
|---|---|---|
| `WaitFeedback` | 1 | 等两个电机反馈 |
| `SeekPitch` | 2 | 6020 速度环爬行顶机械挡块 |
| `MovePitch` | 3 | M2006 驱动：阶段 A 抬 20° → 阶段 B 再转 20° |
| `Done` | 4 | 锁定保持（M2006 刚度 400，上限 1.2 N·m）|
| `Fault` | 5 | 故障（读 ch5 故障码）|
M2006 的两级力矩设计（关键）：
```
静止/极慢（|ω| < 0.06 rad/s） → BREAKAWAY 力矩 0.65 N·m   用于突破静摩擦
已经在动                      → CRUISE 力矩 0.28 N·m      防止越跑越快
全程叠加阻尼                  → −0.30 × ω                自动限速
稳态速度 = CRUISE / DAMPING = 0.93 rad/s
```
---
## 五、红外自动流程（`infrared_pitch_auto`）
`INFRARED_PITCH_AUTO_ENABLE = true` 时启用，**会抢占整个流程**。
| Stage | 值 | 动作 |
|---|---|---|
| `WaitImu` | 0 | 等 IMU 校准完成 |
| `WaitFeedback` | 1 | 等电机反馈 |
| `SeekPitch` | 2 | 顶挡块找零 |
| `MoveDefault` | 3 | 抬到默认位置（25°）|
| `WaitInfrared` | 4 | 等红外测到 ≤ 50 cm |
| `Descent` | 5 | 下降（M2006 轴角到 2.4 rad，专用力矩 0.55 N·m）|
| `DescentHold` | 6 | **最低点保持 5 秒**（`INFRARED_AUTO_DESCENT_HOLD_MS`）|
| `FoldBack` | 7 | 回折到 0.8 rad |
| `FoldHold` | 8 | 保持 0.8 rad 直到交接就绪 |
| `PitchStart` | 9 | 交给 `pitch_start` |
| `Tracking` | 10 | 轨迹跟随 |
| `Holding` | 11 | 停止后弱保持 |
| `Fault` | 12 | 故障 |
### 三个时间参数
| 参数 | 值 | 作用 |
|---|---|---|
| `INFRARED_AUTO_DESCENT_HOLD_MS` | 5000 ms | 最低点停顿 |
| `INFRARED_AUTO_FOLD_SETTLE_MS` | 200 ms | 回折后判稳 |
| `INFRARED_AUTO_PREPARE_DELAY_MS` | **20000 ms** | **从首次触发到交接的总时长门槛** |
### 流程优先级
同时只能有一个流程驱动电机：
```
infrared_pitch_auto  >  pitch_start  >  trajectory_follow
```
---
## 六、串口调试
USART1，**921600** 8N1。协议：`AA BB` + 长度字节 + 若干 little-endian float。
配套 `sp_middleware::sp::Plotter`，上位机用 SerialPlot 之类软件。
**通道数随模式变化，接之前先确认当前模式：**
| 模式 | 通道数 | 内容 |
|---|---|---|
| 红外自动流程（精简） | **7** | yaw 目标/实际(°)、pitch 目标/实际(°)、两轴分数、红外距离(cm) |
| 红外自动流程（诊断） | 18 | 上面 6 个 + stage / fault / 距离 / M2006 角度力矩 / 交接状态 |
| 双轴跟随 | 18 | 前 6 个同上（带°）＋ 跟踪诊断 |
| 找零 / 启动流程 | 8 或 12 | stage、角度、fault、力矩、M2006 角度等 |
### ⚠️ 角度单位说明
- **前 4 个角度通道已改为"度"**，方便直接读图（±22° / ±30°）
- 通道 7 以后的**诊断量里，误差仍是弧度**
- 这只是显示转换，**不影响控制与打分**（内部全程用弧度）
---
## 七、主要开关（`control_constants.hpp`）
```cpp
INFRARED_PITCH_AUTO_ENABLE = true;   // 红外自动流程
PITCH_START_ENABLED        = false;  // 启动流程
FOLLOW_BOTH_ENABLE         = true;   // 双轴轨迹跟随
FOLLOW_START_ENABLED       = ...;    // 是否自动开始跟随
INFRARED_AUTO_PLOT_COMPACT = true;   // 串口 7 通道精简模式
```
---
## 八、编译与烧录
需要 `arm-none-eabi-gcc` + CMake + Ninja（STM32CubeCLT 或独立工具链）。
```bash
cmake --preset Debug
cmake --build --preset Debug        # 产物 build/Debug/cboard.elf
```
编译正常时的占用参考（FLASH 1 MB / RAM 128 KB）：
```
FLASH: ~147 KB  (14%)
RAM:   ~73 KB   (56%)
```
**如果 FLASH 占用突然大幅变化**，说明有文件被误加进/移出编译，先查 `CMakeLists.txt`。
---
## 九、常见问题排查
| 现象 | 优先怀疑 | 怎么查 |
|---|---|---|
| **完全没声音、没灯** | 烧录没成功 / 板子没起来 | 下载器有无 `Verified`；调试器暂停看 PC |
| **电机不动** | CAN 收发 | 有 CAN 初始化吗；通道里 rx/tx 帧计数是否为 0 |
| **有反馈但没指令** | 流程没进入 / 保护 latch | 看 stage 与 fault 通道 |
| **跟踪只跟出轮廓、细节丢失** | 带宽不足（物理上限）| 见下面第十节 |
| **峰值幅值矮** | `MAX_ACCEL` / `MAX_TORQUE` 削平 | 提高限幅，不是加增益 |
| **高频嗡嗡** | 内环过高 | 降 `SPEED_KP` |
| **低频摆头 / 超调** | 外环过高 / 积分过量 | 降 `ANGLE_KP` 或 `ANGLE_KI` |
| **突然停止且不再恢复** | 某条保护 latch 成 Fault | 看 fault 码；本工程已放开跟踪误差 latch |
| **两个流程抢电机** | 优先级没关掉 | 只保留一个 `*_ENABLE = true` |
---
## 十、性能上限说明（被问到时有用）
云台的跟踪精度受**闭环带宽**限制，而带宽受两件事限制：
1. **CAN 命令周期 1 ms** → 内环带宽理论上限约 30~40 Hz，实际稳定在 15~20 Hz
2. **IMU 数据延迟与噪声** → 速度反馈有相位滞后
因此：
- **平滑轨迹**（如 pitch 的目标）→ 跟踪很好，峰值误差可到 1° 以内
- **含高频细节的轨迹**（如 yaw 峰值处的"三尖"，含 10 Hz 以上分量）→ 只能跟出包络，细节跟不出来
**这是带宽的物理上限，不是参数没调好。** 想改善只能提高控制频率、改善 IMU 滤波，或对参考做限带平滑。
---
## 十一、注意
- `trajectory_task.cpp` 是**目标曲线数据**，验收用，**不要修改**
- `grading.cpp` 是**评委给定的打分算法**，**不要修改**
- 提交前确认 `build/` 未被跟踪（已在 `.gitignore` 中，约 100 MB）
- 调参时**一次只改一个**，并立即烧录验证，否则无法定位原因
