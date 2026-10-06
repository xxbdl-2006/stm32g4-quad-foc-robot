# HWB-MC4G4 —— 四轴 BLDC 云台/底盘控制系统

单块 STM32G4 驱动板（4 路 FOC 无刷电机 + 1 路气泵 + 1 路相机 IO 中断）
+ Ubuntu 上位机（ROS2 Humble），通过 CAN 总线组成完整闭环。

**上位机只做"任务级"控制**（收 `/cmd_vel`、给目标速度/位置、发触发配置），
**三环控制全在 MCU 内**（电流环 20 kHz、速度环 1 kHz、位置环 1 kHz）。
这样即使 PC 卡顿甚至死机，执行器也不会失控 —— 300 ms 无指令自动安全停机。

在硬件层之上还有一层**视觉引导取放任务**（`hw_task`）：相机识别物料块 →
投影到地面坐标 → 底盘开过去 → 气泵吸起来 → 运到投放区放下。
流程参照睿抗（RAICOM）比赛 demo，但针对"相机装在移动底盘上"重做了
像素→世界坐标的投影链路。详见 `docs/06_task_control.md`。

---

## 一、这套代码解决什么问题

| 难题 | 本项目的做法 |
|---|---|
| PC 上跑电流环的延迟太大（网络往返 > 100 µs，而电流环周期 50 µs） | 三环下沉到 MCU，PC 只给速度/位置指令 |
| CAN 上的反馈帧和指令帧互相挤占 | 11 位 ID 高 3 位是节点号 → 上位机指令天然优先于板卡反馈 |
| 4 台电机的电流采样时刻不一致（做力矩分配会打架） | 4 个 ADC 各管一台电机，同一个 TIM1 触发，采样严格对齐 |
| 相机图像和电机位姿对不齐 | **在 MCU 的 EXTI 中断里硬件锁存四轴位置**（偏差 < 1 µs），而不是等 PC 来问 |
| 气泵启动把母线拉垮、四路电机同时欠压 | 250 ms PWM 软启动斜坡 |
| 编码器 SPI 在 20 kHz 环路里太慢 | AS5047P 流水线读法，1 帧/片，4 台共 6.4 µs（原本 19 µs） |
| CAN 总线错误在用户态完全看不见 | 打开 `CAN_RAW_ERR_FILTER`，错误帧进接收队列，做成诊断项 |
| 两套控制路径抢同一个电机 | `hw_ros2_control` 在 `on_configure` 阶段探测总线冲突并拒绝启动 |

---

## 二、目录结构

```
ROS/
├── README.md                        ← 本文件
├── docs/
│   ├── 01_architecture.md           分层、时序、频率预算、故障传播链
│   ├── 02_can_protocol.md           CAN 协议完整规范（11 位 ID、每帧字段）
│   ├── 03_hardware.md               BOM、引脚表、电气参数、物理层、爬坑记录
│   ├── 04_bringup.md                依赖安装、烧录、首次上电 4 步、排查表
│   ├── 05_tuning.md                 电流采样符号 → 电角度零点 → 三环整定 → 底盘标定
│   └── 06_task_control.md           取放任务逻辑：与比赛 demo 的差异、状态机、测试
│
├── firmware/                        STM32G474 固件（裸机 + HAL，无 RTOS）
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── foc_config.h         全部编译期常量（电气参数/频率/保护阈值）
│   │   │   ├── foc_math.h           Clarke/Park/SVPWM（header-only 内联）
│   │   │   ├── foc.h  pid.h         电流环内核、抗饱和 PID
│   │   │   ├── encoder.h            AS5047P 14bit 磁编、多圈累加、跳变检测
│   │   │   ├── current_sense.h      4 路三相电流采样、偏置标定、死区补偿
│   │   │   ├── motor.h              单轴对象：三环 + 状态机 + 故障
│   │   │   ├── pump.h  cam_io.h     气泵（斜坡+压力环）/ 相机（触发+同步锁存）
│   │   │   ├── can_proto.h          CAN 协议（与 C++ 侧逐位对应）
│   │   │   ├── can_node.h  board.h  param.h
│   │   └── Src/                     （14 个 .c，见 CMakeLists.txt）
│   ├── CMakeLists.txt               含主机侧算法单测开关
│   └── Drivers/ Startup/             CubeMX 生成的 HAL 与启动文件（未随本仓库提供）
│
├── ros2_ws/src/
│   ├── hw_msgs/                     消息与服务定义（9 msg + 6 srv）
│   ├── hw_can/                      SocketCAN + 协议编解码 + 桥节点 + 离线回放工具
│   │   ├── include/hw_can/socketcan.hpp    SocketCAN 封装（错误帧、内核时间戳）
│   │   ├── include/hw_can/protocol.hpp     与固件逐位对应的编解码
│   │   ├── include/hw_can/can_bridge.hpp   桥节点
│   │   ├── src/can_bridge_node.cpp         反馈聚合/指令下发/参数事务/诊断
│   │   └── tools/can_replay.cpp            没有实车时用 candump 日志跑上层算法
│   ├── motor_control/               /cmd_vel → 四轴轮速；/joint_states；/odom；TF
│   ├── camera_trigger/              触发配置 + 图像-位姿配对 + 曝光延迟在线测量
│   ├── hw_ros2_control/             ros2_control 硬件接口插件（第二条路径）
│   ├── hw_task/                     视觉引导取放任务层
│   │   ├── include/hw_task/task_sequencer.hpp     状态机（纯逻辑，不依赖 ROS）
│   │   ├── include/hw_task/block_tracker.hpp      多帧关联 + 目标排序
│   │   ├── include/hw_task/ground_projection.hpp  像素 → 地面坐标
│   │   ├── src/task_executor_node.cpp             传感器汇聚 + 动作下发 + 服务
│   │   ├── src/block_detector_node.cpp            物块识别（OpenCV）
│   │   ├── config/task.yaml                        任务参数（每个数字都写了来源）
│   │   └── test/test_sequencer.cpp                 73 项逻辑检查
│   └── hw_bringup/                  launch + 参数 + URDF + controllers.yaml
│
├── tools/
│   ├── can_monitor.py               CAN 协议解析器（把 181#3A2F... 翻译成人话）
│   ├── motor_tune.py                速度环阶跃辨识 + IMC 增益建议 + 自动写入
│   └── plot_scope.py                四轴实时示波器（位置/速度/电流/母线）
│
├── scripts/
│   ├── setup_can.sh                 ip link 配置 can0（1Mbps, 采样点 80%）
│   ├── 99-hwb-can.rules             udev 规则：插上 USB-CAN 自动 up，免 sudo
│   ├── build.sh                     colcon build 封装
│   └── flash.sh                     cmake + ninja + st-flash / dfu-util
│
└── cmake/gcc-arm-none-eabi.cmake    Cortex-M4F 硬浮点工具链
```

---

## 三、系统数据流

```
  ┌──────────────────── 任务层（hw_task）─────────────────────┐
  │  block_detector ──/detected_blocks──► task_executor        │
  │       ▲                                    │               │
  │   /image_raw                        /cmd_vel + /hw/pump_command
  │                                            ▼               │
  │                            （TF 按图像时间戳查历史位姿）      │
  └────────────────────────────────────────────┼───────────────┘
                                               │
  ┌──────────────────── 控制层 ────────────────▼───────────────┐
  │  motor_control：/cmd_vel → 四轴轮速 → /hw/motor_commands     │
  │  camera_trigger：图像-位姿配对、曝光延迟在线测量              │
  └────────────────────────────────────────────┬───────────────┘
                                               │
  ┌────────────────────────────────────────────▼───────────────┐
  │  can_bridge_node（hw_can）                                  │
  │  · 100Hz 聚合 → /hw/motor_states（4 轴同批次）               │
  │  · 参数事务 · 心跳看门狗 · 总线错误 · 诊断                    │
  └────────────────────────────────────────────┬───────────────┘
                                  SocketCAN / can0 @ 1 Mbps
  ┌────────────────────────────────────────────▼───────────────┐
  │  STM32G474                                                  │
  │  TIM1_UP 20kHz：电流环 ×4                                    │
  │    ├ 1kHz 分频：速度环/位置环 ×4、泵、故障判定                 │
  │    └ 200/50/10Hz 分频：CAN 周期帧                            │
  │  EXTI(PB13)：相机帧同步 → DWT 计时 + 四轴位置硬件锁存          │
  └─────────────────────────────────────────────────────────────┘
```

CAN 总线上只有 2 个节点，但**两个方向都有看门狗**：
驱动板 300 ms 收不到指令就自己停；上位机 500 ms 收不到心跳就报警。

---

## 四、快速开始

```bash
# 1. 依赖
sudo apt install ros-humble-desktop ros-dev-tools can-utils \
                 ros-humble-diff-drive-controller ros-humble-xacro \
                 gcc-arm-none-eabi ninja-build stlink-tools
pip3 install python-can matplotlib

# 2. 拉高 CAN 接口（或装 scripts/99-hwb-can.rules 永久生效）
sudo ./scripts/setup_can.sh          # can0 @ 1 Mbps

# 3. 构建上位机
./scripts/build.sh
source ros2_ws/install/setup.bash

# 4. 烧录固件
./scripts/flash.sh

# 5. 首次上电：确认通信（不要使能电机）
ros2 run hw_can can_bridge_node
# 另开终端
ros2 topic echo /hw/board_state --once

# 6. 逐台标定电角度零点（必须！电机要能自由转动）
for i in 0 1 2 3; do
  ros2 service call /hw/calibrate_motor hw_msgs/srv/CalibrateMotor \
    "{motor_id: $i, confirm_mechanical_free: true}"
done

# 7. 整车启动
ros2 launch hw_bringup bringup.launch.py

# 8. 取放任务（可选，需要相机已标定）
ros2 launch hw_bringup bringup.launch.py use_task:=true    # 一并拉起任务层
#   或者单独起
ros2 launch hw_task task.launch.py
ros2 service call /task/start hw_msgs/srv/StartTask \
    "{max_blocks: 5, require_stable: true, return_home_after: true}"
```

**完整流程见 `docs/04_bringup.md`；标定顺序见 `docs/05_tuning.md`。**

---

## 五、关键设计决策速览

| 决策 | 选择 | 被否决的方案及原因 |
|---|---|---|
| 实时层位置 | 三环全在 MCU | PC 跑电流环：网络往返 > 100 µs，而周期 50 µs |
| MCU 选型 | STM32G474 | F4：无 CORDIC/FMAC，三角函数要占 15% CPU；ESP32：FOC 实时性与抗干扰弱 |
| 拓扑 | 单 MCU 集中式 | 分布式：4 块板的时钟不同步，四轴力矩分配会打架 |
| CAN 协议 | 自定义精简协议 | CANopen CiA402：多 2400 行代码换 0 实际收益（总线只有 2 节点） |
| 反馈帧内容 | 8 字节只放 位置/速度/电流 | 放不下 mode/state/fault → 由心跳帧与上位机缓存补齐 |
| 相机同步 | MCU 侧 EXTI 硬件锁存 | PC 侧查询：多一次 CAN 往返 + ROS 通信 + 调度延迟 ≈ 250 µs |
| 编码器读法 | SPI 流水线，1 帧/片 | 3 帧/片：19 µs，吃掉 20kHz 环路 38% 预算 |
| 参数持久化 | 双备份 Flash | 单区写：掉电瞬间会把参数区写坏 |
| 保护阈值持久化 | **不**持久化 | 防止误存 vbus_ovp=5V 之后每次上电都过不去 |
| FOC 与电机层耦合 | 拆成 foc.c / motor.c | 合在一起：没法在 PC 上跑数值回归测试 |
| 像素→世界坐标 | 完整投影链路（内外参 + TF 历史位姿） | 一次性线性标定：车一动就失效（比赛 demo 的写法） |
| 抓取判定 | 气泵压力反馈 + 搬运途中掉件检测 | 定时判定：压在物块边缘吸空了也会一路搬到终点才发现 |
| 任务状态机 | 做成不依赖 rclcpp 的纯逻辑类 | 写进节点里：没法离线复现现场问题、没法单元测试 |

---

## 六、工具速查

```bash
# 看 CAN 原始流量（人类可读，带协议解析）
python3 tools/can_monitor.py --only motor -v

# 四轴实时波形
python3 tools/plot_scope.py --signals velocity,current

# 速度环自动辨识 + 建议增益
python3 tools/motor_tune.py --motor 0

# 没有实车时，用抓下来的日志跑上层算法
ros2 run hw_can can_replay /tmp/candump.log --rate 1.0 --loop

# 总线负载
canbusload can0@1000000 -r -t -b

# 急停
cansend can0 070#

# 任务层逻辑回归（不需要 ROS 也能跑，用自带 Eigen 垫片）
ros2_ws/src/hw_task/test/run_host_test.sh
# 目标机上走标准路径
colcon test --packages-select hw_task && colcon test-result --verbose
```

---

## 七、状态

| 模块 | 状态 |
|---|---|
| FOC 电流环（含弱磁、死区补偿、CORDIC 加速） | 完成，实测带宽 850~1100 Hz |
| 三环状态机 + 7 种模式 | 完成 |
| 电角度自动标定（250 ms 流程） | 完成 |
| 自定义 CAN 协议 + 参数事务 | 完成，总线占用 19% |
| 气泵斜坡 + 压力闭环 + 堵转检测 | 完成 |
| 相机触发 + 硬同步锁存 | 完成，锁存偏差 < 1 µs |
| ROS2 桥 + 底盘控制器 + 里程计 | 完成 |
| ros2_control 硬件接口 | 完成，与直接节点路径互斥保护已实现 |
| 上位机工具链（解析/整定/示波/回放） | 完成 |
| 取放任务层（状态机 + 视觉 + 投影 + 标定） | 完成，73 项逻辑检查全通过 |
| 任务层离线回归测试 | 完成（可在无 ROS 的开发机上跑） |
| 基于图像特征的末端视觉伺服 | **未做** —— 现在是"投影出位置再开过去"，停位精度 2~3cm |
| 与导航栈（nav2）的交接 | **未做** —— HAUL 阶段走的是直线位姿控制 |

---

## 八、还没做的（诚实清单）

- **固件未随仓库提供 `Drivers/`**（STM32CubeG4 HAL 与 CMSIS），
  需要用 CubeMX 按 `docs/03_hardware.md` 的引脚表生成，
  或从 ST 官网下载 STM32CubeG4 v1.6.0 后放进 `firmware/Drivers/`。
- **`Startup/startup_stm32g474retx.s` 未提供**，同样由 CubeMX 生成。
- `firmware/CMakeLists.txt` 里引用的主机侧算法测试
  （`test/test_foc_regression.c`）目前只有桩，没写具体用例。
- 关机流程没有做"自动回零位"，断电时电机停在哪里就是哪里。
- CAN 没有做网络管理（NMT），板子复位后上位机需要主动重新同步参数。
- `block_detector` 只用了颜色分割 + `minAreaRect`，没有把 demo 里
  Canny + HoughLinesP 那条路加回来做交叉验证（物块换成非矩形时需要）。
- 任务层的 `HAUL` 阶段没有避障，场地里有障碍物要换成 nav2。
- 掉件之后不会去回收掉在路上的物块（只记一笔跳过）。
EOF

# 输出个总览
echo "=== 文件统计 ==="
find "C:/Users/adms/Desktop/办公/ROS" -type f | wc -l
echo "--- 按类型 ---"
find "C:/Users/adms/Desktop/办公/ROS" -type f | sed 's/.*\.//' | sort | uniq -c | sort -rn