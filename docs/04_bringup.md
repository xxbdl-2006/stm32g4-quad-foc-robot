# 部署与启动

## 一、依赖安装（Ubuntu 22.04）

```bash
# ---- ROS2 Humble ----
sudo apt install software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools

# ---- 本项目额外依赖 ----
sudo apt install \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-velocity-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-controller-manager \
    ros-humble-teleop-twist-keyboard \
    can-utils                           # candump / cansend / cangen

# ---- 固件工具链 ----
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi \
                 cmake ninja-build stlink-tools

# ---- 工具脚本依赖 ----
pip3 install python-can matplotlib
```

## 二、固件烧录

```bash
cd /path/to/ROS
./scripts/flash.sh --build-only    # 先只构建，确认工具链 OK
./scripts/flash.sh                 # 用 ST-Link 烧录
```

烧录前的三件事：
1. **电机母线断电**，或至少让电机可以自由转动（复位瞬间引脚状态不可控）。
2. 确认没有别的进程占用 ST-Link（`ps aux | grep st-`）。
3. 确认供电正常 —— 只靠 ST-Link 的 3.3V 带不动这块板。

烧录后检查：
```bash
# 运行灯应该 1Hz 闪烁
candump can0 | head -20            # 应该能看到 001#... 的心跳和反馈帧
```

## 三、CAN 接口配置

```bash
# 一次性（需要重新插拔或重启后重做）
sudo ip link set can0 type can bitrate 1000000 sample-point 0.800 restart-ms 100
sudo ip link set can0 up

# 或者用脚本
sudo ./scripts/setup_can.sh

# 永久自动（推荐）
sudo cp scripts/99-hwb-can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

验证：
```bash
ip -details link show can0
# 应看到：
#   can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 ...
#       can bitrate 1000000 sample-point 0.800
```

## 四、首次上电流程（重要）

**新板子 / 换过电机后必须按顺序做这三步，否则一定会出问题。**

### 步骤 1：确认能通信，但不要使能电机

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build && source install/setup.bash

# 只起桥，不起控制器
ros2 run hw_can can_bridge_node --ros-args -p can_interface:=can0
```

另开终端：
```bash
ros2 topic echo /hw/board_state --once
# 应该看到 vbus ≈ 24V，node_state = 2(READY)，fault = 0
```

如果这里就没数据，**不要往下走**，先看第八节的排查表。

### 步骤 2：电角度零点标定（每台电机都要做）

标定会把电机强行转到 A 相轴线（约 250ms，会有明显的"啪"一声），
**必须确保电机轴上没有负载、可以自由转动**。

```bash
# 逐台标定，M0 为例
ros2 service call /hw/calibrate_motor hw_msgs/srv/CalibrateMotor \
  "{motor_id: 0, confirm_mechanical_free: true}"
```

期望输出：
```
success: true
elec_offset: 3.1412
message: 标定完成，零点已保存到驱动板 Flash
```

四台都做完。错误的零点会让电机在低速时抖动、在高速时失控。

### 步骤 3：单轴低速验证

```bash
# 让 M0 以 2 rad/s 转 2 秒
ros2 topic pub --rate 50 /hw/motor_commands hw_msgs/msg/MotorCommandArray "{
  commands: [{motor_id: 0, mode: 2, setpoint: 2.0, current_limit: 2.0, enable: true}]
}" --times 100
```

同时用示波器脚本看：
```bash
python3 tools/plot_scope.py --signals velocity,current
```

判断标准：
- 速度在 2 rad/s 附近，稳态误差 < 2%
- 电流在稳态时 < 0.5 A（空载）
- **听**：不应该有尖锐的高频啸叫（那是电流环在振荡）

四台逐一验证。

### 步骤 4：整车启动

```bash
# 终端 1
ros2 launch hw_bringup bringup.launch.py

# 终端 2 —— 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 五、话题与服务清单

### 话题

| 话题 | 类型 | 方向 | 频率 |
|---|---|---|---|
| `/hw/motor_states` | `hw_msgs/MotorStateArray` | 发布 | 100 Hz |
| `/hw/board_state` | `hw_msgs/BoardState` | 发布 | 100 Hz |
| `/hw/pump_state` | `hw_msgs/PumpState` | 发布 | 50 Hz |
| `/hw/camera_sync` | `hw_msgs/CameraSync` | 发布 | 触发频率 |
| `/hw/motor_commands` | `hw_msgs/MotorCommandArray` | 订阅 | 100 Hz |
| `/hw/pump_command` | `hw_msgs/PumpCommand` | 订阅 | 事件 |
| `/joint_states` | `sensor_msgs/JointState` | 发布 | 100 Hz |
| `/odom` | `nav_msgs/Odometry` | 发布 | 100 Hz |
| `/camera/sync_joint_states` | `sensor_msgs/JointState` | 发布 | 触发频率 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发布 | 1 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 上游决定 |

### 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `/hw/set_motor_mode` | `SetMotorMode` | 切换单轴模式 |
| `/hw/set_motor_gains` | `SetMotorGains` | 在线整定 PID |
| `/hw/calibrate_motor` | `CalibrateMotor` | 电角度零点标定 |
| `/hw/set_pump` | `SetPump` | 气泵控制 |
| `/hw/configure_camera` | `ConfigureCamera` | 相机触发配置 |
| `/hw/save_params` | `SaveParams` | 参数写 Flash |

## 六、常用操作速查

```bash
# 急停（最高优先级，立即停所有执行器）
ros2 topic pub --once /hw/motor_commands hw_msgs/msg/MotorCommandArray \
  "{commands: [{motor_id: 0, mode: 0, enable: false}]}"
# 说明：真正的急停走 CLS_ESTOP，需要直接用 cansend：
cansend can0 070#

# 开气泵，40% 占空比
ros2 service call /hw/set_pump hw_msgs/srv/SetPump \
  "{cmd: 1, mode: 0, value: 0.4}"

# 开气泵，压力闭环到 30 kPa
ros2 service call /hw/set_pump hw_msgs/srv/SetPump \
  "{cmd: 1, mode: 1, value: 30.0}"

# 相机 30Hz 触发，脉宽 100us
ros2 service call /hw/configure_camera hw_msgs/srv/ConfigureCamera \
  "{enable: true, rate_hz: 30.0, pulse_width_us: 100.0, mode: 0}"

# 看 CAN 原始流量（人类可读）
python3 tools/can_monitor.py --only motor

# 总线负载统计
canbusload can0@1000000 -r -t -b

# 看四轴波形
python3 tools/plot_scope.py

# 速度环自动整定
python3 tools/motor_tune.py --motor 0
```

## 七、ros2_control 路径

当需要复用标准控制器（如 `joint_trajectory_controller` 做多轴同步轨迹）时，
走这条路。**注意：两条路不能同时开指令下发。**

```bash
ros2 launch hw_bringup bringup.launch.py use_ros2_control:=true use_teleop:=false
```

这个命令会自动：
1. 把 `can_bridge` 的 `motion_output_enabled` 设为 `false`（只做反馈）；
2. 起 `controller_manager` + `joint_state_broadcaster` + `diff_drive_controller`；
3. 加载 `hw_ros2_control/HwbSystem` 作为硬件接口。

`hw_ros2_control` 在 `on_configure` 时会检查总线上是否已有其他节点在发
运动指令，发现冲突会**拒绝启动**并打印明确的原因 —— 这比让你花两小时
排查"为什么车不动"要好。

## 八、故障排查表

| 现象 | 最可能的原因 | 怎么确认 |
|---|---|---|
| `candump` 完全没输出 | 波特率不一致 / CAN_H、L 接反 / 无终端电阻 | `ip -details link show can0` 看 bitrate；万用表量 H-L 间电阻应为 60Ω |
| 偶发丢帧、总线错误计数增长 | 只有一端有 120Ω / 未双绞 / 走线过长 | `ip -statistics link show can0` 看 errors；量电阻 |
| `can_bridge` 报 "interface not found" | 接口名不对或驱动没加载 | `ip link`；gs_usb: `lsmod \| grep gs_usb` |
| 有反馈但服务调用全部超时 | 驱动板在主循环里卡住（Flash 写失败循环） | 运行灯是否还在 1Hz 闪 |
| 电机不转但反馈正常 | 未使能 / 处于 FAULT / 电角度零点错 | `ros2 topic echo /hw/board_state --once` 看 fault 位 |
| 电机低速抖动 + 啸叫 | 电流环 Kp 过大，或死区补偿过量 | 用 `motor_tune.py` 重新辨识；把 `current_sense_deadtime_comp` 的 base 从 8 降到 5 试 |
| 电机高速失控 | 电角度零点标定错（尤其是编码器方向反了） | 重新标定；检查 `encoder.c` 的 `delta` 符号 |
| 一个轮子转向反了 | 电机接线相序反 / 编码器方向反 | 改 `hardware.yaml` 的 `invert_motor_direction` 对应位 |
| 车走直线会偏 | 滑移转向的等效轮距 ≠ 几何轮距 | 按 `05_tuning.md` 第 5 节标定 `wheel_separation_multiplier`（当前 1.18） |
| 里程计漂移大 | 轮子打滑 / 轮半径标定不准 | 推车走 5m，量实际距离，按比例修正 `wheel_radius` |
| 相机图像与位姿对不上 | `exposure_delay` 估计不准 / 配对窗口太小 | 看 `/diagnostics` 的 `match_rate` 与 `exposure_delay_ms` |
| 上位机一重启电机就停 | 这是**正确行为**（300ms 看门狗） | 无需处理 |
| 上电后板子无反应、灯不闪 | 卡在 HardFault（多半是时钟配置） | 接 SWD，看 `board_clock_init` 的断言是否命中 |

### 用 candump 快速定位

```bash
# 只看主板发的帧（高 3 位 ID = 001）
candump can0,100:700

# 只看高速反馈（CLASS=1）
candump can0,100:7F0

# 看全部并带时间戳
candump -t d can0

# 统计各 ID 的帧数（判断丢帧）
timeout 5 candump can0 | awk '{print $3}' | sort | uniq -c | sort -rn
```

正常运行时，5 秒内应该看到大约：
- `181`~`184`（M0~M3 反馈）：各 ~1000 帧
- `185`（板级状态）：~1000 帧
- `100`（心跳）：~50 帧
- `141`（气泵状态）：~250 帧
