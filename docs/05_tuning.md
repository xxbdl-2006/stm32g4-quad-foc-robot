# 标定与整定流程

按顺序做。每一步都依赖前一步的结果，跳步会浪费更多时间。

---

## 0. 前置检查

```bash
# 母线电压正常
ros2 topic echo /hw/board_state --once | grep vbus      # 应为 23~25 V

# 无故障位
ros2 topic echo /hw/board_state --once | grep fault     # 应为 0

# 反馈频率正常
ros2 topic hz /hw/motor_states                          # 应为 ~100 Hz
```

---

## 1. 电流采样链路的符号与偏置

**为什么必须做**：分流电阻的正负接法、INA240 的 IN+/IN- 接法在不同批次
板子上可能不同。符号错了电流环会正反馈 → 上电即飞车。**这一步绝不能省。**

### 1.1 偏置（固件自动做，但要知道它在做什么）

`motor_init_all()` 里有：
```c
current_sense_calib_one(i, &oa, &ob, &oc, 512U);
```
它要求三相桥**全关断**（`board_all_motors_off()` 已在上电流程第 3 步调用）。
512 次平均把噪声压到 0.3 LSB（≈2.4 mA）。

人工验证偏置是否正确：
```bash
ros2 topic echo /hw/motor_states --field motors[0].current
# 电机静止、未使能时应为 0 ± 0.05 A
```
如果看到 ±0.3 A 以上的随机跳动，说明 INA240 的 REF 引脚有问题（见
`03_hardware.md` 第七节第 1 条）。

### 1.2 符号

**方法**：给某台电机施加一个**纯 d 轴**的小电流，用手感受（或用电流表量）
A 相实际方向。

```bash
# 给 M0 施加 id=1A, iq=0 的电流矢量（电机不会转，只是把 A 相拉向轴线）
# 用开环模式 + 固定电角度更直接：
ros2 topic pub --once /hw/motor_commands hw_msgs/msg/MotorCommandArray \
  "{commands: [{motor_id: 0, mode: 1, setpoint: 1.0, current_limit: 1.0, enable: true}]}"
```

如果发现某相电流读数的符号与实际相反，改 `current_sense.c` 的
`phase_sign[][]` 表：

```c
static const float phase_sign[MOTOR_COUNT][3] = {
    { -1.0f, -1.0f, -1.0f },    /* M0 */
    ...
};
```

**判据**：符号正确时，给定正 iq 电流，电机正转；给定正 id 电流，
电机朝编码器读数增大的方向转。

---

## 2. 电角度零点标定

固件已实现自动化流程（`motor.c` 的 `motor_calib_step`），上位机只需：

```bash
ros2 service call /hw/calibrate_motor hw_msgs/srv/CalibrateMotor \
  "{motor_id: 0, confirm_mechanical_free: true}"
```

### 流程内部的四个阶段（250 ms）

| 时间 | 动作 | 目的 |
|---|---|---|
| 0~100 ms | 施加 1.5 V 的 +d 轴电压 | 把转子拉到 A 相轴线 |
| 100~160 ms | 保持 | 等机械振荡衰减 |
| 160~195 ms | 撤压 | 转子停在最近稳定点 |
| 195 ms | 记录编码器角为电角度零点 | |
| 195~250 ms | 写 Flash、回 READY | |

### 验证标定是否正确

```bash
# 用小电流闭环，让电机慢速转 —— 应该平顺、无抖动
python3 tools/motor_tune.py --motor 0 --amp 1.0 --no-write
```

**如果零点错了 180°（编码器方向反了）**，症状是：电机想转但一动就
"锁死"或猛烈抖动，电流飙升。这时改 `encoder.c` 里：

```c
int32_t delta = (int32_t)raw - (int32_t)e->raw_prev;   // 加负号
```

然后再重新标定。

---

## 3. 电流环 PI（理论计算 + 微调）

固件在上电时按被控对象自动算初值：

```c
foc_auto_tune_pi(&m->foc, PHASE_R_OHM, PHASE_L_HENRY, 1000.0f);
// Kp = 2π·fc·L = 2π×1000×190e-6 = 1.19
// Ki = Kp·R/L  = 1.19 × 0.42/190e-6 = 2631
```

### 验证方法

把电流环的阶跃响应抓出来：

```bash
# 临时把速度环 Kp 设成 0，让电流指令直接来自速度环输出（相当于开环）
ros2 service call /hw/set_motor_gains hw_msgs/srv/SetMotorGains \
  "{motor_id: 0, loop: 1, kp: 0.0, ki: 0.0, kd: 0.0}"
```

然后施加一个速度阶跃，用 `plot_scope.py --signals current` 观察电流波形。

| 现象 | 调整 |
|---|---|
| 上升慢（>1 ms 到 90%） | Kp 太小，乘 1.5 |
| 有超调且振铃 | Kp 太大，乘 0.7 |
| 稳态有静差 | Ki 太小，乘 1.5~2 |
| 低频振荡（几十 Hz） | Ki 太大，乘 0.5 |

**正常波形**：上升时间 ~300 µs，超调 < 10%，无振铃。
理论带宽 1 kHz，实测 -3dB 点在 850~1100 Hz 之间都算正常。

---

## 4. 速度环 PI（用工具自动辨识）

```bash
python3 tools/motor_tune.py --motor 0
```

工具会做 4 次阶跃（+2、-2、+1、-1 rad/s），对每次做一阶惯性+纯延迟
拟合，取中位数，再用 IMC 算建议值。

### 读结果

```
  直流增益 K      : 0.9821  (rad/s) / (rad/s 指令)
  时间常数 τ      : 42.3 ms
  纯延迟 L        : 3.1 ms
  IMC 理论值      : Kp=0.1132  Ki=2.6741
  建议写入值(×0.7): Kp=0.0792  Ki=1.8719
```

| 指标 | 正常范围 | 异常含义 |
|---|---|---|
| K | 0.95 ~ 1.02 | < 0.9 说明有静差（电流限幅或摩擦大） |
| τ | 30 ~ 80 ms | 与负载惯量成正比；空载应偏小 |
| L | < 8 ms | > 20 ms 说明上位机侧有排队或 CAN 丢帧 |

### 微调时的经验

- **装上实际负载后必须重做** —— 空载辨识出的 τ 只有 40 ms，
  装上负载可能是 150 ms，Kp 要相应降低。
- 如果关节在低速时"一顿一顿"，是速度环积分器在低速量化噪声下爬行。
  把 `encoder_zero_speed_hint` 的阈值从 0.4 提到 0.8 rad/s。
- 别追求"最优"。IMC 的理论值在实物上通常偏激进，
  乘 0.7 是四个关节实测下来比较稳的系数。

---

## 5. 机械臂标定

前四节整定的是"单个关节的伺服好坏"，这一节校的是"整条手臂的几何对不对"。
几何错一点，逆解就会把误差原样搬到末端 —— 伺服整得再好也没用。

### 5.1 关节零位与方向

每个关节单独小速度点动，确认转向与零位。**一次只动一个** —— 四个关节一起动
的时候出了错分不清是哪个，末端还可能甩到料框上。

```bash
# 走 ros2_control 路径，用关节组速度控制器（点动接口没有加速度限制，
# 务必用小速度；ros2_control.launch.py 已激活）
ros2 topic pub --once /arm_joint_group_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2, 0, 0, 0]}"
ros2 topic pub --once /arm_joint_group_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0]}"

# 看四个关节的实测位置
ros2 topic echo /joint_states --once
```

判据与处置：

| 现象 | 处置 |
|---|---|
| 关节转向与预期相反 | 重新做电角度零点标定（第 2 节），或检查编码器计数方向。**不要**在指令侧加负号绕过 —— 正逆解、URDF、关节限位全都建立在"角度增大 = 逆时针"的约定上，指令侧翻一次号等于全局都错了 |
| 零位漂移 | 机械零位是装配时打表定的，漂了说明联轴器松了，先紧机械再谈标定 |

**J4 的专项核对：丝杠导程**。让 J4 走一段已知位移，量实际值：

```bash
# 指令 J4 下降 50mm（关节组位置目标，单位米）
ros2 topic pub --once /arm_joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{joint_names: [joint_4], points: [{positions: [-0.050], time_from_start: {sec: 2}}]}"
```

用直尺量末端实际位移。50mm 指令应该得到 50mm ± 0.5mm；如果得到
48mm，说明导程实际是 0.0096 而不是 0.010 —— 改 `arm_model.hpp` 的
`lead`（那是唯一真源），再同步 `hardware.yaml`。导程差 2% 看着不大，
累积到 135mm 行程就是 2.7mm，直接吃掉 `touch_z` 的过盈量。

### 5.2 工作空间与正逆解校核

沿几条半径走末端，记录指令位置与实测位置的偏差：

```bash
# 让末端沿 +x 轴走几个点（先使能！）
for x in 0.15 0.20 0.25 0.30 0.35; do
  ros2 topic pub --once /arm/target_pose hw_msgs/msg/ArmTarget \
    "{target_pose: {position: {x: $x, y: 0.0, z: 0.10}}, speed_scale: 0.5}"
  sleep 2
  ros2 topic echo /arm/tool_pose --once | grep -A3 position
done
```

判据：
- `x` 方向偏差 < 1mm（同一方向上 D-H 表错的体现是**恒定偏差**）；
- `/arm/status` 的 `ik_fail_count` 全程为 0；
- 换一个方向（比如 y = 0.20）再走一遍。**不同方向偏差不同** 说明某个连杆
  长度错了（a1 或 a2 差 1mm，末端误差会随姿态在 0~2mm 之间变化），
  回去改 D-H 表，不要在视觉标定里"补偿"它 —— 那只在一个姿态下有效。

### 5.3 手眼标定（**最重要的一步**）

相机外参来自 URDF（`camera_joint` 的位置与 `camera_optical_joint` 的
rpy=(π,0,0)），所以先核对 URDF 与实物安装：

- 相机光轴是否正对工作台中心（装歪了先掰正，比改 URDF 里硬补一个角度好）；
- `camera_optical_joint` 的 rpy **必须是 (π,0,0)**。俯视相机若把 y 轴写成
  与基座同向，外参就成了镜像（行列式 −1），投影会静默地左右反 ——
  症状是抓取点总在物块的镜像位置。

残差校正用 `/task/calibrate_mapping` 采点：把一个物块摆在已知台面坐标
（用直尺从基座中心量），点它的像素坐标：

```bash
ros2 service call /task/calibrate_mapping hw_msgs/srv/CalibrateMapping \
  "{block_color: 1, pixel_x: 318.0, pixel_y: 356.0, world_x: 0.05, world_y: 0.35}"
```

**标定块必须铺开**：单点只解平移；要解出完整的 2D 仿射至少要 3 个不共线的
点，且跨度要超过 8cm（`TableProjection::kMinCalibSpread`）。低于这个跨度
时代码会**拒绝**解完整仿射、只校正平移，并在返回 message 里说明 ——
这是刻意的安全阀：点挤在一起时解出的仿射矩阵在标定点附近很准、离开那个
小区域完全失效，但它看起来"标定成功了"。

建议采点位置：工作台四个角 + 中心，共 5 点。

### 5.4 取放精度验证

把物块摆在可达环带内 4 个不同位置（近/远/左/右），每个位置完整取放 3 次：

```bash
ros2 service call /task/start hw_msgs/srv/StartTask \
  "{max_blocks: 1, require_stable: true}"
```

判据：
- 12 次里成功 ≥ 11 次（失败的那次要看 `/task_status` 的 message 是哪个环节）；
- 放置位置偏差 < 5mm（用直尺量物块中心到槽位中心的距离）；
- 抓取点在物块中心 ± 4mm 内（看吸盘压痕）。

### 5.5 码垛层高

叠 4 层，量顶层物块相对底层的横向偏移：

| 现象 | 处置 |
|---|---|
| 每层等量平移 | `place.layer_height` 与实际物块高度不符（必须填**实测值**，不是标称值） |
| 越往上越歪 | 丝杠背隙（反向间隙）。升降轴全程只往下压这一个方向用，正常不该出现；出现了检查丝杠预紧 |
| 第 3 层起吸不住 | `touch_z` 的过盈量被层高累积误差吃掉了，把过盈从 2mm 加到 3mm |

---

## 6. 气泵

### 6.1 斜坡时间

```bash
# 观察泵启动时的母线跌落
python3 tools/plot_scope.py --signals vbus
```

- 跌落到 20 V 以下 → `PUMP_RAMP_MS` 加长（从 250 提到 400）
- 启动太慢影响节拍 → 可以缩到 150，但必须确认母线不掉

### 6.2 压力环

泵的带宽很低（管路容腔大），`kp=0.9 / ki=0.35` 是比较稳的起点。
**不要试图调快** —— 超过 5 Hz 一定会振。

调法：
```bash
ros2 service call /hw/set_pump hw_msgs/srv/SetPump \
  "{cmd: 1, mode: 1, value: 30.0}"
python3 tools/plot_scope.py   # 看 PumpState.pressure 的收敛曲线
```

| 现象 | 调整 |
|---|---|
| 到不了目标 | `duty_target` 上限不够，提高 SetPump 的 value 上限或检查管路漏气 |
| 在目标附近来回摆 | ki 太大，减半 |
| 一直偏低 | ki 太小，加 50% |

---

## 7. 相机同步

```bash
ros2 service call /hw/configure_camera hw_msgs/srv/ConfigureCamera \
  "{enable: true, rate_hz: 30.0, pulse_width_us: 100.0, mode: 0}"

ros2 topic hz /hw/camera_sync        # 应为 30 Hz
ros2 topic echo /diagnostics --field status[0].values
```

看 `camera_trigger/sync` 诊断项：

| 字段 | 正常 | 异常处理 |
|---|---|---|
| `match_rate` | > 0.95 | < 0.8 说明配对窗口太小，把 `match_window_s` 从 0.040 提到 0.060 |
| `dropped_frames_total` | 0 | 持续增长说明相机跟不上触发频率，降 `rate_hz` |
| `period_jitter_ms` | < 0.5 | 大说明触发输出被中断打扰（检查 TIM8 中断优先级） |
| `exposure_delay_ms` | 1.0~3.0 | 与相机型号有关，首次运行会收敛到这个值 |

### 曝光延迟的物理含义

测量值 = SYNC 沿 → 曝光 → 读出 → USB 传输 → ROS 发布 的总时间。
它**不是**曝光时长。做视觉伺服时应该用：

```
图像对应的真实位姿时刻 = image.header.stamp - exposure_delay
```

`/camera/sync_joint_states` 的 `header.stamp` 已经做了这个校正。

---

## 8. 整定结果记录表

填完这张表，换板子 / 换负载时就有基准可比。

| 项目 | 值 | 日期 | 备注 |
|---|---|---|---|
| M0 电角度零点 | | | |
| M1 电角度零点 | | | |
| M2 电角度零点 | | | |
| M3 电角度零点 | | | |
| 电流环 Kp / Ki | | | 四台应一致（同型号电机） |
| 速度环 Kp / Ki（空载） | | | |
| 速度环 Kp / Ki（满载） | | | |
| 速度环 τ（满载） | | | |
| joint_1..4 关节零位 | | | 装配打表值，联轴器重装过就要重记 |
| 丝杠导程实测 | | | m/rev，标称 0.010 |
| 手眼标定残差 | | | m，/task/calibrate_mapping 的返回值 |
| 码垛层高实测 | | | m，实测物块高度 |
| 泵压力环 Kp / Ki | | | |
| 相机曝光延迟 | | | ms |

记录命令：
```bash
# 一键把全部参数读出来存档
ros2 service call /hw/save_params hw_msgs/srv/SaveParams "{motor_id: 255}"
python3 tools/can_monitor.py --only param -v > tune_log_$(date +%F).txt
```
