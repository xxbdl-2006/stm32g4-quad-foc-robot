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
- 如果车在低速时"一顿一顿"，是速度环积分器在低速量化噪声下爬行。
  把 `encoder_zero_speed_hint` 的阈值从 0.4 提到 0.8 rad/s。
- 别追求"最优"。IMC 的理论值在实物上通常偏激进，
  乘 0.7 是四台车实测下来比较稳的系数。

---

## 5. 底盘标定

### 5.1 电机转向

顶起车，单独给每个轮子 +1 rad/s，看实际转向：

```bash
for i in 0 1 2 3; do
  ros2 topic pub --once /hw/motor_commands hw_msgs/msg/MotorCommandArray \
    "{commands: [{motor_id: $i, mode: 2, setpoint: 1.0, current_limit: 1.0, enable: true}]}"
  sleep 1
done
```

若某个轮子与"前进时该轮应转的方向"相反，把
`hw_bringup/config/hardware.yaml` 里 `invert_motor_direction` 对应位置改成 `-1`。

### 5.2 轮半径

推车走 5 m（地上量好），看 `/odom` 的 x：

```bash
ros2 topic echo /odom --field pose.pose.position.x
```

校正：`wheel_radius_new = wheel_radius_old × (5.0 / 实测值)`

### 5.3 等效轮距（滑移转向特有，**最重要的一步**）

滑移转向的车，左右轮在转向时存在侧向滑移，真实转弯半径比几何计算的大。
表现为：**让车原地转 360°，里程计显示转了不止 360°。**

```bash
# 原地转 10 圈，看 odom 的 yaw 累计
ros2 topic pub --rate 20 /hw/motor_commands ...   # wz = 1.0 rad/s，持续 63 秒
```

校正：`multiplier_new = multiplier_old × (2π / 实测 yaw)`

本车实测值 **1.18**，写在 `controllers.yaml` 的
`wheel_separation_multiplier`。地面材质不同（地毯 vs 瓷砖）会差 ±10%，
值得按使用场地重标一次。

### 5.4 直线跑偏

如果 `invert_motor_direction` 都正确，但直行会偏：
- 检查四轮直径是否一致（用卷尺量，差 0.5mm 就会偏）
- 检查 `motor_control` 里四个 `current_budget_per_wheel` 是否一致

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
| 轮半径 | | | |
| 等效轮距倍数 | | | |
| 泵压力环 Kp / Ki | | | |
| 相机曝光延迟 | | | ms |

记录命令：
```bash
# 一键把全部参数读出来存档
ros2 service call /hw/save_params hw_msgs/srv/SaveParams "{motor_id: 255}"
python3 tools/can_monitor.py --only param -v > tune_log_$(date +%F).txt
```
