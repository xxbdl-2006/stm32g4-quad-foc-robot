# 任务控制逻辑（视觉引导取放）

参照 `C:\Users\adms\Desktop\睿抗比赛demo` 的取放流程实现，但**不是照抄** —— 因为
那个 demo 的前提（固定安装的相机 + SCARA 机械臂）与这里（车载相机 + 移动底盘）
差得很远。本文档说明搬了什么、改了什么、为什么改。

---

## 一、demo 做了什么

三份代码构成一个完整的取放闭环：

| 文件 | 职责 |
|---|---|
| `RK_SCARA_camera.ipynb` | 视觉进程。OpenNI 取流 → 裁 ROI → HSV 分割红/蓝 → 形态学 → `findContours` → `minAreaRect` 得到中心与角度；做 socket **服务端**（localhost:6000） |
| `calculate_angles.py` | 同一套识别的单机调试版，额外用 Canny + `HoughLinesP` 求物块边的角度，做 5 帧滑动平均 |
| `RK_SCARA_DEMO.ipynb` / `RK_competiton_procedure.ipynb` | 机器人进程。socket **客户端**，`send('begin')` → 收到 `"x,y"` → 像素映射到机械臂坐标 → 下降吸附 → 抬起 → 移到堆叠位 → 放气 → 回家。循环 5 次 |

关键的三段逻辑：

```python
# ① 像素 -> 机械臂基座坐标（一次性线性标定）
robot_x = 0.0008*object_y + 0.2563
robot_y = -0.0010*object_x + 0.1450

# ② 越界保护
if robot_x < 1000 and robot_y < 1000: ...

# ③ 抓取 -> 码垛（同一 XY 上按 55mm 一层往上叠）
arm.cartesian_space_interpolated_motion(robot_x, robot_y, 0.30, 0)   # 抬高
arm.cartesian_space_interpolated_motion(robot_x, robot_y, 0.21, 0)   # 下降
arm.cartesian_space_interpolated_motion(robot_x, robot_y, 0.30, 0)   # 抬起（吸住了）
arm.cartesian_space_interpolated_motion(0.052, 0.24, 0.21+0.055*i+0.05, 0)  # 移到码垛位上方
arm.cartesian_space_interpolated_motion(0.052, 0.24, 0.21+0.055*i, 0)       # 下降
gripper.off()                                                        # 放气
```

稳定判据在视觉侧：

```python
ThetaList[i] = Theta1
count = ThetaList[ThetaList == Theta1].size
if count == 2:      # 同一个角度出现两次就认为稳定
    ...开始抓
```

---

## 二、搬到本项目后必须改的六处

### 1. 像素→世界坐标：线性标定 → 完整投影链路

**demo 的做法在车载相机上直接失效。** 那两行线性系数隐含了一个前提：
相机不动、工作平面固定。车一动，同一个像素对应完全不同的世界坐标，
而线性映射会给出一个"看起来合理"的错值 —— 最危险的那种错。

本项目走完整链路（`ground_projection.cpp`）：

```
像素 (u,v)
  → 去畸变（Brown 径向）
  → 归一化射线 d_cam = K⁻¹[u,v,1]
  → 由 TF 提供的 T_oc 旋转到 odom 系
  → 与地面 z=0 求交
  → 叠加一个 2D 仿射残差（吃掉装配公差）
```

`T_oc` 用**图像时间戳**去 TF 里查历史值。这一点很关键：驱动板在收到相机
SYNC 上升沿的那一刻锁存了四轮位姿（见 `docs/01_architecture.md` 3.3 节），
用同一时刻的位姿才不会把图像和位姿错开一个里程计周期。

残差标定只需 3 个点，但有个陷阱：**标定点必须铺开**。三个点挤在一起时
设计矩阵近乎共线，正规方程会把条件数平方，解出的矩阵线性部分乱飞、
靠平移去凑 —— 在标定点附近误差很小，出了那个小区就完全不可用，而它
看起来"标定成功了"。所以 `GroundProjection` 里有一道 `kMinCalibSpread = 8cm`
的安全阀，跨度不够就退化成只校正平移，并在服务返回的 message 里明确
告诉操作员。这条是写测试时才发现的（见第五节）。

### 2. 稳定判据：按图像角度匹配 → 按地面坐标做数据关联

demo 的 `ThetaList` 判据有两个问题：

- 它按**角度值**匹配。方块的对称性让它只有两个可能的角度值，
  两个不同位置的方块很容易被判成同一个。
- 相机一动，同一个静止物块的像素坐标连续两帧完全不同，
  任何基于像素量的匹配都会失败。

改成 `block_tracker.cpp`：每帧的观测先投到 odom 系，用**最近邻 + 门限**
（默认 60mm）做数据关联，再累计连续命中帧数。门限的取值有依据：
车最快 0.85 m/s、控制周期 20ms 时位移 17mm，加上投影噪声与里程计漂移，
60mm 有足够余量，又小于物块尺寸（80mm）不会串到隔壁物块。

### 3. "抓取"的判定：定时 → 压力反馈

demo 里 `gripper.on()` 之后直接按固定时长往下走 —— 压在物块边缘吸空了
也会一路搬到码垛位，然后在放气的瞬间发现手里什么都没。

本项目的驱动板有压力传感器，所以 `GRASP` 状态是**等压力达标**
（`|压力| ≥ vacuum_threshold_kpa`），并且 `LIFT` 阶段再确认一次。
搬运途中压力掉到阈值一半以下就判为掉件，放弃当前物块重新扫描。

另外把单次吸空的重试路径设计成"回 ALIGN 重新对位再试"而不是原地再抽一次 ——
吸不住通常是吸盘没压正，重新对位的成功率明显更高。

### 4. 执行器的对应关系

| demo | 本项目 |
|---|---|
| `vgripper.on()` / `.off()`（`PwmDriver` 控制气泵） | `/hw/set_pump`，`PumpCommand.cmd = RUN/STOP` |
| `arm.cartesian_space_interpolated_motion(x,y,z)` | `/cmd_vel` 速度控制 + 位姿控制器 |
| `arm.joint_space_interpolated_motion(...)` | 同上 |
| `arm.home(n)` | `StartTask.return_home_after` / `AbortTask.return_home` |
| `arm.init()` / `arm.enable()` / `arm.disable()` | `/hw/motor_commands` 的 enable 位 |
| `arm.get_joints_poses()` | `/hw/motor_states` |

气泵既可以给固定占空比（`MODE_OPEN_DUTY`），也可以让驱动板用压力环
维持真空（`MODE_PRESSURE` + `vacuum_target_kpa`）。后者更省电：漏气时
驱动板自动加大功率，上位机只需要看压力是否达标。

### 5. 码垛 → 排放阵列

demo 在同一个 XY 上按 55mm 一层往上叠。本项目是**地面移动平台，没有 Z 轴**，
无法叠高，因此改成沿投放区铺开的一维/二维阵列：

```yaml
place:
  x: 0.00
  y: 1.20
  yaw: 1.5707963
  pitch: 0.12        # 物块 80mm + 30mm 取放余量
  columns: 4         # 每行 4 个，超出自动换行
  row_pitch: 0.12
```

这不是"简化"，是因为执行器物理能力不同。要叠高必须给车加一个升降机构。

### 6. 定点停靠：三环插补 → 两段式位姿控制器

demo 用 `cartesian_space_interpolated_motion(..., duration=5)` —— 机械臂
有精确的运动学模型，给定时间插补就能到位。

滑移转向底盘没有横向自由度，而且轮子会打滑，不能这么干。
`drive_to_pose()` 用两段式：

```
第一段（位置没到）:
    朝向 = atan2(目标 - 当前位置)          ← 注意是"指向目标"，不是期望朝向
    |朝向误差| > 0.6 rad → 原地转
    否则 v = kp·d·cos(e_h)，ω = kω·e_h
第二段（位置到了）:
    原地转到期望朝向
```

**为什么不能按期望朝向分解误差**：差速底盘唯一的横向纠偏手段就是转向。
如果按期望朝向分解（`ex` 沿期望朝向、`ey` 垂直期望朝向），当车已经走到
目标的同一纵坐标、只剩横向偏差时 `ex ≈ 0 → v ≈ 0`，而横向修正项是
`ky·ey·v_norm`，`v_norm = 0` 让它整个消失 —— 车就停在离目标 20cm 的地方
再也不动。这不是理论担忧，是第一次跑测试时实测到的（见第五节）。

---

## 三、状态机

```
IDLE ─start─► SCAN ─找到目标─► APPROACH ─进入精定位区─► ALIGN ─对准─► SETTLE
               ▲                                                   │
               │                                                DESCEND
               │                                                   │
               │                                                 GRASP
               │                                       ┌───────────┴───────────┐
               │                                    压力达标              超时/重试耗尽
               │                                       │                       │
               │                                     LIFT                   跳过该物块
               │                                       │                       │
               │                                     HAUL ◄────────────────────┘
               │                                       │
               │                                     PLACE ─对准完成─► RELEASE
               │                                       ▲                   │
               └──────── RETREAT ◄─────────────────────┘                RETREAT
                     (还有配额)                                            │
                                                                      (配额完成) DONE
```

每个状态的超时、重试上限、以及超时后的去向都在 `config/task.yaml` 里，
每条注释都写了数字的来源。

### 三个容易写错的地方

**① 重试计数必须按阶段独立。** 这版踩过坑：GRASP 失败后的重试路径是
`GRASP → ALIGN → SETTLE → DESCEND → GRASP`，而 ALIGN 成功时会清零重试
计数 —— 于是 `max_grasp_retry` 永远达不到，吸不住的物块会让状态机在
这几个状态之间无限循环（实测跑满 8000 步 160 秒都没出来）。现在四个阶段
各自有计数器（`scan_fail_` / `approach_fail_` / `align_fail_` / `grasp_fail_`）。

**② `aborted` 标志必须挂在返回的那个对象上。** 曾经的写法是
```cpp
a.aborted = true;
return stop_all("...");   // stop_all 新建了一个对象，aborted 丢了
```
结果是状态进了 `FAILED` 但上层收不到 `aborted`，以为任务还在跑。
同样的坑在掉件路径里也有一份。

**③ 中止+回起点不能退化成"回起点后继续干活"。** `abort(return_home=true)`
借用 `RETREAT` 状态做回程，但 `RETREAT` 原本的职责是"退开后找下一个物块"。
如果不加 `aborting_` 标志，操作员按下急停之后车会回到起点然后**接着扫描物块**。

### 失败处理策略

| 情况 | 处理 |
|---|---|
| 扫描超时 | 重试 `max_scan_retry` 轮；配额没完成且排除列表非空时，清空排除列表**补扫一轮**（只补一轮，避免死循环） |
| 接近/精定位超时 | 跳过该物块，去找别的 |
| 吸不住 | 回 ALIGN 重新对位重试 `max_grasp_retry` 次；仍不行则跳过 |
| 搬运途中掉件 | 放弃当前物块并计入跳过，回 SCAN |
| 连续跳过 5 个 | 判定 FAILED（气路或吸盘有问题，再试也没用） |
| 底盘故障 / 急停 | 立刻 FAILED，释放负载，停所有执行器 |

---

## 四、接口一览

### 话题

| 话题 | 类型 | 方向 |
|---|---|---|
| `/detected_blocks` | `hw_msgs/DetectedBlockArray` | block_detector → task_executor |
| `/grasp_targets` | `geometry_msgs/PoseArray` | 发布（RViz 可视化，能看到每个物块的位置与角度） |
| `/task_status` | `hw_msgs/TaskStatus` | 发布，20 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 |
| `/hw/pump_command` | `hw_msgs/PumpCommand` | 发布 |

### 服务

| 服务 | 说明 |
|---|---|
| `/task/start` | 启动任务（可指定最多几个、颜色顺序、是否要求稳定） |
| `/task/abort` | 中止（可指定就地释放 / 保持吸附、是否回起点） |
| `/task/pause` | 暂停（可恢复，恢复时会扣掉暂停时长） |
| `/task/resume` | 恢复 |
| `/task/calibrate_mapping` | 采集一个标定点，校正投影残差 |

### 典型操作序列

```bash
# 1. 硬件层
ros2 launch hw_bringup bringup.launch.py use_teleop:=false use_rviz:=true

# 2. 任务层（或直接用 bringup 的 use_task:=true 一并拉起）
ros2 launch hw_task task.launch.py

# 3. 确认视觉在跑
ros2 topic hz /detected_blocks          # 应等于相机帧率

# 4. 确认投影对不对（在 RViz 里看 /grasp_targets 的箭头是否落在物块上）
ros2 topic echo /grasp_targets --once
#   整体固定偏移   -> 用 calibrate_mapping 校正
#   误差随距离变化 -> 相机内参错了，重新标定

# 5. 标定投影残差（把物块放在一个量得准的位置，记下它的像素坐标）
ros2 service call /task/calibrate_mapping hw_msgs/srv/CalibrateMapping \
  "{block_color: 1, pixel_x: 318.0, pixel_y: 356.0, world_x: 0.0, world_y: 0.45}"

# 6. 开跑
ros2 service call /task/start hw_msgs/srv/StartTask \
  "{max_blocks: 5, require_stable: true, return_home_after: true}"

# 7. 看状态
ros2 topic echo /task_status

# 8. 出问题就中止
ros2 service call /task/abort hw_msgs/srv/AbortTask \
  "{release_payload: true, return_home: true}"
```

---

## 五、测试

`test/test_sequencer.cpp` —— 73 项检查，全部通过。假的是传感器与底盘，
状态机跑的是与真车完全相同的那份代码。

```bash
# 目标机上（用真正的 Eigen3）
colcon build --packages-select hw_task
colcon test --packages-select hw_task && colcon test-result --verbose

# 没有 ROS 的开发机上（用自带的极简 Eigen 垫片）
ros2_ws/src/hw_task/test/run_host_test.sh
```

### 覆盖的场景

| 用例 | 内容 |
|---|---|
| A | 地面投影对照**解析解**（相机正对下方时有闭式解可比）；投影残差标定；标定点太集中时的降级 |
| B | 多帧跟踪：连续帧数阈值、抖动不新建轨迹、远离新建轨迹、丢失后删除、颜色优先+距离排序、可抓范围筛选 |
| C | 完整取放流程（正前方物块），检查终态、放置数、结束位姿 |
| D | 侧前方物块（29° 方位角，考验转向收敛） |
| E | 吸不住 → 重试 → 跳过 → 不卡死 |
| F | 急停中断 |
| G | 中止并返回起点 |
| H | 搬运途中掉件 |
| I | 暂停与恢复（计时补偿） |

### 这一轮测试实际抓出来的问题

写完之后跑第一遍就失败了 17 项，其中**五个是代码的真缺陷**：

1. **位姿控制器死锁**（最严重）。横向偏差大、纵向偏差小时，
   `v → 0` 让横向修正项消失，车停在离目标 20cm 处不动。
   实测表现：投放位对不准（x 偏 18.8cm）、侧前方物块永远进不了 ALIGN。
2. **重试计数被跨阶段清零**，`max_grasp_retry` 永远触发不了，
   吸不住的物块让状态机无限循环。
3. **`aborted` 标志被覆盖**（`stop_all()` 新建对象），
   状态进了 FAILED 但上层不知道。
4. **中止回起点会退化成继续干活**（缺 `aborting_` 标志）。
5. **暂停恢复的计时补偿是死代码**：先 `t_state_ = s.t` 再
   `t_task_ += (s.t - t_state_)`，括号里恒为 0。

另外两个是真实存在的**工程风险**，测试把它们逼出来了：

6. **标定点太集中时 2D 仿射是病态问题**。三个点挤在 2.5cm 范围内时，
   解出的矩阵线性部分乱飞、靠平移去凑 —— 标定点附近很准，出了那个
   小区就完全失效，而它看起来"标定成功了"。加了 `kMinCalibSpread` 安全阀。
7. **HAUL 到位判据缺角速度条件**，车在原地转着恰好扫过目标朝向的瞬间
   会被误判为到达。

剩下 10 项失败是测试自己写错了（观测没给面积被滤掉、三个标定点配了同一个
世界坐标、理想投影用了带残差的 `project()` 导致偏移翻倍、推演步数选得太长
跑过了想测的状态）。这些也都记在代码注释里了 —— 下次改这块的人能少踩一遍。

---

## 六、还没做的

- **`block_detector` 只做了颜色 + `minAreaRect`**，没有用 demo 里
  `calculate_angles.py` 的 Canny + HoughLinesP 那条路。原因是两条路都在
  求同一个"物块边缘角度"，而 `minAreaRect` 对矩形物块更稳（HoughLines
  在有纹理的地面上会检出一堆杂线）。如果物块换成非矩形或者表面有花纹，
  需要把 Hough 那条路加回来做交叉验证。
- **没有做视觉伺服的最后一段**。现在是"投影出目标位置 → 位姿控制器开过去"，
  没有用图像特征做闭环微调。滑移转向的停位精度实测在 2~3cm 量级，
  如果物块更小或者场地更滑，需要在 ALIGN 阶段加一层基于图像的伺服。
- **颜色顺序只在任务启动时指定**，中途不能改。
- **没有多车协同 / 与上层导航栈的交接**。`HAUL` 阶段是自己走的直线位姿控制，
  场地里有障碍物的话需要换成 nav2 的 NavigateToPose。
- **掉件之后的物块位置没有重建**。物块掉在路上，代码只记一笔跳过，
  不会去把掉在路上的物块捡起来（它可能滚到了别处）。
