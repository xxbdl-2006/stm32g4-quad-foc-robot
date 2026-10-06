# CAN 应用层协议规范 v1.0

> 固件实现：`firmware/Core/Inc/can_proto.h`
> 上位机实现：`ros2_ws/src/hw_can/include/hw_can/protocol.hpp`
> **两个文件必须同步修改。** C++ 侧有 `static_assert` 保证帧长一致，
> 但字段语义错位只能靠人 —— 所以任何改动都要同时更新本文档。

## 一、为什么不用 CANopen

| 维度 | CANopen CiA402 | 本协议 |
|---|---|---|
| 代码量 | 对象字典 + SDO/PDO 映射 + NMT ≈ 3000 行 | 编解码 + 节点管理 ≈ 600 行 |
| 总线节点数 | 设计给 127 节点 | 本项目 2~3 节点 |
| 调试可读性 | `0x181#00...` 需查 EDS 文件 | 11 位 ID 直接携带路由，candump 一眼可读 |
| 与第三方互换 | 可以 | 不可以（本项目无此需求） |
| 参数访问 | SDO 分段/块传输 | 单帧 8 字节，够用 |

结论：本项目**没有驱动器互换需求**，而**有强烈的现场可调试需求**
（装在现场的机器人上，能 SSH 上去直接 candump 看懂发生了什么）。
基于这个判断选了自定义协议。

## 二、11 位标识符布局

```
 bit  10 9 8 │ 7 6 5 4 │ 3 2 1 0
      NODE   │  CLASS  │   IDX
      3 bit  │  4 bit  │  4 bit
```

### NODE（bit 10..8）

| 值 | 节点 |
|---|---|
| 0 | `NODE_HOST` 上位机 |
| 1 | `NODE_MAINBOARD` 主板（本项目唯一的执行器板） |
| 2 | `NODE_EXPANSION` 预留（IMU / 扩展 IO） |
| 3..7 | 保留 |

### CLASS（bit 7..4）

| 值 | 名称 | 方向 | 说明 |
|---|---|---|---|
| 0x0 | `CLS_HEARTBEAT` | 板→PC | 10 Hz 心跳，含节点状态与故障位图 |
| 0x1 | `CLS_FAST` | 板→PC | 200 Hz 高速反馈 |
| 0x2 | `CLS_MOTION` | PC→板 | 运动指令 |
| 0x3 | `CLS_PARAM` | 双向 | 参数读写（含板卡主动上报的事件） |
| 0x4 | `CLS_IO` | 双向 | 气泵 / 相机 IO |
| 0x5 | `CLS_FLASH` | PC→板 | 参数持久化 |
| 0x6 | `CLS_DIAG` | 双向 | 诊断统计 |
| 0x7 | `CLS_ESTOP` | PC→板 | 广播急停（IDX 忽略） |

### IDX（bit 3..0）

语义随 CLASS 变化，见各节。

## 三、报文定义

所有多字节整数 **小端**。浮点仅在运动指令的 setpoint 字段使用 IEEE754 binary32。

### 3.1 `CLS_HEARTBEAT` / IDX=0（板→PC，10 Hz）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u16 | `fw_version` = major<<12 \| minor<<8 \| patch |
| 2 | u8 | `node_state`（0=boot 1=idle 2=ready 3=running 4=fault） |
| 3 | u8 | `fault_index` = 首个置位故障的位序号 |
| 4 | u16 | `fault_flags` 故障位图（与 `FAULT_*` 一致） |
| 6 | u8 | `motors_enabled` bit0..3 对应 M0..M3 |
| 7 | u8 | `uptime_s` |

**用途**：这是上位机判断"板子还活着"的唯一依据。500 ms 无心跳 → 上位机报警。

### 3.2 `CLS_FAST` / IDX=0..3（板→PC，每电机 200 Hz）

| 偏移 | 类型 | 字段 | 换算 |
|---|---|---|---|
| 0 | i32 | `position_mrad` | ×1e-3 → rad（多圈） |
| 4 | i16 | `velocity_mrads` | ×1e-3 → rad/s（钳 ±32767） |
| 6 | i16 | `current_ma` | ×1e-3 → A（钳 ±32767） |

只有 8 字节，因此**不含** mode/state/duty/fault。上位机把最近一次下发的
mode 缓存在本地，真实状态以心跳帧的故障位为准。

### 3.3 `CLS_FAST` / IDX=4（板→PC，200 Hz）板级状态

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u16 | `vbus_mv` |
| 2 | i16 | `mcu_temp_c10`（0.1 °C） |
| 4 | i16 | `power_w10`（0.1 W，四轴有功和） |
| 6 | u8 | `fault_flags` 低 8 位 |
| 7 | u8 | `node_state` |

### 3.4 `CLS_MOTION` / IDX=0..3（PC→板，100 Hz）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u8 | `mode`（`motor_mode_t`） |
| 1 | f32 | `setpoint`：A / rad/s / rad / 开环占空比，随 mode |
| 5 | u16 | `limit_ma` 电流上限（速度/位置模式用） |
| 7 | u8 | `flags` bit0=enable bit1=brake bit2=reset_fault bit3=estop |

**模式切换语义**：`mode` 与上一次不同时，固件在**下一个 50 µs 环路边界**
统一处理：清积分器、按目标模式重置内环。这样做是为了避免在 Park 变换
中途换环导致输出尖峰。

### 3.5 `CLS_MOTION` / IDX=15（4 轴同步广播，拆 2 帧）

第一帧用 `setpoint_q16[2]` 装 M0/M1 的速度（q16.16），第二帧装 M2/M3。
IDX 用 15。之所以不用 float，是为了在 8 字节里塞下两个设定值 + 两个限流。

> 注：当前上位机实现的常规路径是**逐轴发 4 帧**（IDX=0..3），
> 同步广播仅用于需要严格同时刻起停的场合（此时还应配合 CLS_ESTOP 语义）。

### 3.6 `CLS_PARAM`（双向）

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u16 | `param_id` |
| 2 | f32 | `value` |
| 6 | u8 | `op`：0=read_req 1=write 2=write_ack 3=read_resp 4=err |
| 7 | u8 | `reserved`：事件通道里用于携带 motor_id |

**事件通道**：`param_id` 的 bit15 置 1 表示这是板卡主动上报的事件，
低 15 位是事件码：

| 事件码 | 含义 |
|---|---|
| 1 | 标定完成（`value` = 电角度零点） |
| 2 | 故障发生（`value` = 故障位图，`reserved` = motor_id） |
| 3 | 模式切换 |
| 4 | 参数保存结果（`value` 1=成功 0=失败） |
| 5 | 板上电 |
| 6 | 相机同步丢失 |

**参数事务**：上位机发 `read_req` 或 `write`，板子在同一个 IDX 上回
`read_resp` / `write_ack`。上位机的 `param_transact()` 用条件变量等待，
默认超时 100 ms。

参数 ID 表见 `can_proto.h` 的 `PID_* / LIMIT_* / BOARD_* / PUMP_* / CAM_*`。
其中 `ENC_ELEC_OFFSET` **只读** —— 禁止上位机直接写，写错会直接烧驱动。

### 3.7 `CLS_IO`

| IDX | 名称 | 方向 | 内容 |
|---|---|---|---|
| 0 | `PUMP_CMD` | PC→板 | cmd / duty_permille / mode / target_kpa10 |
| 1 | `PUMP_STATE` | 板→PC | state / duty / pressure / fault（50 Hz） |
| 2 | `CAM_CFG` | PC→板 | enable / rate_hz / pulse_us / mode |
| 3 | `CAM_SYNC_EVT` | 板→PC | frame_id / dt_us / dropped |
| 4 | `BOARD_IO_STATE` | 板→PC | 压力 / 相机在线 / 帧号低 16 位 |
| 5..8 | `CAM_LATCH0..3` | 板→PC | M0..M3 的同步锁存位置 |

`CAM_LATCH` 帧结构：

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | i32 | `position_mrad` |
| 4 | i16 | `velocity_mrads` |
| 6 | u16 | `frame_id_lo`（帧号低 16 位） |

帧号用低 16 位是因为 8 字节塞不下完整 32 位 + 位置 + 速度。
上位机收到 `CAM_SYNC_EVT`（含完整 32 位帧号）后，校验紧接着到来的
4 帧 `frame_id_lo` 是否匹配 —— 因为顺序有保证（同一批次连续发出），
所以不存在歧义。

### 3.8 `CLS_ESTOP` / 任意 IDX（PC→板，广播）

收到即：四轴 `motor_emergency_stop()` + 气泵立即停 + 相机触发停。
**这是最高优先级的帧**，ID 高 3 位为 0（HOST），会赢得所有仲裁。

### 3.9 `CLS_FLASH` / IDX=0

`value > 0.5` 表示保存，否则表示重新载入。写盘在主循环里做（关中断 20 ms）。

### 3.10 `CLS_DIAG`

| IDX | 内容 |
|---|---|
| 0 | 环路统计（超时次数 ×4） |
| 1 | 编码器统计（CRC 错误 / 超时） |
| 2 | CAN 统计（TX/RX 错误计数） |
| 3 | 运行时间 |

## 四、超时与看门狗

| 项 | 阈值 | 超时行为 |
|---|---|---|
| 驱动板等上位机指令 | 300 ms | **四轴安全停机** + `FAULT_CAN_TIMEOUT` |
| 上位机等驱动板心跳 | 500 ms | 报警 + 诊断升级 + 让导航栈停车 |
| 相机同步超时 | 500 ms | 标记相机掉线 + 上报事件 |
| 参数事务响应 | 100 ms | 上位机侧抛异常，服务返回失败 |

两个方向的超时阈值刻意设成 300 / 500 —— 板子的反应更快，因为它是
最后一环，不能指望上游一定正常。

## 五、扩展预留

- NODE 段还有 5 个未分配值（3..7），够再加 5 个执行器板。
- CLASS 段已用满，但 IDX 在 `CLS_IO`(0..15) 和 `CLS_DIAG`(0..15) 里各有大量空位。
- 若将来需要 CAN FD（64 字节负载），`CLS_MOTION` 的 4 轴同步广播可以
  合并成 1 帧，`CAM_LATCH` 可以 1 帧装下四轴。ID 布局完全不用改。
