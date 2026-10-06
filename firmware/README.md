# HWB-MC4G4 固件

STM32G474VET6（LQFP100，170 MHz）上的四轴 FOC + 气泵 + 相机 IO 固件。
裸机 + HAL，不用 RTOS —— 实时性由中断优先级保证，比引入 RTOS 再调优先级翻转简单。

## 编译

```bash
# 需要 arm-none-eabi-gcc 12.3+ 与 ninja
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake
cmake --build build
arm-none-eabi-size build/hwb_mc4g4.elf
```

或直接用仓库根目录的脚本：
```bash
./scripts/flash.sh --build-only    # 只构建
./scripts/flash.sh                 # 构建 + ST-Link 烧录
./scripts/flash.sh --dfu           # 构建 + DFU 烧录
```

### 缺失的目录

`Drivers/`（STM32CubeG4 HAL + CMSIS）与 `Startup/`（startup_stm32g474vetx.s）
**未包含在本仓库**，需要用 CubeMX 按 `docs/03_hardware.md` 的引脚表生成，
或从 ST 官网下载 [STM32CubeG4](https://www.st.com/en/embedded-software/stm32cubeg4.html) v1.6.0
后把 `Drivers/` 复制过来。

CubeMX 配置要点（漏掉任何一条都会出问题）：

| 项 | 值 | 为什么 |
|---|---|---|
| RCC HSE | 8 MHz 晶振，PLL → 170 MHz | `board_clock_init()` 里有 170MHz 断言 |
| TIM1 | 中心对齐，ARR=2399，PSC=0，TRGO=Update | 20 kHz 电流环时基 + SVPWM 计数范围 |
| TIM1 CH1/2/3 | PWM Generation CH1/2/3，Pulse=1200 | 上电中位，避免使能瞬间满压 |
| TIM2/TIM8/TIM4 | 同上（ARR=2399，PSC=0） | M1/M2/M3 |
| ADC1~ADC4 | 注入组 3 通道，External Trigger = TIM1 TRGO | 中心对齐下溢后 1.2 µs 采样 |
| ADC5 | 规则组 6 通道 + DMA 循环，1 kHz（TIM1 分频） | 母线/压力/NTC 慢量 |
| SPI1 | Master，10 MHz，CPOL=Low/CPHA=1Edge（mode 1），8/16bit | AS5047P |
| FDCAN1 | Classic CAN，1 Mbps，采样点 80%，TX FIFO Queue | CAN 总线 |
| CORDIC | COSINE，6 cycles，NbRead=2（cos+sin） | 电流环三角函数 |
| NVIC | TIM1_UP=1，FDCAN1_IT0=2，EXTI15_10=3 | 见 `stm32g4xx_it.c` 注释 |
| Flash | 保留 sector 3（0x0803E000）给参数区 | `param.c` |
| 优化 | -O2（**不要 -Ofast**） | 见 `CMakeLists.txt` 注释 |

## 模块结构

```
                       ┌──────────────┐
      20kHz ISR ──────►│  motor.c     │  三环 + 状态机 + 故障
                       │   ├ foc.c    │  Clarke/Park/PI/SVPWM/弱磁
                       │   ├ pid.c    │  抗饱和 PID
                       │   └ encoder.c│  AS5047P 流水线读 + 多圈累加
                       │   └ current_sense.c  4 路 ADC 注入组 + 死区补偿
                       └──────┬───────┘
                              │
      EXTI ISR ──────► cam_io.c       触发输出 + 同步锁存（DWT 计时）
      1kHz 任务 ─────► pump.c         斜坡 + 压力环 + 堵转检测
                              │
                       ┌──────▼───────┐
      FDCAN RX ISR ──►│  can_node.c  │  周期帧 / 指令解析 / 参数事务 / 看门狗
                      │  can_proto.c │  布局自检 / 名称表 / 急停帧
                      └──────┬───────┘
                             │
                      board.c / param.c / foc_math.c   底层
```

## 关键实现的取舍记录

### 1. 编码器用 SPI 流水线读法（`encoder.c`）

AS5047P 的响应滞后一帧。因为我们永远读同一个寄存器（ANGLECOM = 0x3FFF），
命令是常量，所以可以把上一拍的响应当本拍用，**1 帧 SPI 换一个角度**。

代价是角度领先/滞后一个周期（50 µs）。4 台电机共 6.4 µs（对比 3 帧读法的
19 µs），在 50 µs 的环路预算里这是 25% 的差别。

### 2. 死区补偿在软件里做（`current_sense.c`）

DRV8323 的 180 ns 死区在 20 kHz 下相当于 0.36% 的占空比损失。
正电流时实际输出偏小，负电流时偏大。补偿量按母线电压缩放，
并对 |i| < 0.1 A 的过零区**不做补偿** —— 那里电流方向不确定，
补偿会在过零点引起振荡。

### 3. 模式切换在环路边界生效（`motor.c`）

`motor_set_mode()` 只写 `mode_pending`，真正的切换发生在下一个
20 kHz 边界。如果在 Park 变换中途换了环，可能读到半更新的 pid 积分器，
输出一个巨大的电流尖峰。

### 4. 看门狗的方向是"板子不信任上位机"（`can_node.c`）

300 ms 收不到上位机指令 → 四轴安全停机 + 泵停 + 相机停。
这样 PC 死机 / ROS 崩了 / 网线掉了都不会导致执行器按最后一条指令一直跑。

### 5. 保护阈值不落 Flash（`param.c`）

`BOARD_VBUS_OVP` 这类阈值可以在线改，但 `param_save()` 不保存它们。
理由是：如果有人误存 `vbus_ovp = 5 V`，之后每次上电都会被这张自己的
配置卡住，只能重新烧固件。持久化"安全参数"是个陷阱。

### 6. 协议布局开机自检（`can_proto.c`）

`can_proto_self_check()` 在 `main()` 的第 1.5 步跑，校验线格式结构体的
长度、关键字段偏移、以及 ID 编解码的往返一致性。失败直接进 `Error_Handler`。
因为"帧格式错位"这种故障在现场表现为"通信完全正常但数值全乱"，极难定位。

## 时序预算（实测，170 MHz）

| 环节 | 周期 | 实测耗时 | 占比 |
|---|---|---|---|
| 电流环（含 4 路编码器 + 4 路 ADC + 4 轴 FOC） | 50 µs | ≈14 µs | 28% |
| 速度/位置环 | 1 ms | ≈4 µs | 0.4% |
| 气泵 1 kHz 任务 | 1 ms | ≈2 µs | 0.2% |
| CAN 周期帧（20 kHz 内的分频发送） | 50 µs | ≈0.4 µs（均值） | 0.8% |
| 主循环（Flash 写、诊断） | 空闲 | — | — |
| **CPU 峰值占用** | | | **< 35%** |

余量 65%，足够再加一层观测器、自适应律或者第二块扩展板。

## 调试建议

```bash
# 1. 用 SWD + OpenOCD 看变量（不需要改代码）
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg

# 2. 临时插桩：在 ISR 里翻转一个空闲 GPIO，用示波器量环路耗时
#    （board.h 里 PC15 在 v1.4 被 NTC 占用，可用 PB15 或飞线）

# 3. 环路抖动统计走 CAN 诊断通道，不用接示波器
ros2 topic pub --once /hw/diag ...   # 或直接看 /diagnostics 的 loop_overrun
```

常见坑：
- **HardFault 大概率是时钟没配到 170 MHz**。`board_clock_init()` 里有断言，
  但断言本身在 HardFault 处理前就触发了 —— 先查 `SystemClock_Config()`。
- **`-Wdouble-promotion` 的告警务必当错误处理**。Cortex-M4F 只有单精度，
  一个隐式的 double 运算会让该行从 1 个周期变成 20+ 个周期。
- **不要在 ISR 里调 `sinf/cosf`**，用 `foc_math.h` 的 `fast_sincos()`。
