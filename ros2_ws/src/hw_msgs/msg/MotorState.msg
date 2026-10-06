# 单台 BLDC 电机的完整状态快照。
# 来源：CAN CLS_FAST(IDX=0..3)，200Hz/电机。
# 时间戳由上位机在收到该 CAN 帧时打，不是驱动板的本地时钟 —— 因为
# CAN 帧本身不带时间信息，且板卡的 SysTick 与 PC 的 CLOCK_MONOTONIC 不同源。

std_msgs/Header header

uint8 motor_id                 # 0..3

# 与固件 motor_state_t 一一对应
uint8 STATE_DISABLED = 0
uint8 STATE_READY    = 1
uint8 STATE_RUNNING  = 2
uint8 STATE_FAULT    = 3
uint8 STATE_CALIB    = 4
uint8 state

# 与固件 motor_mode_t 一一对应
uint8 MODE_IDLE      = 0
uint8 MODE_CURRENT   = 1
uint8 MODE_VELOCITY  = 2
uint8 MODE_POSITION  = 3
uint8 MODE_OPENLOOP  = 4
uint8 MODE_CALIBRATE = 5
uint8 MODE_BRAKE     = 6
uint8 mode

float32 position               # rad，多圈累计（输出轴侧，已含减速比折算）
float32 velocity               # rad/s
float32 current                # A，q 轴电流
float32 duty                   # -1..1，A 相占空比（调试用）

# 与固件 FAULT_* 位图一致
uint16 fault
uint16 FAULT_UNDERVOLTAGE = 1
uint16 FAULT_OVERVOLTAGE  = 2
uint16 FAULT_OVERCURRENT  = 4
uint16 FAULT_MCU_OVERTEMP = 8
uint16 FAULT_MOS_OVERTEMP = 16
uint16 FAULT_ENCODER_SPI  = 32
uint16 FAULT_ENCODER_JUMP = 64
uint16 FAULT_DRV_FAULT_N  = 128
uint16 FAULT_CAN_BUSOFF   = 256
uint16 FAULT_CAN_TIMEOUT  = 512
uint16 FAULT_WATCHDOG     = 1024
uint16 FAULT_CTRL_LATE    = 2048
uint16 FAULT_PUMP_STALL   = 4096
uint16 FAULT_CAM_SYNC_LOST = 8192
