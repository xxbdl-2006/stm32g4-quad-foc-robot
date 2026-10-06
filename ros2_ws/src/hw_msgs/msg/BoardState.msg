# 板级状态，CAN CLS_FAST(IDX=4)，200Hz。
std_msgs/Header header

float32 vbus                   # V，母线电压
float32 mcu_temperature        # °C
float32 mos_temperature_max    # °C，四路 MOS NTC 的最高值
float32 power                  # W，四轴有功功率之和
uint16 fault                   # 板级故障位图
uint8  node_state              # 0=boot 1=idle 2=ready 3=running 4=fault
uint8  NODE_BOOT    = 0
uint8  NODE_IDLE    = 1
uint8  NODE_READY   = 2
uint8  NODE_RUNNING = 3
uint8  NODE_FAULT   = 4
bool   host_online             # 驱动板是否还认为上位机在线（看门狗未超时）
