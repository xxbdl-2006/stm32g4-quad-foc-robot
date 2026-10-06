# 气泵状态，CAN CLS_IO(IDX=1)，50Hz。
std_msgs/Header header

uint8 STATE_IDLE     = 0
uint8 STATE_RAMPING  = 1
uint8 STATE_RUNNING  = 2
uint8 STATE_FAULT    = 3
uint8 STATE_SOFTSTOP = 4
uint8 state

float32 duty                   # 当前实际占空比 0..1
float32 pressure               # kPa
uint8 fault
uint8 FAULT_STALL       = 1
uint8 FAULT_SENSOR_OPEN = 2
uint8 FAULT_OVERCURRENT = 4
