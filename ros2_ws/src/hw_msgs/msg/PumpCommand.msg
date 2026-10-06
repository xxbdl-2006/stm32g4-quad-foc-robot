# 气泵指令，CAN CLS_IO(IDX=0)。
std_msgs/Header header

uint8 CMD_STOP      = 0
uint8 CMD_RUN       = 1
uint8 CMD_SOFT_STOP = 2
uint8 cmd

uint8 MODE_OPEN_DUTY = 0       # 直接给占空比
uint8 MODE_PRESSURE  = 1       # 压力闭环
uint8 mode

float32 duty                   # 0.0 .. 1.0
float32 target_pressure        # kPa，mode=MODE_PRESSURE 时有效
