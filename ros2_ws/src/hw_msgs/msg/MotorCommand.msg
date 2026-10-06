# 单台电机的运动指令。上位机 -> 驱动板，CAN CLS_MOTION(IDX=motor_id)。
std_msgs/Header header

uint8 motor_id

uint8 MODE_IDLE      = 0
uint8 MODE_CURRENT   = 1
uint8 MODE_VELOCITY  = 2
uint8 MODE_POSITION  = 3
uint8 MODE_OPENLOOP  = 4
uint8 MODE_CALIBRATE = 5
uint8 MODE_BRAKE     = 6
uint8 mode

float32 setpoint               # 语义随 mode 变化：A / rad/s / rad / 开环占空比
float32 current_limit          # A，动态限流；<=0 表示用板卡默认值
bool    enable
bool    brake
bool    reset_fault
