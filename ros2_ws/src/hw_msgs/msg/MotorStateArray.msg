# 四台电机的状态集合，由 hw_can 的 bridge 节点聚合后以固定频率发布。
# 之所以要聚合而不是逐台发：下游控制器（arm_control / ros2_control）
# 需要看到"同一时刻"的四关节状态才能做正确的正解、速度前馈和电流分配 ——
# 四个值若来自不同时刻，末端位姿本身就是错的（100Hz 下差一帧就是几毫米）。
# bridge 节点按 CAN 帧的接收顺序把 4 帧缓存起来，4 帧齐了才发一帧
# MotorStateArray，因此四个 MotorState 来自同一个 CAN 反馈周期。

std_msgs/Header header

# 触发本次聚合的板级状态
BoardState board

MotorState[] motors            # 长度恒为 4，顺序固定 M0..M3 = J1..J4
uint8 MOTOR_COUNT = 4
