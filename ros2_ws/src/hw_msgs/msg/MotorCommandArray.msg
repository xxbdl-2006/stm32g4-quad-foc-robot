# 四关节指令集合。arm_control 节点以 100Hz 发布，hw_can 逐帧转换后发出。
# 下标 0..3 = J1..J4，与 MotorStateArray、URDF joint_1..joint_4 一致。
# 带宽核算：4 帧 × 100Hz = 400 帧/s，每帧 8 字节，1Mbps 下占用约 7%。
std_msgs/Header header
MotorCommand[] commands        # 长度恒为 4
