# 视觉检测到的一个物块。由 block_detector 节点发布，task_executor 消费。
#
# 字段设计对应睿抗比赛 demo 里 minAreaRect 的输出：
#   demo 取 (x+w/2, y+h/2) 作为中心、Theta1 作为姿态角，
#   并用 ThetaList 统计"同一角度出现两次"作为稳定判据。
# 这里把那个判据的结果直接物化成 confidence，让下游不必重做滤波。

std_msgs/Header header

uint8 COLOR_UNKNOWN = 0
uint8 COLOR_RED     = 1
uint8 COLOR_BLUE    = 2
uint8 COLOR_GREEN   = 3
uint8 color

float32 pixel_x                # 像素坐标 u（原始图像，未裁剪）
float32 pixel_y                # 像素坐标 v
float32 yaw                    # rad，物块绕垂直轴的偏转角，已归一到 [-pi/2, pi/2)
                               # 注：本机构用圆形真空吸盘，抓取不需要物块朝向；
                               # 这个字段只用于显示与标定时的人工核对
float32 area                   # 像素面积，用于尺度/有效性判据
float32 confidence             # 0..1，连续帧一致度（0 表示只出现过一次）
uint32  stable_frames          # 连续被判为同一物块的帧数
