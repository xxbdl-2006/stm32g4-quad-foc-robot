# 从 DetectedBlock 投影到地面并排序后的抓取目标。
# position.z 恒为 0（物块在地面上）。

std_msgs/Header header

uint32 index                   # 第几个目标，0 起
uint8  color
geometry_msgs/Point position   # odom 系地面坐标（m）
float32 yaw                    # 期望车体朝向（rad），使吸盘长边与物块对齐
float32 distance               # 距车体当前位置的距离（m）
float32 bearing                # 相对车头方向的方位角（rad，左正右负）
bool   reachable               # 是否在可抓范围内（距离与方位都满足）
bool   is_current              # 是否是当前正在执行的目标
