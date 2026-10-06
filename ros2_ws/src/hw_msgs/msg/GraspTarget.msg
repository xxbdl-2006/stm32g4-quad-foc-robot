# 从 DetectedBlock 投影到工作台面并排序后的抓取目标。
# position.z 恒为 0（物块放在工作台面上）。

std_msgs/Header header

uint32 index                   # 第几个目标，0 起
uint8  color
geometry_msgs/Point position   # 基座系下的台面坐标（m）
float32 yaw                    # 物块长边方向（rad）。仅用于显示 —— 吸盘是圆形的，
                               # 抓取时不需要对齐物块朝向
float32 distance               # 距基座回转轴的距离（m），也就是极径
float32 bearing                # 相对基座 +x 轴的方位角（rad）
bool   reachable               # 是否落在可达环带内（距离在 radius_min..max 之间）
bool   is_current              # 是否是当前正在执行的目标
