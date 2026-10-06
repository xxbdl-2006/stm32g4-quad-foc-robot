# 一帧图像里检出的全部物块。
#
# 注意：这里的每一个 DetectedBlock 都来自**同一帧**图像，
# 该帧在驱动板收到相机 SYNC 上升沿的那一刻锁存了四轮位姿。
# task_executor 用 header.stamp 去 TF 里查那时的 odom->base_link，
# 从而把像素投到地面。所以这个时间戳必须原样透传，不能被下游改写。

std_msgs/Header header

DetectedBlock[] blocks
uint32 image_seq               # 相机驱动给的帧序号，用于丢帧检测
bool   roi_clipped             # true 表示只处理了 ROI 区域（pixel_* 已换算回全图）
