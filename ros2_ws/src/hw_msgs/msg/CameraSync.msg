# 相机帧同步事件 + 该时刻四轴位置的硬件锁存结果。
#
# 这是整个系统里对时间一致性要求最高的一条消息：
# 驱动板在收到相机 SYNC 上升沿的那一刻，用硬件中断锁存了四台电机的
# 编码器位置，然后通过 CAN CLS_IO(IDX=5..8) 把 4 帧位置发上来。
# 上位机收到后与本地的图像缓存按 frame_id 关联，得到"这一帧图像对应的
# 四轴真实位姿"，而不是"图像到达时电机在哪"。
#
# 同步精度：EXTI 到锁存 < 1us；CAN 传输延迟 0.2~1ms（不参与对齐）；
# 因此图像与位姿的实际时间偏差约等于相机自身的曝光-同步延迟，
# 由 camera_trigger 节点测量并发布在 exposure_delay 里。

std_msgs/Header header

uint32 frame_id                # 驱动板的单调帧号，用于检测丢帧
float32[] latched_positions    # rad，长度 4，顺序 M0..M3
float32[] latched_velocities   # rad/s，长度 4
float32 period                 # s，与上一帧的实际间隔
uint16 dropped                 # 自上一帧以来丢失的同步脉冲数
float32 exposure_delay         # s，SYNC 上升沿到图像就绪的估计延迟
uint32 capture_cycles          # 驱动板 DWT 周期计数，供跨帧比较（非绝对时间）
