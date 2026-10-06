# 任务执行状态。task_executor 以 20Hz 发布，比赛时也用它驱动上位机的状态灯/UI。

std_msgs/Header header

# ---- 状态机 ----
uint8 STATE_IDLE     = 0    # 未启动
uint8 STATE_SCAN     = 1    # 扫描寻找物块
uint8 STATE_APPROACH = 2    # 粗定位：末端移动到物块上方（允许绕行）
uint8 STATE_ALIGN    = 3    # 精定位：用最新视觉结果刷新目标，直到残差收敛
uint8 STATE_SETTLE   = 4    # 停稳等待
uint8 STATE_DESCEND  = 5    # 直线下压到物块顶面
uint8 STATE_GRASP    = 6    # 建立负压并判定吸住
uint8 STATE_LIFT     = 7    # 抬到搬运高度
uint8 STATE_HAUL     = 8    # 在搬运高度平移到投放位上方
uint8 STATE_PLACE    = 9    # 下放到投放面（或码垛的下一层）
uint8 STATE_RELEASE  = 10   # 放气释放
uint8 STATE_RETREAT  = 11   # 退开
uint8 STATE_DONE     = 12   # 全部完成
uint8 STATE_FAILED   = 13   # 失败终止
uint8 STATE_PAUSED   = 14   # 暂停（可恢复）
uint8 state
uint8 prev_state

# ---- 进度 ----
uint32 target_index            # 当前目标序号
uint32 total_targets           # 本次任务计划抓取的总数
uint8  grasped_count           # 已成功放置
uint8  skipped_count           # 已跳过（吸不住/越界）
uint8  retry_count             # 当前目标的当前状态重试次数
float32 elapsed                # 本次任务已运行时间（s）

# ---- 现场信息（比赛时界面直接显示这些）----
string  message                # 人类可读的一行说明
float32 pump_pressure          # kPa
float32 target_range           # 当前物块到基座回转轴的距离（m），-1 表示无目标
float32 pose_confidence        # 基于相机同步与目标可见性的置信度 0..1
bool    vision_ok
uint8   place_layer            # 当前码垛层号（0 起）。Array 模式下恒为 0
