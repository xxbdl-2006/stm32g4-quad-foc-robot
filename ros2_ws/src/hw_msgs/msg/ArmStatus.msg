# 机械臂控制器状态。arm_control 以 20Hz 发布。
std_msgs/Header header

uint8 STATE_IDLE     = 0    # 使能但无运动指令
uint8 STATE_MOVING   = 1    # 正在执行轨迹
uint8 STATE_HOLDING  = 2    # 到达目标，位置保持
uint8 STATE_REJECTED = 3    # 目标不可达/路径不可行，未执行（message 里给原因）
uint8 STATE_FAULT    = 4    # 有关节故障，已停止接受新目标
uint8 state

geometry_msgs/PoseStamped target_pose     # 当前目标（基座系）
geometry_msgs/PoseStamped current_pose    # 当前实测末端位姿（基座系）

# 末端线速度（基座系，m/s）。任务层用它判断"末端是否停稳"。
# 之所以由控制器算好发出来、而不是让任务层自己用关节速度推：
# 那需要在任务层再建一套运动学（几何参数、雅可比），一旦两边的参数
# 不一致，就会出现"控制器认为到了、任务层认为还在动"这种最难查的分歧。
geometry_msgs/Twist tool_twist

# 长度恒为 4，顺序 J1..J4。J1~J3 单位 rad，J4 单位 m（直线轴）。
float32[] joint_target
float32[] joint_measured
float32[] joint_velocity
float32[] joint_current_limit             # 当前分配到的电流上限（A）

uint8   waypoints_total                   # 本次运动被拆成了几段
uint8   waypoint_index
float32 progress                          # 当前段进度 0..1
float32 speed_scale

uint8   ik_fail_count                     # 累计被拒绝的目标数
float32 min_sin_theta2                    # 路径上最小的 |sinθ2|，越小越接近奇异
bool    ik_ok                             # 当前是否有可行运动
string  message                           # 最近一次拒绝或异常的原因
