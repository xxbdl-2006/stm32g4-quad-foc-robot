// ============================================================================
//  task_types.hpp
//  任务层的类型与配置。刻意不依赖 rclcpp —— 与固件里把 foc.c 从 motor.c
//  里拆出来是同一个理由：逻辑能被单元测试单独驱动，不用起 ROS。
//
//  对应关系（睿抗比赛 demo -> 这里）
//  ------------------------------------------------------------------
//   demo                                 本项目
//   ----------------------------------   ----------------------------------
//   robot_x = 0.0008*obj_y + 0.2563      地面投影 + 在线残差校正
//   robot_y = -0.001*obj_x + 0.1450      （见 ground_projection.cpp）
//   0.21 + 0.055*i  码垛高度              排放阵列 place_pitch（底盘无 Z 轴）
//   gripper.on() / off()                 /hw/set_pump + 压力反馈判定
//   arm.joint_space_interpolated_motion  /cmd_vel 速度控制
//   if robot_x < 1000 and ...             reachable 判据（距离 + 方位 + 视觉置信）
//   ThetaList 同角度出现两次               BlockTracker 的多帧一致性
// ============================================================================
#ifndef HW_TASK__TASK_TYPES_HPP_
#define HW_TASK__TASK_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace hw_task
{

// ---------------------------------------------------------------------------
//  状态机
// ---------------------------------------------------------------------------
enum class TaskState : uint8_t
{
  Idle     = 0,
  Scan     = 1,
  Approach = 2,
  Align    = 3,
  Settle   = 4,
  Descend  = 5,
  Grasp    = 6,
  Lift     = 7,
  Haul     = 8,
  Place    = 9,
  Release  = 10,
  Retreat  = 11,
  Done     = 12,
  Failed   = 13,
  Paused   = 14,
};

const char * to_string(TaskState s);

// ---------------------------------------------------------------------------
//  物块颜色
// ---------------------------------------------------------------------------
enum class BlockColor : uint8_t
{
  Unknown = 0,
  Red     = 1,
  Blue    = 2,
  Green   = 3,
};

const char * to_string(BlockColor c);

// ---------------------------------------------------------------------------
//  配置
// ---------------------------------------------------------------------------
struct TaskConfig
{
  // ---- 视觉可接受的物块 ----
  float min_confidence{0.55f};        ///< 低于此值不进目标列表
  uint32_t min_stable_frames{3};      ///< 连续帧数下限（替代 demo 的 ThetaList 判据）
  float min_area_px{2000.0f};         ///< 对应 demo 的 min_area = 2000
  float max_area_px{30000.0f};        ///< 对应 demo 的 max_area = 30000

  // ---- 可抓范围（底盘正前方扇形）----
  float grasp_radius_min{0.18f};      ///< 太近了吸盘对不准
  float grasp_radius_max{0.85f};      ///< 太远了视觉投影误差大
  float grasp_bearing_max{0.60f};     ///< rad，±34°

  // ---- 运动控制 ----
  float approach_speed{0.35f};        ///< m/s，粗定位速度
  float align_speed{0.09f};           ///< m/s，精定位速度（必须慢，否则过冲）
  float scan_yaw_rate{0.45f};         ///< rad/s，扫描时原地旋转速度
  float standoff{0.34f};              ///< m，停位时物块到车体中心的期望距离
  float standoff_tol{0.030f};         ///< m
  float bearing_tol{0.045f};          ///< rad
  float yaw_kp{1.8f};                 ///< 转向 P 增益
  float lateral_kp{1.6f};             ///< 横向（bearing）纠正 P 增益
  float longitudinal_kp{1.2f};        ///< 纵向（距离）纠正 P 增益
  float speed_min{0.02f};             ///< m/s，低于这个值就贴着死区不动

  // ---- 各状态超时（秒）----
  float scan_timeout{12.0f};
  float approach_timeout{15.0f};
  float align_timeout{8.0f};
  float settle_time{0.35f};           ///< 停稳保持时间；等轮胎/悬架形变恢复
  float descend_time{0.30f};          ///< 抽真空时间
  float grasp_timeout{1.5f};          ///< 等压力达标
  float lift_time{0.40f};
  float haul_timeout{45.0f};
  float place_timeout{15.0f};
  float release_time{0.50f};
  float retreat_time{1.20f};
  float vision_lost_timeout{0.60f};   ///< 视觉丢失超过此时长就退回 SCAN

  // ---- 重试 ----
  uint32_t max_scan_retry{3};
  uint32_t max_align_retry{2};
  uint32_t max_grasp_retry{2};        ///< 吸不住时重新试几次，超过就跳过该物块

  // ---- 气泵（真空吸盘）----
  float grasp_duty{1.00f};            ///< 抓取时的 PWM 占空比 0..1
  float hold_duty{0.75f};             ///< 搬运途中维持的占空比（降占空比省电降温）
  float release_duty{0.00f};
  float vacuum_threshold_kpa{6.0f};   ///< 压力低于此值（相对大气）判为吸住
  bool  use_pressure_check{true};     ///< false 则退化为 demo 的纯定时判断

  /// 让驱动板自己用压力环维持真空，而不是上位机给固定占空比。
  /// 好处是漏气时驱动板会自动加大功率，而上位机只需要看压力是否达标。
  /// 对应驱动板上的 PUMP mode=1（压力闭环），这条路径是硬件已经实现的。
  bool  use_pump_pressure_mode{true};
  float vacuum_target_kpa{-25.0f};    ///< 压力环目标（表压，真空为负值）

  // ---- 投放 ----
  double place_x{0.0};                ///< odom 系投放阵列起点
  double place_y{0.0};
  double place_yaw{0.0};              ///< 放置时车体朝向
  double place_pitch{0.12};           ///< m，相邻放置位间距（物块尺寸 + 间隙）
  int    place_columns{4};            ///< 每行几个，超出换行
  double place_row_pitch{0.12};

  // ---- 起点 ----
  double home_x{0.0};
  double home_y{0.0};
  double home_yaw{0.0};
};

// ---------------------------------------------------------------------------
//  启动请求
// ---------------------------------------------------------------------------
struct StartRequest
{
  uint32_t max_blocks{0};             ///< 0 = 抓到没有为止
  std::vector<BlockColor> color_order;///< 空 = 按发现顺序
  bool require_stable{true};
  bool return_home_after{false};
};

// ---------------------------------------------------------------------------
//  传感器快照：每个控制周期喂给状态机
// ---------------------------------------------------------------------------
struct SensorSnapshot
{
  double t{0.0};                      ///< 单调时间（秒）

  // ---- 车体 ----
  double vehicle_x{0.0};
  double vehicle_y{0.0};
  double vehicle_yaw{0.0};
  double vehicle_vx{0.0};             ///< odom 系线速度，用于停稳判定
  double vehicle_wz{0.0};

  // ---- 当前目标 ----
  bool   target_valid{false};
  double target_x{0.0};               ///< odom 系
  double target_y{0.0};
  double target_yaw{0.0};
  BlockColor target_color{BlockColor::Unknown};
  float  target_confidence{0.0f};

  // ---- 视觉整体 ----
  bool   vision_ok{false};            ///< 相机在线且本周期收到了检测结果
  uint32_t blocks_visible{0};         ///< 当前视野内可用物块数

  // ---- 气泵 ----
  bool   pump_online{false};
  double pump_pressure_kpa{0.0};
  uint8_t pump_fault{0};

  // ---- 底盘 ----
  bool   drive_faulted{false};        ///< 任一电机处于 FAULT
  bool   estop{false};
};

// ---------------------------------------------------------------------------
//  状态机输出：每个控制周期由它驱动实际动作
// ---------------------------------------------------------------------------
struct Actuation
{
  double cmd_vx{0.0};                 ///< m/s
  double cmd_vy{0.0};
  double cmd_wz{0.0};                 ///< rad/s

  bool   pump_run{false};
  double pump_duty{0.0};              ///< 0..1

  int    select_block{-1};            ///< >=0 时要求切到该序号的物块（-1 = 不变）
  bool   cancel_block{false};         ///< 放弃当前目标
  bool   finished{false};             ///< 任务正常结束
  bool   aborted{false};              ///< 任务异常终止

  std::string note;                   ///< 写进 TaskStatus.message
};

}  // namespace hw_task

#endif  // HW_TASK__TASK_TYPES_HPP_
