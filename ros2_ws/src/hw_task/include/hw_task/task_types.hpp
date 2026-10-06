// ============================================================================
//  task_types.hpp
//  任务层的类型与配置。刻意不依赖 rclcpp —— 与固件里把 foc.c 从 motor.c
//  里拆出来是同一个理由：逻辑能被单元测试单独驱动，不用起 ROS。
//
//  对应关系（睿抗比赛 demo -> 这里）
//  ------------------------------------------------------------------
//   demo                                  本项目
//   -----------------------------------   ------------------------------------
//   robot_x = 0.0008*obj_y + 0.2563       工作台投影 + 在线残差校正
//   robot_y = -0.001*obj_x + 0.1450       （见 table_projection.cpp）
//   0.21 + 0.055*i  码垛层高                place_layer_height（可叠放，见下）
//   gripper.on() / off()                  /hw/set_pump + 压力反馈判定
//   arm.joint_space_interpolated_motion   末端目标位姿 -> arm_control 逆解+轨迹规划
//   if robot_x < 1000 and ...              reachable 判据（工作半径 + 视觉置信）
//   ThetaList 同角度出现两次                BlockTracker 的多帧一致性
//
//  动作接口是"末端目标位姿"而不是关节角
//  ------------------------------------
//  任务层说"末端去 (x, y, z)"，arm_control 负责解逆解、选分支、规划轨迹。
//  这样分工的原因：同一个末端目标有两组关节解（肘上/肘下），选哪一组取决于
//  当前姿态与连续性 —— 那是运动控制该管的事。任务层去操心关节角只会让
//  "改一下机械尺寸就要改一遍任务逻辑"。
// ============================================================================
#ifndef HW_TASK__TASK_TYPES_HPP_
#define HW_TASK__TASK_TYPES_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace hw_task
{

// ---------------------------------------------------------------------------
//  状态机
//
//  与底盘方案相比状态名字一个没改 —— 它们描述的是"抓取流程的阶段"，
//  与执行器是轮子还是机械臂无关。变的只是每个阶段下发的动作。
// ---------------------------------------------------------------------------
enum class TaskState : uint8_t
{
  Idle     = 0,
  Scan     = 1,    ///< 等视觉给出稳定目标（机械臂不动）
  Approach = 2,    ///< 末端移动到物块上方（允许绕行，路径可以长）
  Align    = 3,    ///< 视觉伺服精调（小幅直线修正，直到残差收敛）
  Settle   = 4,    ///< 停稳等待（等残余振动衰减）
  Descend  = 5,    ///< 直线下压到物块顶面
  Grasp    = 6,    ///< 建立负压并判定吸住
  Lift     = 7,    ///< 抬到搬运高度
  Haul     = 8,    ///< 在搬运高度平移到投放位上方
  Place    = 9,    ///< 下放到投放面（或下一层）
  Release  = 10,   ///< 放气释放
  Retreat  = 11,   ///< 抬起退开
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
//  投放方式
// ---------------------------------------------------------------------------
enum class PlaceMode : uint8_t
{
  /// 单层阵列排开：每抓一个换个槽位，层高不变
  Array = 0,
  /// 码垛：在同一个槽位往上叠，每层抬高一个物块高度（比赛 demo 的做法）
  Stack = 1,
};

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

  // ---- 工作台平面上的可达环带（距基座回转轴的距离，单位 m）----
  /* 下限不是几何上的最小半径（那是 |a1-(a2+a3)| = 6cm），而是机械干涉
   * 决定的：再往里走，小臂会撞上基座法兰。上限取 0.40 而不是理论最大
   * 半径 0.42 —— 末端在最大伸展处接近奇异，关节速度被放大，视觉定位的
   * 几毫米误差会被放大成很大的关节角速度，实物上表现为"伸手时末梢抖"。 */
  float reach_radius_min{0.12f};
  float reach_radius_max{0.40f};

  // ---- 末端高度（基座系 z，台面 z = 0）----
  float hover_z{0.050f};              ///< 接近时的悬停高度（台面上方 50mm）
  float touch_z{0.023f};              ///< 下压到位的高度：物块高 25mm，压下 2mm 过盈
  float travel_z{0.100f};             ///< 搬运高度（越过工作台上最高的夹具）
  float retreat_z{0.080f};            ///< 释放后退开的高度

  // ---- 到位判据 ----
  /* 位置容差 2mm：视觉投影的重复精度约 1mm，机械重复定位精度约 0.2mm，
   * 留一倍余量。比物块尺寸（80mm）小得多，不会误判到隔壁物块。
   * 速度容差用末端合成速度，0.01 m/s 相当于"基本停住"。 */
  float pos_tol{0.002f};
  float vel_tol{0.010f};
  /// 视觉伺服收敛判据：目标位置的新估计与已下达目标之差小于此值就认为收敛
  float servo_tol{0.0015f};

  // ---- 各状态超时（秒）----
  float scan_timeout{12.0f};
  float approach_timeout{8.0f};
  float align_timeout{6.0f};
  float settle_time{0.30f};           ///< 停稳保持；等谐波减速器的残余振动衰减
  float descend_time{0.25f};
  float grasp_timeout{1.5f};
  float lift_time{0.25f};
  float haul_timeout{12.0f};
  float place_timeout{6.0f};
  float release_time{0.40f};
  float retreat_time{0.60f};
  float vision_lost_timeout{0.60f};

  // ---- 重试 ----
  uint32_t max_scan_retry{3};
  uint32_t max_align_retry{2};
  uint32_t max_grasp_retry{2};

  // ---- 气泵（真空吸盘）----
  float grasp_duty{1.00f};
  float hold_duty{0.75f};
  float release_duty{0.00f};
  float vacuum_threshold_kpa{6.0f};
  bool  use_pressure_check{true};
  bool  use_pump_pressure_mode{true};
  float vacuum_target_kpa{-25.0f};

  // ---- 投放 ----
  PlaceMode place_mode{PlaceMode::Stack};
  double place_x{0.0};                ///< 投放点（基座系）
  double place_y{0.28};
  double place_yaw{0.0};
  /// 阵列模式下的槽位间距（m）。物块外接圆直径 80mm + 10mm 取放余量。
  double place_pitch{0.090};
  double place_row_pitch{0.090};
  int    place_columns{3};
  int    place_rows{3};               ///< 每层几行。与 columns 一起决定总容量
  /// 码垛模式下的层高（m）= 物块高度
  double place_layer_height{0.025};
  /// 码垛最大层数。超过就停：Z 轴行程只有 135mm，第 5 层就要顶到上限了
  int    place_max_layers{4};

  // ---- 起点（回原点用）----
  double home_x{0.28};
  double home_y{0.0};
  double home_z{0.100};
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

  // ---- 末端（吸盘中心）在基座系下的位姿与速度 ----
  double tool_x{0.0};
  double tool_y{0.0};
  double tool_z{0.0};
  double tool_vx{0.0};
  double tool_vy{0.0};
  double tool_vz{0.0};

  /// 末端合成速度（m/s）。停稳判定用它：逐轴比较要写三个阈值，
  /// 而且三轴各自"差一点点没超"时合起来其实还在动。
  double tool_speed() const
  {
    return std::sqrt(tool_vx * tool_vx + tool_vy * tool_vy + tool_vz * tool_vz);
  }

  // ---- 当前目标 ----
  bool   target_valid{false};
  double target_x{0.0};               ///< 基座系，台面上的物块中心
  double target_y{0.0};
  double target_yaw{0.0};
  BlockColor target_color{BlockColor::Unknown};
  float  target_confidence{0.0f};

  // ---- 视觉整体 ----
  bool   vision_ok{false};
  uint32_t blocks_visible{0};

  // ---- 气泵 ----
  bool   pump_online{false};
  double pump_pressure_kpa{0.0};
  uint8_t pump_fault{0};

  // ---- 机械臂 ----
  bool   arm_faulted{false};          ///< 任一关节处于 FAULT
  bool   ik_ok{true};                 ///< 上一个下达的目标是否被 arm_control 接受
  bool   estop{false};
};

// ---------------------------------------------------------------------------
//  状态机输出：每个控制周期由它驱动实际动作
// ---------------------------------------------------------------------------
struct Actuation
{
  /**
   * @brief 是否本次要下达一个新的末端目标。
   *
   * 为什么需要这个标志而不是"每周期都发一次目标"：arm_control 收到目标就会
   * 重新规划一条从当前位置到目标的轨迹。如果每个周期都发同一个目标，就会
   * 每 20ms 重新规划一次 —— 轨迹的起点一直在变（实测/指令值在动），
   * 规划出的五次多项式每次都不一样，末端会持续抖动，而且永远"到不了"。
   * 所以状态机只在**目标真的变了**的那一个周期置位。
   */
  bool   set_pose{false};
  double pose_x{0.0};
  double pose_y{0.0};
  double pose_z{0.0};
  double pose_yaw{0.0};
  /// 本次运动是否允许 arm_control 自动绕行（大范围移动时开，下压时必须关）
  bool   allow_waypoint{true};
  /// 时间缩放：精调阶段放慢
  float  speed_scale{1.0f};

  bool   pump_run{false};
  double pump_duty{0.0};

  int    select_block{-1};            ///< >=0 时要求切到该序号的物块（-1 = 不变）
  bool   cancel_block{false};
  bool   finished{false};
  bool   aborted{false};

  std::string note;                   ///< 写进 TaskStatus.message
};

}  // namespace hw_task

#endif  // HW_TASK__TASK_TYPES_HPP_
