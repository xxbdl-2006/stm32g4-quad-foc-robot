// ============================================================================
//  task_sequencer.hpp
//  取放任务的状态机。纯逻辑，不依赖 rclcpp。
//
//  为什么要把状态机从 ROS 节点里拆出来
//  ----------------------------------
//  现场最怕的是"流程跑飞了但没人知道卡在哪一步"。把状态机做成一个
//  只吃 SensorSnapshot、只吐 Actuation 的纯函数对象，意味着可以：
//    · 用运动学积分器 + 假传感器在 PC 上把完整流程跑几百遍
//    · 对每个状态单独写单元测试，不用起 ROS、不用接硬件
//    · 给投影加噪声做扰动测试，看会不会卡死
//  这和固件里把 foc.c 从 motor.c 拆出来是同一个理由。
//
//  状态流转（<x,y,z> 表示下发给 arm_control 的末端目标点）
//  ------------------------------------------------------
//    IDLE ─start─► SCAN ─找到目标─► APPROACH <物块上方 hover_z>
//                     ▲                     │
//                     │                  ALIGN（视觉伺服精调）
//                     │                     │
//                     │                  SETTLE（停稳）
//                     │                     │
//                     │                  DESCEND <touch_z>（直线下压）
//                     │                     │
//                     │                   GRASP
//                     │            ┌────────┴────────┐
//                     │        压力达标          超时/重试耗尽
//                     │            │                 │
//                     │          LIFT <travel_z>   跳过该物块
//                     │            │                 │
//                     │          HAUL <投放位上方> ◄──┘
//                     │            │
//                     │          PLACE <place_z>
//                     │            │
//                     │          RELEASE
//                     │            │
//                     └── RETREAT <retreat_z> ◄──────┘
//                          (还有物块)
//                              │
//                          (没了) DONE
// ============================================================================
#ifndef HW_TASK__TASK_SEQUENCER_HPP_
#define HW_TASK__TASK_SEQUENCER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hw_task/task_types.hpp"

namespace hw_task
{

/// 供外部（ROS 节点）读取的完整状态
struct SequencerStatus
{
  TaskState state{TaskState::Idle};
  TaskState prev_state{TaskState::Idle};
  TaskState last_non_terminal{TaskState::Idle};

  uint32_t target_index{0};
  uint32_t total_targets{0};
  uint8_t  grasped_count{0};
  uint8_t  skipped_count{0};
  uint8_t  retry_count{0};
  float    elapsed{0.0f};

  std::string message;
  float  pump_pressure{0.0f};
  float  target_range{-1.0f};        ///< 物块到基座回转轴的距离（m），-1 表示无目标
  float  pose_confidence{0.0f};
  bool   vision_ok{false};

  /// 当前目标
  bool   has_target{false};
  double target_x{0.0};
  double target_y{0.0};
  double target_yaw{0.0};
  BlockColor target_color{BlockColor::Unknown};
  uint32_t target_track_id{0};

  /// 末端当前的目标点（已下达但可能还没走完）
  double cmd_x{0.0};
  double cmd_y{0.0};
  double cmd_z{0.0};

  /// 已放置的槽位序号（供界面显示进度）
  uint8_t placed_slots{0};
  /// 当前码垛层数（Array 模式恒为 0）
  uint8_t place_layer{0};
};

class TaskSequencer
{
public:
  TaskSequencer() = default;

  void configure(const TaskConfig & cfg);

  /**
   * @brief 启动任务。
   * @param req  启动参数
   * @param s    当前传感器快照（用来确定起始位姿与视野内物块数）
   * @return false 表示当前状态不允许启动（例如已经在跑）
   */
  bool start(const StartRequest & req, const SensorSnapshot & s);

  /// 中止。release=true 就地放气，false 保持吸住。
  bool abort(bool release, bool return_home, const SensorSnapshot & s);

  /// 暂停（可恢复）
  bool pause(const SensorSnapshot & s);
  bool resume(const SensorSnapshot & s);

  /**
   * @brief 状态机主循环。每个控制周期（建议 50 Hz）调用一次。
   * @param s   传感器快照
   * @param dt  距上次调用的时间（秒）
   */
  Actuation update(const SensorSnapshot & s, double dt);

  const SequencerStatus & status() const { return st_; }
  TaskState state() const { return st_.state; }
  bool running() const;

  /// 本次任务已经处理过的轨迹 id（ROS 节点用它给 tracker 做排除）
  const std::vector<uint32_t> & excluded_ids() const { return excluded_; }

  /// 切换当前目标（由 ROS 节点在 tracker 选出新目标后调用）
  void bind_target(uint32_t track_id, double x, double y,
                   BlockColor color, double yaw, float confidence);
  void clear_target();

private:
  // ---- 状态处理 ----
  Actuation st_idle(const SensorSnapshot &, double);
  Actuation st_scan(const SensorSnapshot &, double);
  Actuation st_approach(const SensorSnapshot &, double);
  Actuation st_align(const SensorSnapshot &, double);
  Actuation st_settle(const SensorSnapshot &, double);
  Actuation st_descend(const SensorSnapshot &, double);
  Actuation st_grasp(const SensorSnapshot &, double);
  Actuation st_lift(const SensorSnapshot &, double);
  Actuation st_haul(const SensorSnapshot &, double);
  Actuation st_place(const SensorSnapshot &, double);
  Actuation st_release(const SensorSnapshot &, double);
  Actuation st_retreat(const SensorSnapshot &, double);

  // ---- 内部工具 ----
  void transition(TaskState next, const std::string & note);
  Actuation hold(const std::string & note) const;
  Actuation stop_all(const std::string & note) const;
  Actuation with_pump(Actuation a, double duty) const;

  /**
   * @brief 下达一个新的末端目标位姿。
   *
   * **只在下达点与上一次不同的时候才置 set_pose**。每周期重复下发同一个
   * 目标会让 arm_control 每 20ms 重新规划一次轨迹（起点一直在动），
   * 末端反而永远停不下来 —— 这个坑在底盘方案里不存在（速度指令可以重复
   * 下发），换到机械臂方案上就变成了必须处理的问题。
   *
   * @param z 末端高度。给 -1 表示"保持当前高度不变"
   */
  Actuation goto_pose(double x, double y, double z, double yaw,
                      bool allow_waypoint, float speed_scale,
                      const std::string & note);

  /// 末端是否已经停在 (x, y, z) 附近并停稳
  bool tool_settled(const SensorSnapshot & s, double x, double y, double z) const;

  /// 物块是否在工作半径环带内
  bool reachable(double x, double y) const;

  /// 投放槽位坐标（含码垛高度）。Array 模式只用 slot 的 0 层。
  void place_slot(uint32_t placed_count, double * x, double * y,
                  double * yaw, double * z) const;

  /// 气泵压力是否已达标（或用定时模式时的替代判据）
  bool vacuum_ok(const SensorSnapshot & s) const;

  /// 把内部分解出来的当前"抓取点 + 高度"写进 Actuation（供各状态复用）
  Actuation goto_grasp_point(double z, bool allow_waypoint, float speed_scale,
                             const std::string & note);

  // ---- 状态 ----
  TaskConfig    cfg_{};
  StartRequest  req_{};
  SequencerStatus st_{};
  std::vector<uint32_t> excluded_;

  double  t_state_{0.0};       ///< 进入当前状态的时刻
  double  t_task_{0.0};        ///< 任务开始的时刻
  double  t_last_{0.0};
  bool    have_last_{false};

  /// 中止流程标志。非零表示 RETREAT 结束后要走"回原点并终止"，
  /// 而不是"继续找下一个物块"。没有这个标志的话，abort(return_home=true)
  /// 会退化成"回原点后重新开始扫描" —— 用户按了急停，机械臂却继续干活。
  bool    aborting_{false};
  bool    abort_release_{false};

  /// 暂停起始时刻，用于恢复时把两个计时器平移掉暂停时长
  double  t_paused_{0.0};

  /// 是否已经做过一轮"清空排除列表、从头再找一遍"的补扫。
  /// 吸不住的物块会被加进 excluded_，如果就这样结束，可能本来还有
  /// 能抓的物块没抓到。所以允许一轮补扫，但只允许一轮。
  bool    second_pass_done_{false};

  uint32_t placed_slots_{0};

  // ---- 已下达的末端目标（用于"只在下达点变化时才发新目标"）----
  bool   cmd_valid_{false};
  double cmd_x_{0.0};
  double cmd_y_{0.0};
  double cmd_z_{0.0};
  double cmd_yaw_{0.0};

  // ---- 当前目标 ----
  bool     has_target_{false};
  uint32_t target_track_id_{0};
  double   target_x_{0.0};
  double   target_y_{0.0};
  double   target_yaw_{0.0};
  BlockColor target_color_{BlockColor::Unknown};
  float    target_conf_{0.0f};

  // 视觉丢失计时
  double t_vision_lost_{0.0};
  bool   vision_was_ok_{false};

  // 起始位姿（return_home 用）
  double home_x_{0.0}, home_y_{0.0}, home_z_{0.1}, home_yaw_{0.0};

  // 上一次的压力（用于检测搬运途中掉件）
  double last_pressure_{0.0};
  bool   payload_attached_{false};

  // 连败计数（连续跳过多个物块后降级为 FAILED，避免无限循环）
  uint32_t consecutive_skips_{0};
  static constexpr uint32_t kMaxConsecutiveSkips = 5;

  /* 每个阶段各自的重试计数。
   * 共用同一个计数器会出事：GRASP 失败后的重试路径要经过 ALIGN，
   * 而 ALIGN 成功时会清零计数器 —— 于是 GRASP 的重试上限永远达不到。
   * 四个阶段的目标互不相同，计数本来就不该互相干扰。 */
  uint32_t scan_fail_{0};
  uint32_t approach_fail_{0};
  uint32_t align_fail_{0};
  uint32_t grasp_fail_{0};
};

}  // namespace hw_task

#endif  // HW_TASK__TASK_SEQUENCER_HPP_
