// ============================================================================
//  task_sequencer.hpp
//  取放任务的状态机。纯逻辑，不依赖 rclcpp。
//
//  为什么要把状态机从 ROS 节点里拆出来
//  ----------------------------------
//  比赛现场最怕的是"流程跑飞了但没人知道卡在哪一步"。把状态机做成一个
//  只吃 SensorSnapshot、只吐 Actuation 的纯函数对象，意味着可以：
//    · 用录制下来的传感器序列离线回放，复现现场问题（test/replay_task.cpp）
//    · 对每个状态单独写单元测试，不用起 ROS、不用接硬件
//    · 在 PC 上跑蒙特卡洛式扰动测试（给投影加噪声，看会不会卡死）
//  这和固件里把 foc.c 从 motor.c 拆出来是同一个理由。
//
//  状态流转
//  --------
//    IDLE ─start─► SCAN ─找到目标─► APPROACH ─进入可抓区─► ALIGN ─对准─► SETTLE
//                   ▲                                                  │
//                   │                                               DESCEND
//                   │                                                  │
//                   │                                                GRASP
//                   │                                            ┌─────┴─────┐
//                   │                                        压力达标    超时/重试耗尽
//                   │                                            │           │
//                   │                                          LIFT       跳过该物块
//                   │                                            │           │
//                   │                                          HAUL ◄────────┘
//                   │                                            │
//                   │                                          PLACE ─对齐─► RELEASE
//                   │                                            ▲             │
//                   └──────────────── RETREAT ◄──────────────────┘          RETREAT
//                          (还有物块)                                        │
//                                                                       (没了) DONE
// ============================================================================
#ifndef HW_TASK__TASK_SEQUENCER_HPP_
#define HW_TASK__TASK_SEQUENCER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

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
  float  target_distance{-1.0f};
  float  pose_confidence{0.0f};
  bool   vision_ok{false};

  /// 当前目标（用于 GraspTarget.is_current）
  bool   has_target{false};
  double target_x{0.0};
  double target_y{0.0};
  double target_yaw{0.0};
  BlockColor target_color{BlockColor::Unknown};
  uint32_t target_track_id{0};

  /// 已放置的槽位序号（供界面显示进度条）
  uint8_t placed_slots{0};
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
  void bind_target(uint32_t track_id, const Eigen::Vector2d & pos,
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
   * @brief 位姿控制器：把车体开到指定的 (x, y, yaw)。
   *
   * 用"期望位姿 + 沿/横轨迹误差"的标准形式，而不是直接对着物块做视觉伺服。
   * 原因是本任务要求的不只是"开到物块上方"，还要"车体朝向与物块长边对齐"
   * （吸盘是长条形的）。位姿控制器天然同时满足这两点，而纯视觉伺服需要
   * 额外叠加一个朝向环 —— 两个环互相耦合，调起来更容易振荡。
   *
   * 误差定义（在**期望朝向**的坐标系里分解）：
   *     ex =  cos(θd)·Δx + sin(θd)·Δy    沿轨迹误差
   *     ey = -sin(θd)·Δx + cos(θd)·Δy    横轨迹误差
   *     eθ = wrap(θd - θ)
   * 控制律：
   *     v  = kx · ex
   *     ω  = kθ · eθ + ky · ey · sat(v/v_ref)     ← 横向误差只在车前进时才转，
   *                                                否则停车时会原地打转
   *
   * @param speed_cap 本状态允许的最大线速度（ALIGN 阶段要压得很低）
   * @param angular_cap 最大角速度
   */
  Actuation drive_to_pose(const SensorSnapshot & s, double tx, double ty, double tyaw,
                          double speed_cap, double angular_cap) const;

  /// 车体期望位姿 = 物块坐标 - standoff · [cos,sin](期望朝向)
  void target_to_vehicle_pose(double block_x, double block_y, double block_yaw,
                              double * out_x, double * out_y, double * out_yaw) const;

  /// 投放槽位坐标（按已放置数量在阵列里排）
  void place_slot(uint32_t slot_index, double * x, double * y, double * yaw) const;

  /// 是否停在期望位姿附近
  bool pose_settled(const SensorSnapshot & s, double tx, double ty, double tyaw) const;

  /// 气泵压力是否已达标（或用定时模式时的替代判据）
  bool vacuum_ok(const SensorSnapshot & s) const;

  // ---- 状态 ----
  TaskConfig    cfg_{};
  StartRequest  req_{};
  SequencerStatus st_{};
  std::vector<uint32_t> excluded_;

  double  t_state_{0.0};       ///< 进入当前状态的时刻
  double  t_task_{0.0};        ///< 任务开始的时刻
  double  t_last_{0.0};
  bool    have_last_{false};

  /// 中止流程标志。非零表示 RETREAT 结束后要走"回起点并终止"，
  /// 而不是"继续找下一个物块"。没有这个标志的话，abort(return_home=true)
  /// 会退化成"回到起点后重新开始扫描" —— 用户按了急停，车却继续干活。
  bool    aborting_{false};
  bool    abort_release_{false};

  /// 暂停起始时刻，用于恢复时把任务计时平移掉暂停时长
  double  t_paused_{0.0};

  /// 是否已经做过一轮"清空排除列表、从头再找一遍"的补扫。
  /// 吸不住的物块会被加进 excluded_，如果就这样结束，可能本来还有
  /// 能抓的物块没抓到。所以允许一轮补扫，但只允许一轮。
  bool    second_pass_done_{false};

  uint32_t placed_slots_{0};

  // 当前目标
  bool     has_target_{false};
  uint32_t target_track_id_{0};
  Eigen::Vector2d target_pos_{0.0, 0.0};
  double   target_yaw_{0.0};
  BlockColor target_color_{BlockColor::Unknown};
  float    target_conf_{0.0f};

  // 视觉丢失计时
  double t_vision_lost_{0.0};
  bool   vision_was_ok_{false};

  // 起始位姿（return_home 用）
  double home_x_{0.0}, home_y_{0.0}, home_yaw_{0.0};

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
