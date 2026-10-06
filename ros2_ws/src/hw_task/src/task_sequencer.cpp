// ============================================================================
//  task_sequencer.cpp
// ============================================================================
#include "hw_task/task_sequencer.hpp"

#include <algorithm>
#include <cmath>

namespace hw_task
{

namespace
{
/// 本文件里所有三角运算统一用这个常量。
/// 不用 <cmath> 里的 M_PI：glibc 只在 __USE_MISC 下导出它，
/// 用 `-std=c++17`（严格模式）编译时它会消失 —— 而 gnu++17 又有。
/// 自带的常量不会有这个坑，也能保证主机侧测试与目标机行为一致。
constexpr double kPi = 3.14159265358979323846;

/// 判定"下达点是否变了"的死区。1e-5 m = 10 µm，小于任何机械重复定位精度，
/// 又大于 double 在 0.1~0.5 m 量级上的舍入误差（~1e-16），不会因为
/// 浮点噪声反复重发目标。
constexpr double kPoseEps = 1e-5;

double wrap_pi(double a)
{
  while (a > kPi) { a -= 2.0 * kPi; }
  while (a <= -kPi) { a += 2.0 * kPi; }
  return a;
}
}  // namespace

// ---------------------------------------------------------------------------
const char * to_string(TaskState s)
{
  switch (s) {
    case TaskState::Idle:     return "IDLE";
    case TaskState::Scan:     return "SCAN";
    case TaskState::Approach: return "APPROACH";
    case TaskState::Align:    return "ALIGN";
    case TaskState::Settle:   return "SETTLE";
    case TaskState::Descend:  return "DESCEND";
    case TaskState::Grasp:    return "GRASP";
    case TaskState::Lift:     return "LIFT";
    case TaskState::Haul:     return "HAUL";
    case TaskState::Place:    return "PLACE";
    case TaskState::Release:  return "RELEASE";
    case TaskState::Retreat:  return "RETREAT";
    case TaskState::Done:     return "DONE";
    case TaskState::Failed:   return "FAILED";
    case TaskState::Paused:   return "PAUSED";
  }
  return "?";
}

const char * to_string(BlockColor c)
{
  switch (c) {
    case BlockColor::Red:   return "红";
    case BlockColor::Blue:  return "蓝";
    case BlockColor::Green: return "绿";
    default:                return "未知";
  }
}

// ---------------------------------------------------------------------------
void TaskSequencer::configure(const TaskConfig & cfg)
{
  cfg_ = cfg;
}

bool TaskSequencer::running() const
{
  return st_.state != TaskState::Idle &&
         st_.state != TaskState::Done &&
         st_.state != TaskState::Failed;
}

// ---------------------------------------------------------------------------
void TaskSequencer::transition(TaskState next, const std::string & note)
{
  st_.prev_state = st_.state;
  st_.state = next;
  st_.message = note;
  t_state_ = t_last_;

  if (next != TaskState::Paused && next != TaskState::Idle &&
      next != TaskState::Done && next != TaskState::Failed) {
    st_.last_non_terminal = next;
  }

  /* 这里**刻意不**自动清 retry_count。
   * 曾经的写法是"只要目标状态和上一个状态不同就把重试计数清零"，
   * 结果 GRASP 失败 -> ALIGN 重试 -> GRASP 这一圈每轮都清零，
   * max_grasp_retry 永远触发不了，吸不住的物块会让状态机无限循环。
   * 现在改为由各状态在"确实取得进展"时显式清零：
   *   SCAN 找到目标、ALIGN 判定对准完成、GRASP 判定吸住。 */
}

Actuation TaskSequencer::hold(const std::string & note) const
{
  Actuation a;
  a.note = note;
  // 保持位置：把最后下达过的目标再"说"一遍，但 set_pose 为 false，
  // 所以 arm_control 不会重新规划，机械臂就停在原地。
  a.pose_x = cmd_x_;
  a.pose_y = cmd_y_;
  a.pose_z = cmd_z_;
  a.pose_yaw = cmd_yaw_;
  a.speed_scale = 1.0f;
  return a;
}

Actuation TaskSequencer::stop_all(const std::string & note) const
{
  Actuation a = hold(note);
  a.pump_run = false;
  a.pump_duty = 0.0;
  return a;
}

Actuation TaskSequencer::with_pump(Actuation a, double duty) const
{
  a.pump_run = duty > 0.001;
  a.pump_duty = duty;
  return a;
}

// ---------------------------------------------------------------------------
//  目标下达
// ---------------------------------------------------------------------------
Actuation TaskSequencer::goto_pose(double x, double y, double z, double yaw,
                                   bool allow_waypoint, float speed_scale,
                                   const std::string & note)
{
  Actuation a;
  a.note = note;

  if (z < 0.0) { z = cmd_z_; }   // -1 = 保持当前高度

  const bool changed = !cmd_valid_ ||
                       std::fabs(x - cmd_x_) > kPoseEps ||
                       std::fabs(y - cmd_y_) > kPoseEps ||
                       std::fabs(z - cmd_z_) > kPoseEps ||
                       std::fabs(wrap_pi(yaw - cmd_yaw_)) > kPoseEps;

  if (changed) {
    a.set_pose = true;
    a.pose_x = x;
    a.pose_y = y;
    a.pose_z = z;
    a.pose_yaw = yaw;
    a.allow_waypoint = allow_waypoint;
    a.speed_scale = speed_scale;
    cmd_valid_ = true;
    cmd_x_ = x;
    cmd_y_ = y;
    cmd_z_ = z;
    cmd_yaw_ = yaw;
  }
  return a;
}

Actuation TaskSequencer::goto_grasp_point(double z, bool allow_waypoint,
                                          float speed_scale,
                                          const std::string & note)
{
  /* 末端朝向恒为 0。吸盘是圆形真空吸盘，物块朝向与抓取无关 —— 这也是本机构
   * 不需要第四个回转轴、可以把它做成升降轴的原因。物块的 yaw 只用于显示。 */
  return goto_pose(target_x_, target_y_, z, 0.0, allow_waypoint, speed_scale, note);
}

bool TaskSequencer::tool_settled(const SensorSnapshot & s,
                                 double x, double y, double z) const
{
  const double dx = s.tool_x - x;
  const double dy = s.tool_y - y;
  const double dz = s.tool_z - z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  /* 两个条件缺一不可：
   *   · 位置进入容差 —— 到位；
   *   · 合成速度低于容差 —— 停稳。
   * 只看位置会误判：末端以 0.2 m/s 掠过目标点时只有一两个控制周期落在
   * 容差球内，那时下达"下压"会让轨迹在下压中途被新目标打断。 */
  return dist < cfg_.pos_tol && s.tool_speed() < cfg_.vel_tol;
}

bool TaskSequencer::reachable(double x, double y) const
{
  const double r = std::sqrt(x * x + y * y);
  return (r >= cfg_.reach_radius_min) && (r <= cfg_.reach_radius_max);
}

void TaskSequencer::place_slot(uint32_t placed, double * x, double * y,
                               double * yaw, double * z) const
{
  /* 槽位编号规则：
   *   阵列模式 —— 每放一个换一个槽位，都在台面上（z = touch_z）
   *   码垛模式 —— 同一个槽位往上叠 place_max_layers 层，然后再换一个槽位
   *
   * 码垛的可行性来自 J4 是升降轴：底盘方案里没有 Z 自由度，只能排放阵列。
   * 但码垛对 Z 的绝对精度敏感 —— 每一层的落点误差都会累积。这里靠
   * touch_z 留的 2mm 过盈量吸收：宁可压得比理论值低一点，也不要悬空释放。 */
  const int cols = std::max(1, cfg_.place_columns);
  const uint32_t max_layers =
    (cfg_.place_mode == PlaceMode::Stack)
      ? static_cast<uint32_t>(std::max(1, cfg_.place_max_layers)) : 1U;

  const uint32_t layer = (cfg_.place_mode == PlaceMode::Stack) ? (placed % max_layers) : 0U;
  const uint32_t slot  = (cfg_.place_mode == PlaceMode::Stack) ? (placed / max_layers) : placed;

  const int col = static_cast<int>(slot) % cols;
  const int row = static_cast<int>(slot) / cols;
  const double along = col * cfg_.place_pitch;
  const double side = row * cfg_.place_row_pitch;

  // 阵列沿投放区朝向铺开
  const double c = std::cos(cfg_.place_yaw);
  const double s = std::sin(cfg_.place_yaw);
  *x = cfg_.place_x + along * c - side * s;
  *y = cfg_.place_y + along * s + side * c;
  *yaw = cfg_.place_yaw;
  *z = cfg_.touch_z + static_cast<double>(layer) * cfg_.place_layer_height;
}

bool TaskSequencer::vacuum_ok(const SensorSnapshot & s) const
{
  if (!cfg_.use_pressure_check) {
    // 退化为 demo 的纯定时判断：DESCEND 结束时就算抓住了。
    // 这条路径只在没有压力传感器时用 —— 它会漏判"压在物块边缘上吸空"的情况。
    return true;
  }
  if (!s.pump_online) {
    return false;
  }
  // 真空度约定：压力传感器读的是表压，抽真空后为负值（或低于环境压力）。
  // 阈值用绝对值比较，避免不同传感器的零点约定差异。
  return std::fabs(s.pump_pressure_kpa) >= cfg_.vacuum_threshold_kpa;
}

// ---------------------------------------------------------------------------
//  启动 / 中止 / 暂停
// ---------------------------------------------------------------------------
bool TaskSequencer::start(const StartRequest & req, const SensorSnapshot & s)
{
  if (running()) {
    return false;
  }
  if (s.arm_faulted || s.estop) {
    transition(TaskState::Failed, "机械臂有故障或处于急停，拒绝启动");
    return false;
  }

  req_ = req;
  excluded_.clear();
  placed_slots_ = 0;
  has_target_ = false;
  payload_attached_ = false;
  consecutive_skips_ = 0;
  t_vision_lost_ = 0.0;
  vision_was_ok_ = false;
  have_last_ = false;
  aborting_ = false;
  abort_release_ = false;
  second_pass_done_ = false;
  scan_fail_ = approach_fail_ = align_fail_ = grasp_fail_ = 0;

  /* 起点取配置里的 home，而不是"当前末端位姿"。
   * 原因是结束时要回到一个**确定**的位置：如果拿当前位姿当 home，那么
   * "回原点"就变成了"回到任务开始时那个时刻碰巧待着的地方"，
   * 换一个开始时机就换一个终点，现场没法预设料框位置。 */
  home_x_ = cfg_.home_x;
  home_y_ = cfg_.home_y;
  home_z_ = cfg_.home_z;
  home_yaw_ = cfg_.home_yaw;

  cmd_valid_ = false;

  st_ = SequencerStatus{};
  st_.total_targets = req.max_blocks;
  st_.pump_pressure = static_cast<float>(s.pump_pressure_kpa);
  t_task_ = s.t;
  t_last_ = s.t;
  t_state_ = s.t;

  transition(TaskState::Scan,
             "任务启动，开始扫描物块（最多 " +
             (req.max_blocks ? std::to_string(req.max_blocks) : std::string("不限")) +
             " 个）");
  return true;
}

bool TaskSequencer::abort(bool release, bool return_home, const SensorSnapshot & s)
{
  if (!running()) {
    return false;
  }

  aborting_ = true;
  abort_release_ = release;

  if (return_home) {
    payload_attached_ = false;
    transition(TaskState::Retreat, "中止：正在返回原点");
    return true;
  }

  if (release) {
    payload_attached_ = false;
    transition(TaskState::Failed, "中止：已就地释放负载");
  } else {
    transition(TaskState::Failed, "中止：保持吸住并停止");
  }
  st_.elapsed = static_cast<float>(s.t - t_task_);
  return true;
}

bool TaskSequencer::pause(const SensorSnapshot & s)
{
  if (!running()) { return false; }
  st_.last_non_terminal = st_.state;
  t_paused_ = s.t;
  transition(TaskState::Paused, "暂停");
  st_.elapsed = static_cast<float>(s.t - t_task_);
  return true;
}

bool TaskSequencer::resume(const SensorSnapshot & s)
{
  if (st_.state != TaskState::Paused) { return false; }
  const TaskState back = st_.last_non_terminal;

  /* 把暂停的时长从两个计时器里扣掉，否则恢复后状态会立刻超时。
   * 之前这里是 t_state_ = s.t 之后再 t_task_ += (s.t - t_state_)，
   * 而 t_state_ 已经被改成 s.t，括号里恒为 0 —— 等于没补偿。
   * 现在先把差值算出来再赋值。 */
  const double paused_span = s.t - t_paused_;
  t_state_ += paused_span;
  t_task_  += paused_span;

  transition(back, "恢复");
  return true;
}

// ---------------------------------------------------------------------------
//  目标绑定
// ---------------------------------------------------------------------------
void TaskSequencer::bind_target(uint32_t track_id, double x, double y,
                                BlockColor color, double yaw, float confidence)
{
  // 换目标时如果已经吸着东西，说明上层逻辑有问题 —— 不要静默覆盖
  if (has_target_ && target_track_id_ != track_id && payload_attached_) {
    return;
  }
  has_target_ = true;
  target_track_id_ = track_id;
  target_x_ = x;
  target_y_ = y;
  target_yaw_ = yaw;
  target_color_ = color;
  target_conf_ = confidence;
  st_.has_target = true;
  st_.target_track_id = track_id;
}

void TaskSequencer::clear_target()
{
  has_target_ = false;
  st_.has_target = false;
}

// ---------------------------------------------------------------------------
//  主循环
// ---------------------------------------------------------------------------
Actuation TaskSequencer::update(const SensorSnapshot & s, double dt)
{
  if (!have_last_) {
    t_last_ = s.t;
    have_last_ = true;
  }
  const double eff_dt = (dt > 0.0) ? dt : std::max(0.0, s.t - t_last_);
  t_last_ = s.t;

  // ---- 全局急停：任何状态下都最高优先 ----
  if (s.estop) {
    if (st_.state != TaskState::Failed) {
      transition(TaskState::Failed, "急停触发，任务终止");
    }
    return stop_all("急停");
  }

  // ---- 关节故障：搬着东西也要先放下，否则一直吸着会过热 ----
  if (s.arm_faulted && st_.state != TaskState::Idle &&
      st_.state != TaskState::Done && st_.state != TaskState::Failed) {
    payload_attached_ = false;
    transition(TaskState::Failed, "机械臂关节故障，任务终止（负载已释放）");
    return stop_all("关节故障");
  }

  st_.elapsed = static_cast<float>(s.t - t_task_);
  st_.pump_pressure = static_cast<float>(s.pump_pressure_kpa);
  st_.vision_ok = s.vision_ok;

  // ---- 视觉在线/离线计时（供各状态判断）----
  if (s.vision_ok) {
    vision_was_ok_ = true;
    t_vision_lost_ = 0.0;
  } else if (vision_was_ok_) {
    t_vision_lost_ += eff_dt;
  }

  // ---- 更新对外可见的目标信息 ----
  if (has_target_) {
    st_.target_range = static_cast<float>(std::sqrt(target_x_ * target_x_ +
                                                   target_y_ * target_y_));
    st_.target_x = target_x_;
    st_.target_y = target_y_;
    st_.target_yaw = target_yaw_;
    st_.target_color = target_color_;
    st_.has_target = true;
  } else {
    st_.target_range = -1.0f;
    st_.has_target = false;
  }
  st_.cmd_x = cmd_x_;
  st_.cmd_y = cmd_y_;
  st_.cmd_z = cmd_z_;

  // ---- 搬运途中掉件检测：吸住之后压力突然消失 ----
  if (payload_attached_ && cfg_.use_pressure_check) {
    const bool lost = std::fabs(s.pump_pressure_kpa) <
                      (cfg_.vacuum_threshold_kpa * 0.5);
    if (lost && (st_.state == TaskState::Lift || st_.state == TaskState::Haul ||
                 st_.state == TaskState::Place)) {
      /* 掉件了。物块落在工作台上某处、位置未知，只能放弃当前目标重新扫描。
       * 注意这里**不**去"捡回掉落的物块"：它多半躺在工作台中间、姿态也变了，
       * 与其去处理这种不确定状态，不如把它排除掉先完成配额。 */
      payload_attached_ = false;
      if (has_target_) { excluded_.push_back(target_track_id_); }
      clear_target();
      st_.skipped_count++;
      consecutive_skips_++;
      // 换了目标，各阶段的重试计数都重新开始
      scan_fail_ = approach_fail_ = align_fail_ = grasp_fail_ = 0;

      // 同样注意：aborted 必须挂在返回的那个对象上（见 st_grasp 里的注释）
      Actuation drop_stop = stop_all("搬运途中掉件");
      if (consecutive_skips_ >= kMaxConsecutiveSkips) {
        transition(TaskState::Failed, "连续掉件，任务终止");
        drop_stop.aborted = true;
        return drop_stop;
      }
      transition(TaskState::Scan, "搬运途中掉件，已放弃该物块，重新扫描");
      return drop_stop;
    }
    last_pressure_ = s.pump_pressure_kpa;
  }

  // ---- 分派 ----
  Actuation out;
  switch (st_.state) {
    case TaskState::Idle:     out = st_idle(s, eff_dt);     break;
    case TaskState::Scan:     out = st_scan(s, eff_dt);     break;
    case TaskState::Approach: out = st_approach(s, eff_dt); break;
    case TaskState::Align:    out = st_align(s, eff_dt);    break;
    case TaskState::Settle:   out = st_settle(s, eff_dt);   break;
    case TaskState::Descend:  out = st_descend(s, eff_dt);  break;
    case TaskState::Grasp:    out = st_grasp(s, eff_dt);    break;
    case TaskState::Lift:     out = st_lift(s, eff_dt);     break;
    case TaskState::Haul:     out = st_haul(s, eff_dt);     break;
    case TaskState::Place:    out = st_place(s, eff_dt);    break;
    case TaskState::Release:  out = st_release(s, eff_dt);  break;
    case TaskState::Retreat:  out = st_retreat(s, eff_dt);  break;
    case TaskState::Paused:
      out = hold("已暂停");
      break;
    case TaskState::Done:
    case TaskState::Failed:
    default:
      out = stop_all(st_.message);
      break;
  }

  // ---- 位姿置信度：用视觉在线性与目标可见性合成 ----
  {
    float conf = 0.0f;
    if (s.vision_ok) { conf += 0.5f; }
    if (has_target_) { conf += 0.5f * target_conf_; }
    if (t_vision_lost_ > cfg_.vision_lost_timeout * 0.5) { conf *= 0.5f; }
    st_.pose_confidence = conf;
  }

  // ---- 把"当前阶段"的重试计数映射到对外可见的 retry_count ----
  switch (st_.state) {
    case TaskState::Scan:     st_.retry_count = static_cast<uint8_t>(scan_fail_);     break;
    case TaskState::Approach: st_.retry_count = static_cast<uint8_t>(approach_fail_); break;
    case TaskState::Align:    st_.retry_count = static_cast<uint8_t>(align_fail_);    break;
    case TaskState::Grasp:
    case TaskState::Lift:     st_.retry_count = static_cast<uint8_t>(grasp_fail_);    break;
    default: break;
  }

  // ---- 消息前缀：把状态与关键量拼成一行，方便现场看日志 ----
  out.note = std::string("[") + to_string(st_.state) + "] " + out.note;
  st_.message = out.note;
  return out;
}

// ---------------------------------------------------------------------------
//  IDLE
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_idle(const SensorSnapshot &, double)
{
  Actuation a = stop_all("待机");
  a.select_block = -1;
  return a;
}

// ---------------------------------------------------------------------------
//  SCAN：把末端停到待机位（避开相机视野），等视觉给出稳定目标
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_scan(const SensorSnapshot & s, double dt)
{
  (void)dt;

  // 找到稳定目标 -> 进入 APPROACH。目标由 ROS 节点通过 bind_target 送进来，
  // 所以这里只判断"有没有目标"。
  if (has_target_ && s.target_valid) {
    if (!reachable(target_x_, target_y_)) {
      /* 视觉给出的物块在工作半径环带之外。不去抓它，也不把它当成失败 ——
       * 它可能只是被放在工作台边上。加进排除列表换下一个。 */
      excluded_.push_back(target_track_id_);
      clear_target();
      st_.skipped_count++;
      Actuation a = hold("目标超出工作半径，跳过");
      a.select_block = -1;
      return a;
    }

    // 扫描有结果了：新一轮接近/对准/抓取，把后面几个计数全部清零
    scan_fail_ = 0;
    approach_fail_ = 0;
    align_fail_ = 0;
    grasp_fail_ = 0;
    transition(TaskState::Approach,
               "锁定" + std::string(to_string(target_color_)) + "色物块，距基座 " +
               std::to_string(st_.target_range).substr(0, 4) + "m");
    return hold("锁定目标");
  }

  /* 扫描期间把末端收到待机位（= 原点）。
   * 这一步不是可有可无的：相机是固定安装在工作台上方的（eye-to-hand），
   * 机械臂如果停在视野中间会把物块挡住，视觉侧永远等不到稳定目标 ——
   * 症状是"卡在 SCAN 不动"，看起来像视觉坏了，其实是自己挡了镜头。 */
  Actuation a = goto_pose(home_x_, home_y_, home_z_, home_yaw_,
                          /*allow_waypoint=*/true, 1.0f, "扫描：末端回到待机位");
  a.select_block = -1;   // 请求 ROS 节点挑选目标

  if ((s.t - t_state_) > cfg_.scan_timeout) {
    scan_fail_++;
    if (scan_fail_ >= cfg_.max_scan_retry) {
      /* 配额还没完成，但视野里已经找不到能抓的物块。两种可能：
       *   · 之前有几块因为"吸不住/接近超时"被排除了，但它们其实还在台上
       *   · 台上的物块确实都被抓完了
       * 所以先清空排除列表补扫一轮，还是找不到就收工。
       * 只允许补扫一轮，避免"反复扫不到又反复补扫"的死循环。 */
      const bool quota_left = (req_.max_blocks == 0) ||
                              (placed_slots_ < req_.max_blocks);
      if (quota_left && !second_pass_done_ && !excluded_.empty()) {
        excluded_.clear();
        second_pass_done_ = true;
        scan_fail_ = 0;
        t_state_ = s.t;
        a.note = "首次扫描未果，清空排除列表补扫一轮";
        return a;
      }

      if (st_.grasped_count > 0 || placed_slots_ > 0) {
        transition(TaskState::Done,
                   "工作台上已无可用物块，任务结束（放置 " +
                   std::to_string(placed_slots_) + " 个，跳过 " +
                   std::to_string(st_.skipped_count) + " 个）");
        Actuation fin = stop_all("任务完成");
        fin.finished = true;
        return fin;
      }
      transition(TaskState::Failed, "扫描超时且未发现任何物块，任务失败");
      Actuation fin = stop_all("扫描失败");
      fin.aborted = true;
      return fin;
    }
    t_state_ = s.t;   // 重新开始一轮扫描
    a.note = "扫描超时，第 " + std::to_string(scan_fail_ + 1) + " 轮重试";
  }
  return a;
}

// ---------------------------------------------------------------------------
//  APPROACH：末端移动到物块上方的悬停位（允许绕行）
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_approach(const SensorSnapshot & s, double dt)
{
  (void)dt;

  // ---- 视觉丢失 ----
  if (t_vision_lost_ > cfg_.vision_lost_timeout) {
    approach_fail_++;
    clear_target();
    if (approach_fail_ > cfg_.max_align_retry) {
      transition(TaskState::Failed, "接近过程中持续丢失视觉，任务终止");
      Actuation fin = stop_all("丢失视觉");
      fin.aborted = true;
      return fin;
    }
    transition(TaskState::Scan, "接近过程中丢失目标，退回扫描");
    return hold("丢失目标");
  }

  // ---- 目标失效（被 tracker 删掉或换人）----
  if (!has_target_ || !s.target_valid) {
    transition(TaskState::Scan, "目标失效，退回扫描");
    return hold("目标失效");
  }

  /* ---- 目标被 arm_control 拒绝 ----
   * arm_control 在规划前会做可达性自检，不可达的目标会在 ik_ok 上报 false。
   * 这是"视觉认为可达但运动学上不可达"的情形（比如投影误差把物块投到了
   * 内孔里），必须当成不可抓处理，否则会一直在原地重试。 */
  if (!s.ik_ok) {
    excluded_.push_back(target_track_id_);
    clear_target();
    st_.skipped_count++;
    consecutive_skips_++;
    Actuation a = hold("目标超出机械臂可达空间，跳过该物块");
    a.select_block = -1;
    return a;
  }

  Actuation a = goto_grasp_point(cfg_.hover_z, /*allow_waypoint=*/true, 1.0f,
                                 "移动到物块上方");

  if (tool_settled(s, cmd_x_, cmd_y_, cmd_z_)) {
    approach_fail_ = 0;
    transition(TaskState::Align, "已到达悬停位，开始视觉精调");
    return hold("到达悬停位");
  }

  if ((s.t - t_state_) > cfg_.approach_timeout) {
    approach_fail_++;
    if (approach_fail_ > cfg_.max_align_retry) {
      // 超时通常意味着物块在不可达位置（工作台边缘、被夹具挡住）
      excluded_.push_back(target_track_id_);
      clear_target();
      st_.skipped_count++;
      consecutive_skips_++;
      transition(TaskState::Scan, "接近超时，跳过该物块");
      return hold("接近超时");
    }
    transition(TaskState::Scan, "接近超时，重新定位");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  ALIGN：视觉伺服精调
//
//  与底盘方案最大的不同：底盘是靠"移动车体"把吸盘挪到物块上，误差源是
//  行驶与滑移；机械臂这里是**用最新一帧视觉结果刷新末端目标**，直到
//  目标不再变化。判据因此从一个"位置误差"变成了"目标估计的收敛性" ——
//  因为机械臂有能力把末端精确送到任何给定点，真正不确定的是"物块到底在哪儿"。
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_align(const SensorSnapshot & s, double dt)
{
  (void)dt;

  if (!has_target_ || !s.target_valid) {
    if (t_vision_lost_ > cfg_.vision_lost_timeout * 0.5) {
      align_fail_++;
      transition(TaskState::Scan, "精调过程中丢失目标");
      return hold("丢失目标");
    }
    return hold("等待视觉恢复");
  }

  const double dx = target_x_ - cmd_x_;
  const double dy = target_y_ - cmd_y_;
  const double residual = std::sqrt(dx * dx + dy * dy);

  /* 残差已经收敛：最新估计与已下达目标之差在阈值内，说明视觉不再带来
   * 新信息（要么物块真的在那儿，要么只是噪声在 1mm 上下抖）。
   * 此时再等末端走完，然后进入停稳。 */
  if (residual < cfg_.servo_tol && tool_settled(s, cmd_x_, cmd_y_, cfg_.hover_z)) {
    align_fail_ = 0;              // 对准成功，只清对准阶段自己的计数
    transition(TaskState::Settle, "视觉收敛且末端到位，停稳等待");
    return hold("对准完成");
  }

  /* 精调用小幅直线运动 + 降速。allow_waypoint 必须关掉：绕行会让末端先抬
   * 到安全高度再回来，而这时候末端离物块只有几毫米 —— 抬起来再压回去
   * 反而更容易碰倒物块。 */
  Actuation a = goto_grasp_point(cfg_.hover_z, /*allow_waypoint=*/false, 0.4f,
                                 "视觉精调 残差 " +
                                 std::to_string(residual * 1000.0).substr(0, 4) + "mm");

  if ((s.t - t_state_) > cfg_.align_timeout) {
    align_fail_++;
    if (align_fail_ > cfg_.max_align_retry) {
      excluded_.push_back(target_track_id_);
      clear_target();
      st_.skipped_count++;
      consecutive_skips_++;
      transition(TaskState::Scan, "精调反复失败，跳过该物块");
      return hold("精调失败");
    }
    transition(TaskState::Scan, "精调超时，重新接近");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  SETTLE：停稳，等谐波减速器的残余振动衰减
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_settle(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = hold("停稳等待");

  /* 为什么必须等：谐波减速器有柔性，末端到位后还会以 10~30Hz 的频带余振
   * 零点几秒。带着余振下压会让吸盘在物块表面蹭，压在边缘上就吸空了 ——
   * 而"吸空了"要到 LIFT 之后才发现，浪费一个完整周期。 */
  if ((s.t - t_state_) >= cfg_.settle_time) {
    transition(TaskState::Descend, "开始下压到物块顶面");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  DESCEND：直线下压到物块顶面
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_descend(const SensorSnapshot & s, double dt)
{
  (void)dt;

  /* 必须走直线（allow_waypoint=false）：
   * 下压段如果走弧线，吸盘会沿物块表面滑动，压力分布偏离中心，容易漏气。
   * 这也正是 arm_control 里 CartesianMove 存在的理由。 */
  Actuation a = goto_grasp_point(cfg_.touch_z, /*allow_waypoint=*/false, 0.5f,
                                 "下压到物块顶面");

  const bool arrived = tool_settled(s, cmd_x_, cmd_y_, cfg_.touch_z);
  if (arrived && (s.t - t_state_) >= cfg_.descend_time) {
    transition(TaskState::Grasp,
               "等待真空建立（阈值 " +
               std::to_string(cfg_.vacuum_threshold_kpa).substr(0, 4) + "kPa）");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  GRASP：开泵并判定是否吸住
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_grasp(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = with_pump(hold("判定吸附"), cfg_.grasp_duty);

  if (vacuum_ok(s)) {
    payload_attached_ = true;
    grasp_fail_ = 0;
    consecutive_skips_ = 0;
    transition(TaskState::Lift, "已吸住，抬升到搬运高度");
    return a;
  }

  if ((s.t - t_state_) > cfg_.grasp_timeout) {
    grasp_fail_++;
    if (grasp_fail_ >= cfg_.max_grasp_retry) {
      /* 吸不住：可能是物块倾斜、表面不平、吸盘压不到位。
       * 加进排除列表先去抓别的 —— 换一个物块抓成功的概率比在原地
       * 反复试要高。如果最后配额没完成，SCAN 的补扫机制会清空排除列表
       * 再来一轮，那时物块可能已经被旁边的东西挪动过、角度也变了。 */
      if (has_target_) { excluded_.push_back(target_track_id_); }
      clear_target();
      st_.skipped_count++;
      consecutive_skips_++;
      /* 注意别把 aborted 标志挂在 a 上再返回 stop_all() ——
       * stop_all() 会新建一个 Actuation，挂在 a 上的标志就丢了。
       * 之前正是这么写的，结果"连续吸不住"进了 FAILED 状态但
       * 没有把 aborted 报给上层，上层以为任务还在跑。 */
      Actuation stop = stop_all("跳过物块");
      if (consecutive_skips_ >= kMaxConsecutiveSkips) {
        transition(TaskState::Failed, "连续多块吸不住，任务终止（检查气路与吸盘）");
        stop.aborted = true;
        stop.note = "连续吸不住";
      } else {
        transition(TaskState::Scan, "吸不住，跳过该物块");
      }
      return stop;
    }
    // 重试：回 ALIGN 重新对位（比原地再抽一次成功率高）
    transition(TaskState::Align, "吸附失败，重新对位后重试");
    return stop_all("重新对位");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  LIFT：抬到搬运高度，确认真的离地
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_lift(const SensorSnapshot & s, double dt)
{
  (void)dt;

  /* 抬升用直线（不走绕行）。虽然不是"压"的动作，但抬升时末端贴着物块，
   * 任何横向位移都会把物块蹭倒 —— 而绕行的第一段正是"抬到安全高度"，
   * 高度可能比 travel_z 还高，走完还要下来。直接直线抬到 travel_z 就够。 */
  Actuation a = with_pump(goto_grasp_point(cfg_.travel_z, /*allow_waypoint=*/false, 0.7f,
                                           "抬升到搬运高度"),
                          cfg_.hold_duty);

  // 保持期间再确认一次压力，防止"看着吸住了其实只是贴上了"
  if (cfg_.use_pressure_check && !vacuum_ok(s)) {
    payload_attached_ = false;
    grasp_fail_++;
    if (grasp_fail_ >= cfg_.max_grasp_retry) {
      if (has_target_) { excluded_.push_back(target_track_id_); }
      clear_target();
      st_.skipped_count++;
      consecutive_skips_++;
      transition(TaskState::Scan, "离地后压力不足，跳过该物块");
      return stop_all("吸附不稳");
    }
    transition(TaskState::Align, "离地后压力不足，重新对位");
    return stop_all("重新对位");
  }

  if (tool_settled(s, cmd_x_, cmd_y_, cfg_.travel_z) &&
      (s.t - t_state_) >= cfg_.lift_time) {
    transition(TaskState::Haul, "开始运送");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  HAUL：在搬运高度平移到投放位上方（对应 demo 里那两段长距离插补）
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_haul(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = with_pump(hold("运送中"), cfg_.hold_duty);

  double px, py, pyaw, pz;
  place_slot(placed_slots_, &px, &py, &pyaw, &pz);
  (void)pyaw;
  (void)pz;

  a = with_pump(goto_pose(px, py, cfg_.travel_z, 0.0,
                          /*allow_waypoint=*/true, 1.0f, "运送到投放位上方"),
                cfg_.hold_duty);

  /* 到位判据用"位置 + 速度"而不是 demo 里的固定 duration：
   * duration 是开环时间，负载一变就会提前或滞后停下。
   * 而且必须同时看速度 —— 只看位置的话，末端以 0.3 m/s 掠过目标点
   * 的那一两个周期也会被判成"到达"。tool_settled 把这两点都验证了。 */
  if (tool_settled(s, cmd_x_, cmd_y_, cfg_.travel_z)) {
    transition(TaskState::Place, "到达投放位上方");
    return a;
  }

  if ((s.t - t_state_) > cfg_.haul_timeout) {
    // 超时说明被卡住或路径不通。带着负载悬在那里没有意义，就地放掉。
    payload_attached_ = false;
    transition(TaskState::Failed, "运送超时，已就地释放负载");
    Actuation fin = stop_all("运送超时");
    fin.aborted = true;
    return fin;
  }
  return a;
}

// ---------------------------------------------------------------------------
//  PLACE：下放到投放面（或码垛的下一层）
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_place(const SensorSnapshot & s, double dt)
{
  (void)dt;

  double px, py, pyaw, pz;
  place_slot(placed_slots_, &px, &py, &pyaw, &pz);
  (void)pyaw;

  Actuation a = with_pump(goto_pose(px, py, pz, 0.0,
                                    /*allow_waypoint=*/false, 0.5f, "下放到投放面"),
                          cfg_.hold_duty);

  if (tool_settled(s, cmd_x_, cmd_y_, pz)) {
    transition(TaskState::Release, "下放到位，释放");
    return a;
  }

  if ((s.t - t_state_) > cfg_.place_timeout) {
    /* 投放位对不准通常不影响功能（物块落地有容差），降级放行并记账。
     * 但码垛模式下必须更保守：放到一半松手会让物块歪倒，砸到已码好的垛。
     * 所以码垛超时就保持吸住不动，等上层处理（/task/abort）。 */
    if (cfg_.place_mode == PlaceMode::Stack) {
      transition(TaskState::Failed, "码垛下放超时，保持吸住等待人工处理");
      Actuation fin = with_pump(stop_all("码垛下放超时"), cfg_.hold_duty);
      fin.aborted = true;
      return fin;
    }
    payload_attached_ = false;
    st_.skipped_count++;
    transition(TaskState::Retreat, "投放位下放超时，就地释放");
    /* 就地释放之后必须真的"抬起"，也就是要下达新的目标点。
     * 之前这里只改了返回对象的 pose_z 却没置 set_pose —— 那个字段是给
     * 人看的，arm_control 根本不会因为这个字段动一下，末端于是留在
     * 投放面高度上，下一个动作再往下压时就撞到了刚放下的物块。 */
    Actuation r = goto_pose(cmd_x_, cmd_y_, cfg_.retreat_z, cmd_yaw_,
                            /*allow_waypoint=*/false, 1.0f, "就地释放后抬起");
    r.pump_run = false;
    r.pump_duty = 0.0;
    return r;
  }
  return a;
}

// ---------------------------------------------------------------------------
//  RELEASE：放气（对应 demo 的 gripper.off()）
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_release(const SensorSnapshot & s, double dt)
{
  (void)dt;

  /* 主动泄气：把占空比降到 0 让吸盘内的负压通过泄气阀释放。
   * 不用"反向吹气"是因为本项目的气泵是单向隔膜泵，没有吹气能力。 */
  Actuation a = stop_all("放气中");

  if ((s.t - t_state_) >= cfg_.release_time) {
    payload_attached_ = false;
    placed_slots_++;
    st_.grasped_count = static_cast<uint8_t>(
      std::min<uint32_t>(255U, st_.grasped_count + 1));
    st_.placed_slots = static_cast<uint8_t>(std::min<uint32_t>(255U, placed_slots_));
    consecutive_skips_ = 0;

    if (has_target_) { excluded_.push_back(target_track_id_); }
    clear_target();
    st_.target_index++;

    /* 更新码垛层号（供界面显示）。
     * 注意是 (placed_slots_ - 1)，不是 placed_slots_：placed_slots_ 在一个
     * 语句之前刚刚自增过，现在它表示"已经放好的数量"，而 place_layer 要报的
     * 是"刚刚放下的那个物块在第几层"。用 placed_slots_ 会让层号整体偏 1 ——
     * 界面上第一个物块显示成"第 1 层"，而它其实在最底下那一层（第 0 层）。 */
    const uint32_t max_layers = static_cast<uint32_t>(std::max(1, cfg_.place_max_layers));
    st_.place_layer = static_cast<uint8_t>(
      (cfg_.place_mode == PlaceMode::Stack) ? ((placed_slots_ - 1U) % max_layers) : 0U);

    transition(TaskState::Retreat, "已放置第 " + std::to_string(placed_slots_) + " 个");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  RETREAT：抬起退开，然后决定下一个目标还是收工
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_retreat(const SensorSnapshot & s, double dt)
{
  (void)dt;

  /* 抬起退开。注意是**垂直抬起**而不是"往后退"：
   * 机械臂没有底盘那种"退开一段距离"的能力，垂直抬起就能离开刚放下的物块，
   * 而且不会碰到相邻的物块。 */
  Actuation a = goto_pose(cmd_x_, cmd_y_, cfg_.retreat_z, cmd_yaw_,
                          /*allow_waypoint=*/false, 1.0f, "抬起退开");

  // 中止流程：抬起后回原点，然后终止。
  if (aborting_) {
    Actuation go = goto_pose(home_x_, home_y_, home_z_, home_yaw_,
                             /*allow_waypoint=*/true, 1.0f, "中止：返回原点");
    if (!abort_release_) {
      go.pump_run = true;
      go.pump_duty = cfg_.hold_duty;
    }
    if (tool_settled(s, home_x_, home_y_, home_z_)) {
      const std::string tail = abort_release_ ? "，负载已释放" : "，保持吸附";
      transition(TaskState::Failed, "已中止并返回原点" + tail);
      Actuation fin = stop_all("已中止");
      if (!abort_release_) {
        fin.pump_run = true;
        fin.pump_duty = cfg_.hold_duty;
      }
      fin.aborted = true;
      return fin;
    }
    return go;
  }

  const bool lifted = tool_settled(s, cmd_x_, cmd_y_, cfg_.retreat_z);
  if (!lifted && (s.t - t_state_) < cfg_.retreat_time) {
    return a;
  }

  // ---- 是否还有活 ----
  const bool quota_done = (req_.max_blocks > 0) && (placed_slots_ >= req_.max_blocks);
  /* 投放区容量：层数 × 每层槽位数。
   * 底盘方案里没有这一条（单层排开，场地够大就一直摆得下）；机械臂的
   * Z 行程只有 135mm，码垛到第 5 层就顶到上限了，所以必须有一个"放满了"
   * 的判断，否则第五个物块会被要求下放到一个够不到的高度。 */
  const uint32_t place_capacity =
    static_cast<uint32_t>(std::max(1, cfg_.place_max_layers)) *
    static_cast<uint32_t>(std::max(1, cfg_.place_columns)) *
    static_cast<uint32_t>(std::max(1, cfg_.place_rows));
  const bool stack_full = (cfg_.place_mode == PlaceMode::Stack) &&
                          (placed_slots_ >= place_capacity);
  if (quota_done || stack_full) {
    if (req_.return_home_after) {
      Actuation go = goto_pose(home_x_, home_y_, home_z_, home_yaw_,
                               /*allow_waypoint=*/true, 1.0f, "返回原点");
      if (tool_settled(s, home_x_, home_y_, home_z_)) {
        transition(TaskState::Done,
                   "任务完成，共放置 " + std::to_string(placed_slots_) + " 个，" +
                   "跳过 " + std::to_string(st_.skipped_count) + " 个");
        Actuation fin = stop_all("完成");
        fin.finished = true;
        return fin;
      }
      return go;
    }
    transition(TaskState::Done,
               "任务完成，共放置 " + std::to_string(placed_slots_) + " 个，" +
               "跳过 " + std::to_string(st_.skipped_count) + " 个");
    Actuation fin = stop_all("任务完成");
    fin.finished = true;
    return fin;
  }

  transition(TaskState::Scan, "寻找下一个物块");
  a.select_block = -1;
  a.note = "寻找下一个";
  return a;
}

}  // namespace hw_task
