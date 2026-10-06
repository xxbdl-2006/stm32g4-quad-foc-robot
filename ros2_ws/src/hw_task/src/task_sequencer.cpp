// ============================================================================
//  task_sequencer.cpp
// ============================================================================
#include "hw_task/task_sequencer.hpp"
#include "hw_task/ground_projection.hpp"

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

double wrap_pi(double a)
{
  while (a > kPi) { a -= 2.0 * kPi; }
  while (a <= -kPi) { a += 2.0 * kPi; }
  return a;
}

double clampd(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
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
  // 换状态就清重试计数 —— 重试是"同一状态内的重试"，跨状态不累计
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
  return a;
}

Actuation TaskSequencer::stop_all(const std::string & note) const
{
  Actuation a;
  a.pump_run = false;
  a.pump_duty = 0.0;
  a.note = note;
  return a;
}

Actuation TaskSequencer::with_pump(Actuation a, double duty) const
{
  a.pump_run = duty > 0.001;
  a.pump_duty = duty;
  return a;
}

// ---------------------------------------------------------------------------
//  启动 / 中止 / 暂停
// ---------------------------------------------------------------------------
bool TaskSequencer::start(const StartRequest & req, const SensorSnapshot & s)
{
  if (running()) {
    return false;
  }
  if (s.drive_faulted || s.estop) {
    transition(TaskState::Failed, "底盘有故障或处于急停，拒绝启动");
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

  home_x_ = s.vehicle_x;
  home_y_ = s.vehicle_y;
  home_yaw_ = s.vehicle_yaw;

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
    transition(TaskState::Retreat, "中止：正在返回起点");
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
void TaskSequencer::bind_target(uint32_t track_id, const Eigen::Vector2d & pos,
                                BlockColor color, double yaw, float confidence)
{
  // 换目标时如果已经吸着东西，说明上层逻辑有问题 —— 不要静默覆盖
  if (has_target_ && target_track_id_ != track_id && payload_attached_) {
    return;
  }
  has_target_ = true;
  target_track_id_ = track_id;
  target_pos_ = pos;
  target_color_ = color;
  target_yaw_ = yaw;
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
//  几何工具
// ---------------------------------------------------------------------------
void TaskSequencer::target_to_vehicle_pose(double bx, double by, double byaw,
                                           double * ox, double * oy, double * oyaw) const
{
  /* 吸盘装在车体正前方，所以"车体期望位姿"是把车体沿 -期望朝向 方向
   * 后退 standoff 距离，使得吸盘正落在物块上。
   * 期望朝向取物块长边方向再加一个安装偏置（0 或 π/2，取决于吸盘是
   * 长边对齐还是短边对齐）。 */
  const double yaw_d = byaw;
  *oyaw = yaw_d;
  *ox = bx - cfg_.standoff * std::cos(yaw_d);
  *oy = by - cfg_.standoff * std::sin(yaw_d);
}

void TaskSequencer::place_slot(uint32_t slot, double * x, double * y, double * yaw) const
{
  const int cols = std::max(1, cfg_.place_columns);
  const int col = static_cast<int>(slot) % cols;
  const int row = static_cast<int>(slot) / cols;
  const double along = col * cfg_.place_pitch;
  const double side = row * cfg_.place_row_pitch;

  // 阵列沿投放区的朝向铺开
  const double c = std::cos(cfg_.place_yaw);
  const double s = std::sin(cfg_.place_yaw);
  *x = cfg_.place_x + along * c - side * s;
  *y = cfg_.place_y + along * s + side * c;
  *yaw = cfg_.place_yaw;
}

bool TaskSequencer::pose_settled(const SensorSnapshot & s, double tx, double ty,
                                 double tyaw) const
{
  const double ex = tx - s.vehicle_x;
  const double ey = ty - s.vehicle_y;
  const double dist = std::hypot(ex, ey);
  const double eyaw = std::abs(wrap_pi(tyaw - s.vehicle_yaw));

  const bool position_ok = dist < (cfg_.standoff_tol * 1.5);
  const bool heading_ok = eyaw < (cfg_.bearing_tol * 2.0);
  const bool still = std::abs(s.vehicle_vx) < 0.03 && std::abs(s.vehicle_wz) < 0.06;
  return position_ok && heading_ok && still;
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
//  位姿控制器
// ---------------------------------------------------------------------------
Actuation TaskSequencer::drive_to_pose(const SensorSnapshot & s,
                                       double tx, double ty, double tyaw,
                                       double speed_cap, double angular_cap) const
{
  Actuation a;

  const double ex = tx - s.vehicle_x;
  const double ey = ty - s.vehicle_y;
  const double d = std::hypot(ex, ey);

  /* ---- 第一段：走到目标点 ----
   * 方向取"当前位置指向目标点"，而不是"期望朝向"。因为差速底盘没有横向
   * 自由度，只能靠转向前进；如果按期望朝向分解误差，横向偏差就无解。 */
  if (d > cfg_.standoff_tol) {
    const double heading_to_goal = std::atan2(ey, ex);
    const double e_h = wrap_pi(heading_to_goal - s.vehicle_yaw);

    // 目标在大角度侧后方时先原地转过去，避免画圈
    if (std::abs(e_h) > 0.60) {
      a.cmd_wz = clampd(cfg_.yaw_kp * e_h, -angular_cap, angular_cap);
      a.note = "定向";
      return a;
    }

    double v = cfg_.longitudinal_kp * d * std::cos(e_h);
    v = clampd(v, 0.0, speed_cap);
    if (v < cfg_.speed_min) {
      // 保证逼近过程不会因为"误差小于速度死区"而停住 —— 原来的实现正是在
      // 这里卡死的。给一个速度下限，20ms 走 0.4mm，不会有过冲问题。
      v = cfg_.speed_min;
    }
    a.cmd_vx = v;
    a.cmd_wz = clampd(cfg_.yaw_kp * e_h, -angular_cap, angular_cap);
    a.note = "逼近";
    return a;
  }

  /* ---- 第二段：位置到位了，原地转到期望朝向 ----
   * 差速底盘的原地旋转不改变车体中心位置，所以两段之间不会有耦合。 */
  const double e_final = wrap_pi(tyaw - s.vehicle_yaw);
  if (std::abs(e_final) > cfg_.bearing_tol) {
    a.cmd_wz = clampd(cfg_.yaw_kp * e_final, -angular_cap, angular_cap);
    a.note = "对齐朝向";
    return a;
  }

  // 全零 = 已到位且已对准，调用方的 pose_settled() 会据此进入下一步
  return a;
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

  // ---- 底盘故障：搬着东西也要先放下，否则一直吸着会过热 ----
  if (s.drive_faulted && st_.state != TaskState::Idle &&
      st_.state != TaskState::Done && st_.state != TaskState::Failed) {
    payload_attached_ = false;
    transition(TaskState::Failed, "底盘故障，任务终止（负载已释放）");
    return stop_all("底盘故障");
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
    const PolarTarget p = to_polar(target_pos_.x(), target_pos_.y(),
                                   s.vehicle_x, s.vehicle_y, s.vehicle_yaw);
    st_.target_distance = static_cast<float>(p.range);
    st_.target_x = target_pos_.x();
    st_.target_y = target_pos_.y();
    st_.target_yaw = target_yaw_;
    st_.target_color = target_color_;
    st_.has_target = true;
  } else {
    st_.target_distance = -1.0f;
    st_.has_target = false;
  }

  // ---- 搬运途中掉件检测：吸住之后压力突然消失 ----
  if (payload_attached_ && cfg_.use_pressure_check) {
    const bool lost = std::fabs(s.pump_pressure_kpa) <
                      (cfg_.vacuum_threshold_kpa * 0.5);
    if (lost && (st_.state == TaskState::Lift || st_.state == TaskState::Haul ||
                 st_.state == TaskState::Place)) {
      // 掉件了。物块落在路上，位置未知，只能放弃当前目标重新扫描。
      payload_attached_ = false;
      if (has_target_) { excluded_.push_back(target_track_id_); }
      clear_target();
      st_.skipped_count++;
      consecutive_skips_++;
      // 换了目标，各阶段的重试计数都重新开始
      scan_fail_ = approach_fail_ = align_fail_ = grasp_fail_ = 0;

      // 同样注意：aborted 必须挂在返回的那个对象上（见 st_grasp 里的注释）
      Actuation drop_stop = stop_all("连续掉件");
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

  // ---- 位姿置信度：用相机同步对齐质量与目标可见性合成 ----
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
//  SCAN：原地慢速旋转找物块
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_scan(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a;

  // 找到稳定目标 -> 进入 APPROACH。目标由 ROS 节点通过 bind_target 送进来，
  // 所以这里只判断"有没有目标"。
  if (has_target_ && s.target_valid) {
    const PolarTarget p = to_polar(target_pos_.x(), target_pos_.y(),
                                   s.vehicle_x, s.vehicle_y, s.vehicle_yaw);
    // 扫描有结果了：新一轮接近/对准/抓取，把后面几个计数全部清零
    scan_fail_ = 0;
    approach_fail_ = 0;
    align_fail_ = 0;
    grasp_fail_ = 0;
    transition(TaskState::Approach,
               "锁定" + std::string(to_string(target_color_)) + "色物块，距离 " +
               std::to_string(p.range).substr(0, 4) + "m");
    return hold("锁定目标");
  }

  // 扫描：朝物块更可能多的方向转。没有先验知识时按固定方向匀速转。
  a.cmd_wz = cfg_.scan_yaw_rate;
  a.note = "原地扫描";
  a.select_block = -1;   // 请求 ROS 节点重新挑选目标

  if ((s.t - t_state_) > cfg_.scan_timeout) {
    scan_fail_++;
    if (scan_fail_ >= cfg_.max_scan_retry) {
      /* 配额还没完成，但视野里已经找不到能抓的物块。两种可能：
       *   · 之前有几块因为"吸不住/接近超时"被排除了，但它们其实还在场上
       *   · 场上的物块确实都被抓完了
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
                   "视野内已无可用物块，任务结束（放置 " +
                   std::to_string(placed_slots_) + " 个，跳过 " +
                   std::to_string(st_.skipped_count) + " 个）");
        a.finished = true;
        a.note = "任务完成";
        return a;
      }
      transition(TaskState::Failed, "扫描超时且未发现任何物块，任务失败");
      a.aborted = true;
      a.note = "扫描失败";
      return a;
    }
    t_state_ = s.t;   // 重新开始一轮扫描
    a.note = "扫描超时，第 " + std::to_string(scan_fail_ + 1) + " 轮重试";
  }
  return a;
}

// ---------------------------------------------------------------------------
//  APPROACH：粗定位，全速开到物块附近
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_approach(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a;

  // ---- 视觉丢失 ----
  if (t_vision_lost_ > cfg_.vision_lost_timeout) {
    approach_fail_++;
    clear_target();
    if (approach_fail_ > cfg_.max_align_retry) {
      transition(TaskState::Failed, "接近过程中持续丢失视觉，任务终止");
      a.aborted = true;
      return stop_all("丢失视觉");
    }
    transition(TaskState::Scan, "接近过程中丢失目标，退回扫描");
    return hold("丢失目标");
  }

  // ---- 目标失效（被 tracker 删掉或换人）----
  if (!has_target_ || !s.target_valid) {
    transition(TaskState::Scan, "目标失效，退回扫描");
    return hold("目标失效");
  }

  double vx, vy, vyaw;
  target_to_vehicle_pose(target_pos_.x(), target_pos_.y(), target_yaw_, &vx, &vy, &vyaw);
  const PolarTarget p = to_polar(target_pos_.x(), target_pos_.y(),
                                 s.vehicle_x, s.vehicle_y, s.vehicle_yaw);

  // 进入精定位区：距离已经接近 standoff，且方位角不大
  const double approach_band = cfg_.standoff * 0.6;
  if (p.range < (cfg_.standoff + approach_band) &&
      p.range > (cfg_.standoff - approach_band) &&
      std::abs(p.bearing) < 0.45) {
    approach_fail_ = 0;
    transition(TaskState::Align, "进入精定位区，切换到视觉伺服");
    // 保留目标，直接返回零指令让 ALIGN 接管
    return hold("切换精定位");
  }

  a = drive_to_pose(s, vx, vy, vyaw, cfg_.approach_speed, 1.4);
  a.note = "接近中 " + std::to_string(p.range).substr(0, 4) + "m";
  a.select_block = -1;

  if ((s.t - t_state_) > cfg_.approach_timeout) {
    approach_fail_++;
    if (approach_fail_ > cfg_.max_align_retry) {
      // 超时通常意味着物块在不可达位置（墙边、车底下）
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
//  ALIGN：精定位，低速视觉伺服
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_align(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a;

  if (!has_target_ || !s.target_valid) {
    if (t_vision_lost_ > cfg_.vision_lost_timeout * 0.5) {
      align_fail_++;
      transition(TaskState::Scan, "精定位丢失目标");
      return hold("丢失目标");
    }
    a.note = "等待视觉恢复";
    return a;
  }

  double vx, vy, vyaw;
  // 期望位姿随目标实时更新（目标被 tracker 平滑过，本身已经很稳）
  target_to_vehicle_pose(target_pos_.x(), target_pos_.y(), target_yaw_, &vx, &vy, &vyaw);

  const double dist_err = std::hypot(s.vehicle_x - vx, s.vehicle_y - vy);
  const double head_err = std::abs(wrap_pi(vyaw - s.vehicle_yaw));
  const bool still = std::abs(s.vehicle_vx) < 0.02 && std::abs(s.vehicle_wz) < 0.05;

  if (dist_err < cfg_.standoff_tol && head_err < cfg_.bearing_tol && still) {
    align_fail_ = 0;              // 对准成功，只清对准阶段自己的计数
    transition(TaskState::Settle, "对准完成，停稳");
    return hold("对准完成");
  }

  a = drive_to_pose(s, vx, vy, vyaw, cfg_.align_speed, 0.5);
  a.note = "精调 Δ" + std::to_string(dist_err * 1000.0).substr(0, 4) + "mm";
  a.select_block = -1;

  if ((s.t - t_state_) > cfg_.align_timeout) {
    align_fail_++;
    if (align_fail_ > cfg_.max_align_retry) {
      excluded_.push_back(target_track_id_);
      clear_target();
      st_.skipped_count++;
      consecutive_skips_++;
      transition(TaskState::Scan, "精定位反复失败，跳过该物块");
      return hold("精定位失败");
    }
    transition(TaskState::Scan, "精定位超时，重新接近");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  SETTLE：停稳，等轮胎/悬架形变恢复
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_settle(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = hold("停稳等待");

  if ((s.t - t_state_) >= cfg_.settle_time) {
    transition(TaskState::Descend, "开始建立负压");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  DESCEND：开泵抽真空（对应 demo 的 gripper.on()）
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_descend(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = with_pump(hold("抽真空"), cfg_.grasp_duty);

  if ((s.t - t_state_) >= cfg_.descend_time) {
    transition(TaskState::Grasp,
               "等待真空建立（阈值 " +
               std::to_string(cfg_.vacuum_threshold_kpa).substr(0, 4) + "kPa）");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  GRASP：判定是否吸住
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_grasp(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = with_pump(hold("判定吸附"), cfg_.grasp_duty);

  if (vacuum_ok(s)) {
    payload_attached_ = true;
    grasp_fail_ = 0;
    consecutive_skips_ = 0;
    transition(TaskState::Lift, "已吸住，确认离地");
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
//  LIFT：确认离地
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_lift(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = with_pump(hold("确认离地"), cfg_.hold_duty);

  // 保持期间再确认一次压力，防止"看着吸住了其实只是贴上了"
  if (cfg_.use_pressure_check && !vacuum_ok(s)) {
    payload_attached_ = false;
    grasp_fail_++;
    if (grasp_fail_ >= cfg_.max_grasp_retry) {
      clear_target();
      st_.skipped_count++;
      consecutive_skips_++;
      transition(TaskState::Scan, "离地后压力不足，跳过该物块");
      return stop_all("吸附不稳");
    }
    transition(TaskState::Align, "离地后压力不足，重新对位");
    return stop_all("重新对位");
  }

  if ((s.t - t_state_) >= cfg_.lift_time) {
    transition(TaskState::Haul, "开始运送");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  HAUL：运送到投放区（对应 demo 里那两段长距离插补）
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_haul(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = with_pump(hold("运送中"), cfg_.hold_duty);

  double px, py, pyaw;
  place_slot(placed_slots_, &px, &py, &pyaw);
  const PolarTarget p = to_polar(px, py, s.vehicle_x, s.vehicle_y, s.vehicle_yaw);

  // 到位判据用"距离 + 朝向"，比 demo 里固定 duration 的插补更可靠：
  // duration 是开环时间，地面摩擦一变就会提前或滞后停下。
  // 到位判据必须同时看线速度与角速度 —— 只看线速度的话，车在"原地转着
  // 恰好扫过目标朝向"的那一瞬间也会被判成到达。
  if (p.range < (cfg_.standoff_tol * 2.0) &&
      std::abs(wrap_pi(pyaw - s.vehicle_yaw)) < (cfg_.bearing_tol * 2.5) &&
      std::abs(s.vehicle_vx) < 0.04 && std::abs(s.vehicle_wz) < 0.10) {
    transition(TaskState::Place, "到达投放区");
    return a;
  }

  // 投放区在车后方时，approach_speed 下先掉头
  Actuation drive = drive_to_pose(s, px, py, pyaw, cfg_.approach_speed, 1.4);
  drive.pump_run = a.pump_run;
  drive.pump_duty = a.pump_duty;
  drive.note = "运送 " + std::to_string(p.range).substr(0, 4) + "m";

  // 运送过程中视觉可以不看物块（已经吸在车上了），所以不判视觉丢失
  if ((s.t - t_state_) > cfg_.haul_timeout) {
    // 超时说明被卡住或路径不通。带着负载原地等没有意义，就地放掉再报错。
    payload_attached_ = false;
    transition(TaskState::Failed, "运送超时，已就地释放负载");
    drive.pump_run = false;
    drive.pump_duty = 0.0;
    drive.aborted = true;
    drive.note = "运送超时";
    return drive;
  }
  return drive;
}

// ---------------------------------------------------------------------------
//  PLACE：对准投放槽位
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_place(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = with_pump(hold("对准投放位"), cfg_.hold_duty);

  double px, py, pyaw;
  place_slot(placed_slots_, &px, &py, &pyaw);

  if (pose_settled(s, px, py, pyaw)) {
    transition(TaskState::Release, "对准完成，释放");
    return a;
  }

  Actuation drive = drive_to_pose(s, px, py, pyaw, cfg_.align_speed, 0.5);
  drive.pump_run = a.pump_run;
  drive.pump_duty = a.pump_duty;
  drive.note = "投放位微调";

  if ((s.t - t_state_) > cfg_.place_timeout) {
    // 投放位对不准通常不影响功能（物块落地有容差），降级放行并记账
    payload_attached_ = false;
    st_.skipped_count++;
    transition(TaskState::Retreat, "投放位对准超时，就地释放");
    drive.pump_run = false;
    drive.pump_duty = 0.0;
    drive.note = "就地释放";
    return drive;
  }
  return drive;
}

// ---------------------------------------------------------------------------
//  RELEASE：放气（对应 demo 的 gripper.off()）
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_release(const SensorSnapshot & s, double dt)
{
  (void)dt;
  // 主动泄气：把占空比降到 0 让吸盘内的负压通过泄气阀释放。
  // 不用"反向吹气"是因为本项目的气泵是单向隔膜泵，没有吹气能力。
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

    transition(TaskState::Retreat, "已放置第 " + std::to_string(placed_slots_) + " 个");
  }
  return a;
}

// ---------------------------------------------------------------------------
//  RETREAT：退开，然后决定下一个目标还是收工
// ---------------------------------------------------------------------------
Actuation TaskSequencer::st_retreat(const SensorSnapshot & s, double dt)
{
  (void)dt;
  Actuation a = stop_all("退开");

  // 直退：保持当前朝向后退，避免刚放下的物块被车轮碾到。
  // 后退速度压得很低（0.12 m/s），因为后方没有视觉。
  a.cmd_vx = -0.12;

  // 中止流程：不管退开多久，退完就回起点然后终止。
  if (aborting_) {
    const bool home_ok = pose_settled(s, home_x_, home_y_, home_yaw_);
    if (home_ok && (s.t - t_state_) > 0.5) {
      const std::string tail = abort_release_ ? "，负载已释放" : "，保持吸附";
      transition(TaskState::Failed, "已中止并返回起点" + tail);
      Actuation fin = stop_all("已中止");
      fin.aborted = true;
      return fin;
    }
    Actuation go = drive_to_pose(s, home_x_, home_y_, home_yaw_,
                                 cfg_.approach_speed, 1.4);
    if (abort_release_) {
      go.pump_run = false;
      go.pump_duty = 0.0;
    }
    go.note = "返回起点中";
    return go;
  }

  if ((s.t - t_state_) >= cfg_.retreat_time) {
    // ---- 是否还有活 ----
    const bool quota_done = (req_.max_blocks > 0) &&
                            (placed_slots_ >= req_.max_blocks);
    if (quota_done) {
      if (req_.return_home_after) {
        // 用 Haul 的位姿控制器回起点，但不再需要泵
        Actuation go = drive_to_pose(s, home_x_, home_y_, home_yaw_,
                                     cfg_.approach_speed, 1.4);
        if (pose_settled(s, home_x_, home_y_, home_yaw_)) {
          transition(TaskState::Done,
                     "任务完成，共放置 " + std::to_string(placed_slots_) + " 个，" +
                     "跳过 " + std::to_string(st_.skipped_count) + " 个");
          Actuation fin = stop_all("完成");
          fin.finished = true;
          return fin;
        }
        go.note = "返回起点";
        return go;
      }
      transition(TaskState::Done,
                 "任务完成，共放置 " + std::to_string(placed_slots_) + " 个，" +
                 "跳过 " + std::to_string(st_.skipped_count) + " 个");
      a.finished = true;
      a.note = "任务完成";
      return a;
    }

    transition(TaskState::Scan, "寻找下一个物块");
    a.select_block = -1;
    a.note = "寻找下一个";
  }
  return a;
}

}  // namespace hw_task
