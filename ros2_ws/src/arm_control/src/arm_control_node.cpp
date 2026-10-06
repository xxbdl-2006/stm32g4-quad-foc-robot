// ============================================================================
//  arm_control_node.cpp
//  机械臂控制器：末端目标位姿 -> 逆解 + 轨迹规划 -> 四关节位置指令
//
//  数据流
//  ------
//     /arm/target_pose  (hw_msgs/ArmTarget)  ──┐
//                                              ├─► 规划（CartesianMove / JointQuinticMove）
//     /hw/motor_states  (关节反馈)            ──┘        │
//                                                          ▼
//                       100Hz 定时器 ──► 采样轨迹 ──► 逆解 ──► /hw/motor_commands
//                                          │
//                                          ├──► /joint_states（给 robot_state_publisher 算 TF）
//                                          ├──► /arm/tool_pose（实测末端位姿）
//                                          └──► /arm/status
//
//  四个必须守住的约定
//  ------------------
//  1. **定时器必须无条件发帧**。驱动板 300ms 收不到指令就主动安全停机，所以
//     即使没有任何运动指令、机械臂静止不动，也要按 100Hz 把"保持当前位置"
//     发下去。写成"有指令才发"会让机械臂每隔 300ms 掉一次使能，症状是
//     静止时轻微抖动、一给指令就报看门狗 —— 非常难查。
//  2. **J4 的 setpoint 是丝杠转角，不是毫米**。驱动板不知道丝杠的存在，
//     换算在这里做（lift_to_motor_rad）。而 /joint_states 里必须是米，
//     因为 URDF 里 J4 是 prismatic 关节，单位不一致会让 RViz 里的臂形变。
//  3. **先规划、后执行**。收到目标时先把整条路径解一遍，路径不可行就直接
//     拒绝并给出原因，绝不允许"走到一半才发现无解"。
//  4. **运动学用指令值递推，不用实测值**。轨迹的起点取上一次的**指令**
//     关节值：实测值带编码器噪声与跟随误差，用它当起点会让每次规划的
//     起点都在抖，末端轨迹跟着抖。实测值只用于显示与安全判断。
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hw_msgs/msg/arm_status.hpp"
#include "hw_msgs/msg/arm_target.hpp"
#include "hw_msgs/msg/motor_command_array.hpp"
#include "hw_msgs/msg/motor_state_array.hpp"

#include "arm_control/arm_model.hpp"
#include "arm_control/current_alloc.hpp"
#include "arm_control/dh.hpp"
#include "arm_control/scara_ik.hpp"
#include "arm_control/trajectory.hpp"

using namespace std::chrono_literals;

namespace arm_control
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

/// 从四元数里取绕 Z 的 yaw。本机构末端只有绕 Z 的自由度，所以不需要
/// 通用的 RPY 分解，也不存在万向锁问题。
double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

// ---------------------------------------------------------------------------
//  一段已规划好的运动
// ---------------------------------------------------------------------------
struct PlannedSegment
{
  bool     is_cartesian{true};
  CartesianMove    cart;
  JointQuinticMove joint;
  double   duration{0.0};
  Pose3    from{};
  Pose3    to{};

  /// speed_scale 缩放前的原始时长（供显示）
  double   nominal_duration{0.0};

  /// 本段终点对应的关节解。多段直线首尾相接时，下一段拿它做逆解种子 ——
  /// 这就是为什么整条路径的每一段在规划阶段都要把终点解保留下来。
  double   joint_end_q[kJointCount]{0.0, 0.0, 0.0, 0.0};

  /// 本段路径上最小的 |sin θ2|（接近奇异程度）
  double   min_sin_t2{1.0};
};

class ArmControlNode : public rclcpp::Node
{
public:
  ArmControlNode()
  : rclcpp::Node("arm_control")
  {
    load_parameters();

    sub_target_ = create_subscription<hw_msgs::msg::ArmTarget>(
      "/arm/target_pose", rclcpp::QoS(5),
      [this](hw_msgs::msg::ArmTarget::SharedPtr m) { on_target(m); });

    sub_states_ = create_subscription<hw_msgs::msg::MotorStateArray>(
      "/hw/motor_states", rclcpp::QoS(10),
      [this](hw_msgs::msg::MotorStateArray::SharedPtr m) { on_states(m); });

    pub_cmd_    = create_publisher<hw_msgs::msg::MotorCommandArray>("/hw/motor_commands", 10);
    pub_joint_  = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pub_pose_   = create_publisher<geometry_msgs::msg::PoseStamped>("/arm/tool_pose", 10);
    pub_status_ = create_publisher<hw_msgs::msg::ArmStatus>("/arm/status", 10);

    srv_home_ = create_service<std_srvs::srv::Trigger>(
      "/arm/go_home",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = go_home(res->message);
      });

    srv_stop_ = create_service<std_srvs::srv::Trigger>(
      "/arm/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lk(mtx_);
        plan_.clear();
        seg_idx_ = 0;
        motion_state_ = hw_msgs::msg::ArmStatus::STATE_HOLDING;
        res->success = true;
        res->message = "已停止当前运动，位置保持";
        RCLCPP_WARN(get_logger(), "运动被 /arm/stop 中止");
      });

    srv_enable_ = create_service<std_srvs::srv::SetBool>(
      "/arm/set_enabled",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        enabled_ = req->data;
        if (!enabled_) {
          std::lock_guard<std::mutex> lk(mtx_);
          plan_.clear();
          seg_idx_ = 0;
          motion_state_ = hw_msgs::msg::ArmStatus::STATE_IDLE;
        }
        res->success = true;
        res->message = enabled_ ? "已使能" : "已下使能（伺服保持位置但不接受运动指令）";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ctrl_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });

    const auto status_period = std::chrono::duration<double>(1.0 / status_rate_hz_);
    timer_status_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
      [this]() { publish_status(); });

    last_tick_ = now();

    RCLCPP_INFO(get_logger(),
                "机械臂控制器就绪：SCARA(RRPR) 大臂 %.3fm 小臂 %.3fm 腕偏 %.3fm "
                "基座 %.3fm 丝杠导程 %.1fmm",
                geo_.a1, geo_.a2, geo_.a3, geo_.d1, geo_.lead * 1000.0);
    RCLCPP_INFO(get_logger(),
                "工作半径 %.3f~%.3f m，升降行程 %.0f~%.0f mm，控制 %0.0f Hz",
                geo_.reach_min(), geo_.reach_max(),
                lim_.lo[kJ4Lift] * 1000.0, lim_.hi[kJ4Lift] * 1000.0, control_rate_hz_);
    RCLCPP_INFO(get_logger(),
                "发目标：ros2 topic pub --once /arm/target_pose hw_msgs/msg/ArmTarget "
                "\"{target_pose: {position: {x: 0.25, y: 0.0, z: 0.11}}, speed_scale: 1.0, "
                "allow_waypoint: true}\"");
  }

private:
  // -------------------------------------------------------------------------
  //  参数
  // -------------------------------------------------------------------------
  void load_parameters()
  {
    auto p = [this](const char * n, auto def) { return declare_parameter(n, def); };

    geo_.a1   = p("a1", 0.180);
    geo_.a2   = p("a2", 0.180);
    geo_.a3   = p("a3", 0.060);
    geo_.d1   = p("d1", 0.140);
    geo_.lead = p("lead", 0.010);

    // 关节限位：默认值与 arm_model.hpp 一致，允许按实际机型覆盖
    const std::vector<double> lo = p("joint_lower", std::vector<double>{
      lim_.lo[0], lim_.lo[1], lim_.lo[2], lim_.lo[3]});
    const std::vector<double> hi = p("joint_upper", std::vector<double>{
      lim_.hi[0], lim_.hi[1], lim_.hi[2], lim_.hi[3]});
    if (lo.size() == kJointCount && hi.size() == kJointCount) {
      for (int i = 0; i < kJointCount; ++i) {
        lim_.lo[i] = lo[static_cast<std::size_t>(i)];
        lim_.hi[i] = hi[static_cast<std::size_t>(i)];
      }
    }

    const std::vector<double> vmax = p("joint_vel_max", std::vector<double>{
      tlim_.vmax[0], tlim_.vmax[1], tlim_.vmax[2], tlim_.vmax[3]});
    const std::vector<double> amax = p("joint_acc_max", std::vector<double>{
      tlim_.amax[0], tlim_.amax[1], tlim_.amax[2], tlim_.amax[3]});
    if (vmax.size() == kJointCount && amax.size() == kJointCount) {
      for (int i = 0; i < kJointCount; ++i) {
        tlim_.vmax[i] = vmax[static_cast<std::size_t>(i)];
        tlim_.amax[i] = amax[static_cast<std::size_t>(i)];
      }
    }

    const std::vector<double> nom = p("current.nominal", std::vector<double>{
      budget_.nominal[0], budget_.nominal[1], budget_.nominal[2], budget_.nominal[3]});
    const std::vector<double> bst = p("current.boost_max", std::vector<double>{
      budget_.boost_max[0], budget_.boost_max[1], budget_.boost_max[2], budget_.boost_max[3]});
    if (nom.size() == kJointCount && bst.size() == kJointCount) {
      for (int i = 0; i < kJointCount; ++i) {
        budget_.nominal[i]   = nom[static_cast<std::size_t>(i)];
        budget_.boost_max[i] = bst[static_cast<std::size_t>(i)];
      }
    }
    budget_.total_cap = p("current.total_cap", 12.0);

    control_rate_hz_ = p("control_rate_hz", 100.0);
    status_rate_hz_  = p("status_rate_hz", 20.0);

    /* 安全高度：任何"抬起来再平移"的动作都走这个高度。
     * 取基座平面下方 20mm —— 足够越过工作台上最高 40mm 的物块与夹具，
     * 又不会高到让 J1 回转时手臂甩太大范围。 */
    safe_z_ = p("safe_z", geo_.d1 - 0.020);

    home_.x   = p("home.x", 0.28);
    home_.y   = p("home.y", 0.0);
    home_.z   = p("home.z", safe_z_);
    home_.yaw = p("home.yaw", 0.0);

    auto_waypoint_ = p("auto_waypoint", true);
    default_speed_scale_ = p("speed_scale", 1.0);
    base_frame_ = p("base_frame", std::string("base_link"));
  }

  // -------------------------------------------------------------------------
  //  订阅：新目标
  // -------------------------------------------------------------------------
  void on_target(const hw_msgs::msg::ArmTarget::SharedPtr m)
  {
    Pose3 to;
    to.x   = m->target_pose.position.x;
    to.y   = m->target_pose.position.y;
    to.z   = m->target_pose.position.z;
    to.yaw = yaw_from_quaternion(m->target_pose.orientation.x, m->target_pose.orientation.y,
                                 m->target_pose.orientation.z, m->target_pose.orientation.w);

    const bool allow_wp = m->allow_waypoint && auto_waypoint_;
    const double scale = std::clamp(static_cast<double>(m->speed_scale), 0.05, 3.0);
    const bool cart = (m->move_kind == hw_msgs::msg::ArmTarget::MOVE_CARTESIAN);
    current_scale_ = scale;

    std::lock_guard<std::mutex> lk(mtx_);

    if (faulted_) {
      reject("有关节处于故障状态，拒绝执行新目标（先排除故障）");
      return;
    }
    if (!enabled_) {
      reject("未使能，拒绝执行（先调 /arm/set_enabled）");
      return;
    }
    if (!have_meas_) {
      /* 还没收到任何关节反馈时不能动。此时 joint_cmd_ 全是 0，规划出的
       * 起点是"零位"，而真实机械臂可能在别处 —— 直接执行会让它以最快速度
       * 冲向零位。等第一帧 /hw/motor_states 到了再接受目标。 */
      reject("尚未收到关节反馈（检查 CAN 总线与驱动板），拒绝执行");
      return;
    }

    /* 规划的起点用**上一次的指令关节值**算出的末端位姿，而不是实测位姿。
     * 见文件头第 4 条。首次（还没发过任何指令）时退化为实测位姿。 */
    Pose3 from = forward_tool_pose(geo_, joint_cmd_);
    from.yaw = last_cmd_pose_valid_ ? last_cmd_pose_.yaw : from.yaw;

    if (plan_to(from, to, cart, scale, allow_wp)) {
      target_pose_ = to;
      motion_state_ = hw_msgs::msg::ArmStatus::STATE_MOVING;
      last_message_ = "开始执行 " + std::to_string(plan_.size()) + " 段运动";
      RCLCPP_INFO(get_logger(), "接受目标 (%.3f, %.3f, %.3f) yaw %.1f°，%zu 段，预计 %.2fs",
                  to.x, to.y, to.z, to.yaw * 180.0 / kPi, plan_.size(), total_duration_);
    }
    // plan_to 内部失败时会自己调 reject()
  }

  // -------------------------------------------------------------------------
  //  规划
  // -------------------------------------------------------------------------
  bool plan_to(const Pose3 & from, const Pose3 & to, bool cartesian, double scale,
               bool allow_waypoint)
  {
    plan_.clear();
    seg_idx_ = 0;
    seg_t_ = 0.0;
    total_duration_ = 0.0;
    min_sin_all_ = 1.0;

    // ---- 先做一次可达性自检，把"目标本身就不可达"与"路径不可行"分开报 ----
    IkSolution probe;
    if (!scara_inverse(geo_, lim_, to.x, to.y, to.z, to.yaw, joint_cmd_, &probe)) {
      reject(std::string("目标不可达：") + to_string(probe.status) +
             (probe.status == IkStatus::OutOfReach
                ? ("，差 " + std::to_string(probe.reach_error * 1000.0) + " mm")
                : std::string()));
      return false;
    }

    if (!cartesian) {
      // 关节空间插补：不会撞奇异，但末端路径不保证是直线
      PlannedSegment seg;
      seg.is_cartesian = false;
      seg.from = from;
      seg.to = to;
      seg.joint.reset(joint_cmd_, probe.q, tlim_);
      for (int k = 0; k < kJointCount; ++k) { seg.joint_end_q[k] = probe.q[k]; }
      seg.nominal_duration = seg.joint.duration();
      seg.duration = std::max(1e-3, seg.nominal_duration / scale);
      total_duration_ = seg.duration;
      plan_.push_back(seg);
      return true;
    }

    // ---- 直线插补：先试直线，不行再考虑绕行 ----
    if (append_cartesian(from, to, scale)) {
      return true;
    }

    if (!allow_waypoint) {
      reject("直线路径不可行，且未允许绕行（allow_waypoint=false）");
      return false;
    }

    /* 绕行：抬到安全高度 -> 平移到目标上方 -> 下压到目标。
     * 这是工业机械臂的常规做法（"先抬后走"），也是把一条穿过基座内孔的
     * 不可行直线拆成三段可行直线的标准手段。 */
    const Pose3 lift{from.x, from.y, safe_z_, from.yaw};
    const Pose3 travel{to.x, to.y, safe_z_, to.yaw};

    plan_.clear();
    if (!append_cartesian(from, lift, scale) ||
        !append_cartesian(lift, travel, scale) ||
        !append_cartesian(travel, to, scale)) {
      plan_.clear();
      reject("目标可达，但直线与绕行路径都不可行（检查工作台上是否有干涉）");
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "直线路径不可行，已自动改为绕行：抬升到 z=%.3f -> 平移 -> 下压", safe_z_);
    return true;
  }

  /// 追加一段直线运动；起点用 plan_ 里最后一段的终点（保证段间连续）
  bool append_cartesian(const Pose3 & from, const Pose3 & to, double scale)
  {
    PlannedSegment seg;
    seg.is_cartesian = true;
    seg.from = from;
    seg.to = to;

    // 段的起点关节值：第一段用当前指令值，后续段用上一段终点的逆解
    const double * seed = joint_cmd_;
    if (!plan_.empty()) {
      seed = plan_.back().joint_end_q;
    }

    if (!seg.cart.plan(geo_, lim_, tlim_, seed, from, to)) {
      return false;
    }
    seg.cart.end_q(seg.joint_end_q);

    seg.nominal_duration = seg.cart.duration();
    seg.duration = std::max(1e-3, seg.nominal_duration / scale);
    seg.min_sin_t2 = seg.cart.min_sin_theta2();

    total_duration_ += seg.duration;
    min_sin_all_ = std::min(min_sin_all_, seg.min_sin_t2);
    plan_.push_back(seg);
    return true;
  }

  void reject(const std::string & why)
  {
    ++ik_fail_count_;
    last_message_ = why;
    motion_state_ = hw_msgs::msg::ArmStatus::STATE_REJECTED;
    RCLCPP_ERROR(get_logger(), "目标被拒绝：%s", why.c_str());
  }

  // -------------------------------------------------------------------------
  //  反馈
  // -------------------------------------------------------------------------
  void on_states(const hw_msgs::msg::MotorStateArray::SharedPtr m)
  {
    if (m->motors.size() < static_cast<std::size_t>(kJointCount)) { return; }

    std::lock_guard<std::mutex> lk(mtx_);
    faulted_ = false;
    for (int i = 0; i < kJointCount; ++i) {
      const auto & ms = m->motors[static_cast<std::size_t>(i)];

      /* /hw/motor_states 里 J4 的位置是**丝杠转角**（rad），这里换算成米，
       * 因为 URDF 里 joint_4 是 prismatic 关节。两处单位不一致时 RViz 里
       * 会看到升降轴以 60 倍的幅度运动 —— 一眼就知道是单位问题，但排查
       * 要花时间，所以在这一行上面留个记号。 */
      if (is_prismatic(i)) {
        joint_meas_[i] = motor_rad_to_lift(ms.position, geo_.lead);
        joint_meas_vel_[i] = motor_rad_to_lift(ms.velocity, geo_.lead);
      } else {
        joint_meas_[i] = ms.position;
        joint_meas_vel_[i] = ms.velocity;
      }

      faults_[i] = static_cast<uint8_t>((ms.fault != 0U) ||
                                        ms.state == hw_msgs::msg::MotorState::STATE_FAULT);
      if (faults_[i] != 0U) { faulted_ = true; }
      last_current_[i] = ms.current;
    }
    have_meas_ = true;

    if (faulted_ && motion_state_ == hw_msgs::msg::ArmStatus::STATE_MOVING) {
      // 有关节报故障：立刻停止轨迹推进，保持当前位置。
      // 不主动下使能 —— 下使能会让失电的关节失去保持力矩，手臂有可能
      // 因为重力塌下来。让驱动板自己去处理它自己的故障策略。
      plan_.clear();
      seg_idx_ = 0;
      motion_state_ = hw_msgs::msg::ArmStatus::STATE_FAULT;
      last_message_ = "关节故障，运动已中止（保持当前位置）";
      RCLCPP_ERROR(get_logger(), "%s", last_message_.c_str());
    }

    publish_joint_states(m->header.stamp);
  }

  // -------------------------------------------------------------------------
  //  控制主循环
  // -------------------------------------------------------------------------
  void tick()
  {
    const rclcpp::Time t = now();
    double dt = (t - last_tick_).seconds();
    last_tick_ = t;
    if (dt <= 0.0 || dt > 0.5) {
      /* 时间跳变（首帧、或调度卡了半秒以上）时不要按这个 dt 推进轨迹 ——
       * 一次跳变会把轨迹推进到很后面，末端等于"瞬移"。宁可这一周期不推进。 */
      dt = 0.0;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    advance(dt);
    send_command(t);
  }

  void advance(double dt)
  {
    if (plan_.empty() || seg_idx_ >= plan_.size()) { return; }

    seg_t_ += dt;

    // 段结束：把超出的时间带到下一段，避免每段接缝处丢半个周期
    while (seg_idx_ < plan_.size() && seg_t_ >= plan_[seg_idx_].duration) {
      seg_t_ -= plan_[seg_idx_].duration;
      ++seg_idx_;
    }

    if (seg_idx_ >= plan_.size()) {
      // 全部走完
      motion_state_ = hw_msgs::msg::ArmStatus::STATE_HOLDING;
      last_message_ = "运动完成，位置保持";
      target_pose_ = plan_.back().to;
      plan_.clear();
      seg_idx_ = 0;
      seg_t_ = 0.0;
      return;
    }

    const PlannedSegment & seg = plan_[seg_idx_];
    double q[kJointCount];

    if (seg.is_cartesian) {
      if (!seg.cart.sample_joint(seg_t_, q)) {
        // 规划阶段已经验过整条路径，这里失败说明出现了计划外的状况
        // （比如编码器读数突变导致种子跳变）。停住比硬走安全。
        plan_.clear();
        seg_idx_ = 0;
        motion_state_ = hw_msgs::msg::ArmStatus::STATE_REJECTED;
        last_message_ = "轨迹执行中出现无解（路径与规划不一致），已停止";
        RCLCPP_ERROR(get_logger(), "%s", last_message_.c_str());
        return;
      }
    } else {
      seg.joint.sample(seg_t_, q);
    }

    // 限位兜底：规划阶段已经保证在限位内，这一层是防止数值边界与
    // 参数被在线改动之后出现越界指令。宁可按限位截断，也不发出越界指令。
    for (int i = 0; i < kJointCount; ++i) {
      joint_cmd_[i] = std::clamp(q[i], lim_.lo[i], lim_.hi[i]);
    }
  }

  /// 无条件按控制周期发帧（见文件头第 1 条）
  void send_command(const rclcpp::Time & t)
  {
    bool healthy[kJointCount];
    for (int i = 0; i < kJointCount; ++i) { healthy[i] = (faults_[i] == 0U); }

    double limit[kJointCount];
    allocate_current(budget_, healthy, limit);
    for (int i = 0; i < kJointCount; ++i) { joint_limit_[i] = limit[i]; }

    hw_msgs::msg::MotorCommandArray arr;
    arr.header.stamp = t;
    arr.header.frame_id = base_frame_;
    arr.commands.resize(static_cast<std::size_t>(kJointCount));

    for (int i = 0; i < kJointCount; ++i) {
      auto & c = arr.commands[static_cast<std::size_t>(i)];
      c.header = arr.header;
      c.motor_id = static_cast<uint8_t>(i);
      c.mode = hw_msgs::msg::MotorCommand::MODE_POSITION;

      // J4：米 -> 丝杠转角（rad）。其余三个关节本来就是 rad。
      c.setpoint = static_cast<float>(is_prismatic(i)
                                        ? lift_to_motor_rad(joint_cmd_[i], geo_.lead)
                                        : joint_cmd_[i]);
      c.current_limit = static_cast<float>(limit[i]);
      c.enable = enabled_;
      c.brake = false;
      c.reset_fault = false;
    }
    pub_cmd_->publish(arr);

    last_cmd_pose_ = forward_tool_pose(geo_, joint_cmd_);
    last_cmd_pose_valid_ = true;
  }

  // -------------------------------------------------------------------------
  //  发布
  // -------------------------------------------------------------------------
  void publish_joint_states(const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    for (int i = 0; i < kJointCount; ++i) {
      js.name.emplace_back(joint_name(i));
      js.position.push_back(joint_meas_[i]);
      js.velocity.push_back(joint_meas_vel_[i]);
      // 用电流代替力矩：这四个关节都没有力矩传感器，而电流与力矩在
      // 永磁同步电机里是严格成比例的（力矩常数 × iq）。
      js.effort.push_back(last_current_[i]);
    }
    pub_joint_->publish(js);
  }

  void publish_status()
  {
    std::lock_guard<std::mutex> lk(mtx_);

    const ToolPose cur = forward_tool_pose(geo_, joint_meas_);
    const Vec3 tool_v = tool_velocity();

    geometry_msgs::msg::PoseStamped cur_msg;
    cur_msg.header.stamp = now();
    cur_msg.header.frame_id = base_frame_;
    cur_msg.pose.position.x = cur.x;
    cur_msg.pose.position.y = cur.y;
    cur_msg.pose.position.z = cur.z;
    set_yaw(cur_msg.pose, cur.yaw);
    pub_pose_->publish(cur_msg);

    hw_msgs::msg::ArmStatus st;
    st.header = cur_msg.header;
    st.state = motion_state_;
    st.message = last_message_;
    st.ik_ok = !plan_.empty();

    st.target_pose = cur_msg;
    st.target_pose.pose.position.x = target_pose_.x;
    st.target_pose.pose.position.y = target_pose_.y;
    st.target_pose.pose.position.z = target_pose_.z;
    set_yaw(st.target_pose.pose, target_pose_.yaw);

    st.current_pose = cur_msg;
    st.tool_twist.linear.x = tool_v.x;
    st.tool_twist.linear.y = tool_v.y;
    st.tool_twist.linear.z = tool_v.z;

    for (int i = 0; i < kJointCount; ++i) {
      st.joint_target.push_back(static_cast<float>(joint_cmd_[i]));
      st.joint_measured.push_back(static_cast<float>(joint_meas_[i]));
      st.joint_velocity.push_back(static_cast<float>(joint_meas_vel_[i]));
      st.joint_current_limit.push_back(static_cast<float>(joint_limit_[i]));
    }

    st.waypoints_total = static_cast<uint8_t>(std::min<std::size_t>(255, plan_.size()));
    st.waypoint_index = static_cast<uint8_t>(std::min<std::size_t>(255, seg_idx_));
    st.progress = (!plan_.empty() && seg_idx_ < plan_.size() && plan_[seg_idx_].duration > 0.0)
                    ? static_cast<float>(seg_t_ / plan_[seg_idx_].duration) : 0.0f;
    st.speed_scale = static_cast<float>(current_scale_);
    st.ik_fail_count = static_cast<uint8_t>(std::min<uint32_t>(255, ik_fail_count_));
    st.min_sin_theta2 = min_sin_all_;
    pub_status_->publish(st);
  }

  /**
   * @brief 末端线速度，由关节速度经数值雅可比映射得到。
   *
   * 为什么不用"末端位置的差分"：位置来自 100Hz 聚合的话题，差分出来的
   * 速度噪声比信号还大（1mm 的位置抖动 ÷ 10ms = 0.1 m/s 的假速度），
   * 而停稳判据的阈值就是 10mm/s 量级 —— 会被噪声淹没。
   * 用关节速度（驱动板做过一阶低通 + 编码器差分）经雅可比映射要干净得多。
   */
  Vec3 tool_velocity() const
  {
    double J[3][kJointCount];
    numerical_jacobian(make_scara_table(geo_), joint_meas_, J);

    Vec3 v;
    for (int c = 0; c < kJointCount; ++c) {
      // J4 在运动学里的单位是米，实测速度数组里已经是米/秒，直接相乘
      const double qd = joint_meas_vel_[c];
      v.x += J[0][c] * qd;
      v.y += J[1][c] * qd;
      v.z += J[2][c] * qd;
    }
    return v;
  }

  static void set_yaw(geometry_msgs::msg::Pose & p, double yaw)
  {
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = std::sin(yaw * 0.5);
    p.orientation.w = std::cos(yaw * 0.5);
  }

  // -------------------------------------------------------------------------
  //  服务
  // -------------------------------------------------------------------------
  bool go_home(std::string & msg)
  {
    if (faulted_) {
      msg = "有关节故障，无法回原点";
      return false;
    }
    if (!enabled_) {
      msg = "未使能";
      return false;
    }
    if (!have_meas_) {
      msg = "尚未收到关节反馈，无法安全回原点";
      return false;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    Pose3 from = forward_tool_pose(geo_, joint_cmd_);
    from.yaw = last_cmd_pose_valid_ ? last_cmd_pose_.yaw : from.yaw;

    if (!plan_to(from, home_, /*cartesian=*/true, default_speed_scale_,
                 /*allow_waypoint=*/true)) {
      msg = "回原点规划失败：" + last_message_;
      return false;
    }
    target_pose_ = home_;
    motion_state_ = hw_msgs::msg::ArmStatus::STATE_MOVING;
    msg = "正在回原点";
    return true;
  }

  // -------------------------------------------------------------------------
  //  成员
  // -------------------------------------------------------------------------
  ArmGeometry      geo_{};
  JointLimits      lim_{};
  TrajectoryLimits tlim_{};
  CurrentBudget    budget_{};

  double control_rate_hz_{100.0};
  double status_rate_hz_{20.0};
  double safe_z_{0.120};
  double default_speed_scale_{1.0};
  double current_scale_{1.0};
  Pose3  home_{0.28, 0.0, 0.120, 0.0};
  bool   auto_waypoint_{true};
  std::string base_frame_{"base_link"};

  // ---- 反馈 ----
  std::mutex mtx_;
  double  joint_meas_[kJointCount]{0.0, 0.0, 0.0, 0.0};
  double  joint_meas_vel_[kJointCount]{0.0, 0.0, 0.0, 0.0};
  double  joint_cmd_[kJointCount]{0.0, 0.0, 0.0, 0.0};
  double  joint_limit_[kJointCount]{0.0, 0.0, 0.0, 0.0};
  double  last_current_[kJointCount]{0.0, 0.0, 0.0, 0.0};
  uint8_t faults_[kJointCount]{0, 0, 0, 0};
  bool    have_meas_{false};
  bool    faulted_{false};
  bool    enabled_{false};

  // ---- 运动 ----
  std::vector<PlannedSegment> plan_;
  std::size_t seg_idx_{0};
  double seg_t_{0.0};
  double total_duration_{0.0};
  double min_sin_all_{1.0};
  Pose3  target_pose_{};
  Pose3  last_cmd_pose_{};
  bool   last_cmd_pose_valid_{false};

  uint8_t  motion_state_{hw_msgs::msg::ArmStatus::STATE_IDLE};
  uint32_t ik_fail_count_{0};
  std::string last_message_{"待机"};

  rclcpp::Time last_tick_;

  rclcpp::Subscription<hw_msgs::msg::ArmTarget>::SharedPtr      sub_target_;
  rclcpp::Subscription<hw_msgs::msg::MotorStateArray>::SharedPtr sub_states_;

  rclcpp::Publisher<hw_msgs::msg::MotorCommandArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_joint_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<hw_msgs::msg::ArmStatus>::SharedPtr         pub_status_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_home_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;

  rclcpp::TimerBase::SharedPtr timer_ctrl_;
  rclcpp::TimerBase::SharedPtr timer_status_;
};

}  // namespace arm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_control::ArmControlNode>());
  rclcpp::shutdown();
  return 0;
}
