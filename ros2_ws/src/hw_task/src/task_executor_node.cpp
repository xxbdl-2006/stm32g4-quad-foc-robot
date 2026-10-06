// ============================================================================
//  task_executor_node.cpp
//  把 TaskSequencer 接到 ROS 上：喂传感器、执行动作、发布状态。
//
//  这个文件刻意只做"翻译"，不含任何决策逻辑 —— 所有 if/else 都在
//  task_sequencer.cpp 里。这样比赛的流程改动只需要动那一个文件。
//
//  依赖的关键上游
//  --------------
//    /detected_blocks        block_detector_node（视觉）
//    /hw/camera_sync         驱动板硬同步锁存的四轮位姿（时间基准）
//    /hw/motor_states        轮速/电流/故障
//    /hw/pump_state          真空压力反馈（判定是否吸住）
//    /hw/board_state         母线电压、板级故障
//    /odom                   motor_control 的里程计 + TF odom->base_link
//
//  下发的
//  ------
//    /cmd_vel                底盘速度
//    /hw/pump_command        气泵（真空吸盘）
//    /task_status            状态机状态
//    /grasp_targets          当前视野内的可抓目标（给 RViz/UI 看）
// ============================================================================
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hw_msgs/msg/board_state.hpp"
#include "hw_msgs/msg/camera_sync.hpp"
#include "hw_msgs/msg/detected_block_array.hpp"
#include "hw_msgs/msg/grasp_target.hpp"
#include "hw_msgs/msg/motor_state_array.hpp"
#include "hw_msgs/msg/pump_command.hpp"
#include "hw_msgs/msg/pump_state.hpp"
#include "hw_msgs/msg/task_status.hpp"
#include "hw_msgs/srv/abort_task.hpp"
#include "hw_msgs/srv/calibrate_mapping.hpp"
#include "hw_msgs/srv/start_task.hpp"

#include "hw_task/block_tracker.hpp"
#include "hw_task/ground_projection.hpp"
#include "hw_task/task_sequencer.hpp"

using namespace std::chrono_literals;

namespace hw_task
{

class TaskExecutorNode : public rclcpp::Node
{
public:
  TaskExecutorNode()
  : rclcpp::Node("task_executor")
  {
    load_parameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---------------- 发布 ----------------
    pub_cmd_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_pump_   = create_publisher<hw_msgs::msg::PumpCommand>("/hw/pump_command", 10);
    pub_status_ = create_publisher<hw_msgs::msg::TaskStatus>("/task_status", 10);
    pub_targets_ = create_publisher<geometry_msgs::msg::PoseArray>("/grasp_targets", 10);

    // ---------------- 订阅 ----------------
    sub_blocks_ = create_subscription<hw_msgs::msg::DetectedBlockArray>(
      "/detected_blocks", rclcpp::SensorDataQoS(),
      [this](hw_msgs::msg::DetectedBlockArray::SharedPtr m) { on_blocks(m); });

    sub_sync_ = create_subscription<hw_msgs::msg::CameraSync>(
      "/hw/camera_sync", rclcpp::QoS(20),
      [this](hw_msgs::msg::CameraSync::SharedPtr m) { on_camera_sync(m); });

    sub_motors_ = create_subscription<hw_msgs::msg::MotorStateArray>(
      "/hw/motor_states", rclcpp::QoS(10),
      [this](hw_msgs::msg::MotorStateArray::SharedPtr m) { on_motors(m); });

    sub_board_ = create_subscription<hw_msgs::msg::BoardState>(
      "/hw/board_state", rclcpp::QoS(10),
      [this](hw_msgs::msg::BoardState::SharedPtr m) { on_board(m); });

    sub_pump_state_ = create_subscription<hw_msgs::msg::PumpState>(
      "/hw/pump_state", rclcpp::QoS(10),
      [this](hw_msgs::msg::PumpState::SharedPtr m) { on_pump_state(m); });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::SharedPtr m) { on_odom(m); });

    // ---------------- 服务 ----------------
    srv_start_ = create_service<hw_msgs::srv::StartTask>(
      "/task/start",
      [this](const std::shared_ptr<hw_msgs::srv::StartTask::Request> req,
             std::shared_ptr<hw_msgs::srv::StartTask::Response> res) {
        handle_start(req, res);
      });

    srv_abort_ = create_service<hw_msgs::srv::AbortTask>(
      "/task/abort",
      [this](const std::shared_ptr<hw_msgs::srv::AbortTask::Request> req,
             std::shared_ptr<hw_msgs::srv::AbortTask::Response> res) {
        handle_abort(req, res);
      });

    srv_calib_ = create_service<hw_msgs::srv::CalibrateMapping>(
      "/task/calibrate_mapping",
      [this](const std::shared_ptr<hw_msgs::srv::CalibrateMapping::Request> req,
             std::shared_ptr<hw_msgs::srv::CalibrateMapping::Response> res) {
        handle_calibrate(req, res);
      });

    srv_pause_ = create_service<std_srvs::srv::Trigger>(
      "/task/pause",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const bool ok = seq_.pause(snapshot());
        res->success = ok;
        res->message = ok ? "已暂停" : "当前状态无法暂停";
      });

    srv_resume_ = create_service<std_srvs::srv::Trigger>(
      "/task/resume",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const bool ok = seq_.resume(snapshot());
        res->success = ok;
        res->message = ok ? "已恢复" : "当前状态无法恢复";
      });

    // ---------------- 控制循环 ----------------
    const double rate = declare_parameter("control_rate_hz", 50.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      [this]() { control_loop(); });

    RCLCPP_INFO(get_logger(),
                "任务执行器就绪：控制 %0.0f Hz，可抓范围 %.2f~%.2f m / ±%.0f°，"
                "投放阵列起点 (%.2f, %.2f) 间距 %.3f m",
                rate, cfg_.grasp_radius_min, cfg_.grasp_radius_max,
                cfg_.grasp_bearing_max * 180.0 / M_PI,
                cfg_.place_x, cfg_.place_y, cfg_.place_pitch);
    RCLCPP_INFO(get_logger(),
                "启动任务： ros2 service call /task/start hw_msgs/srv/StartTask "
                "\"{max_blocks: 5, require_stable: true}\"");
  }

private:
  // -------------------------------------------------------------------------
  //  参数
  // -------------------------------------------------------------------------
  void load_parameters()
  {
    auto p = [this](const char * name, auto def) { return declare_parameter(name, def); };

    CameraIntrinsics K;
    K.fx = p("camera.fx", 600.0);
    K.fy = p("camera.fy", 600.0);
    K.cx = p("camera.cx", 320.0);
    K.cy = p("camera.cy", 240.0);
    K.width = static_cast<int>(p("camera.width", 640L));
    K.height = static_cast<int>(p("camera.height", 480L));
    K.k1 = p("camera.k1", 0.0);
    K.k2 = p("camera.k2", 0.0);
    projection_.set_intrinsics(K);

    // 相机光学系在 odom 里的位姿来自 TF，不需要在这里配外参 ——
    // 外参在 URDF 里（camera_joint 的 origin + camera_optical_joint 的 rpy）。
    // 这么做的意义是：改机械结构只需要改 URDF，投影链路自动跟随。
    odom_frame_ = p("odom_frame", std::string("odom"));
    camera_frame_ = p("camera_frame", std::string("camera_optical_frame"));

    // ---- 视觉判据（对应 demo 的 min_area=2000 / max_area=30000）----
    cfg_.min_area_px       = static_cast<float>(p("vision.min_area_px", 2000.0));
    cfg_.max_area_px       = static_cast<float>(p("vision.max_area_px", 30000.0));
    cfg_.min_confidence    = static_cast<float>(p("vision.min_confidence", 0.55));
    cfg_.min_stable_frames = static_cast<uint32_t>(p("vision.min_stable_frames", 3L));

    TrackerConfig tc;
    tc.match_radius        = p("tracker.match_radius", 0.060);
    tc.min_stable_frames   = cfg_.min_stable_frames;
    tc.max_missing_frames  = static_cast<uint32_t>(p("tracker.max_missing_frames", 5L));
    tc.track_ttl           = p("tracker.track_ttl", 20.0);
    tracker_.configure(tc);

    // ---- 可抓范围 ----
    cfg_.grasp_radius_min  = static_cast<float>(p("grasp.radius_min", 0.18));
    cfg_.grasp_radius_max  = static_cast<float>(p("grasp.radius_max", 0.85));
    cfg_.grasp_bearing_max = static_cast<float>(p("grasp.bearing_max", 0.60));

    // ---- 运动 ----
    cfg_.approach_speed    = static_cast<float>(p("motion.approach_speed", 0.35));
    cfg_.align_speed       = static_cast<float>(p("motion.align_speed", 0.09));
    cfg_.scan_yaw_rate     = static_cast<float>(p("motion.scan_yaw_rate", 0.45));
    cfg_.standoff          = static_cast<float>(p("motion.standoff", 0.34));
    cfg_.standoff_tol      = static_cast<float>(p("motion.standoff_tol", 0.030));
    cfg_.bearing_tol       = static_cast<float>(p("motion.bearing_tol", 0.045));
    cfg_.yaw_kp            = static_cast<float>(p("motion.yaw_kp", 1.8));
    cfg_.lateral_kp        = static_cast<float>(p("motion.lateral_kp", 1.6));
    cfg_.longitudinal_kp   = static_cast<float>(p("motion.longitudinal_kp", 1.2));
    cfg_.speed_min         = static_cast<float>(p("motion.speed_min", 0.02));

    // ---- 超时 ----
    cfg_.scan_timeout      = static_cast<float>(p("timeout.scan", 12.0));
    cfg_.approach_timeout  = static_cast<float>(p("timeout.approach", 15.0));
    cfg_.align_timeout     = static_cast<float>(p("timeout.align", 8.0));
    cfg_.settle_time       = static_cast<float>(p("timeout.settle", 0.35));
    cfg_.descend_time      = static_cast<float>(p("timeout.descend", 0.30));
    cfg_.grasp_timeout     = static_cast<float>(p("timeout.grasp", 1.5));
    cfg_.lift_time         = static_cast<float>(p("timeout.lift", 0.40));
    cfg_.haul_timeout      = static_cast<float>(p("timeout.haul", 45.0));
    cfg_.place_timeout     = static_cast<float>(p("timeout.place", 15.0));
    cfg_.release_time      = static_cast<float>(p("timeout.release", 0.50));
    cfg_.retreat_time      = static_cast<float>(p("timeout.retreat", 1.20));
    cfg_.vision_lost_timeout = static_cast<float>(p("timeout.vision_lost", 0.60));

    cfg_.max_scan_retry    = static_cast<uint32_t>(p("retry.scan", 3L));
    cfg_.max_align_retry   = static_cast<uint32_t>(p("retry.align", 2L));
    cfg_.max_grasp_retry   = static_cast<uint32_t>(p("retry.grasp", 2L));

    // ---- 气泵 ----
    cfg_.grasp_duty          = static_cast<float>(p("pump.grasp_duty", 1.0));
    cfg_.hold_duty           = static_cast<float>(p("pump.hold_duty", 0.75));
    cfg_.use_pressure_check  = p("pump.use_pressure_check", true);
    cfg_.use_pump_pressure_mode = p("pump.use_pressure_mode", true);
    cfg_.vacuum_target_kpa   = static_cast<float>(p("pump.vacuum_target_kpa", -25.0));
    cfg_.vacuum_threshold_kpa = static_cast<float>(p("pump.vacuum_threshold_kpa", 6.0));

    // ---- 投放阵列 ----
    cfg_.place_x         = p("place.x", 0.0);
    cfg_.place_y         = p("place.y", 1.20);
    cfg_.place_yaw       = p("place.yaw", 1.5707963);
    cfg_.place_pitch     = p("place.pitch", 0.12);
    cfg_.place_columns   = static_cast<int>(p("place.columns", 4L));
    cfg_.place_row_pitch = p("place.row_pitch", 0.12);

    cfg_.home_x   = p("home.x", 0.0);
    cfg_.home_y   = p("home.y", 0.0);
    cfg_.home_yaw = p("home.yaw", 0.0);

    publish_targets_ = p("publish_targets", true);

    seq_.configure(cfg_);
  }

  // -------------------------------------------------------------------------
  //  传感器汇聚
  // -------------------------------------------------------------------------
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    vehicle_x_ = m->pose.pose.position.x;
    vehicle_y_ = m->pose.pose.position.y;
    tf2::Quaternion q(m->pose.pose.orientation.x, m->pose.pose.orientation.y,
                      m->pose.pose.orientation.z, m->pose.pose.orientation.w);
    double r, pi, ya;
    tf2::Matrix3x3(q).getRPY(r, pi, ya);
    vehicle_yaw_ = ya;
    vehicle_vx_ = m->twist.twist.linear.x;
    vehicle_wz_ = m->twist.twist.angular.z;
    have_odom_ = true;
  }

  void on_motors(const hw_msgs::msg::MotorStateArray::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    drive_faulted_ = false;
    for (const auto & ms : m->motors) {
      if (ms.state == hw_msgs::msg::MotorState::STATE_FAULT || ms.fault != 0U) {
        drive_faulted_ = true;
        break;
      }
    }
  }

  void on_board(const hw_msgs::msg::BoardState::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    board_fault_ = m->fault;
  }

  void on_pump_state(const hw_msgs::msg::PumpState::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pump_online_ = true;
    pump_pressure_ = m->pressure;
    pump_fault_ = m->fault;
  }

  void on_camera_sync(const hw_msgs::msg::CameraSync::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_sync_stamp_ = rclcpp::Time(m->header.stamp);
    last_sync_frame_ = m->frame_id;
    last_sync_dropped_ = m->dropped;
    have_sync_ = true;
  }

  // -------------------------------------------------------------------------
  //  视觉：投影 + 跟踪
  // -------------------------------------------------------------------------
  void on_blocks(const hw_msgs::msg::DetectedBlockArray::SharedPtr m)
  {
    const rclcpp::Time stamp(m->header.stamp);

    // ---- 拿图像时间戳那一刻的相机位姿 ----
    Eigen::Matrix4d T_oc;
    if (!lookup_camera_transform(stamp, T_oc)) {
      static int warn_div = 0;
      if ((warn_div++ % 100) == 0) {
        RCLCPP_WARN(get_logger(),
                    "无法在 %f 时刻查到 %s -> %s 的 TF，本帧检测被丢弃",
                    stamp.seconds(), camera_frame_.c_str(), odom_frame_.c_str());
      }
      std::lock_guard<std::mutex> lk(mtx_);
      vision_ok_ = false;
      return;
    }

    std::vector<BlockObservation> obs;
    obs.reserve(m->blocks.size());

    for (const auto & b : m->blocks) {
      const GroundPoint gp = projection_.project(b.pixel_x, b.pixel_y, T_oc);
      if (!gp.valid) { continue; }
      // 地面上的合理范围：太近的东西不在地面（可能是车体自身），太远的超出工作区
      if (gp.range < 0.05 || gp.range > 8.0) { continue; }

      BlockObservation o;
      o.pos = Eigen::Vector2d(gp.x, gp.y);
      o.color = static_cast<BlockColor>(b.color);
      o.area_px = b.area;
      o.confidence_px = b.confidence;
      o.pixel_x = b.pixel_x;
      o.pixel_y = b.pixel_y;

      // 物块长边像素长度：由面积和长宽比反推不可靠，这里用 sqrt(area) 作为
      // 特征长度。斜俯视下它会随距离变化，但 project_yaw 只用它来定方向，
      // 长度误差只带来很小的角度误差（一阶小量）。
      const double length_px = std::sqrt(std::max(1.0f, b.area));
      o.yaw = projection_.project_yaw(b.pixel_x, b.pixel_y, b.yaw, length_px, T_oc);
      obs.push_back(o);
    }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      tracker_.update(obs, stamp.seconds());
      vision_ok_ = true;
      blocks_visible_ = static_cast<uint32_t>(obs.size());
    }
  }

  /// 查 camera_optical_frame -> odom 的 4x4 变换
  bool lookup_camera_transform(const rclcpp::Time & stamp, Eigen::Matrix4d & out)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(odom_frame_, camera_frame_, stamp,
                                       rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &) {
      // 时间戳太新（TF 还没跟上）时退回最新可用值。这会让投影带上
      // "最近一帧里程计"的误差，但总比丢帧好 —— 而且这个分支在正常
      // 运行时不应该被走到（图像时间戳必然早于当前时间）。
      try {
        tf = tf_buffer_->lookupTransform(odom_frame_, camera_frame_,
                                         tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        return false;
      }
    }

    const auto & t = tf.transform.translation;
    const auto & q = tf.transform.rotation;
    Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
    eq.normalize();

    out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) = eq.toRotationMatrix();
    out(0, 3) = t.x;
    out(1, 3) = t.y;
    out(2, 3) = t.z;
    return true;
  }

  // -------------------------------------------------------------------------
  //  快照
  // -------------------------------------------------------------------------
  SensorSnapshot snapshot()
  {
    SensorSnapshot s;
    std::lock_guard<std::mutex> lk(mtx_);

    s.t = now().seconds();
    s.vehicle_x = vehicle_x_;
    s.vehicle_y = vehicle_y_;
    s.vehicle_yaw = vehicle_yaw_;
    s.vehicle_vx = vehicle_vx_;
    s.vehicle_wz = vehicle_wz_;

    s.vision_ok = vision_ok_;
    s.blocks_visible = blocks_visible_;

    s.pump_online = pump_online_;
    s.pump_pressure_kpa = pump_pressure_;
    s.pump_fault = pump_fault_;

    s.drive_faulted = drive_faulted_;
    s.estop = false;      // 急停走 EStop 服务/话题，这里由 Sequencer 的 estop 字段承载

    // 当前绑定目标（由 pick_target 更新）
    if (bound_) {
      s.target_valid = true;
      s.target_x = bound_pos_.x();
      s.target_y = bound_pos_.y();
      s.target_yaw = bound_yaw_;
      s.target_color = bound_color_;
      s.target_confidence = bound_conf_;
    }
    return s;
  }

  // -------------------------------------------------------------------------
  //  目标挑选
  // -------------------------------------------------------------------------
  void pick_target(const SensorSnapshot & s)
  {
    // 只有在需要目标的状态才去挑，避免在搬运途中把目标换掉
    const TaskState st = seq_.state();
    const bool need = (st == TaskState::Scan) || (st == TaskState::Approach) ||
                      (st == TaskState::Align);
    if (!need || !s.vision_ok) {
      return;
    }

    const Eigen::Vector2d veh(s.vehicle_x, s.vehicle_y);
    const auto excluded = seq_.excluded_ids();

    /* tracker_ 的读也走同一把锁。当前用的是单线程 executor，订阅回调与
     * 定时器回调天然串行，理论上不会有竞态；但只要有任何人把 main() 里的
     * spin 换成 MultiThreadedExecutor，这里就会变成数据竞争。加锁的成本
     * （纳秒级、无争用）远低于这个潜在风险。 */
    BlockTrack best;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      best = tracker_.pick(cfg_, veh, s.vehicle_yaw, excluded);
    }
    if (!best.stable) {
      return;
    }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      bound_ = true;
      bound_pos_ = best.pos;
      bound_yaw_ = best.yaw;
      bound_color_ = best.color;
      bound_conf_ = static_cast<float>(best.confidence);
      bound_id_ = best.id;
    }
    seq_.bind_target(best.id, best.pos, best.color, best.yaw,
                     static_cast<float>(best.confidence));
  }

  // -------------------------------------------------------------------------
  //  执行动作
  // -------------------------------------------------------------------------
  void apply(const Actuation & a)
  {
    // ---- 底盘 ----
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = clampd(a.cmd_vx, -0.5, 0.5);
    cmd.linear.y = clampd(a.cmd_vy, -0.5, 0.5);
    cmd.angular.z = clampd(a.cmd_wz, -1.5, 1.5);
    pub_cmd_->publish(cmd);

    // ---- 气泵 ----
    if (a.pump_run != last_pump_run_ || std::abs(a.pump_duty - last_pump_duty_) > 0.02) {
      hw_msgs::msg::PumpCommand pc;
      pc.header.stamp = now();
      pc.cmd = hw_msgs::msg::PumpCommand::CMD_RUN;
      if (cfg_.use_pump_pressure_mode) {
        // 让驱动板的压力环去维持真空。target_pressure 用绝对值语义，
        // 负号由固件的压力环方向处理（见 firmware pump.c 的符号约定）。
        pc.mode = hw_msgs::msg::PumpCommand::MODE_PRESSURE;
        pc.target_pressure = cfg_.vacuum_target_kpa;
        pc.duty = 0.0f;
      } else {
        pc.mode = hw_msgs::msg::PumpCommand::MODE_OPEN_DUTY;
        pc.duty = static_cast<float>(a.pump_duty);
        pc.target_pressure = 0.0f;
      }
      pub_pump_->publish(pc);
      last_pump_run_ = a.pump_run;
      last_pump_duty_ = a.pump_duty;
    } else if (!a.pump_run && last_pump_run_) {
      hw_msgs::msg::PumpCommand pc;
      pc.header.stamp = now();
      pc.cmd = hw_msgs::msg::PumpCommand::CMD_STOP;
      pub_pump_->publish(pc);
      last_pump_run_ = false;
      last_pump_duty_ = 0.0;
    }

    // ---- 目标切换请求 ----
    if (a.cancel_block) {
      std::lock_guard<std::mutex> lk(mtx_);
      bound_ = false;
      seq_.clear_target();
    }
  }

  // -------------------------------------------------------------------------
  //  控制循环
  // -------------------------------------------------------------------------
  void control_loop()
  {
    if (!have_odom_) {
      RCLCPP_WARN_ONCE(get_logger(), "还没收到 /odom，等待 motor_control 启动");
      return;
    }

    SensorSnapshot s = snapshot();
    pick_target(s);
    s = snapshot();          // 重新取一次，带上刚绑定的目标

    const Actuation a = seq_.update(s, 0.0);
    apply(a);
    publish_status(s, a);

    if (a.finished || a.aborted) {
      // 终态：确保底盘停住、气泵按 Sequencer 的决定执行
      RCLCPP_INFO(get_logger(), "%s", a.note.c_str());
    }
  }

  // -------------------------------------------------------------------------
  //  状态发布
  // -------------------------------------------------------------------------
  void publish_status(const SensorSnapshot & s, const Actuation & a)
  {
    const auto & st = seq_.status();

    hw_msgs::msg::TaskStatus msg;
    msg.header.stamp = now();
    msg.state = static_cast<uint8_t>(st.state);
    msg.prev_state = static_cast<uint8_t>(st.prev_state);
    msg.target_index = st.target_index;
    msg.total_targets = st.total_targets;
    msg.grasped_count = st.grasped_count;
    msg.skipped_count = st.skipped_count;
    msg.retry_count = st.retry_count;
    msg.elapsed = st.elapsed;
    msg.message = st.message;
    msg.pump_pressure = static_cast<float>(s.pump_pressure_kpa);
    msg.target_distance = st.target_distance;
    msg.pose_confidence = st.pose_confidence;
    msg.vision_ok = s.vision_ok;
    pub_status_->publish(msg);

    // ---- 可抓目标可视化 ----
    if (publish_targets_) {
      geometry_msgs::msg::PoseArray arr;
      arr.header.stamp = now();
      arr.header.frame_id = odom_frame_;

      std::lock_guard<std::mutex> lk(mtx_);
      const auto stable = tracker_.stable_tracks();
      for (const auto & tr : stable) {
        geometry_msgs::msg::Pose p;
        p.position.x = tr.pos.x();
        p.position.y = tr.pos.y();
        p.position.z = 0.0;
        // 用四元数把物块朝向编码进去，RViz 里能看到方块的角度
        const Eigen::Quaterniond q(Eigen::AngleAxisd(tr.yaw, Eigen::Vector3d::UnitZ()));
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();
        arr.poses.push_back(p);
      }
      pub_targets_->publish(arr);
    }
  }

  // -------------------------------------------------------------------------
  //  服务
  // -------------------------------------------------------------------------
  void handle_start(const std::shared_ptr<hw_msgs::srv::StartTask::Request> req,
                    std::shared_ptr<hw_msgs::srv::StartTask::Response> res)
  {
    StartRequest sr;
    sr.max_blocks = req->max_blocks;
    sr.require_stable = req->require_stable;
    sr.return_home_after = req->return_home_after;
    for (auto c : req->color_order) {
      sr.color_order.push_back(static_cast<BlockColor>(c));
    }

    const bool ok = seq_.start(sr, snapshot());
    res->success = ok;
    if (ok) {
      res->planned_targets = sr.max_blocks;
      res->message = "任务已启动";
      RCLCPP_INFO(get_logger(), "任务启动（max_blocks=%u）", sr.max_blocks);
    } else {
      res->planned_targets = 0;
      res->message = "启动失败：任务已在运行或底盘故障";
      RCLCPP_WARN(get_logger(), "任务启动被拒绝：%s", res->message.c_str());
    }
  }

  void handle_abort(const std::shared_ptr<hw_msgs::srv::AbortTask::Request> req,
                    std::shared_ptr<hw_msgs::srv::AbortTask::Response> res)
  {
    const auto before = seq_.state();
    const bool ok = seq_.abort(req->release_payload, req->return_home, snapshot());
    res->success = ok;
    res->last_state = static_cast<uint8_t>(before);
    res->message = ok
      ? (req->release_payload ? "已中止并释放负载" : "已中止，保持吸附")
      : "当前没有运行中的任务";
  }

  void handle_calibrate(const std::shared_ptr<hw_msgs::srv::CalibrateMapping::Request> req,
                        std::shared_ptr<hw_msgs::srv::CalibrateMapping::Response> res)
  {
    Eigen::Matrix4d T_oc;
    if (!lookup_camera_transform(now(), T_oc)) {
      res->success = false;
      res->message = "TF 不可用，无法标定";
      return;
    }

    // 用当前残差先算一次投影，得到"校正前"的误差
    const GroundPoint before = projection_.project(req->pixel_x, req->pixel_y, T_oc);
    const float err_before = before.valid
      ? static_cast<float>(std::hypot(before.x - req->world_x, before.y - req->world_y))
      : -1.0f;

    const double err_after = projection_.add_calibration_point(
      req->pixel_x, req->pixel_y, req->world_x, req->world_y, T_oc);

    if (err_after < 0.0) {
      res->success = false;
      res->message = "该像素无法投到地面（超出视野或射线朝天）";
      return;
    }

    res->success = true;
    res->residual_before_m = err_before;
    res->residual_after_m = static_cast<float>(err_after);

    /* 把"到底解出了什么"告诉调用方。
     * 标定点太少或挤在一起时，完整的 2D 仿射是病态问题，代码会自动退化成
     * 只校正平移。这件事必须让操作员知道 —— 否则他会以为旋转/尺度也校好了，
     * 而实际上离开标定区域几米之后误差会很大。 */
    const std::size_t n = projection_.calibration_points();
    const double spread = projection_.calibration_spread();
    if (projection_.affine_solved()) {
      res->message = "已采集第 " + std::to_string(n) +
                     " 个标定点，解出完整 2D 仿射（跨度 " +
                     std::to_string(spread * 100).substr(0, 4) + " cm）";
    } else {
      res->message = "已采集第 " + std::to_string(n) +
                     " 个标定点，仅校正平移（标定点跨度 " +
                     std::to_string(spread * 100).substr(0, 4) +
                     " cm，小于 " +
                     std::to_string(GroundProjection::kMinCalibSpread * 100).substr(0, 4) +
                     " cm）。要校正旋转/尺度请把标定块分散摆到工作区四角再采点。";
    }

    RCLCPP_INFO(get_logger(),
                "标定点 #%zu：像素(%.1f, %.1f) -> 世界(%.3f, %.3f)，误差 %.4f m -> %.4f m，%s",
                n, req->pixel_x, req->pixel_y,
                req->world_x, req->world_y, err_before, err_after,
                projection_.affine_solved() ? "完整仿射" : "仅平移");
  }

  static double clampd(double v, double lo, double hi)
  {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  // -------------------------------------------------------------------------
  //  成员
  // -------------------------------------------------------------------------
  TaskConfig       cfg_{};
  TaskSequencer    seq_{};
  BlockTracker     tracker_{};
  GroundProjection projection_{};

  std::string odom_frame_{"odom"};
  std::string camera_frame_{"camera_optical_frame"};
  bool publish_targets_{true};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mtx_;
  double vehicle_x_{0.0}, vehicle_y_{0.0}, vehicle_yaw_{0.0};
  double vehicle_vx_{0.0}, vehicle_wz_{0.0};
  bool   have_odom_{false};
  bool   drive_faulted_{false};
  uint16_t board_fault_{0};

  bool   pump_online_{false};
  double pump_pressure_{0.0};
  uint8_t pump_fault_{0};

  bool   vision_ok_{false};
  uint32_t blocks_visible_{0};
  rclcpp::Time last_sync_stamp_{0, 0, RCL_ROS_TIME};
  uint32_t last_sync_frame_{0};
  uint16_t last_sync_dropped_{0};
  bool   have_sync_{false};

  bool     bound_{false};
  uint32_t bound_id_{0};
  Eigen::Vector2d bound_pos_{0.0, 0.0};
  double   bound_yaw_{0.0};
  BlockColor bound_color_{BlockColor::Unknown};
  float    bound_conf_{0.0f};

  bool   last_pump_run_{false};
  double last_pump_duty_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     pub_cmd_;
  rclcpp::Publisher<hw_msgs::msg::PumpCommand>::SharedPtr     pub_pump_;
  rclcpp::Publisher<hw_msgs::msg::TaskStatus>::SharedPtr      pub_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_targets_;

  rclcpp::Subscription<hw_msgs::msg::DetectedBlockArray>::SharedPtr sub_blocks_;
  rclcpp::Subscription<hw_msgs::msg::CameraSync>::SharedPtr         sub_sync_;
  rclcpp::Subscription<hw_msgs::msg::MotorStateArray>::SharedPtr    sub_motors_;
  rclcpp::Subscription<hw_msgs::msg::BoardState>::SharedPtr         sub_board_;
  rclcpp::Subscription<hw_msgs::msg::PumpState>::SharedPtr          sub_pump_state_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          sub_odom_;

  rclcpp::Service<hw_msgs::srv::StartTask>::SharedPtr         srv_start_;
  rclcpp::Service<hw_msgs::srv::AbortTask>::SharedPtr         srv_abort_;
  rclcpp::Service<hw_msgs::srv::CalibrateMapping>::SharedPtr  srv_calib_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          srv_pause_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          srv_resume_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hw_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hw_task::TaskExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
