// ============================================================================
//  task_executor_node.cpp
//  把 TaskSequencer 接到 ROS 上：喂传感器、执行动作、发布状态。
//
//  这个文件刻意只做"翻译"，不含任何决策逻辑 —— 所有 if/else 都在
//  task_sequencer.cpp 里。这样现场改流程只需要动那一个文件，
//  而且改完可以先在 PC 上跑回归再上机。
//
//  依赖的关键上游
//  --------------
//    /detected_blocks        block_detector_node（视觉）——只给像素与颜色
//    /hw/camera_sync         驱动板硬同步的帧事件（时间基准、丢帧统计）
//    /arm/status             arm_control：末端实测位姿与速度、ik_ok
//    /hw/pump_state          真空压力反馈（判定是否吸住）
//    /hw/board_state         母线电压、板级故障
//
//  下发的
//  ------
//    /arm/target_pose        末端目标位姿（arm_control 负责逆解与轨迹规划）
//    /hw/pump_command        气泵（真空吸盘）
//    /task_status            状态机状态
//    /grasp_targets          当前视野内的可抓目标（给 RViz / 上位机界面看）
//
//  一个容易踩的坑：末端目标不能每个周期都发
//  ----------------------------------------
//  arm_control 收到目标就重新规划一条从当前位置到目标的轨迹。如果每 20ms
//  重复发同一个目标，就会每个周期重规划一次，而轨迹起点一直在动 ——
//  末端会持续抖动而且永远"到不了"。所以只有 Actuation.set_pose 为真
//  （也就是状态机确实换了目标点）时才发。这个判断在状态机里做，
//  这一层照做即可。
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hw_msgs/msg/arm_status.hpp"
#include "hw_msgs/msg/arm_target.hpp"
#include "hw_msgs/msg/board_state.hpp"
#include "hw_msgs/msg/camera_sync.hpp"
#include "hw_msgs/msg/detected_block_array.hpp"
#include "hw_msgs/msg/grasp_target.hpp"
#include "hw_msgs/msg/pump_command.hpp"
#include "hw_msgs/msg/pump_state.hpp"
#include "hw_msgs/msg/task_status.hpp"
#include "hw_msgs/srv/abort_task.hpp"
#include "hw_msgs/srv/calibrate_mapping.hpp"
#include "hw_msgs/srv/start_task.hpp"

#include "hw_task/block_tracker.hpp"
#include "hw_task/table_projection.hpp"
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
    pub_target_  = create_publisher<hw_msgs::msg::ArmTarget>("/arm/target_pose", 10);
    pub_pump_    = create_publisher<hw_msgs::msg::PumpCommand>("/hw/pump_command", 10);
    pub_status_  = create_publisher<hw_msgs::msg::TaskStatus>("/task_status", 10);
    pub_targets_ = create_publisher<geometry_msgs::msg::PoseArray>("/grasp_targets", 10);

    // ---------------- 订阅 ----------------
    sub_blocks_ = create_subscription<hw_msgs::msg::DetectedBlockArray>(
      "/detected_blocks", rclcpp::SensorDataQoS(),
      [this](hw_msgs::msg::DetectedBlockArray::SharedPtr m) { on_blocks(m); });

    sub_sync_ = create_subscription<hw_msgs::msg::CameraSync>(
      "/hw/camera_sync", rclcpp::QoS(20),
      [this](hw_msgs::msg::CameraSync::SharedPtr m) { on_camera_sync(m); });

    sub_arm_ = create_subscription<hw_msgs::msg::ArmStatus>(
      "/arm/status", rclcpp::QoS(20),
      [this](hw_msgs::msg::ArmStatus::SharedPtr m) { on_arm(m); });

    sub_board_ = create_subscription<hw_msgs::msg::BoardState>(
      "/hw/board_state", rclcpp::QoS(10),
      [this](hw_msgs::msg::BoardState::SharedPtr m) { on_board(m); });

    sub_pump_state_ = create_subscription<hw_msgs::msg::PumpState>(
      "/hw/pump_state", rclcpp::QoS(10),
      [this](hw_msgs::msg::PumpState::SharedPtr m) { on_pump_state(m); });

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
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        const bool ok = seq_.pause(snapshot());
        res->success = ok;
        res->message = ok ? "已暂停" : "当前状态无法暂停";
      });

    srv_resume_ = create_service<std_srvs::srv::Trigger>(
      "/task/resume",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
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
                "任务执行器就绪：控制 %0.0f Hz，工作半径 %.2f~%.2f m，"
                "悬停高度 %.3f m，投放点 (%.2f, %.2f)",
                rate, cfg_.reach_radius_min, cfg_.reach_radius_max,
                cfg_.hover_z, cfg_.place_x, cfg_.place_y);
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

    /* 相机外参来自 TF（URDF 里的 camera_joint + camera_optical_joint），
     * 这里不配外参。意义是：相机是装在工作台支架上还是装在末端上，
     * 只影响 URDF，投影链路一行都不用改。 */
    base_frame_ = p("base_frame", std::string("base_link"));
    camera_frame_ = p("camera_frame", std::string("camera_optical_frame"));

    // ---- 视觉判据（沿用 demo 的 min_area=2000 / max_area=30000）----
    cfg_.min_area_px       = static_cast<float>(p("vision.min_area_px", 2000.0));
    cfg_.max_area_px       = static_cast<float>(p("vision.max_area_px", 30000.0));
    cfg_.min_confidence    = static_cast<float>(p("vision.min_confidence", 0.55));
    cfg_.min_stable_frames = static_cast<uint32_t>(p("vision.min_stable_frames", 3L));

    TrackerConfig tc;
    tc.match_radius       = p("tracker.match_radius", 0.040);
    tc.min_stable_frames  = cfg_.min_stable_frames;
    tc.max_missing_frames = static_cast<uint32_t>(p("tracker.max_missing_frames", 5L));
    tc.track_ttl          = p("tracker.track_ttl", 20.0);
    tracker_.configure(tc);

    // ---- 可达范围与末端高度 ----
    cfg_.reach_radius_min = static_cast<float>(p("reach.radius_min", 0.12));
    cfg_.reach_radius_max = static_cast<float>(p("reach.radius_max", 0.40));
    cfg_.hover_z          = static_cast<float>(p("height.hover_z", 0.050));
    cfg_.touch_z          = static_cast<float>(p("height.touch_z", 0.023));
    cfg_.travel_z         = static_cast<float>(p("height.travel_z", 0.100));
    cfg_.retreat_z        = static_cast<float>(p("height.retreat_z", 0.080));

    // ---- 到位判据 ----
    cfg_.pos_tol   = static_cast<float>(p("tolerance.pos_m", 0.002));
    cfg_.vel_tol   = static_cast<float>(p("tolerance.vel_mps", 0.010));
    cfg_.servo_tol = static_cast<float>(p("tolerance.servo_m", 0.0015));

    // ---- 超时 ----
    cfg_.scan_timeout        = static_cast<float>(p("timeout.scan", 12.0));
    cfg_.approach_timeout    = static_cast<float>(p("timeout.approach", 8.0));
    cfg_.align_timeout       = static_cast<float>(p("timeout.align", 6.0));
    cfg_.settle_time         = static_cast<float>(p("timeout.settle", 0.30));
    cfg_.descend_time        = static_cast<float>(p("timeout.descend", 0.25));
    cfg_.grasp_timeout       = static_cast<float>(p("timeout.grasp", 1.5));
    cfg_.lift_time           = static_cast<float>(p("timeout.lift", 0.25));
    cfg_.haul_timeout        = static_cast<float>(p("timeout.haul", 12.0));
    cfg_.place_timeout       = static_cast<float>(p("timeout.place", 6.0));
    cfg_.release_time        = static_cast<float>(p("timeout.release", 0.40));
    cfg_.retreat_time        = static_cast<float>(p("timeout.retreat", 0.60));
    cfg_.vision_lost_timeout = static_cast<float>(p("timeout.vision_lost", 0.60));

    cfg_.max_scan_retry  = static_cast<uint32_t>(p("retry.scan", 3L));
    cfg_.max_align_retry = static_cast<uint32_t>(p("retry.align", 2L));
    cfg_.max_grasp_retry = static_cast<uint32_t>(p("retry.grasp", 2L));

    // ---- 气泵 ----
    cfg_.grasp_duty             = static_cast<float>(p("pump.grasp_duty", 1.0));
    cfg_.hold_duty              = static_cast<float>(p("pump.hold_duty", 0.75));
    cfg_.use_pressure_check     = p("pump.use_pressure_check", true);
    cfg_.use_pump_pressure_mode = p("pump.use_pressure_mode", true);
    cfg_.vacuum_target_kpa      = static_cast<float>(p("pump.vacuum_target_kpa", -25.0));
    cfg_.vacuum_threshold_kpa   = static_cast<float>(p("pump.vacuum_threshold_kpa", 6.0));

    // ---- 投放 ----
    const std::string mode = p("place.mode", std::string("stack"));
    cfg_.place_mode          = (mode == "array") ? PlaceMode::Array : PlaceMode::Stack;
    cfg_.place_x             = p("place.x", 0.28);
    cfg_.place_y             = p("place.y", -0.14);
    cfg_.place_yaw           = p("place.yaw", 0.0);
    cfg_.place_pitch         = p("place.pitch", 0.090);
    cfg_.place_row_pitch     = p("place.row_pitch", 0.090);
    cfg_.place_columns       = static_cast<int>(p("place.columns", 3L));
    cfg_.place_rows          = static_cast<int>(p("place.rows", 3L));
    cfg_.place_layer_height  = p("place.layer_height", 0.025);
    cfg_.place_max_layers    = static_cast<int>(p("place.max_layers", 4L));

    // ---- 起点 ----
    cfg_.home_x   = p("home.x", 0.28);
    cfg_.home_y   = p("home.y", 0.0);
    cfg_.home_z   = p("home.z", 0.100);
    cfg_.home_yaw = p("home.yaw", 0.0);

    publish_targets_ = p("publish_targets", true);

    seq_.configure(cfg_);
  }

  // -------------------------------------------------------------------------
  //  传感器汇聚
  // -------------------------------------------------------------------------
  void on_arm(const hw_msgs::msg::ArmStatus::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    tool_x_ = m->current_pose.pose.position.x;
    tool_y_ = m->current_pose.pose.position.y;
    tool_z_ = m->current_pose.pose.position.z;
    tool_vx_ = m->tool_twist.linear.x;
    tool_vy_ = m->tool_twist.linear.y;
    tool_vz_ = m->tool_twist.linear.z;

    /* ik_ok 的语义是"上一个下达的目标被接受了"。刚启动、还没下达过任何目标时
     * arm_control 报的是 true，正好符合"不需要因为不可达而跳过目标"的默认。 */
    ik_ok_ = m->ik_ok;

    /* 只有 MOVING/FAULT 才算"机械臂有问题"。
     * REJECTED 只代表"上一次目标被拒"，那个由 ik_ok 单独承载 ——
     * 把它也算进 arm_faulted 会让状态机直接判 FAILED 而不是跳过该物块。 */
    arm_faulted_ = (m->state == hw_msgs::msg::ArmStatus::STATE_FAULT);

    have_arm_ = true;
  }

  void on_board(const hw_msgs::msg::BoardState::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    board_fault_ = m->fault;
    // 板级故障也会让关节失去响应，对任务层来说和关节故障等价
    if (m->fault != 0U) { arm_faulted_ = true; }
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
    /* 相机是固定安装的，这个变换其实是静态的。仍然按图像时间戳查，理由是：
     *   · 如果将来把相机装到末端上（eye-in-hand），这条链路不用改；
     *   · 图像在传感器里曝光到发布之间有一段延迟，用"当前时刻"的 TF
     *     会让每一帧都带上一份与图像不对应的位姿误差。
     */
    Eigen::Matrix4d T_bc;
    if (!lookup_camera_transform(stamp, T_bc)) {
      static int warn_div = 0;
      if ((warn_div++ % 100) == 0) {
        RCLCPP_WARN(get_logger(),
                    "无法在 %f 时刻查到 %s -> %s 的 TF，本帧检测被丢弃",
                    stamp.seconds(), camera_frame_.c_str(), base_frame_.c_str());
      }
      std::lock_guard<std::mutex> lk(mtx_);
      vision_ok_ = false;
      return;
    }

    std::vector<BlockObservation> obs;
    obs.reserve(m->blocks.size());

    for (const auto & b : m->blocks) {
      const TablePoint tp = projection_.project(b.pixel_x, b.pixel_y, T_bc);
      if (!tp.valid) { continue; }
      /* 工作台上的合理范围：太近的（离相机正下方 5cm 内，落在基座法兰上）
       * 与太远的（超出工作台）都不要。这里的 8m 只是个"离谱值"的兜底，
       * 真正决定能不能抓的是 cfg_.reach_radius_*。 */
      if (tp.range < 0.05 || tp.range > 8.0) { continue; }

      BlockObservation o;
      o.pos = Eigen::Vector2d(tp.x, tp.y);
      o.color = static_cast<BlockColor>(b.color);
      o.area_px = b.area;
      o.confidence_px = b.confidence;
      o.pixel_x = b.pixel_x;
      o.pixel_y = b.pixel_y;

      // 物块长边方向。吸盘是圆形的，用不到它，但它在 RViz 里能直观
      // 显示"视觉认为物块朝哪边"，标定时很有用。
      const double length_px = std::sqrt(std::max(1.0f, b.area));
      o.yaw = projection_.project_yaw(b.pixel_x, b.pixel_y, b.yaw, length_px, T_bc);
      obs.push_back(o);
    }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      tracker_.update(obs, stamp.seconds());
      vision_ok_ = true;
      blocks_visible_ = static_cast<uint32_t>(obs.size());
    }
  }

  /// 查 camera_optical_frame -> base_link 的 4x4 变换
  bool lookup_camera_transform(const rclcpp::Time & stamp, Eigen::Matrix4d & out)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, camera_frame_, stamp,
                                       rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &) {
      // 时间戳太新（静态 TF 还没进入缓冲）时退回最新可用值。
      try {
        tf = tf_buffer_->lookupTransform(base_frame_, camera_frame_,
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

    s.tool_x = tool_x_;
    s.tool_y = tool_y_;
    s.tool_z = tool_z_;
    s.tool_vx = tool_vx_;
    s.tool_vy = tool_vy_;
    s.tool_vz = tool_vz_;

    s.vision_ok = vision_ok_;
    s.blocks_visible = blocks_visible_;

    s.pump_online = pump_online_;
    s.pump_pressure_kpa = pump_pressure_;
    s.pump_fault = pump_fault_;

    s.arm_faulted = arm_faulted_;
    s.ik_ok = ik_ok_;
    s.estop = false;      // 急停走 CLS_ESTOP / 服务，不在常规快照里

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

    /* 参考点取基座回转轴在台面上的投影 (0, 0)：排序与可达性判断都是
     * "离基座多远"。底盘方案里这里传的是车体位置（还要带车头朝向），
     * 机械臂没有可移动的机体，基座就是唯一合理的参考点。 */
    const auto excluded = seq_.excluded_ids();

    /* tracker_ 的读也走同一把锁。当前用的是单线程 executor，订阅回调与
     * 定时器回调天然串行，理论上不会有竞态；但只要有人把 main() 里的
     * spin 换成 MultiThreadedExecutor，这里就会变成数据竞争。加锁的成本
     * （纳秒级、无争用）远低于这个潜在风险。 */
    BlockTrack best;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      best = tracker_.pick(cfg_, Eigen::Vector2d(0.0, 0.0), excluded);
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
    seq_.bind_target(best.id, best.pos.x(), best.pos.y(), best.color, best.yaw,
                     static_cast<float>(best.confidence));
  }

  // -------------------------------------------------------------------------
  //  执行动作
  // -------------------------------------------------------------------------
  void apply(const Actuation & a)
  {
    // ---- 末端目标 ----
    // 只在状态机换了目标点时发（见文件头的说明）
    if (a.set_pose) {
      hw_msgs::msg::ArmTarget t;
      t.header.stamp = now();
      t.header.frame_id = base_frame_;
      t.target_pose.position.x = a.pose_x;
      t.target_pose.position.y = a.pose_y;
      t.target_pose.position.z = a.pose_z;
      // 末端只有绕 Z 的一个自由度，四元数里只填 yaw
      t.target_pose.orientation.z = std::sin(a.pose_yaw * 0.5);
      t.target_pose.orientation.w = std::cos(a.pose_yaw * 0.5);
      t.move_kind = hw_msgs::msg::ArmTarget::MOVE_CARTESIAN;
      t.speed_scale = a.speed_scale;
      t.allow_waypoint = a.allow_waypoint;
      pub_target_->publish(t);
    }

    // ---- 气泵 ----
    if (a.pump_run != last_pump_run_) {
      hw_msgs::msg::PumpCommand pc;
      pc.header.stamp = now();
      if (a.pump_run) {
        pc.cmd = hw_msgs::msg::PumpCommand::CMD_RUN;
        if (cfg_.use_pump_pressure_mode) {
          /* 让驱动板的压力环去维持真空。target_pressure 用绝对值语义，
           * 负号由固件的压力环方向处理（见 firmware pump.c 的符号约定）。
           * 好处是漏气时驱动板自动加大功率，上位机只需看压力是否达标。 */
          pc.mode = hw_msgs::msg::PumpCommand::MODE_PRESSURE;
          pc.target_pressure = std::fabs(cfg_.vacuum_target_kpa);
          pc.duty = 0.0f;
        } else {
          pc.mode = hw_msgs::msg::PumpCommand::MODE_OPEN_DUTY;
          pc.duty = static_cast<float>(a.pump_duty);
          pc.target_pressure = 0.0f;
        }
      } else {
        pc.cmd = hw_msgs::msg::PumpCommand::CMD_STOP;
        pc.mode = hw_msgs::msg::PumpCommand::MODE_OPEN_DUTY;
        pc.duty = 0.0f;
      }
      pub_pump_->publish(pc);
      last_pump_run_ = a.pump_run;
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
    if (!have_arm_) {
      RCLCPP_WARN_ONCE(get_logger(),
                       "还没收到 /arm/status，等待 arm_control 启动"
                       "（它负责给出末端实测位姿，任务层靠它判到位）");
      return;
    }

    SensorSnapshot s = snapshot();
    pick_target(s);
    s = snapshot();          // 重新取一次，带上刚绑定的目标

    const Actuation a = seq_.update(s, 0.0);
    apply(a);
    publish_status(s, a);
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
    msg.target_range = st.target_range;
    msg.pose_confidence = st.pose_confidence;
    msg.vision_ok = s.vision_ok;
    msg.place_layer = st.place_layer;
    pub_status_->publish(msg);

    // ---- 可抓目标可视化 ----
    if (publish_targets_) {
      geometry_msgs::msg::PoseArray arr;
      arr.header.stamp = now();
      arr.header.frame_id = base_frame_;

      std::lock_guard<std::mutex> lk(mtx_);
      const auto stable = tracker_.stable_tracks();
      for (const auto & tr : stable) {
        geometry_msgs::msg::Pose p;
        p.position.x = tr.pos.x();
        p.position.y = tr.pos.y();
        p.position.z = 0.0;   // 工作台面
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
      res->message = "启动失败：任务已在运行，或机械臂处于故障状态";
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
    Eigen::Matrix4d T_bc;
    if (!lookup_camera_transform(now(), T_bc)) {
      res->success = false;
      res->message = "TF 不可用，无法标定（检查 robot_state_publisher 是否在跑）";
      return;
    }

    // 用当前残差先算一次投影，得到"校正前"的误差
    const TablePoint before = projection_.project(req->pixel_x, req->pixel_y, T_bc);
    const float err_before = before.valid
      ? static_cast<float>(std::hypot(before.x - req->world_x, before.y - req->world_y))
      : -1.0f;

    const double err_after = projection_.add_calibration_point(
      req->pixel_x, req->pixel_y, req->world_x, req->world_y, T_bc);

    if (err_after < 0.0) {
      res->success = false;
      res->message = "该像素无法投到工作台面（超出视野或射线朝天）";
      return;
    }

    res->success = true;
    res->residual_before_m = err_before;
    res->residual_after_m = static_cast<float>(err_after);

    /* 把"到底解出了什么"告诉调用方。
     * 标定点太少或挤在一起时，完整的 2D 仿射是病态问题，代码会自动退化成
     * 只校正平移。这件事必须让操作员知道 —— 否则他会以为旋转/尺度也校好了，
     * 而实际上离开标定区域之后误差会迅速变大。 */
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
                     std::to_string(TableProjection::kMinCalibSpread * 100).substr(0, 4) +
                     " cm）。要校正旋转/尺度，请把标定块分散摆到工作台四角再采点。";
    }

    RCLCPP_INFO(get_logger(),
                "标定点 #%zu：像素(%.1f, %.1f) -> 台面(%.3f, %.3f)，"
                "误差 %.4f m -> %.4f m，%s",
                n, req->pixel_x, req->pixel_y,
                req->world_x, req->world_y, err_before, err_after,
                projection_.affine_solved() ? "完整仿射" : "仅平移");
  }

  // -------------------------------------------------------------------------
  //  成员
  // -------------------------------------------------------------------------
  TaskConfig      cfg_{};
  TaskSequencer   seq_{};
  BlockTracker    tracker_{};
  TableProjection projection_{};

  std::string base_frame_{"base_link"};
  std::string camera_frame_{"camera_optical_frame"};
  bool publish_targets_{true};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mtx_;
  double tool_x_{0.0}, tool_y_{0.0}, tool_z_{0.0};
  double tool_vx_{0.0}, tool_vy_{0.0}, tool_vz_{0.0};
  bool   ik_ok_{true};
  bool   have_arm_{false};
  bool   arm_faulted_{false};
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

  bool last_pump_run_{false};

  rclcpp::Publisher<hw_msgs::msg::ArmTarget>::SharedPtr        pub_target_;
  rclcpp::Publisher<hw_msgs::msg::PumpCommand>::SharedPtr      pub_pump_;
  rclcpp::Publisher<hw_msgs::msg::TaskStatus>::SharedPtr       pub_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr  pub_targets_;

  rclcpp::Subscription<hw_msgs::msg::DetectedBlockArray>::SharedPtr sub_blocks_;
  rclcpp::Subscription<hw_msgs::msg::CameraSync>::SharedPtr         sub_sync_;
  rclcpp::Subscription<hw_msgs::msg::ArmStatus>::SharedPtr          sub_arm_;
  rclcpp::Subscription<hw_msgs::msg::BoardState>::SharedPtr         sub_board_;
  rclcpp::Subscription<hw_msgs::msg::PumpState>::SharedPtr          sub_pump_state_;

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
