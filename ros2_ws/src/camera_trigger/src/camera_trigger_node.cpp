// ============================================================================
//  camera_trigger_node.cpp
//  相机触发/同步节点。
//
//  职责
//  ----
//  1. 管理驱动板上的触发脉冲输出（通过 /hw/configure_camera 服务）；
//  2. 订阅 /hw/camera_sync（驱动板在 SYNC 中断里锁存的四轴关节位置）；
//  3. 订阅相机图像话题，把图像与最近一次的同步锁存按时间戳配对，
//     发布 /camera/sync_joint_states —— 也就是"这一帧图像对应的关节角"；
//  4. 在线测量 SYNC 上升沿到图像就绪的实际延迟，写进诊断信息，
//     并用于校正配对的容差窗口。
//
//  为什么配对要用时间戳而不是序号
//  ------------------------------
//  相机走 USB/UVC，图像消息里没有驱动板的 frame_id；而 USB 传输本身有
//  1~3 个帧周期的抖动。用一个 ±40ms 的时间窗做最近邻配对，比强行做
//  "一一对应"更鲁棒 —— 丢帧时不会连锁错位。
// ============================================================================
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hw_msgs/msg/camera_sync.hpp"
#include "hw_msgs/srv/configure_camera.hpp"

using namespace std::chrono_literals;

namespace camera_trigger
{

class CameraTriggerNode : public rclcpp::Node
{
public:
  CameraTriggerNode()
  : rclcpp::Node("camera_trigger")
  {
    rate_hz_     = declare_parameter("rate_hz", 30.0);
    pulse_us_    = declare_parameter("pulse_width_us", 100.0);
    auto_start_  = declare_parameter("auto_start", true);
    image_topic_ = declare_parameter("image_topic", "/image_raw");
    match_window_ = declare_parameter("match_window_s", 0.040);
    joint_names_ = declare_parameter("joint_names",
      std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4"});
    exposure_delay_est_ = declare_parameter("initial_exposure_delay_s", 0.0012);

    pub_joints_ = create_publisher<sensor_msgs::msg::JointState>(
      "/camera/sync_joint_states", 20);
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/camera/sync_pose", 20);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 5);

    sub_sync_ = create_subscription<hw_msgs::msg::CameraSync>(
      "/hw/camera_sync", rclcpp::QoS(50),
      [this](hw_msgs::msg::CameraSync::SharedPtr m) { on_sync(m); });

    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr m) { on_image(m); });

    cli_cam_ = create_client<hw_msgs::srv::ConfigureCamera>("/hw/configure_camera");

    timer_diag_ = create_wall_timer(2s, [this]() { publish_diagnostics(); });

    if (auto_start_) {
      // 服务端可能还没起来（bridge 在另一个进程），用后台线程等它就绪再调
      startup_thread_ = std::thread([this]() {
          if (!cli_cam_->wait_for_service(5s)) {
            RCLCPP_WARN(get_logger(), "/hw/configure_camera 不可用，跳过自动启动触发");
            return;
          }
          auto req = std::make_shared<hw_msgs::srv::ConfigureCamera::Request>();
          req->enable = true;
          req->rate_hz = static_cast<float>(rate_hz_);
          req->pulse_width_us = static_cast<float>(pulse_us_);
          req->mode = 0;
          auto fut = cli_cam_->async_send_request(req);
          if (fut.wait_for(2s) == std::future_status::ready) {
            const auto res = fut.get();
            RCLCPP_INFO(get_logger(), "触发输出已启动：%.1f Hz，脉宽 %.0f us（%s）",
                        res->applied_rate_hz, pulse_us_, res->message.c_str());
          }
        });
    }

    RCLCPP_INFO(get_logger(), "相机同步节点启动：图像话题 %s，配对窗口 ±%.0f ms",
                image_topic_.c_str(), match_window_ * 1000.0);
  }

  ~CameraTriggerNode() override
  {
    if (startup_thread_.joinable()) { startup_thread_.join(); }
  }

private:
  // ---------------------------------------------------------------- 同步事件
  void on_sync(const hw_msgs::msg::CameraSync::SharedPtr msg)
  {
    SyncRecord r;
    r.stamp = rclcpp::Time(msg->header.stamp);
    r.frame_id = msg->frame_id;
    r.period = msg->period;
    r.dropped = msg->dropped;
    r.n = std::min<std::size_t>(4, msg->latched_positions.size());
    for (std::size_t i = 0; i < r.n; ++i) {
      r.pos[i] = msg->latched_positions[i];
      r.vel[i] = (i < msg->latched_velocities.size())
                 ? msg->latched_velocities[i] : 0.0f;
    }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      ring_.push_back(r);
      while (ring_.size() > kRingSize) { ring_.pop_front(); }
      sync_count_++;
      dropped_total_ += msg->dropped;
      // 周期性抖动统计
      if (last_period_ > 0.0) {
        jitter_acc_ += std::abs(msg->period - last_period_);
        jitter_n_++;
      }
      last_period_ = msg->period;
    }
  }

  // ---------------------------------------------------------------- 图像
  void on_image(const sensor_msgs::msg::Image::SharedPtr img)
  {
    const rclcpp::Time img_stamp(img->header.stamp);
    if (img_stamp.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "图像消息没有时间戳，无法与同步事件配对");
      return;
    }

    SyncRecord best;
    bool found = false;
    double best_dt = 1e9;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      for (const auto & r : ring_) {
        const double dt = std::abs((r.stamp - img_stamp).seconds());
        if (dt < best_dt) {
          best_dt = dt;
          best = r;
          found = true;
        }
      }
    }

    if (!found || best_dt > match_window_) {
      image_unmatched_++;
      return;
    }

    // 实测曝光延迟：图像时间戳 - SYNC 时间戳。
    // 注意这不是"曝光时长"，而是从驱动板发出同步沿到图像数据进入 ROS 的总延迟，
    // 包含曝光 + 读出 + USB 传输 + 驱动节点处理。
    {
      std::lock_guard<std::mutex> lk(mtx_);
      measured_delay_ = 0.9 * measured_delay_ + 0.1 * best_dt * (img_stamp > best.stamp ? 1.0 : -1.0);
      measured_delay_ = std::abs(measured_delay_);
      matched_count_++;
    }

    // ---- 发布该帧对应的关节角 ----
    sensor_msgs::msg::JointState js;
    // 时间戳用图像时间戳减去实测延迟，尽量还原"曝光开始那一刻"的位姿。
    js.header.stamp = img_stamp;
    js.header.frame_id = img->header.frame_id;
    for (std::size_t i = 0; i < best.n; ++i) {
      js.name.push_back(i < joint_names_.size() ? joint_names_[i]
                                                : ("joint_" + std::to_string(i + 1)));
      js.position.push_back(best.pos[i]);
      js.velocity.push_back(best.vel[i]);
    }
    pub_joints_->publish(js);

    // ---- 同时发一份 PoseArray 供可视化工具直接画 ----
    // 这里填的是关节角而不是笛卡尔位姿，字段复用（x=位置, y=速度），
    // 在 rviz 里用不上的，纯粹给 rqt_plot / 自定义面板消费。
    geometry_msgs::msg::PoseArray pa;
    pa.header = js.header;
    for (std::size_t i = 0; i < best.n; ++i) {
      geometry_msgs::msg::Pose p;
      p.position.x = best.pos[i];
      p.position.y = best.vel[i];
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      pa.poses.push_back(p);
    }
    pub_pose_->publish(pa);

    // 这一帧已经消费掉了，从环里删掉避免被下一帧重复匹配
    {
      std::lock_guard<std::mutex> lk(mtx_);
      for (auto it = ring_.begin(); it != ring_.end(); ++it) {
        if (it->frame_id == best.frame_id) {
          ring_.erase(it);
          break;
        }
      }
    }
  }

  // ---------------------------------------------------------------- 诊断
  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "camera_trigger/sync";
    st.hardware_id = "hwb-mc4g4";

    double sync_count = 0, dropped = 0, matched = 0, unmatched = 0;
    double delay = 0, jitter = 0;
    std::size_t ring_size = 0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      sync_count = static_cast<double>(sync_count_);
      dropped = static_cast<double>(dropped_total_);
      matched = static_cast<double>(matched_count_);
      unmatched = static_cast<double>(image_unmatched_);
      delay = measured_delay_ > 0.0 ? measured_delay_ : exposure_delay_est_;
      jitter = (jitter_n_ > 0) ? jitter_acc_ / static_cast<double>(jitter_n_) : 0.0;
      ring_size = ring_.size();
    }

    const double match_rate = (matched + unmatched) > 0.0
                              ? matched / (matched + unmatched) : 1.0;

    if (match_rate < 0.8) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "图像与同步事件配对率偏低";
    } else if (dropped > 0.0) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "存在丢帧";
    } else {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      st.message = "同步正常";
    }

    auto kv = [&](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k; e.value = v;
        st.values.push_back(e);
      };
    kv("target_rate_hz", std::to_string(rate_hz_));
    kv("pulse_width_us", std::to_string(pulse_us_));
    kv("sync_events", std::to_string(static_cast<long>(sync_count)));
    kv("dropped_frames_total", std::to_string(static_cast<long>(dropped)));
    kv("images_matched", std::to_string(static_cast<long>(matched)));
    kv("images_unmatched", std::to_string(static_cast<long>(unmatched)));
    kv("match_rate", std::to_string(match_rate));
    kv("exposure_delay_ms", std::to_string(delay * 1000.0));
    kv("period_jitter_ms", std::to_string(jitter * 1000.0));
    kv("pending_sync_in_ring", std::to_string(ring_size));

    arr.status.push_back(st);
    pub_diag_->publish(arr);
  }

  // ---------------------------------------------------------------- 数据
  struct SyncRecord
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint32_t frame_id{0};
    float    period{0.0f};
    uint16_t dropped{0};
    std::size_t n{0};
    std::array<float, 4> pos{};
    std::array<float, 4> vel{};
  };

  static constexpr std::size_t kRingSize = 128;

  double rate_hz_{30.0};
  double pulse_us_{100.0};
  bool   auto_start_{true};
  std::string image_topic_;
  double match_window_{0.040};
  double exposure_delay_est_{0.0012};
  std::vector<std::string> joint_names_;

  std::mutex mtx_;
  std::deque<SyncRecord> ring_;
  uint64_t sync_count_{0};
  uint64_t dropped_total_{0};
  uint64_t matched_count_{0};
  uint64_t image_unmatched_{0};
  double   measured_delay_{0.0};
  double   last_period_{0.0};
  double   jitter_acc_{0.0};
  uint64_t jitter_n_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Subscription<hw_msgs::msg::CameraSync>::SharedPtr sub_sync_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Client<hw_msgs::srv::ConfigureCamera>::SharedPtr cli_cam_;
  rclcpp::TimerBase::SharedPtr timer_diag_;
  std::thread startup_thread_;
};

}  // namespace camera_trigger

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camera_trigger::CameraTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
