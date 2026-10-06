// ============================================================================
//  motor_control_node.cpp
//  底盘控制器：/cmd_vel -> 四轴轮速指令 -> /hw/motor_commands
//              /hw/motor_states -> /joint_states + /odom + TF
//
//  设计要点
//  --------
//  1. /cmd_vel 是"事件驱动"进来，但指令必须以固定 100Hz 下发 —— 驱动板的看门狗
//     会在 300ms 收不到上位机帧时安全停机，而且固定节拍让 CAN 负载平稳。
//     因此订阅回调只更新目标值，实际发送由定时器驱动。
//
//  2. 指令斜坡：/cmd_vel 的突变（尤其是导航栈偶尔发的速度跳变）会直接变成
//     电流冲击。这里对 vx/wz 做一阶限速（默认 1.5 m/s²、3.0 rad/s²），
//     斜坡参数可配、可在线关闭。
//
//  3. 里程计用轮速积分而不是电机位置差分：轮速已经过驱动板的一阶低通，
//     比位置差分的量化噪声小一个量级。
// ============================================================================
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "hw_msgs/msg/motor_command_array.hpp"
#include "hw_msgs/msg/motor_state_array.hpp"
#include "motor_control/kinematics.hpp"

using namespace std::chrono_literals;

namespace motor_control
{

class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode()
  : rclcpp::Node("motor_control")
  {
    // ---------------- 参数 ----------------
    const std::string drive = declare_parameter("drive_type", "skid_steer");
    WheelGeometry geo;
    geo.wheel_radius = declare_parameter("wheel_radius", 0.065);
    geo.track_width  = declare_parameter("track_width", 0.42);
    geo.wheel_base   = declare_parameter("wheel_base", 0.30);
    geo.gear_ratio   = declare_parameter("gear_ratio", 1.0);

    kin_ = std::make_unique<Kinematics>(parse_drive_type(drive), geo);

    max_vx_      = declare_parameter("max_linear_velocity", 1.20);      // m/s
    max_wz_      = declare_parameter("max_angular_velocity", 4.00);     // rad/s
    ramp_acc_    = declare_parameter("linear_accel_limit", 1.50);       // m/s^2
    ramp_dec_    = declare_parameter("angular_accel_limit", 3.00);      // rad/s^2
    cmd_rate_hz_ = declare_parameter("command_rate_hz", 100.0);
    current_budget_ = declare_parameter("current_budget_per_wheel", 4.0);  // A
    odom_frame_  = declare_parameter("odom_frame", "odom");
    base_frame_  = declare_parameter("base_frame", "base_link");
    publish_tf_  = declare_parameter("publish_tf", true);
    invert_vector_ = declare_parameter("invert_motor_direction",
                                       std::vector<int64_t>{1, 1, 1, 1});

    if (invert_vector_.size() != 4U) {
      invert_vector_ = {1, 1, 1, 1};
    }

    // ---------------- 发布 / 订阅 ----------------
    pub_cmd_ = create_publisher<hw_msgs::msg::MotorCommandArray>("/hw/motor_commands", 10);
    pub_joint_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);

    sub_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(5),
      [this](geometry_msgs::msg::Twist::SharedPtr m) {
        target_.vx = m->linear.x;
        target_.vy = m->linear.y;
        target_.wz = m->angular.z;
      });

    sub_state_ = create_subscription<hw_msgs::msg::MotorStateArray>(
      "/hw/motor_states", rclcpp::QoS(10),
      [this](hw_msgs::msg::MotorStateArray::SharedPtr m) { on_states(m); });

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // ---------------- 定时器 ----------------
    const auto period = std::chrono::duration<double>(1.0 / cmd_rate_hz_);
    timer_cmd_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() { tick_command(); });

    last_tick_ = now();

    RCLCPP_INFO(get_logger(),
                "底盘控制器已启动：%s，轮半径 %.3f m，轮距 %.3f m，轴距 %.3f m，"
                "指令 %0.0f Hz，限速 %.2f m/s / %.2f rad/s",
                drive.c_str(), geo.wheel_radius, geo.track_width, geo.wheel_base,
                cmd_rate_hz_, max_vx_, max_wz_);
  }

private:
  struct Target
  {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
  };

  // ---------------------------------------------------------------- 指令下发
  void tick_command()
  {
    const rclcpp::Time t = now();
    const double dt = std::max(1e-4, (t - last_tick_).seconds());
    last_tick_ = t;

    // ---- 1. 目标限幅 ----
    const double tvx = std::clamp(target_.vx, -max_vx_, max_vx_);
    const double tvy = std::clamp(target_.vy, -max_vx_, max_vx_);
    const double twz = std::clamp(target_.wz, -max_wz_, max_wz_);

    // ---- 2. 一阶斜坡（按加速度上限逼近目标） ----
    auto approach = [dt](double cur, double tgt, double rate) {
        const double step = rate * dt;
        if (tgt > cur) { return std::min(tgt, cur + step); }
        return std::max(tgt, cur - step);
      };

    cmd_.vx = approach(cmd_.vx, tvx, ramp_acc_);
    cmd_.vy = approach(cmd_.vy, tvy, ramp_acc_);
    cmd_.wz = approach(cmd_.wz, twz, ramp_dec_);

    // ---- 3. 解算轮速 ----
    const auto wheels = kin_->body_to_wheels(cmd_.vx, cmd_.vy, cmd_.wz);

    // ---- 4. 按当前故障状态分配限流 ----
    std::array<uint8_t, 4> faults{};
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      for (std::size_t i = 0; i < 4; ++i) {
        faults[i] = last_faults_[i];
      }
    }
    const auto limits = kin_->allocate_current(current_budget_, faults);

    // ---- 5. 组帧 ----
    hw_msgs::msg::MotorCommandArray arr;
    arr.header.stamp = t;
    arr.header.frame_id = base_frame_;
    arr.commands.resize(4);

    bool any_fault = false;
    for (std::size_t i = 0; i < 4; ++i) {
      auto & c = arr.commands[i];
      c.header = arr.header;
      c.motor_id = static_cast<uint8_t>(i);
      c.mode = hw_msgs::msg::MotorCommand::MODE_VELOCITY;
      c.setpoint = static_cast<float>(wheels[i] * static_cast<double>(invert_vector_[i]));
      c.current_limit = static_cast<float>(limits[i]);
      c.enable = true;
      c.brake = false;
      c.reset_fault = false;
      if (faults[i] != 0U) { any_fault = true; }
    }

    if (any_fault) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "存在故障电机，已按故障状态重新分配电流预算");
    }

    pub_cmd_->publish(arr);
  }

  // ---------------------------------------------------------------- 状态回调
  void on_states(const hw_msgs::msg::MotorStateArray::SharedPtr msg)
  {
    if (msg->motors.size() < 4U) {
      return;
    }

    const auto & geo = kin_->geometry();

    // ---- joint_states：位置与速度都换算到轮侧（除以减速比） ----
    sensor_msgs::msg::JointState js;
    js.header.stamp = msg->header.stamp;

    std::array<double, 4> wheel_rad_s{};
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      for (std::size_t i = 0; i < 4; ++i) {
        js.name.push_back("wheel_" + std::to_string(i) + "_joint");
        // 电机侧位置 -> 轮侧位置
        js.position.push_back(msg->motors[i].position / geo.gear_ratio);
        js.velocity.push_back(msg->motors[i].velocity / geo.gear_ratio);
        js.effort.push_back(msg->motors[i].current);   // 用电流代替力矩（无力矩传感器）
        wheel_rad_s[i] = msg->motors[i].velocity / geo.gear_ratio;
        last_faults_[i] = static_cast<uint8_t>(msg->motors[i].fault & 0xFFU);
      }
    }
    pub_joint_->publish(js);

    // ---- 里程计 ----
    double vx = 0.0, vy = 0.0, wz = 0.0;
    kin_->wheels_to_body(wheel_rad_s, &vx, &vy, &wz);

    const rclcpp::Time t = msg->header.stamp;
    double dt = 0.0;
    if (last_odom_time_.nanoseconds() != 0) {
      dt = (t - last_odom_time_).seconds();
      if (dt <= 0.0 || dt > 0.5) { dt = 0.0; }   // 时间跳变或长间隔不积分
    }
    last_odom_time_ = t;

    if (dt > 0.0) {
      // 用中点法积分，比欧拉法在转向时精度高一个量级
      const double dtheta = wz * dt;
      const double theta_mid = odom_theta_ + 0.5 * dtheta;
      odom_x_ += (vx * std::cos(theta_mid) - vy * std::sin(theta_mid)) * dt;
      odom_y_ += (vx * std::sin(theta_mid) + vy * std::cos(theta_mid)) * dt;
      odom_theta_ += dtheta;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;

    // 滑移转向的车体速度与轮速积分出来的速度存在模型误差（轮子打滑），
    // 这里给它一个诚实的协方差：不打滑时约 2%，转向时能达到 15%。
    const double slip = std::abs(wz) > 0.1 ? 0.15 : 0.02;
    odom.twist.covariance[0]  = slip * slip;          // vx
    odom.twist.covariance[7]  = (slip * 2.0) * (slip * 2.0);  // vy（无侧向测量）
    odom.twist.covariance[35] = 0.05 * 0.05;          // wz

    pub_odom_->publish(odom);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = odom_x_;
      tf.transform.translation.y = odom_y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }
  }

  // ---------------------------------------------------------------- 成员
  std::unique_ptr<Kinematics> kin_;

  double max_vx_{1.2}, max_wz_{4.0};
  double ramp_acc_{1.5}, ramp_dec_{3.0};
  double cmd_rate_hz_{100.0};
  double current_budget_{4.0};
  bool   publish_tf_{true};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::vector<int64_t> invert_vector_{1, 1, 1, 1};

  Target target_;
  Target cmd_;

  double odom_x_{0.0}, odom_y_{0.0}, odom_theta_{0.0};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_;

  std::mutex state_mtx_;
  std::array<uint8_t, 4> last_faults_{};

  rclcpp::Publisher<hw_msgs::msg::MotorCommandArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_joint_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         pub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    sub_vel_;
  rclcpp::Subscription<hw_msgs::msg::MotorStateArray>::SharedPtr sub_state_;
  rclcpp::TimerBase::SharedPtr timer_cmd_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace motor_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_control::MotorControlNode>());
  rclcpp::shutdown();
  return 0;
}
