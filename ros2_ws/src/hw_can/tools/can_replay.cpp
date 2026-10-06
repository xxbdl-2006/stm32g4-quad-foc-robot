// ============================================================================
//  can_replay.cpp
//  把 candump 抓下来的日志回放成 ROS2 话题，用于：没有实车时开发上层算法。
//
//  用法：
//    ros2 run hw_can can_replay <candump.log> --rate 1.0 --loop
//
//  输入格式支持 `candump -L can0` 的文本格式：
//    (1700000000.123456) can0 123#1122334455667788
//
//  回放时会按原始时间戳的相对间隔重放，因此电机反馈的节拍与真实一致，
//  下游控制器的时序行为可以真实复现（这是"用假数据跑仿真"做不到的）。
// ============================================================================
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hw_msgs/msg/board_state.hpp"
#include "hw_msgs/msg/camera_sync.hpp"
#include "hw_msgs/msg/motor_state_array.hpp"
#include "hw_msgs/msg/pump_state.hpp"
#include "hw_can/protocol.hpp"

using namespace std::chrono_literals;

namespace
{

struct LogEntry
{
  double   t;
  uint32_t id;
  uint8_t  dlc;
  uint8_t  data[8];
};

/// 解析一行 candump -L 输出。
bool parse_line(const std::string & line, LogEntry & out)
{
  const auto lp = line.find('(');
  const auto rp = line.find(')');
  const auto hash = line.find('#');
  if (lp == std::string::npos || rp == std::string::npos || hash == std::string::npos) {
    return false;
  }

  out.t = std::stod(line.substr(lp + 1, rp - lp - 1));

  // <iface> <id>#<data>，iface 与 id 之间是空格
  std::string rest = line.substr(rp + 1);
  std::istringstream iss(rest);
  std::string iface, framedata;
  if (!(iss >> iface >> framedata)) { return false; }

  const auto h = framedata.find('#');
  if (h == std::string::npos) { return false; }

  out.id = static_cast<uint32_t>(std::stoul(framedata.substr(0, h), nullptr, 16));
  const std::string hex = framedata.substr(h + 1);
  out.dlc = static_cast<uint8_t>(hex.size() / 2);
  if (out.dlc > 8) { return false; }
  std::memset(out.data, 0, sizeof(out.data));
  for (uint8_t i = 0; i < out.dlc; ++i) {
    out.data[i] = static_cast<uint8_t>(std::stoul(hex.substr(i * 2, 2), nullptr, 16));
  }
  return true;
}

inline double now_sec()
{
  const auto tp = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(tp).count();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() < 2) {
    std::fprintf(stderr,
                 "用法: %s <candump.log> [--rate 1.0] [--loop] [--start N]\n",
                 args.empty() ? "can_replay" : args[0].c_str());
    return 1;
  }

  const std::string path = args[1];
  double rate = 1.0;
  bool loop = false;
  for (std::size_t i = 2; i + 1 < args.size(); ++i) {
    if (args[i] == "--rate") { rate = std::stod(args[i + 1]); }
    if (args[i] == "--loop") { loop = true; }
  }

  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "无法打开 %s\n", path.c_str());
    return 1;
  }

  std::vector<LogEntry> log;
  log.reserve(200000);
  std::string line;
  while (std::getline(in, line)) {
    LogEntry e{};
    if (parse_line(line, e)) { log.push_back(e); }
  }
  std::fprintf(stdout, "已载入 %zu 帧（%.2f 秒）\n", log.size(),
               log.empty() ? 0.0 : log.back().t - log.front().t);

  auto node = rclcpp::Node::make_shared("can_replay");
  auto pub_motors = node->create_publisher<hw_msgs::msg::MotorStateArray>("/hw/motor_states", 10);
  auto pub_board  = node->create_publisher<hw_msgs::msg::BoardState>("/hw/board_state", 10);
  auto pub_pump   = node->create_publisher<hw_msgs::msg::PumpState>("/hw/pump_state", 10);
  auto pub_cam    = node->create_publisher<hw_msgs::msg::CameraSync>("/hw/camera_sync", 20);
  node->declare_parameter("loop", loop);

  using hw_can::CanClass;
  using hw_can::id_class;
  using hw_can::id_idx;

  std::array<hw_msgs::msg::MotorState, 4> motors{};
  hw_msgs::msg::CameraSync cam_accum;
  bool cam_active = false;

  auto publish_cycle = [&]() {
      hw_msgs::msg::MotorStateArray arr;
      arr.header.stamp = node->now();
      arr.motors.assign(motors.begin(), motors.end());
      pub_motors->publish(arr);
    };

  const double t0 = log.empty() ? 0.0 : log.front().t;
  const double wall0 = now_sec();

  do {
    for (std::size_t i = 0; i < log.size(); ++i) {
      const LogEntry & e = log[i];

      // 按原始相对时间休眠，保持与真实一致的节拍
      const double target = (e.t - t0) / rate;
      const double elapsed = now_sec() - wall0;
      if (target > elapsed) {
        std::this_thread::sleep_for(
          std::chrono::duration<double>(target - elapsed));
      }

      const CanClass cls = id_class(e.id);
      const uint8_t  idx = id_idx(e.id);

      switch (cls) {
        case CanClass::Fast:
          if (idx < 4) {
            hw_can::MotorFeedbackWire w{};
            std::memcpy(&w, e.data, 8);
            motors[idx] = hw_can::decode_motor_feedback(w, idx);
            motors[idx].header.stamp = node->now();
            if (idx == 3) { publish_cycle(); }
          } else if (idx == hw_can::fast_idx::kBoardStatus) {
            hw_can::BoardStatusWire w{};
            std::memcpy(&w, e.data, 8);
            hw_msgs::msg::BoardState bs;
            bs.header.stamp = node->now();
            hw_can::decode_board_status(w, &bs.vbus, &bs.mcu_temperature,
                                        &bs.power, &bs.fault, &bs.node_state);
            pub_board->publish(bs);
          }
          break;

        case CanClass::Io:
          if (idx == hw_can::io_idx::kPumpState) {
            hw_can::PumpStateWire w{};
            std::memcpy(&w, e.data, 8);
            auto s = hw_can::decode_pump_state(w);
            s.header.stamp = node->now();
            pub_pump->publish(s);
          } else if (idx == hw_can::io_idx::kCamSyncEvt) {
            hw_can::CamSyncEvtWire w{};
            std::memcpy(&w, e.data, 8);
            cam_accum = hw_msgs::msg::CameraSync();
            cam_accum.header.stamp = node->now();
            cam_accum.header.frame_id = "cam_sync";
            cam_accum.frame_id = w.frame_id;
            cam_accum.period = static_cast<float>(w.dt_us) * 1e-6f;
            cam_accum.dropped = w.dropped;
            cam_accum.latched_positions.assign(4, 0.0f);
            cam_accum.latched_velocities.assign(4, 0.0f);
            cam_active = true;
          } else if (idx >= hw_can::io_idx::kCamLatch0 &&
                     idx < hw_can::io_idx::kCamLatch0 + 4 && cam_active) {
            hw_can::CamLatchWire w{};
            std::memcpy(&w, e.data, 8);
            const auto m = idx - hw_can::io_idx::kCamLatch0;
            cam_accum.latched_positions[m] = static_cast<float>(w.position_mrad) * 1e-3f;
            cam_accum.latched_velocities[m] = hw_can::mrads_to_rads(w.velocity_mrads);
            if (m == 3) {
              pub_cam->publish(cam_accum);
              cam_active = false;
            }
          }
          break;

        default:
          break;
      }
    }
  } while (loop && rclcpp::ok());

  rclcpp::shutdown();
  return 0;
}
