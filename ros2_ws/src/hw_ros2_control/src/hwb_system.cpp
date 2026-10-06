// ============================================================================
//  hwb_system.cpp
// ============================================================================
#include "hw_ros2_control/hwb_system.hpp"

#include <linux/can.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace hw_ros2_control
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

template<typename T>
void pack_into(can_frame & f, const T & w)
{
  static_assert(sizeof(T) <= 8, "payload must fit in one frame");
  f.can_dlc = static_cast<__u8>(sizeof(T));
  std::memset(f.data, 0, sizeof(f.data));
  std::memcpy(f.data, &w, sizeof(T));
}

template<typename T>
T unpack_from(const can_frame & f)
{
  T w{};
  std::memcpy(&w, f.data, std::min<std::size_t>(sizeof(T), f.can_dlc));
  return w;
}
}  // namespace

// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn HwbSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kN) {
    RCLCPP_FATAL(rclcpp::get_logger("HwbSystem"),
                 "URDF 里声明了 %zu 个关节，本驱动要求恰好 %zu 个（M0..M3）",
                 info_.joints.size(), kN);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- 硬件参数（写在 URDF 的 <hardware><param> 里）----
  auto get_param = [&](const std::string & key, const std::string & def) {
      auto it = info_.hardware_parameters.find(key);
      return (it != info_.hardware_parameters.end()) ? it->second : def;
    };

  can_iface_ = get_param("can_interface", "can0");
  invert_direction_ = (get_param("invert_direction", "false") == "true");
  send_enable_on_activate_ = (get_param("send_enable_on_activate", "true") == "true");

  for (std::size_t i = 0; i < kN; ++i) {
    gear_ratio_[i] = std::stod(get_param("gear_ratio_joint_" + std::to_string(i), "1.0"));
    cmd_current_limit_[i] =
      std::stod(get_param("current_limit_joint_" + std::to_string(i), "4.0"));

    // 校验每个关节都声明了 velocity 命令接口（本驱动只支持速度模式为主）
    bool has_vel = false;
    for (const auto & ci : info_.joints[i].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_VELOCITY) { has_vel = true; }
    }
    if (!has_vel) {
      RCLCPP_FATAL(rclcpp::get_logger("HwbSystem"),
                   "关节 %s 缺少 velocity 命令接口；本驱动至少需要它",
                   info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("HwbSystem"),
              "初始化完成：接口=%s，关节=%zu，反向=%s",
              can_iface_.c_str(), info_.joints.size(),
              invert_direction_ ? "是" : "否");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HwbSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    bus_ = std::make_unique<hw_can::SocketCan>(can_iface_);
    bus_->open();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("HwbSystem"), "打开 CAN 失败：%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (detect_motion_conflict()) {
    RCLCPP_FATAL(rclcpp::get_logger("HwbSystem"),
                 "检测到 can_bridge 节点正在下发运动指令（/hw/motor_commands 有发布者）。"
                 "两套控制律会抢同一个电机，拒绝启动。"
                 "请把 can_bridge 的 motion_output_enabled 设为 false，或停掉 arm_control。");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化指令缓存：位置 = 当前位置（避免激活瞬间跳变）
  for (std::size_t i = 0; i < kN; ++i) {
    cmd_position_[i] = hw_position_[i];
    cmd_velocity_[i] = 0.0;
    cmd_effort_[i] = 0.0;
    velocity_cmd_last_[i] = 0.0;
    last_mode_[i] = 0xFFU;
  }
  last_rx_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(rclcpp::get_logger("HwbSystem"), "%s 已配置", can_iface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool HwbSystem::detect_motion_conflict()
{
  // 简单可靠的判据：给驱动板发一次参数读取，观察是否在 20ms 内收到响应。
  // 如果 bus 上已经有别人在发运动帧，说明对方也在跑，直接拒绝。
  //
  // 这里不去解析 /hw/motor_commands 的订阅者（那需要在本组件里建一个 rclcpp
  // Node，成本高、且依赖静态发现延迟）。用总线行为判断更直接。
  //
  // 实测：单独的 can_bridge 会导致驱动板持续收到 CLS_MOTION。本组件在
  // on_configure 阶段先静默 50ms，若期间收到任何来自 NODE_HOST 的 CLS_MOTION，
  // 就判定冲突。
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(50)) {
    hw_can::RxFrame rf{};
    if (!bus_->recv(rf, 5)) { continue; }
    if (rf.is_error) { continue; }
    const uint16_t id = rf.frame.can_id & CAN_SFF_MASK;
    if (hw_can::id_node(id) == hw_can::NODE_HOST &&
        hw_can::id_class(id) == hw_can::CanClass::Motion)
    {
      return true;
    }
  }
  return false;
}

hardware_interface::CallbackReturn HwbSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (send_enable_on_activate_) {
    for (std::size_t i = 0; i < kN; ++i) {
      hw_can::MotionCmdWire w{};
      w.mode = 2;                                     // MODE_VELOCITY
      w.setpoint = 0.0f;
      w.limit_ma = static_cast<uint16_t>(cmd_current_limit_[i] * 1000.0);
      w.flags = 0x01U;                                // enable
      can_frame f{};
      f.can_id = hw_can::make_id(hw_can::NODE_HOST, hw_can::CanClass::Motion,
                                 static_cast<uint8_t>(i));
      pack_into(f, w);
      try {
        bus_->send(f);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("HwbSystem"), "使能下发失败：%s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  active_ = true;
  last_rx_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(rclcpp::get_logger("HwbSystem"), "已激活，四个关节已使能（速度 0）");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HwbSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // 减速到零再关使能 —— 直接断使能会让车带着速度滑行
  for (int step = 0; step < 10; ++step) {
    for (std::size_t i = 0; i < kN; ++i) {
      hw_can::MotionCmdWire w{};
      w.mode = 2;
      w.setpoint = 0.0f;
      w.limit_ma = static_cast<uint16_t>(cmd_current_limit_[i] * 1000.0);
      w.flags = 0x01U;
      can_frame f{};
      f.can_id = hw_can::make_id(hw_can::NODE_HOST, hw_can::CanClass::Motion,
                                 static_cast<uint8_t>(i));
      pack_into(f, w);
      bus_->send(f);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  for (std::size_t i = 0; i < kN; ++i) {
    hw_can::MotionCmdWire w{};
    w.mode = 0;                                       // MODE_IDLE
    w.setpoint = 0.0f;
    w.limit_ma = 0;
    w.flags = 0x00U;                                  // 不使能
    can_frame f{};
    f.can_id = hw_can::make_id(hw_can::NODE_HOST, hw_can::CanClass::Motion,
                               static_cast<uint8_t>(i));
    pack_into(f, w);
    bus_->send(f);
  }

  active_ = false;
  RCLCPP_INFO(rclcpp::get_logger("HwbSystem"), "已停用（先减速后断使能）");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HwbSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (bus_) { bus_->close(); }
  bus_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
//  接口导出
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> HwbSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(kN * 4U);

  for (std::size_t i = 0; i < kN; ++i) {
    const std::string & n = info_.joints[i].name;
    out.emplace_back(n, hardware_interface::HW_IF_POSITION, &hw_position_[i]);
    out.emplace_back(n, hardware_interface::HW_IF_VELOCITY, &hw_velocity_[i]);
    out.emplace_back(n, hardware_interface::HW_IF_EFFORT,   &hw_effort_[i]);
    out.emplace_back(n, "temperature",                      &hw_temperature_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> HwbSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(kN * 3U);

  for (std::size_t i = 0; i < kN; ++i) {
    const std::string & n = info_.joints[i].name;
    out.emplace_back(n, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
    out.emplace_back(n, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]);
    out.emplace_back(n, hardware_interface::HW_IF_EFFORT,   &cmd_effort_[i]);
  }
  return out;
}

// ---------------------------------------------------------------------------
//  read：把 socket 里积压的帧全部抽干
// ---------------------------------------------------------------------------
hardware_interface::return_type HwbSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !bus_) {
    return hardware_interface::return_type::OK;
  }

  constexpr std::size_t kBatch = 64;
  hw_can::RxFrame batch[kBatch];
  std::size_t n = 0;
  try {
    n = bus_->recv_batch(batch, kBatch, 0);   // 非阻塞
  } catch (const std::exception & e) {
    /* 这里不能用 RCLCPP_*_THROTTLE：本类不是 Node，没有稳定的 clock 对象，
     * 而 rclcpp::Clock() 是临时量，对其 get_clock() 解引用是未定义行为。
     * 用一个简单计数器做限流，效果等价且没有 UB。 */
    if ((++err_log_div_ % 200U) == 1U) {
      RCLCPP_ERROR(rclcpp::get_logger("HwbSystem"), "read() 异常：%s", e.what());
    }
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t k = 0; k < n; ++k) {
    if (batch[k].is_error) { continue; }
    const uint16_t id = batch[k].frame.can_id & CAN_SFF_MASK;
    const auto cls = hw_can::id_class(id);
    const uint8_t idx = hw_can::id_idx(id);

    if (cls == hw_can::CanClass::Fast) {
      if (idx < kN) {
        const auto w = unpack_from<hw_can::MotorFeedbackWire>(batch[k].frame);
        hw_position_[idx] = static_cast<double>(w.position_mrad) * 1e-3 / gear_ratio_[idx];
        hw_velocity_[idx] = static_cast<double>(w.velocity_mrads) * 1e-3 / gear_ratio_[idx];
        hw_effort_[idx]   = static_cast<double>(w.current_ma) * 1e-3;
      } else if (idx == hw_can::fast_idx::kBoardStatus) {
        const auto w = unpack_from<hw_can::BoardStatusWire>(batch[k].frame);
        hw_vbus_ = static_cast<double>(w.vbus_mv) * 1e-3;
        hw_board_temp_ = static_cast<double>(w.mcu_temp_c10) * 0.1;
      }
    } else if (cls == hw_can::CanClass::Heartbeat) {
      const auto w = unpack_from<hw_can::HeartbeatWire>(batch[k].frame);
      last_rx_ = std::chrono::steady_clock::now();
      // 把整板故障位分发到四轴（更细的逐轴故障位走诊断通道）
      for (std::size_t i = 0; i < kN; ++i) { hw_fault_[i] = w.fault_flags; }
    }
  }

  // 驱动板静默超过 500ms：写入 NaN 让上层控制器自己决定怎么处理
  // （通常 joint_limits / velocity_controllers 会把 NaN 当作"无数据"而保持不动，
  //   比我们在这里擅自补 0 更安全 —— 补 0 会让控制器误以为电机真的停住了）
  const auto silence = std::chrono::steady_clock::now() - last_rx_;
  if (silence > std::chrono::milliseconds(500)) {
    if ((++err_log_div_ % 200U) == 1U) {
      RCLCPP_ERROR(rclcpp::get_logger("HwbSystem"), "驱动板静默超过 500ms");
    }
    for (std::size_t i = 0; i < kN; ++i) {
      hw_velocity_[i] = kNaN;
      hw_position_[i] = kNaN;
    }
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
//  write：按指令推导模式并发帧
// ---------------------------------------------------------------------------
hardware_interface::return_type HwbSystem::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_ || !bus_) {
    return hardware_interface::return_type::OK;
  }

  // 选择模式：
  //   位置指令被写过（与反馈差 > 1e-4）-> 位置模式
  //   否则速度指令非零            -> 速度模式
  //   否则                         -> 速度 0（保持使能）
  for (std::size_t i = 0; i < kN; ++i) {
    uint8_t mode = 2;
    float setpoint = static_cast<float>(cmd_velocity_[i]);

    const bool pos_active = std::abs(cmd_position_[i] - hw_position_[i]) > 1e-4 &&
                            std::abs(cmd_velocity_[i]) < 1e-6;
    if (pos_active) {
      mode = 3;   // MODE_POSITION
      setpoint = static_cast<float>(cmd_position_[i] * gear_ratio_[i]);
    }

    // 指令斜坡：按加速度上限把速度限住，避免 position_controller 的阶跃被
    // 直接变成电流冲击。周期用 controller_manager 给的 period（通常是 10ms）。
    const double max_dv = 20.0 * period.seconds();   // 20 rad/s^2
    double v = cmd_velocity_[i];
    const double dv = v - velocity_cmd_last_[i];
    if (std::abs(dv) > max_dv) {
      v = velocity_cmd_last_[i] + (dv > 0 ? max_dv : -max_dv);
    }
    velocity_cmd_last_[i] = v;
    if (mode == 2) { setpoint = static_cast<float>(v); }

    hw_can::MotionCmdWire w{};
    w.mode = mode;
    w.setpoint = invert_direction_ ? -setpoint : setpoint;
    w.limit_ma = static_cast<uint16_t>(cmd_current_limit_[i] * 1000.0);
    w.flags = 0x01U;   // enable

    // 模式变化时额外带一次 reset_fault，防止之前的小故障把电机锁死
    if (last_mode_[i] != mode) {
      w.flags |= 0x04U;
      last_mode_[i] = mode;
    }

    can_frame f{};
    f.can_id = hw_can::make_id(hw_can::NODE_HOST, hw_can::CanClass::Motion,
                               static_cast<uint8_t>(i));
    pack_into(f, w);
    try {
      bus_->send(f);
    } catch (const std::exception & e) {
      if ((++err_log_div_ % 200U) == 1U) {
        RCLCPP_ERROR(rclcpp::get_logger("HwbSystem"), "write() 发送失败：%s", e.what());
      }
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hw_ros2_control

PLUGINLIB_EXPORT_CLASS(hw_ros2_control::HwbSystem, hardware_interface::SystemInterface)
