// ============================================================================
//  hwb_system.hpp
//  ros2_control 的 System 硬件接口：把 HWB-MC4G4 驱动板接进 controller_manager。
//
//  与 hw_can/can_bridge 的关系
//  --------------------------
//  两者都直接打开 can0（Linux SocketCAN 允许多个 socket 同时订阅同一接口，
//  每个 socket 各拿到一份帧副本）。区别在于**谁负责下发指令**：
//
//    · 用 can_bridge + arm_control   ：bridge 发指令，适合逆解/轨迹规划都在自己手里的场合
//    · 用 hw_ros2_control             ：本组件发指令，可以复用
//                                      joint_trajectory_controller、
//                                      velocity_controllers、joint_limits 等标准件
//
//  两者绝不能同时下发指令 —— 会出现两套控制律抢同一个电机。因此本组件在
//  on_configure 时会尝试通过 ROS2 话题探测 can_bridge 的指令输出是否开启
//  （见 check_no_conflicting_publisher），并在检测到冲突时拒绝激活。
//
//  实时性说明
//  ---------
//  本组件跑在 controller_manager 的 update() 线程里（默认 100Hz）。
//  读操作是非阻塞地把 socket 里积压的帧全部抽干（通常 0~2 帧），
//  写操作是 4 次非阻塞 write()。整个过程在 20us 以内，不会拖慢 update 周期。
//  反馈滞后的最坏情况是一个 update 周期（10ms）+ CAN 传输（<1ms）。
// ============================================================================
#ifndef HW_ROS2_CONTROL__HWB_SYSTEM_HPP_
#define HW_ROS2_CONTROL__HWB_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "hw_can/protocol.hpp"
#include "hw_can/socketcan.hpp"

namespace hw_ros2_control
{

class HwbSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HwbSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kN = 4;

  /// 认为"总线上有另一个节点在发运动指令"的判定窗口
  bool detect_motion_conflict();

  std::string can_iface_;
  bool        invert_direction_{false};
  bool        send_enable_on_activate_{true};

  std::unique_ptr<hw_can::SocketCan> bus_;

  // ---- 状态（驱动板 -> controller）----
  std::array<double, kN> hw_position_{};
  std::array<double, kN> hw_velocity_{};
  std::array<double, kN> hw_effort_{};        // 用 iq 电流近似
  std::array<double, kN> hw_temperature_{};
  std::array<uint16_t, kN> hw_fault_{};
  double hw_vbus_{0.0};
  double hw_board_temp_{0.0};

  // ---- 指令（controller -> 驱动板）----
  std::array<double, kN> cmd_position_{};
  std::array<double, kN> cmd_velocity_{};
  std::array<double, kN> cmd_effort_{};
  std::array<double, kN> cmd_current_limit_{};

  // ---- 内部 ----
  std::array<double, kN> gear_ratio_{};
  std::array<double, kN> velocity_cmd_last_{};
  std::array<uint8_t, kN> last_mode_{};
  std::chrono::steady_clock::time_point last_rx_{};
  bool active_{false};
  /// 错误日志限流计数器。不用 RCLCPP_*_THROTTLE 的原因：本类不是 Node，
  /// 没有稳定的 clock 对象，而 rclcpp::Clock() 是临时量，对它取
  /// get_clock() 再解引用属于未定义行为。见 hwb_system.cpp。
  uint32_t err_log_div_{0};
};

}  // namespace hw_ros2_control

#endif  // HW_ROS2_CONTROL__HWB_SYSTEM_HPP_
