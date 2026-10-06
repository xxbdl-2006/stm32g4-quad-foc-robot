// ============================================================================
//  can_bridge.hpp
//  把 SocketCAN 上跑的自定义协议翻译成 ROS2 话题/服务。
//
//  线程模型
//  --------
//   线程 A（rx_thread_）：阻塞在 poll()，把 CAN 帧搬进共享缓冲。除 CameraSync
//                         之外的所有话题都在这个线程里**不**发布，而是攒到
//                         flush 定时器里由 executor 线程发。
//                         CameraSync 是唯一的例外：它必须紧贴 SYNC 中断发出去，
//                         如果等 flush 定时器（最长 10ms）会白白引入抖动。
//                         rclcpp 的 Publisher::publish() 本身是线程安全的，
//                         这里从非 executor 线程发是刻意的取舍。
//   线程 B（executor）   ：两个定时器
//                         · flush 定时器（100Hz）：把攒齐的四轴反馈发出去
//                         · watchdog 定时器（1Hz）：心跳/诊断/总线状态
//                         以及订阅回调与服务回调。
//   两者之间用一个 std::mutex 保护的小结构体通信，临界区里只有 memcpy。
//
//  为什么不用 ros2_control 的 broadcast 接口直接读 CAN
//  ------------------------------------------------
//  因为相机同步锁存需要在**收到 SYNC 帧后立刻**把四轴位置打包发出去，
//  这个动作的频率和时机由驱动板决定，不受 controller_manager 的 update()
//  节奏控制。中间加一层桥节点，让"时间关键"的数据走自己的路径。
// ============================================================================
#ifndef HW_CAN__CAN_BRIDGE_HPP_
#define HW_CAN__CAN_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hw_msgs/msg/board_state.hpp"
#include "hw_msgs/msg/camera_sync.hpp"
#include "hw_msgs/msg/motor_command_array.hpp"
#include "hw_msgs/msg/motor_state_array.hpp"
#include "hw_msgs/msg/pump_command.hpp"
#include "hw_msgs/msg/pump_state.hpp"
#include "hw_msgs/srv/calibrate_motor.hpp"
#include "hw_msgs/srv/configure_camera.hpp"
#include "hw_msgs/srv/save_params.hpp"
#include "hw_msgs/srv/set_motor_gains.hpp"
#include "hw_msgs/srv/set_motor_mode.hpp"
#include "hw_msgs/srv/set_pump.hpp"

#include "hw_can/protocol.hpp"
#include "hw_can/socketcan.hpp"

namespace hw_can
{

class CanBridge : public rclcpp::Node
{
public:
  explicit CanBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanBridge() override;

private:
  // ---------------------------------------------------------------- 接收
  void rx_loop();
  void handle_frame(const RxFrame & rf);
  void on_motor_feedback(uint8_t motor_id, const MotorFeedbackWire & w, double stamp);
  void on_board_status(const BoardStatusWire & w, double stamp);
  void on_heartbeat(const HeartbeatWire & w, double stamp);
  void on_pump_state(const PumpStateWire & w, double stamp);
  void on_cam_sync(const CamSyncEvtWire & w, double stamp);
  void on_cam_latch(uint8_t motor_id, const CamLatchWire & w, double stamp);
  void on_param(const ParamWire & w, double stamp);
  void on_bus_error(const can_frame & f);

  // ---------------------------------------------------------------- 发送
  void publish_motor_array();
  void send_motion(const hw_msgs::msg::MotorCommand & cmd);
  void send_param(uint16_t param_id, float value, uint8_t op, uint8_t idx = 0);

  /// 同步参数事务：发一帧、等响应，超时抛 std::runtime_error。
  float param_transact(uint16_t param_id, float value, uint8_t op,
                       uint8_t motor_idx, int timeout_ms = 100);

  // ---------------------------------------------------------------- 定时器
  void on_flush_timer();
  void on_diag_timer();

  // ---------------------------------------------------------------- 订阅回调
  void on_motor_command(const hw_msgs::msg::MotorCommandArray::SharedPtr msg);
  void on_pump_command(const hw_msgs::msg::PumpCommand::SharedPtr msg);

  // ---------------------------------------------------------------- 服务回调
  void srv_set_motor_mode(
    const std::shared_ptr<hw_msgs::srv::SetMotorMode::Request> req,
    std::shared_ptr<hw_msgs::srv::SetMotorMode::Response> res);
  void srv_set_motor_gains(
    const std::shared_ptr<hw_msgs::srv::SetMotorGains::Request> req,
    std::shared_ptr<hw_msgs::srv::SetMotorGains::Response> res);
  void srv_calibrate(
    const std::shared_ptr<hw_msgs::srv::CalibrateMotor::Request> req,
    std::shared_ptr<hw_msgs::srv::CalibrateMotor::Response> res);
  void srv_set_pump(
    const std::shared_ptr<hw_msgs::srv::SetPump::Request> req,
    std::shared_ptr<hw_msgs::srv::SetPump::Response> res);
  void srv_configure_camera(
    const std::shared_ptr<hw_msgs::srv::ConfigureCamera::Request> req,
    std::shared_ptr<hw_msgs::srv::ConfigureCamera::Response> res);
  void srv_save_params(
    const std::shared_ptr<hw_msgs::srv::SaveParams::Request> req,
    std::shared_ptr<hw_msgs::srv::SaveParams::Response> res);

  // ---------------------------------------------------------------- 参数
  std::string can_iface_;
  int         flush_period_ms_;
  int         heartbeat_timeout_ms_;
  bool        publish_raw_;
  bool        auto_enable_on_cmd_;
  /// 用 ros2_control 时置 false，让 hw_ros2_control 独占指令下发，
  /// 本节点只做反馈聚合与诊断。两套控制律同时发帧会抢同一个电机。
  bool        motion_output_enabled_;

  // ---------------------------------------------------------------- CAN
  std::unique_ptr<SocketCan> bus_;
  std::thread                rx_thread_;
  std::atomic<bool>          running_{false};
  std::atomic<uint64_t>      rx_frames_{0};
  std::atomic<uint64_t>      rx_dropped_{0};
  std::atomic<uint64_t>      tx_frames_{0};
  std::atomic<uint64_t>      tx_dropped_{0};
  std::atomic<uint64_t>      bus_errors_{0};
  std::atomic<bool>          board_online_{false};
  std::chrono::steady_clock::time_point last_heartbeat_{};

  /// SYNC 上升沿到图像就绪的估计延迟（秒）。由 configure_camera 服务在配置时
  /// 用"脉宽 + 相机读出时间"估算，并在 camera_trigger 节点实测后回写。
  std::atomic<float>         exposure_delay_s_{0.002f};

  // ---------------------------------------------------------------- 共享缓冲
  std::mutex mtx_;
  std::array<hw_msgs::msg::MotorState, kMotorCount> motor_buf_{};
  std::array<bool, kMotorCount> motor_fresh_{};
  std::array<hw_msgs::msg::MotorCommand, kMotorCount> last_cmd_{};
  hw_msgs::msg::BoardState board_buf_{};
  bool board_fresh_{false};

  // 相机同步组装：先收到 SYNC 事件，再收 4 帧 latch，凑齐后发 CameraSync
  struct PendingCam
  {
    bool     active{false};
    uint32_t frame_id{0};
    uint16_t dt_us{0};
    uint16_t dropped{0};
    double   stamp{0.0};
    std::array<float, kMotorCount> pos{};
    std::array<float, kMotorCount> vel{};
    std::array<bool, kMotorCount>  got{};
    uint16_t expected_lo{0};
  } cam_pending_;

  // 参数事务：等待响应的条件变量
  struct ParamWaiter
  {
    std::condition_variable cv;
    bool  ready{false};
    float value{0.0f};
    uint8_t op{0xFF};
  };
  std::mutex                                  param_mtx_;
  std::map<uint16_t, std::shared_ptr<ParamWaiter>> param_waiters_;

  // ---------------------------------------------------------------- ROS 接口
  rclcpp::Publisher<hw_msgs::msg::MotorStateArray>::SharedPtr pub_motors_;
  rclcpp::Publisher<hw_msgs::msg::BoardState>::SharedPtr      pub_board_;
  rclcpp::Publisher<hw_msgs::msg::PumpState>::SharedPtr       pub_pump_;
  rclcpp::Publisher<hw_msgs::msg::CameraSync>::SharedPtr      pub_cam_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

  rclcpp::Subscription<hw_msgs::msg::MotorCommandArray>::SharedPtr sub_cmd_;
  rclcpp::Subscription<hw_msgs::msg::PumpCommand>::SharedPtr       sub_pump_;

  rclcpp::Service<hw_msgs::srv::SetMotorMode>::SharedPtr     srv_mode_;
  rclcpp::Service<hw_msgs::srv::SetMotorGains>::SharedPtr    srv_gains_;
  rclcpp::Service<hw_msgs::srv::CalibrateMotor>::SharedPtr   srv_calib_;
  rclcpp::Service<hw_msgs::srv::SetPump>::SharedPtr          srv_pump_;
  rclcpp::Service<hw_msgs::srv::ConfigureCamera>::SharedPtr  srv_cam_;
  rclcpp::Service<hw_msgs::srv::SaveParams>::SharedPtr       srv_save_;

  rclcpp::TimerBase::SharedPtr timer_flush_;
  rclcpp::TimerBase::SharedPtr timer_diag_;
};

}  // namespace hw_can

#endif  // HW_CAN__CAN_BRIDGE_HPP_
