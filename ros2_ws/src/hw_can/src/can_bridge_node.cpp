// ============================================================================
//  can_bridge_node.cpp
// ============================================================================
#include "hw_can/can_bridge.hpp"

#include <linux/can.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>

using namespace std::chrono_literals;

namespace hw_can
{

namespace
{
/// 把 steady_clock 的时间点转成 ROS 时间戳。
/// 不用 node->now()：那会在 rx 线程里调 rclcpp API，且延迟一个调用栈。
builtin_interfaces::msg::Time to_msg_time(const rclcpp::Node * node, double stamp)
{
  (void)node;
  builtin_interfaces::msg::Time t;
  const auto sec = static_cast<int32_t>(stamp);
  t.sec = sec;
  t.nanosec = static_cast<uint32_t>((stamp - static_cast<double>(sec)) * 1e9);
  return t;
}

template<typename T>
void pack_into(can_frame & f, const T & w)
{
  static_assert(sizeof(T) <= 8, "payload must fit in one classic CAN frame");
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
CanBridge::CanBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("can_bridge", options)
{
  can_iface_           = declare_parameter("can_interface", "can0");
  flush_period_ms_     = declare_parameter("flush_period_ms", 10);
  heartbeat_timeout_ms_ = declare_parameter("heartbeat_timeout_ms", 500);
  publish_raw_         = declare_parameter("publish_raw", false);
  auto_enable_on_cmd_  = declare_parameter("auto_enable_on_cmd", true);
  motion_output_enabled_ = declare_parameter("motion_output_enabled", true);

  if (!motion_output_enabled_) {
    RCLCPP_WARN(get_logger(),
                "motion_output_enabled=false：本节点不下发运动指令，"
                "指令下发权归 hw_ros2_control（ros2_control 路径）");
  }

  // ---------------- 发布 ----------------
  pub_motors_ = create_publisher<hw_msgs::msg::MotorStateArray>("/hw/motor_states", 10);
  pub_board_  = create_publisher<hw_msgs::msg::BoardState>("/hw/board_state", 10);
  pub_pump_   = create_publisher<hw_msgs::msg::PumpState>("/hw/pump_state", 10);
  pub_cam_    = create_publisher<hw_msgs::msg::CameraSync>("/hw/camera_sync", 20);
  pub_diag_   = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  // ---------------- 订阅 ----------------
  sub_cmd_ = create_subscription<hw_msgs::msg::MotorCommandArray>(
    "/hw/motor_commands", rclcpp::QoS(10),
    std::bind(&CanBridge::on_motor_command, this, std::placeholders::_1));

  sub_pump_ = create_subscription<hw_msgs::msg::PumpCommand>(
    "/hw/pump_command", rclcpp::QoS(5),
    std::bind(&CanBridge::on_pump_command, this, std::placeholders::_1));

  // ---------------- 服务 ----------------
  srv_mode_ = create_service<hw_msgs::srv::SetMotorMode>(
    "/hw/set_motor_mode",
    std::bind(&CanBridge::srv_set_motor_mode, this, std::placeholders::_1,
              std::placeholders::_2));
  srv_gains_ = create_service<hw_msgs::srv::SetMotorGains>(
    "/hw/set_motor_gains",
    std::bind(&CanBridge::srv_set_motor_gains, this, std::placeholders::_1,
              std::placeholders::_2));
  srv_calib_ = create_service<hw_msgs::srv::CalibrateMotor>(
    "/hw/calibrate_motor",
    std::bind(&CanBridge::srv_calibrate, this, std::placeholders::_1,
              std::placeholders::_2));
  srv_pump_ = create_service<hw_msgs::srv::SetPump>(
    "/hw/set_pump",
    std::bind(&CanBridge::srv_set_pump, this, std::placeholders::_1,
              std::placeholders::_2));
  srv_cam_ = create_service<hw_msgs::srv::ConfigureCamera>(
    "/hw/configure_camera",
    std::bind(&CanBridge::srv_configure_camera, this, std::placeholders::_1,
              std::placeholders::_2));
  srv_save_ = create_service<hw_msgs::srv::SaveParams>(
    "/hw/save_params",
    std::bind(&CanBridge::srv_save_params, this, std::placeholders::_1,
              std::placeholders::_2));

  // ---------------- 定时器 ----------------
  timer_flush_ = create_wall_timer(
    std::chrono::milliseconds(flush_period_ms_),
    std::bind(&CanBridge::on_flush_timer, this));
  timer_diag_ = create_wall_timer(1s, std::bind(&CanBridge::on_diag_timer, this));

  // ---------------- 打开总线 ----------------
  bus_ = std::make_unique<SocketCan>(can_iface_);
  bus_->open();
  RCLCPP_INFO(get_logger(), "CAN 接口 %s 已打开（1 Mbps，自定义精简协议 v%d.%d）",
              can_iface_.c_str(), 2, 4);

  running_ = true;
  rx_thread_ = std::thread(&CanBridge::rx_loop, this);
}

CanBridge::~CanBridge()
{
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (bus_) {
    bus_->close();
  }
  RCLCPP_INFO(get_logger(), "CAN 桥已停止：rx=%lu tx=%lu 丢帧=%lu 总线错误=%lu",
              rx_frames_.load(), tx_frames_.load(),
              rx_dropped_.load() + tx_dropped_.load(), bus_errors_.load());
}

// ===========================================================================
//  接收线程
// ===========================================================================
void CanBridge::rx_loop()
{
  constexpr std::size_t kBatch = 32;
  RxFrame batch[kBatch];

  while (running_) {
    std::size_t n = 0;
    try {
      n = bus_->recv_batch(batch, kBatch, 50);
    } catch (const CanBusError & e) {
      RCLCPP_ERROR(get_logger(), "接收失败：%s", e.what());
      // 总线 down 之后尝试重开，避免整个节点静默失效
      std::this_thread::sleep_for(500ms);
      try {
        bus_->close();
        bus_->open();
        RCLCPP_WARN(get_logger(), "CAN 接口已重新打开");
      } catch (const CanBusError & e2) {
        RCLCPP_ERROR(get_logger(), "重开失败，继续重试：%s", e2.what());
      }
      continue;
    }

    for (std::size_t i = 0; i < n; ++i) {
      rx_frames_++;
      handle_frame(batch[i]);
    }
    if (n == kBatch) {
      // 一次 poll 读到满批，说明内核队列里还有积压 —— 记一次统计，
      // 用于判断 CAN 波特率或 CPU 是否成了瓶颈
      rx_dropped_++;
    }
  }
}

void CanBridge::handle_frame(const RxFrame & rf)
{
  if (rf.is_error) {
    on_bus_error(rf.frame);
    return;
  }

  const uint16_t id  = rf.frame.can_id & CAN_SFF_MASK;
  const uint8_t  src = id_node(id);
  if (src != NODE_MAINBOARD && src != NODE_EXPANSION) {
    return;   // 不是驱动板发的，忽略（默认过滤器理论上已挡掉）
  }

  const CanClass cls = id_class(id);
  const uint8_t  idx = id_idx(id);

  switch (cls) {
    case CanClass::Fast:
      if (idx < kMotorCount) {
        on_motor_feedback(idx, unpack_from<MotorFeedbackWire>(rf.frame), rf.timestamp);
      } else if (idx == fast_idx::kBoardStatus) {
        on_board_status(unpack_from<BoardStatusWire>(rf.frame), rf.timestamp);
      }
      break;

    case CanClass::Heartbeat:
      on_heartbeat(unpack_from<HeartbeatWire>(rf.frame), rf.timestamp);
      break;

    case CanClass::Io:
      if (idx == io_idx::kPumpState) {
        on_pump_state(unpack_from<PumpStateWire>(rf.frame), rf.timestamp);
      } else if (idx == io_idx::kCamSyncEvt) {
        on_cam_sync(unpack_from<CamSyncEvtWire>(rf.frame), rf.timestamp);
      } else if (idx >= io_idx::kCamLatch0 && idx < io_idx::kCamLatch0 + kMotorCount) {
        on_cam_latch(idx - io_idx::kCamLatch0,
                     unpack_from<CamLatchWire>(rf.frame), rf.timestamp);
      }
      break;

    case CanClass::Diag:
      on_param(unpack_from<ParamWire>(rf.frame), rf.timestamp);
      break;

    case CanClass::Param:
      on_param(unpack_from<ParamWire>(rf.frame), rf.timestamp);
      break;

    default:
      break;
  }
}

// ---------------- 各帧处理 ----------------
void CanBridge::on_motor_feedback(uint8_t motor_id, const MotorFeedbackWire & w,
                                  double stamp)
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto st = decode_motor_feedback(w, motor_id);

  // 反馈帧里没有 state/mode/duty（为了省带宽），从最近一次下发的指令补。
  // 注意这是"上位机认为的状态"，真实状态以心跳帧里的故障位为准。
  if (motor_id < kMotorCount) {
    st.mode  = last_cmd_[motor_id].mode;
    st.duty  = 0.0f;
    st.state = (st.fault != 0U) ? hw_msgs::msg::MotorState::STATE_FAULT
                                : hw_msgs::msg::MotorState::STATE_RUNNING;
  }
  st.header.stamp = to_msg_time(this, stamp);

  motor_buf_[motor_id] = st;
  motor_fresh_[motor_id] = true;
}

void CanBridge::on_board_status(const BoardStatusWire & w, double stamp)
{
  std::lock_guard<std::mutex> lk(mtx_);
  float vbus = 0, temp = 0, power = 0;
  uint16_t fault = 0;
  uint8_t state = 0;
  decode_board_status(w, &vbus, &temp, &power, &fault, &state);

  board_buf_.header.stamp = to_msg_time(this, stamp);
  board_buf_.vbus = vbus;
  board_buf_.mcu_temperature = temp;
  board_buf_.mos_temperature_max = temp;   // 精确值走诊断通道，这里先用 MCU 温度占位
  board_buf_.power = power;
  board_buf_.fault = fault;
  board_buf_.node_state = state;
  board_buf_.host_online = board_online_.load();
  board_fresh_ = true;
}

void CanBridge::on_heartbeat(const HeartbeatWire & w, double stamp)
{
  (void)stamp;
  board_online_ = true;
  last_heartbeat_ = std::chrono::steady_clock::now();

  if (w.fault_flags != 0U) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "驱动板故障位 0x%04X（首个故障索引 %u），已使能电机掩码 0x%02X",
                         w.fault_flags, w.fault_index, w.motors_enabled);
  }
}

void CanBridge::on_pump_state(const PumpStateWire & w, double stamp)
{
  auto s = decode_pump_state(w);
  s.header.stamp = to_msg_time(this, stamp);
  pub_pump_->publish(s);
}

void CanBridge::on_cam_sync(const CamSyncEvtWire & w, double stamp)
{
  std::lock_guard<std::mutex> lk(mtx_);

  // 上一帧还没凑齐就来了新 SYNC：说明有 latch 帧丢了，直接丢弃未完成的组装
  if (cam_pending_.active) {
    rx_dropped_++;
  }
  cam_pending_.active    = true;
  cam_pending_.frame_id  = w.frame_id;
  cam_pending_.dt_us     = w.dt_us;
  cam_pending_.dropped   = w.dropped;
  cam_pending_.stamp     = stamp;
  cam_pending_.expected_lo = static_cast<uint16_t>(w.frame_id & 0xFFFFU);
  cam_pending_.got.fill(false);
}

void CanBridge::on_cam_latch(uint8_t motor_id, const CamLatchWire & w, double stamp)
{
  (void)stamp;
  std::lock_guard<std::mutex> lk(mtx_);

  if (!cam_pending_.active) {
    return;   // 没有对应的 SYNC 事件，丢弃
  }
  // 用帧号低 16 位校验，防止上一帧的迟到 latch 混进这一帧
  if (w.frame_id_lo != cam_pending_.expected_lo) {
    return;
  }

  cam_pending_.pos[motor_id] = static_cast<float>(w.position_mrad) * 1e-3f;
  cam_pending_.vel[motor_id] = mrads_to_rads(w.velocity_mrads);
  cam_pending_.got[motor_id] = true;

  const bool all = std::all_of(cam_pending_.got.begin(), cam_pending_.got.end(),
                               [](bool b) { return b; });
  if (!all) {
    return;
  }

  // 四帧齐了，发 CameraSync
  hw_msgs::msg::CameraSync msg;
  msg.header.stamp = to_msg_time(this, cam_pending_.stamp);
  msg.header.frame_id = "cam_sync";
  msg.frame_id = cam_pending_.frame_id;
  msg.latched_positions.assign(cam_pending_.pos.begin(), cam_pending_.pos.end());
  msg.latched_velocities.assign(cam_pending_.vel.begin(), cam_pending_.vel.end());
  msg.period  = static_cast<float>(cam_pending_.dt_us) * 1e-6f;
  msg.dropped = cam_pending_.dropped;
  msg.exposure_delay = exposure_delay_s_.load();
  msg.capture_cycles = 0;

  cam_pending_.active = false;
  pub_cam_->publish(msg);
}

void CanBridge::on_param(const ParamWire & w, double stamp)
{
  (void)stamp;
  const auto d = decode_param(w);

  if (d.is_event()) {
    switch (d.event_code()) {
      case event_code::kBoot:
        RCLCPP_INFO(get_logger(), "驱动板上报启动事件（固件 %.2f）", d.value);
        break;
      case event_code::kCalibDone:
        RCLCPP_INFO(get_logger(), "电机 %u 电角度标定完成，offset=%.4f rad",
                    d.reserved, d.value);
        break;
      case event_code::kFault:
        RCLCPP_ERROR(get_logger(), "驱动板故障事件：motor=%u fault=0x%04X",
                     d.reserved, static_cast<unsigned>(d.value));
        break;
      case event_code::kParamSaved:
        RCLCPP_INFO(get_logger(), "参数已写入 Flash（%s）", d.value > 0.5f ? "成功" : "失败");
        break;
      case event_code::kCamSyncLost:
        RCLCPP_WARN(get_logger(), "相机帧同步丢失，累计丢帧 %u",
                    static_cast<unsigned>(d.value));
        break;
      default:
        break;
    }
    return;
  }

  // 参数响应：唤醒等待中的服务调用
  std::lock_guard<std::mutex> lk(param_mtx_);
  auto it = param_waiters_.find(d.param_id);
  if (it != param_waiters_.end() && it->second) {
    it->second->value = d.value;
    it->second->op = d.op;
    it->second->ready = true;
    it->second->cv.notify_all();
  }
}

void CanBridge::on_bus_error(const can_frame & f)
{
  bus_errors_++;
  if ((f.can_id & CAN_ERR_BUSOFF) != 0U) {
    RCLCPP_ERROR(get_logger(), "CAN 总线进入 BUS-OFF，驱动板已自动停机");
  } else if ((f.can_id & CAN_ERR_ACK) != 0U) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "CAN 帧未被应答（总线上只有上位机在跑？）");
  } else if ((f.can_id & CAN_ERR_CRTL) != 0U) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "CAN 控制器错误，RX 错误计数=%u", f.data[1]);
  }
}

// ===========================================================================
//  发送
// ===========================================================================
void CanBridge::send_motion(const hw_msgs::msg::MotorCommand & cmd)
{
  if (cmd.motor_id >= kMotorCount) {
    return;
  }
  if (!motion_output_enabled_) {
    RCLCPP_WARN_ONCE(get_logger(),
                     "收到运动指令但 motion_output_enabled=false，已忽略。"
                     "若正在使用 ros2_control，这是预期行为。");
    return;
  }
  const auto w = encode_motion_cmd(cmd);

  can_frame f{};
  f.can_id = make_id(NODE_HOST, CanClass::Motion, cmd.motor_id);
  pack_into(f, w);

  if (bus_->send(f)) {
    tx_frames_++;
  } else {
    tx_dropped_++;
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_cmd_[cmd.motor_id] = cmd;
  }
}

void CanBridge::send_param(uint16_t param_id, float value, uint8_t op, uint8_t idx)
{
  const auto w = encode_param(param_id, value, op);
  can_frame f{};
  f.can_id = make_id(NODE_HOST, CanClass::Param, idx);
  pack_into(f, w);
  if (bus_->send(f)) {
    tx_frames_++;
  } else {
    tx_dropped_++;
  }
}

float CanBridge::param_transact(uint16_t param_id, float value, uint8_t op,
                                uint8_t motor_idx, int timeout_ms)
{
  auto waiter = std::make_shared<ParamWaiter>();
  {
    std::lock_guard<std::mutex> lk(param_mtx_);
    param_waiters_[param_id] = waiter;
  }

  send_param(param_id, value, op, motor_idx);

  {
    std::unique_lock<std::mutex> lk(param_mtx_);
    const bool ok = waiter->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                                        [&] { return waiter->ready; });
    param_waiters_.erase(param_id);
    if (!ok) {
      throw std::runtime_error("等待驱动板参数响应超时（param 0x" +
                               std::to_string(param_id) + "）");
    }
    if (waiter->op == param_op::kErr) {
      throw std::runtime_error("驱动板拒绝该参数操作（param 0x" +
                               std::to_string(param_id) + "）");
    }
  }
  return waiter->value;
}

// ===========================================================================
//  定时器
// ===========================================================================
void CanBridge::on_flush_timer()
{
  // ---- 四轴状态：凑齐（或超时）就发 ----
  bool any = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    for (bool b : motor_fresh_) {
      if (b) { any = true; break; }
    }
  }

  if (any) {
    hw_msgs::msg::MotorStateArray arr;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      arr.header.stamp = now();
      arr.header.frame_id = "base_link";
      arr.board = board_buf_;
      arr.motors.assign(motor_buf_.begin(), motor_buf_.end());
      std::fill(motor_fresh_.begin(), motor_fresh_.end(), false);
      board_fresh_ = false;
    }
    pub_motors_->publish(arr);

    // 板级状态单独发，方便只用 board_state 的下游（电源监控、诊断面板）
    if (arr.board.header.stamp.sec != 0) {
      pub_board_->publish(arr.board);
    }
  }
}

void CanBridge::on_diag_timer()
{
  // ---- 心跳超时判定 ----
  const auto now_tp = std::chrono::steady_clock::now();
  const auto silence = std::chrono::duration_cast<std::chrono::milliseconds>(
    now_tp - last_heartbeat_).count();
  const bool online = silence < heartbeat_timeout_ms_;
  if (board_online_ != online) {
    board_online_ = online;
    RCLCPP_WARN(get_logger(), "驱动板%s（静默 %ld ms）",
                online ? "恢复在线" : "失去联系", static_cast<long>(silence));
  }

  // ---- 诊断消息 ----
  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now();

  auto add = [&](const std::string & name, int8_t level,
                 const std::string & message,
                 std::initializer_list<std::pair<std::string, std::string>> kv) {
      diagnostic_msgs::msg::DiagnosticStatus st;
      st.name = name;
      st.hardware_id = "hwb-mc4g4";
      st.level = level;
      st.message = message;
      for (const auto & p : kv) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = p.first;
        e.value = p.second;
        st.values.push_back(e);
      }
      diag.status.push_back(st);
    };

  add("hw_can/bus", board_online_ ? 0 : 2,
      board_online_ ? "总线正常" : "驱动板心跳超时",
      {{"interface", can_iface_},
       {"rx_frames", std::to_string(rx_frames_.load())},
       {"tx_frames", std::to_string(tx_frames_.load())},
       {"tx_dropped", std::to_string(tx_dropped_.load())},
       {"bus_errors", std::to_string(bus_errors_.load())},
       {"heartbeat_silence_ms", std::to_string(silence)}});

  {
    std::lock_guard<std::mutex> lk(mtx_);
    add("hw_can/board", board_buf_.fault == 0 ? 0 : 1,
        board_buf_.fault == 0 ? "无故障" : "存在故障位",
        {{"vbus_V", std::to_string(board_buf_.vbus)},
         {"mcu_temp_C", std::to_string(board_buf_.mcu_temperature)},
         {"power_W", std::to_string(board_buf_.power)},
         {"fault", std::to_string(board_buf_.fault)}});
  }

  pub_diag_->publish(diag);
}

// ===========================================================================
//  订阅回调
// ===========================================================================
void CanBridge::on_motor_command(const hw_msgs::msg::MotorCommandArray::SharedPtr msg)
{
  for (const auto & cmd : msg->commands) {
    auto c = cmd;
    if (auto_enable_on_cmd_ && !c.enable && !c.brake && !c.reset_fault &&
        c.mode == hw_msgs::msg::MotorCommand::MODE_VELOCITY) {
      c.enable = true;   // 速度模式下默认随指令使能，方便上层只发速度
    }
    send_motion(c);
  }
}

void CanBridge::on_pump_command(const hw_msgs::msg::PumpCommand::SharedPtr msg)
{
  const auto w = encode_pump_cmd(msg->cmd, msg->mode, msg->duty, msg->target_pressure);
  can_frame f{};
  f.can_id = make_id(NODE_HOST, CanClass::Io, io_idx::kPumpCmd);
  pack_into(f, w);
  if (bus_->send(f)) { tx_frames_++; } else { tx_dropped_++; }
}

// ===========================================================================
//  服务回调
// ===========================================================================
void CanBridge::srv_set_motor_mode(
  const std::shared_ptr<hw_msgs::srv::SetMotorMode::Request> req,
  std::shared_ptr<hw_msgs::srv::SetMotorMode::Response> res)
{
  if (req->motor_id >= kMotorCount) {
    res->success = false;
    res->message = "motor_id 必须在 0..3";
    return;
  }

  hw_msgs::msg::MotorCommand cmd;
  cmd.motor_id = req->motor_id;
  cmd.mode = req->mode;
  cmd.enable = req->enable;
  cmd.setpoint = 0.0f;
  cmd.current_limit = 0.0f;
  cmd.brake = (req->mode == hw_msgs::msg::MotorCommand::MODE_BRAKE);
  cmd.reset_fault = false;
  send_motion(cmd);

  res->success = true;
  res->active_mode = req->mode;
  res->message = "模式切换指令已下发（驱动板在下一个 50us 环路边界生效）";
}

void CanBridge::srv_set_motor_gains(
  const std::shared_ptr<hw_msgs::srv::SetMotorGains::Request> req,
  std::shared_ptr<hw_msgs::srv::SetMotorGains::Response> res)
{
  uint16_t kp_id = 0, ki_id = 0, kd_id = 0;
  switch (req->loop) {
    case hw_msgs::srv::SetMotorGains::Request::LOOP_CURRENT:
      kp_id = param_id::kPidCurKp; ki_id = param_id::kPidCurKi; kd_id = 0; break;
    case hw_msgs::srv::SetMotorGains::Request::LOOP_VELOCITY:
      kp_id = param_id::kPidVelKp; ki_id = param_id::kPidVelKi;
      kd_id = param_id::kPidVelKd; break;
    case hw_msgs::srv::SetMotorGains::Request::LOOP_POSITION:
      kp_id = param_id::kPidPosKp; ki_id = param_id::kPidPosKi;
      kd_id = param_id::kPidPosKd; break;
    default:
      res->success = false;
      res->message = "未知的环路编号";
      return;
  }

  try {
    param_transact(kp_id, req->kp, param_op::kWrite, req->motor_id);
    res->kp_readback = param_transact(kp_id, 0.0f, param_op::kReadReq, req->motor_id);
    if (req->loop != hw_msgs::srv::SetMotorGains::Request::LOOP_CURRENT) {
      param_transact(ki_id, req->ki, param_op::kWrite, req->motor_id);
      res->ki_readback = param_transact(ki_id, 0.0f, param_op::kReadReq, req->motor_id);
      param_transact(kd_id, req->kd, param_op::kWrite, req->motor_id);
      res->kd_readback = param_transact(kd_id, 0.0f, param_op::kReadReq, req->motor_id);
    } else {
      param_transact(ki_id, req->ki, param_op::kWrite, req->motor_id);
      res->ki_readback = param_transact(ki_id, 0.0f, param_op::kReadReq, req->motor_id);
      res->kd_readback = 0.0f;
    }
    res->success = true;
    res->message = "PID 已写入（RAM），需 SaveParams 才持久化";
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

void CanBridge::srv_calibrate(
  const std::shared_ptr<hw_msgs::srv::CalibrateMotor::Request> req,
  std::shared_ptr<hw_msgs::srv::CalibrateMotor::Response> res)
{
  if (!req->confirm_mechanical_free) {
    res->success = false;
    res->message = "必须显式确认电机可以自由转动（confirm_mechanical_free=true）";
    return;
  }
  if (req->motor_id >= kMotorCount) {
    res->success = false;
    res->message = "motor_id 必须在 0..3";
    return;
  }

  hw_msgs::msg::MotorCommand cmd;
  cmd.motor_id = req->motor_id;
  cmd.mode = hw_msgs::msg::MotorCommand::MODE_CALIBRATE;
  cmd.enable = true;
  cmd.brake = false;
  send_motion(cmd);

  try {
    // 标定流程 250ms，等一下再读回零点
    std::this_thread::sleep_for(400ms);
    res->elec_offset = param_transact(param_id::kEncElecOffset, 0.0f,
                                      param_op::kReadReq, req->motor_id, 300);
    res->success = true;
    res->message = "标定完成，零点已保存到驱动板 Flash";
  } catch (const std::exception & e) {
    res->success = false;
    res->message = std::string("标定后读取零点失败：") + e.what();
  }
}

void CanBridge::srv_set_pump(
  const std::shared_ptr<hw_msgs::srv::SetPump::Request> req,
  std::shared_ptr<hw_msgs::srv::SetPump::Response> res)
{
  float duty = 0.0f, target = 0.0f;
  if (req->mode == hw_msgs::msg::PumpCommand::MODE_OPEN_DUTY) {
    duty = req->value;
  } else {
    target = req->value;
  }

  const auto w = encode_pump_cmd(req->cmd, req->mode, duty, target);
  can_frame f{};
  f.can_id = make_id(NODE_HOST, CanClass::Io, io_idx::kPumpCmd);
  pack_into(f, w);

  res->success = bus_->send(f);
  res->message = res->success ? "气泵指令已下发" : "CAN 发送失败";
  res->pressure = 0.0f;
}

void CanBridge::srv_configure_camera(
  const std::shared_ptr<hw_msgs::srv::ConfigureCamera::Request> req,
  std::shared_ptr<hw_msgs::srv::ConfigureCamera::Response> res)
{
  CamCfgWire w{};
  w.enable = req->enable ? 1U : 0U;
  w.rate_hz = static_cast<uint16_t>(std::clamp(req->rate_hz, 1.0f, 120.0f));
  w.pulse_us = static_cast<uint16_t>(std::clamp(req->pulse_width_us, 5.0f, 10000.0f));
  w.mode = req->mode;

  can_frame f{};
  f.can_id = make_id(NODE_HOST, CanClass::Io, io_idx::kCamCfg);
  pack_into(f, w);

  res->success = bus_->send(f);
  res->applied_rate_hz = static_cast<float>(w.rate_hz);
  res->message = res->success ? "相机触发配置已下发" : "CAN 发送失败";

  // 标定 SYNC 到图像就绪的延迟（用于 CameraSync.exposure_delay）
  // 这里用配置的脉宽 + 相机 datasheet 的读出时间估计；实测值由
  // camera_trigger 节点在收到第一对 (sync, image) 后回写。
  exposure_delay_s_ = static_cast<float>(w.pulse_us) * 1e-6f + 0.0012f;
}

void CanBridge::srv_save_params(
  const std::shared_ptr<hw_msgs::srv::SaveParams::Request> req,
  std::shared_ptr<hw_msgs::srv::SaveParams::Response> res)
{
  (void)req;
  ParamWire w = encode_param(0xFFFFU, 1.0f, param_op::kWrite);
  can_frame f{};
  f.can_id = make_id(NODE_HOST, CanClass::Flash, 0);
  pack_into(f, w);

  res->success = bus_->send(f);
  res->message = res->success
    ? "保存请求已下发（驱动板在主循环里写 Flash，约 20ms 后上报结果）"
    : "CAN 发送失败";
}

}  // namespace hw_can

// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<hw_can::CanBridge>(options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
