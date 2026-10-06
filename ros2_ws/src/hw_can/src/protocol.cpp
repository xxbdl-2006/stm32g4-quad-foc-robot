// ============================================================================
//  protocol.cpp
// ============================================================================
#include "hw_can/protocol.hpp"

namespace hw_can
{

hw_msgs::msg::MotorState decode_motor_feedback(const MotorFeedbackWire & w,
                                               uint8_t motor_id)
{
  hw_msgs::msg::MotorState m;
  m.motor_id = motor_id;
  m.position = static_cast<float>(w.position_mrad) * 1e-3f;      // mrad -> rad
  m.velocity = mrads_to_rads(w.velocity_mrads);                  // mrad/s -> rad/s
  m.current  = static_cast<float>(w.current_ma) * 1e-3f;         // mA -> A
  // duty / state / mode 不在反馈帧里（省带宽），由桥节点从最近一次指令缓存补上
  m.duty  = 0.0f;
  m.state = hw_msgs::msg::MotorState::STATE_RUNNING;
  m.mode  = hw_msgs::msg::MotorState::MODE_VELOCITY;
  m.fault = 0;
  return m;
}

void decode_board_status(const BoardStatusWire & w,
                         float * vbus, float * mcu_temp, float * power,
                         uint16_t * fault, uint8_t * node_state)
{
  if (vbus)       { *vbus       = static_cast<float>(w.vbus_mv) * 1e-3f; }
  if (mcu_temp)   { *mcu_temp   = static_cast<float>(w.mcu_temp_c10) * 0.1f; }
  if (power)      { *power      = static_cast<float>(w.power_w10) * 0.1f; }
  if (fault)      { *fault      = w.fault_flags_lo; }
  if (node_state) { *node_state = w.state; }
}

MotionCmdWire encode_motion_cmd(const hw_msgs::msg::MotorCommand & cmd)
{
  MotionCmdWire w{};
  w.mode = cmd.mode;
  w.setpoint = cmd.setpoint;

  const float limit_a = (cmd.current_limit > 0.0f) ? cmd.current_limit : 0.0f;
  float limit_ma_f = limit_a * 1000.0f;
  if (limit_ma_f > 65535.0f) { limit_ma_f = 65535.0f; }
  w.limit_ma = static_cast<uint16_t>(limit_ma_f);

  uint8_t flags = 0;
  if (cmd.enable)      { flags |= 0x01U; }
  if (cmd.brake)       { flags |= 0x02U; }
  if (cmd.reset_fault) { flags |= 0x04U; }
  w.flags = flags;
  return w;
}

PumpCmdWire encode_pump_cmd(uint8_t cmd, uint8_t mode, float duty, float target_kpa)
{
  PumpCmdWire w{};
  w.cmd = cmd;
  w.mode = mode;
  float d = duty * 1000.0f;
  if (d < 0.0f)     { d = 0.0f; }
  if (d > 1000.0f)  { d = 1000.0f; }
  w.duty_permille = static_cast<uint16_t>(d);
  float kpa10 = target_kpa * 10.0f;
  if (kpa10 < 0.0f)     { kpa10 = 0.0f; }
  if (kpa10 > 65535.0f) { kpa10 = 65535.0f; }
  w.target_kpa10 = static_cast<uint16_t>(kpa10);
  w.reserved = 0;
  return w;
}

hw_msgs::msg::PumpState decode_pump_state(const PumpStateWire & w)
{
  hw_msgs::msg::PumpState s;
  s.state    = w.state;
  s.duty     = static_cast<float>(w.duty_permille) * 1e-3f;
  s.pressure = static_cast<float>(w.pressure_kpa10) * 0.1f;
  s.fault    = w.fault;
  return s;
}

ParamDecoded decode_param(const ParamWire & w)
{
  ParamDecoded d;
  d.param_id = w.param_id;
  d.value    = w.value;
  d.op       = w.op;
  d.reserved = w.reserved;
  return d;
}

ParamWire encode_param(uint16_t param_id, float value, uint8_t op)
{
  ParamWire w{};
  w.param_id = param_id;
  w.value    = value;
  w.op       = op;
  w.reserved = 0;
  return w;
}

}  // namespace hw_can
