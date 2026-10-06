// ============================================================================
//  protocol.hpp
//  CAN 应用层协议的 C++ 编解码。与固件 Core/Inc/can_proto.h 逐位对应。
//
//  两个文件必须同时修改：任何字段增删都要改 PARAM_VERSION 并重新标定，
//  否则会出现"帧长对但字段错位"的静默故障 —— 这是 CAN 协议最常见的翻车方式，
//  所以这里把 layout 断言也写进来了（static_assert 在编译期就会挡住）。
// ============================================================================
#ifndef HW_CAN__PROTOCOL_HPP_
#define HW_CAN__PROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <array>
#include <optional>

#include "hw_msgs/msg/motor_state.hpp"
#include "hw_msgs/msg/motor_command.hpp"
#include "hw_msgs/msg/pump_state.hpp"
#include "hw_msgs/msg/camera_sync.hpp"

namespace hw_can
{

// ---------------------------------------------------------------- 标识符
constexpr uint16_t NODE_HOST       = 0x00;
constexpr uint16_t NODE_MAINBOARD  = 0x01;
constexpr uint16_t NODE_EXPANSION  = 0x02;

enum class CanClass : uint8_t
{
  Heartbeat = 0x0,
  Fast      = 0x1,
  Motion    = 0x2,
  Param     = 0x3,
  Io        = 0x4,
  Flash     = 0x5,
  Diag      = 0x6,
  EStop     = 0x7,
};

constexpr uint8_t kMotorCount = 4;

inline uint16_t make_id(uint16_t node, CanClass cls, uint8_t idx)
{
  return static_cast<uint16_t>(
    ((node & 0x7U) << 8) | ((static_cast<uint8_t>(cls) & 0xFU) << 4) | (idx & 0xFU));
}
inline uint8_t id_node(uint16_t id)  { return static_cast<uint8_t>((id >> 8) & 0x7U); }
inline CanClass id_class(uint16_t id)
{
  return static_cast<CanClass>((id >> 4) & 0xFU);
}
inline uint8_t id_idx(uint16_t id)   { return static_cast<uint8_t>(id & 0xFU); }

// ---------------------------------------------------------------- 线格式结构
#pragma pack(push, 1)

struct HeartbeatWire
{
  uint16_t fw_version;
  uint8_t  node_state;
  uint8_t  fault_index;
  uint16_t fault_flags;
  uint8_t  motors_enabled;
  uint8_t  uptime_s;
};
static_assert(sizeof(HeartbeatWire) == 8, "heartbeat frame must be 8 bytes");

struct MotorFeedbackWire
{
  int32_t position_mrad;
  int16_t velocity_mrads;
  int16_t current_ma;
};
static_assert(sizeof(MotorFeedbackWire) == 8, "motor feedback must be 8 bytes");

struct BoardStatusWire
{
  uint16_t vbus_mv;
  int16_t  mcu_temp_c10;
  int16_t  power_w10;
  uint8_t  fault_flags_lo;
  uint8_t  state;
};
static_assert(sizeof(BoardStatusWire) == 8, "board status must be 8 bytes");

struct MotionCmdWire
{
  uint8_t  mode;
  float    setpoint;
  uint16_t limit_ma;
  uint8_t  flags;
};
static_assert(sizeof(MotionCmdWire) == 8, "motion cmd must be 8 bytes");

struct MotionSyncHalfWire
{
  int32_t  setpoint_q16[2];
  uint16_t limit_ma[2];
};
static_assert(sizeof(MotionSyncHalfWire) == 8, "sync half must be 8 bytes");

struct ParamWire
{
  uint16_t param_id;
  float    value;
  uint8_t  op;
  uint8_t  reserved;
};
static_assert(sizeof(ParamWire) == 8, "param frame must be 8 bytes");

struct PumpCmdWire
{
  uint8_t  cmd;
  uint16_t duty_permille;
  uint8_t  mode;
  uint16_t target_kpa10;
  uint8_t  reserved;
};
static_assert(sizeof(PumpCmdWire) == 8, "pump cmd must be 8 bytes");

struct PumpStateWire
{
  uint8_t  state;
  uint16_t duty_permille;
  uint16_t pressure_kpa10;
  uint8_t  fault;
  uint8_t  reserved;
};
static_assert(sizeof(PumpStateWire) == 8, "pump state must be 8 bytes");

struct CamCfgWire
{
  uint8_t  enable;
  uint16_t rate_hz;
  uint16_t pulse_us;
  uint8_t  mode;
};
static_assert(sizeof(CamCfgWire) == 8, "cam cfg must be 8 bytes");

struct CamSyncEvtWire
{
  uint32_t frame_id;
  uint16_t dt_us;
  uint16_t dropped;
};
static_assert(sizeof(CamSyncEvtWire) == 8, "cam sync must be 8 bytes");

struct CamLatchWire
{
  int32_t  position_mrad;
  int16_t  velocity_mrads;
  uint16_t frame_id_lo;
};
static_assert(sizeof(CamLatchWire) == 8, "cam latch must be 8 bytes");

#pragma pack(pop)

// ---------------------------------------------------------------- IO 子索引
namespace io_idx
{
constexpr uint8_t kPumpCmd     = 0;
constexpr uint8_t kPumpState   = 1;
constexpr uint8_t kCamCfg      = 2;
constexpr uint8_t kCamSyncEvt  = 3;
constexpr uint8_t kBoardIoSt   = 4;
constexpr uint8_t kCamLatch0   = 5;   // 5..8 -> M0..M3
}  // namespace io_idx

namespace fast_idx
{
constexpr uint8_t kBoardStatus = 4;
}  // namespace fast_idx

namespace diag_idx
{
constexpr uint8_t kCanStats = 2;
}  // namespace diag_idx

// ---------------------------------------------------------------- 参数 ID
namespace param_id
{
constexpr uint16_t kPidVelKp      = 0x0100;
constexpr uint16_t kPidVelKi      = 0x0101;
constexpr uint16_t kPidVelKd      = 0x0102;
constexpr uint16_t kPidPosKp      = 0x0103;
constexpr uint16_t kPidPosKi      = 0x0104;
constexpr uint16_t kPidPosKd      = 0x0105;
constexpr uint16_t kPidCurKp      = 0x0106;
constexpr uint16_t kPidCurKi      = 0x0107;
constexpr uint16_t kLimitCurrent  = 0x0110;
constexpr uint16_t kLimitVelocity = 0x0111;
constexpr uint16_t kEncElecOffset = 0x0120;
constexpr uint16_t kGearRatio     = 0x0122;
constexpr uint16_t kBoardVbusOvp  = 0x0200;
constexpr uint16_t kBoardVbusUvp  = 0x0201;
constexpr uint16_t kBoardOtpMcu   = 0x0202;
constexpr uint16_t kBoardOtpMos   = 0x0203;
constexpr uint16_t kCanTimeout    = 0x0204;
constexpr uint16_t kPumpPwmFreq   = 0x0300;
constexpr uint16_t kPumpRampMs    = 0x0301;
constexpr uint16_t kCamTrigRate   = 0x0400;
constexpr uint16_t kCamTrigPulse  = 0x0401;
constexpr uint16_t kCamTrigEnable = 0x0402;
}  // namespace param_id

namespace param_op
{
constexpr uint8_t kReadReq  = 0;
constexpr uint8_t kWrite    = 1;
constexpr uint8_t kWriteAck = 2;
constexpr uint8_t kReadResp = 3;
constexpr uint8_t kErr      = 4;
/** 事件通道：param_id 高位置 1 表示这是板卡主动上报的事件，不是参数响应 */
constexpr uint16_t kEventFlag = 0x8000;
}  // namespace param_op

namespace event_code
{
constexpr uint16_t kCalibDone    = 1;
constexpr uint16_t kFault        = 2;
constexpr uint16_t kModeChanged  = 3;
constexpr uint16_t kParamSaved   = 4;
constexpr uint16_t kBoot         = 5;
constexpr uint16_t kCamSyncLost  = 6;
}  // namespace event_code

// ---------------------------------------------------------------- 编解码
/// 把电机反馈帧解成 ROS 消息（不含 header，header 由桥节点打时间戳）。
hw_msgs::msg::MotorState decode_motor_feedback(const MotorFeedbackWire & w,
                                               uint8_t motor_id);

/// 把板级状态帧解出来。
void decode_board_status(const BoardStatusWire & w,
                         float * vbus, float * mcu_temp, float * power,
                         uint16_t * fault, uint8_t * node_state);

/// 把 ROS 指令编成 CAN 帧。
MotionCmdWire encode_motion_cmd(const hw_msgs::msg::MotorCommand & cmd);

/// 把气泵指令编码。
PumpCmdWire encode_pump_cmd(uint8_t cmd, uint8_t mode, float duty, float target_kpa);

/// 解气泵状态。
hw_msgs::msg::PumpState decode_pump_state(const PumpStateWire & w);

/// 解参数帧。
struct ParamDecoded
{
  uint16_t param_id;
  float    value;
  uint8_t  op;
  uint8_t  reserved;
  bool     is_event() const { return (param_id & param_op::kEventFlag) != 0U; }
  uint16_t event_code() const { return static_cast<uint16_t>(param_id & 0x7FFFU); }
};
ParamDecoded decode_param(const ParamWire & w);
ParamWire encode_param(uint16_t param_id, float value, uint8_t op);

// ---------------------------------------------------------------- 工具
inline float q16_to_float(int32_t v) { return static_cast<float>(v) / 65536.0f; }
inline int32_t float_to_q16(float v)
{
  return static_cast<int32_t>(v * 65536.0f);
}

/// 速度单位换算：驱动板给 mrad/s，ROS 用 rad/s
inline float mrads_to_rads(int16_t v) { return static_cast<float>(v) * 1e-3f; }
inline int16_t rads_to_mrads_clamped(float v)
{
  const float m = v * 1000.0f;
  if (m > 32767.0f) { return 32767; }
  if (m < -32768.0f) { return -32768; }
  return static_cast<int16_t>(m);
}

}  // namespace hw_can

#endif  // HW_CAN__PROTOCOL_HPP_
