// ============================================================================
//  kinematics.cpp
// ============================================================================
#include "motor_control/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motor_control
{

namespace
{
constexpr double kSqrt2 = 1.4142135623730951;
}  // namespace

DriveType parse_drive_type(const std::string & s)
{
  if (s == "skid_steer" || s == "skidsteer" || s == "differential") {
    return DriveType::SkidSteer;
  }
  if (s == "mecanum") {
    return DriveType::Mecanum;
  }
  if (s == "four_steer" || s == "foursteer") {
    return DriveType::FourSteer;
  }
  throw std::invalid_argument("未知的驱动类型: " + s);
}

Kinematics::Kinematics(DriveType type, const WheelGeometry & geo)
: type_(type), geo_(geo)
{
  const double hx = geo.wheel_base / 2.0;
  const double hy = geo.track_width / 2.0;

  // 轮位顺序：0=左前 1=右前 2=左后 3=右后
  // x 向前为正，y 向左为正（ROS REP-103）
  wheel_x_ = { hx,  hx, -hx, -hx};
  wheel_y_ = { hy, -hy,  hy, -hy};
}

std::array<double, 4> Kinematics::body_to_wheels(double vx, double vy, double wz) const
{
  const double hx = geo.wheel_base / 2.0;
  const double hy = geo.track_width / 2.0;
  const double r  = geo.wheel_radius;
  const double g  = geo.gear_ratio;

  std::array<double, 4> w{};   // 轮缘线速度 (m/s)，最后统一除半径转 rad/s

  switch (type_) {
    case DriveType::SkidSteer: {
      // 滑移转向：左侧两轮同速，右侧两轮同速。
      // 转向时内外侧存在滑移，真实线速度差比几何计算大，用 slip_factor 补偿。
      // 这里不给 slip_factor —— 它属于标定参数，应放在 odom 积分侧修正，
      // 而不是在指令侧"假装几何是对的"。
      const double v_left  = vx - wz * hy;
      const double v_right = vx + wz * hy;
      w[0] = w[2] = v_left;
      w[1] = w[3] = v_right;
      (void)vy;
      break;
    }

    case DriveType::Mecanum: {
      // 麦轮标准解算（辊子 45°）。1/√2 的系数是因为辊子与轮面成 45°，
      // 实际推动车体的只有分量。
      const double k = 1.0 / kSqrt2;
      w[0] = (vx - vy - wz * (hx + hy)) * k;   // 左前
      w[1] = (vx + vy + wz * (hx + hy)) * k;   // 右前
      w[2] = (vx + vy - wz * (hx + hy)) * k;   // 左后
      w[3] = (vx - vy + wz * (hx + hy)) * k;   // 右后
      break;
    }

    case DriveType::FourSteer: {
      // 四轮独立转向：先算每个轮位需要的速度矢量，再分解成
      // 转向角 + 轮速。这一路本项目暂未启用转向电机，只保留解算骨架。
      for (std::size_t i = 0; i < 4; ++i) {
        const double vxi = vx - wz * wheel_y_[i];
        const double vyi = vy + wz * wheel_x_[i];
        const double ang = std::atan2(vyi, vxi);
        w[i] = std::hypot(vxi, vyi) * std::cos(ang - geo_.steer_angle[i]);
      }
      break;
    }
  }

  // 线速度 -> 电机角速度： w_rad = v / r * gear
  for (auto & v : w) {
    v = v / r * g;
  }
  return w;
}

void Kinematics::wheels_to_body(const std::array<double, 4> & wheel_rad_s,
                                double * vx, double * vy, double * wz) const
{
  const double hx = geo.wheel_base / 2.0;
  const double hy = geo.track_width / 2.0;
  const double r  = geo.wheel_radius;
  const double g  = geo.gear_ratio;

  // 先把电机角速度换算回轮缘线速度
  std::array<double, 4> v{};
  for (std::size_t i = 0; i < 4; ++i) {
    v[i] = wheel_rad_s[i] * r / g;
  }

  switch (type_) {
    case DriveType::SkidSteer: {
      const double v_left  = 0.5 * (v[0] + v[2]);
      const double v_right = 0.5 * (v[1] + v[3]);
      if (vx) { *vx = 0.5 * (v_left + v_right); }
      if (vy) { *vy = 0.0; }
      if (wz) { *wz = (v_right - v_left) / (2.0 * hy); }
      break;
    }

    case DriveType::Mecanum: {
      const double k = kSqrt2 * 0.25;
      const double l = hx + hy;
      if (vx) { *vx = k * ( v[0] + v[1] + v[2] + v[3]); }
      if (vy) { *vy = k * (-v[0] + v[1] + v[2] - v[3]); }
      if (wz) { *wz = (k / l) * (-v[0] + v[1] - v[2] + v[3]); }
      break;
    }

    case DriveType::FourSteer: {
      // 用最小二乘从四轮速度矢量反解车体速度
      double sx = 0.0, sy = 0.0, sw = 0.0, wsum = 0.0;
      for (std::size_t i = 0; i < 4; ++i) {
        sx += v[i] * std::cos(geo_.steer_angle[i]);
        sy += v[i] * std::sin(geo_.steer_angle[i]);
        wsum += 1.0;
      }
      if (vx) { *vx = sx / wsum; }
      if (vy) { *vy = sy / wsum; }
      if (wz) { *wz = 0.0; }   // 需要角速度时再解算，此处不用
      (void)sw;
      break;
    }
  }
}

std::array<double, 4> Kinematics::allocate_current(
  double total_budget_a, const std::array<uint8_t, 4> & fault_flags) const
{
  std::array<double, 4> out{};
  std::array<bool, 4> healthy{};
  int n_healthy = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    healthy[i] = (fault_flags[i] == 0U);
    if (healthy[i]) { ++n_healthy; }
  }

  if (n_healthy == 0) {
    return out;   // 全故障，一条电流都不给
  }

  // 均分之后，把故障轮的份额按"同轴优先"的原则补给健康轮。
  // 同轴优先的理由：给同侧轮补力矩会产生额外偏航力矩，而同轴补不产生。
  const double per = total_budget_a / static_cast<double>(n_healthy);
  for (std::size_t i = 0; i < 4; ++i) {
    if (!healthy[i]) { continue; }

    double extra = 0.0;
    // 左右配对的同轴轮：0<->1（前轴），2<->3（后轴）
    const std::size_t mate = (i % 2 == 0) ? (i + 1) : (i - 1);
    if (!healthy[mate]) {
      extra += total_budget_a / 4.0;   // 吸收掉对侧故障轮的份额
    }
    out[i] = per + extra;
  }

  // 单轴上限：本项目电机额定 6A，绝对不允许超
  for (auto & c : out) {
    c = std::clamp(c, 0.0, 6.0);
  }
  return out;
}

}  // namespace motor_control
