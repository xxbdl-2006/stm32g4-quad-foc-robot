// ============================================================================
//  kinematics.hpp
//  四轮底盘的轮速解算：支持 skid-steer（差速/滑移转向）与 mecanum（麦轮）。
//
//  为什么不用 diff_drive_controller / mecanum_drive_controller 现成的？
//  因为那两个控制器假设驱动层提供的接口是"rad/s 的轮速 + 位置反馈"，
//  而本项目的驱动板还提供了逐轴的力矩限幅与温度/故障信息，需要按轮位
//  做独立的力矩分配（例如某个轮打滑时降低它的限流，把力矩让给对侧）。
//  这一层薄控制器就是用来做这件事的。
// ============================================================================
#ifndef MOTOR_CONTROL__KINEMATICS_HPP_
#define MOTOR_CONTROL__KINEMATICS_HPP_

#include <array>
#include <string>
#include <vector>

namespace motor_control
{

enum class DriveType
{
  SkidSteer,      // 四轮同侧直连（或链条联接），左右各一组
  Mecanum,        // 麦克纳姆轮，45° 辊子
  FourSteer,      // 四轮独立转向（预留）
};

struct WheelGeometry
{
  /// 轮半径（m）
  double wheel_radius{0.065};
  /// 左右轮距（m）
  double track_width{0.42};
  /// 前后轴距（m）
  double wheel_base{0.30};
  /// 减速比：电机轴 -> 轮轴（本项目电机直驱，=1.0）
  double gear_ratio{1.0};
  /// 轮位顺序约定：0=左前 1=右前 2=左后 3=右后
  std::array<double, 4> steer_angle{0.0, 0.0, 0.0, 0.0};   // 仅 FourSteer 用
};

/// 底盘运动学。全部是纯函数式，方便单元测试与在 PC 上离线验证轨迹。
class Kinematics
{
public:
  explicit Kinematics(DriveType type, const WheelGeometry & geo);

  /// 车体速度 -> 四轮角速度（rad/s，电机轴侧）。
  /// @param vx 前进速度 m/s
  /// @param vy 横向速度 m/s（skid-steer 忽略）
  /// @param wz 偏航角速度 rad/s
  std::array<double, 4> body_to_wheels(double vx, double vy, double wz) const;

  /// 四轮角速度 -> 车体速度。用于里程计。
  void wheels_to_body(const std::array<double, 4> & wheel_rad_s,
                      double * vx, double * vy, double * wz) const;

  /// 轮位 -> 该轮在车体坐标系下的位置（用于把轮速积分成车身位姿）
  const std::array<double, 4> & wheel_positions_x() const { return wheel_x_; }
  const std::array<double, 4> & wheel_positions_y() const { return wheel_y_; }

  /// 按当前工况给每轮分配电流上限（力矩分配）。
  /// 策略：正常均分；某个轮被标记 SLIP/LIMIT 时把它的份额按比例让给对侧同轴轮。
  std::array<double, 4> allocate_current(double total_budget_a,
                                         const std::array<uint8_t, 4> & fault_flags) const;

  DriveType type() const { return type_; }
  const WheelGeometry & geometry() const { return geo_; }

private:
  DriveType     type_;
  WheelGeometry geo_;
  std::array<double, 4> wheel_x_{};
  std::array<double, 4> wheel_y_{};
};

/// 从参数名解析驱动类型字符串（"skid_steer" / "mecanum" / "four_steer"）。
DriveType parse_drive_type(const std::string & s);

}  // namespace motor_control

#endif  // MOTOR_CONTROL__KINEMATICS_HPP_
