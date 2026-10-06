// ============================================================================
//  linalg.hpp
//  运动学层用到的最小线性代数：3 维向量 + 4x4 齐次变换。
//
//  为什么不用 Eigen
//  ---------------
//  这个模块只用到"3 维向量"和"4x4 齐次变换相乘"两件事，加起来不到 80 行。
//  引入 Eigen 的代价是：
//    · 目标机（ROS 环境）要装 eigen 包，主机侧回归测试要准备垫片；
//    · Eigen 的表达式模板在 -O2 下会把简单的矩阵乘法展开成一大堆标量表达式，
//      反汇编时不好对照 —— 而这个模块是"虚实联调"的基准，要能逐行核对。
//  所以这里自己写。求解器（逆解）用的是闭式几何法，也不需要 Eigen 的数值代数。
//  真正需要 Eigen 的是视觉投影那一侧（动态尺寸矩阵），那边照旧用 Eigen。
// ============================================================================
#ifndef ARM_CONTROL__LINALG_HPP_
#define ARM_CONTROL__LINALG_HPP_

#include <cmath>

namespace arm_control
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  double norm() const { return std::sqrt(x * x + y * y + z * z); }
  double dot(const Vec3 & o) const { return x * o.x + y * o.y + z * o.z; }
};

/// 刚体变换：p_parent = R * p_child + t
struct Mat4
{
  double r[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  double t[3]{0, 0, 0};

  static Mat4 identity() { return Mat4{}; }

  static Mat4 from_rpy(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);

    Mat4 m;
    m.r[0][0] = cy * cp;
    m.r[0][1] = cy * sp * sr - sy * cr;
    m.r[0][2] = cy * sp * cr + sy * sr;
    m.r[1][0] = sy * cp;
    m.r[1][1] = sy * sp * sr + cy * cr;
    m.r[1][2] = sy * sp * cr - cy * sr;
    m.r[2][0] = -sp;
    m.r[2][1] = cp * sr;
    m.r[2][2] = cp * cr;
    return m;
  }

  /// 复合：this ∘ other（先 other 再 this）
  Mat4 operator*(const Mat4 & o) const
  {
    Mat4 m;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        m.r[i][j] = r[i][0] * o.r[0][j] + r[i][1] * o.r[1][j] + r[i][2] * o.r[2][j];
      }
      m.t[i] = r[i][0] * o.t[0] + r[i][1] * o.t[1] + r[i][2] * o.t[2] + t[i];
    }
    return m;
  }

  Vec3 apply(const Vec3 & p) const
  {
    Vec3 o;
    o.x = r[0][0] * p.x + r[0][1] * p.y + r[0][2] * p.z + t[0];
    o.y = r[1][0] * p.x + r[1][1] * p.y + r[1][2] * p.z + t[1];
    o.z = r[2][0] * p.x + r[2][1] * p.y + r[2][2] * p.z + t[2];
    return o;
  }

  /// 绕 Z 轴转 + 沿 Z 平移（D-H 里最常用的两个动作的复合）
  static Mat4 rz_tz(double theta, double d)
  {
    const double c = std::cos(theta), s = std::sin(theta);
    Mat4 m;
    m.r[0][0] = c; m.r[0][1] = -s;
    m.r[1][0] = s; m.r[1][1] = c;
    m.t[2] = d;
    return m;
  }

  /// 沿 X 平移 + 绕 X 转（D-H 里剩下的两个动作的复合）
  static Mat4 tx_rx(double a, double alpha)
  {
    const double c = std::cos(alpha), s = std::sin(alpha);
    Mat4 m;
    m.r[1][1] = c; m.r[1][2] = -s;
    m.r[2][1] = s; m.r[2][2] = c;
    m.t[0] = a;
    return m;
  }
};

/// 把末端姿态的 yaw（绕 Z）从旋转矩阵里取出来。本机构末端只有绕 Z 的自由度，
/// 所以这里不做通用的 RPY 分解，只取 atan2(R10, R00)，避免万向锁带来的歧义。
inline double yaw_of(const Mat4 & m)
{
  return std::atan2(m.r[1][0], m.r[0][0]);
}

}  // namespace arm_control

#endif  // ARM_CONTROL__LINALG_HPP_
