// ============================================================================
//  dh.cpp
// ============================================================================
#include "arm_control/dh.hpp"

namespace arm_control
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrap_pi(double a)
{
  while (a > kPi) { a -= 2.0 * kPi; }
  while (a <= -kPi) { a += 2.0 * kPi; }
  return a;
}
}  // namespace

DhTable make_scara_table(const ArmGeometry & geo)
{
  DhTable t;

  // i = 1：基座回转。θ1 是变量，d1 是基座高度（固定），a1 是大臂长度。
  t.row[0].theta = 0.0;
  t.row[0].d     = geo.d1;
  t.row[0].a     = geo.a1;
  t.row[0].alpha = 0.0;
  t.row[0].prismatic = false;

  // i = 2：大臂回转。d 为 0（两个回转轴共面），a2 是小臂长度。
  t.row[1].theta = 0.0;
  t.row[1].d     = 0.0;
  t.row[1].a     = geo.a2;
  t.row[1].alpha = 0.0;
  t.row[1].prismatic = false;

  // i = 3：小臂回转。a3 是"J3 轴 -> 吸盘中心"的腕部偏移。
  t.row[2].theta = 0.0;
  t.row[2].d     = 0.0;
  t.row[2].a     = geo.a3;
  t.row[2].alpha = 0.0;
  t.row[2].prismatic = false;

  // i = 4：升降轴。θ4 恒为 0（末端没有绕 Z 的第四个自由度 —— 吸盘是圆形的，
  //        不需要绕 Z 自转去对齐物块），变量是 d4，单位 m。
  t.row[3].theta = 0.0;
  t.row[3].d     = 0.0;
  t.row[3].a     = 0.0;
  t.row[3].alpha = 0.0;
  t.row[3].prismatic = true;

  // 基座坐标系就在工作台面（z = 0）上，与 d1 一起把回转平面抬到正确高度，
  // 因此这里不需要额外的固定变换。
  t.base_to_shoulder = Mat4::identity();
  return t;
}

Mat4 forward_kinematics(const DhTable & table, const double q[kJointCount])
{
  Mat4 T = table.base_to_shoulder;

  for (int i = 0; i < DhTable::kRows; ++i) {
    const DhRow & r = table.row[i];

    // 关节变量：回转轴吃 q[i]（rad），直线轴也吃 q[i] —— 注意直线轴在
    // 运动学层用的单位就是**米**，不是丝杠转角。丝杠转角 -> 米 的换算在
    // 节点层用 lift_to_motor_rad() 做（见 arm_model.hpp 的说明）。
    const double theta = r.prismatic ? r.theta : (r.theta + q[i]);
    const double d     = r.prismatic ? (r.d + q[i]) : r.d;

    // i-1_T_i = Rz(θ) · Tz(d) · Tx(a) · Rx(α)
    T = T * Mat4::rz_tz(theta, d) * Mat4::tx_rx(r.a, r.alpha);
  }
  return T;
}

ToolPose forward_tool_pose(const ArmGeometry & geo, const double q[kJointCount])
{
  const double t1 = q[kJ1Base];
  const double t12 = t1 + q[kJ2Shoulder];
  const double t123 = t12 + q[kJ3Elbow];

  ToolPose p;
  p.x = geo.a1 * std::cos(t1) + geo.a2 * std::cos(t12) + geo.a3 * std::cos(t123);
  p.y = geo.a1 * std::sin(t1) + geo.a2 * std::sin(t12) + geo.a3 * std::sin(t123);
  p.z = geo.d1 + q[kJ4Lift];
  /* yaw 归一化到 (-π, π]。必须归一：θ1+θ2+θ3 是三个角的和，可以轻易超过 π
   * （例如 2.5 - 1.5 + 2.4 = 3.4），而 D-H 矩阵链给出的朝向由 atan2 得到、
   * 天然落在 (-π, π]。两处表示不一致会让"用正解校验逆解"和"比较期望朝向"
   * 这类判断凭空差出 2π —— 而且只在某些姿态下才出现，特别难查。 */
  p.yaw = wrap_pi(t123);
  return p;
}

void numerical_jacobian(const DhTable & table, const double q[kJointCount],
                        double J[3][kJointCount])
{
  /* 步长选择：回转轴 1e-6 rad，直线轴 1e-6 m。
   * 再小就会被 double 的舍入吃掉（位置量级 0.1~0.5 m，相对精度 ~1e-16，
   * 差分 1e-9 时只剩 7 位有效数字）；再大会把三角函数的非线性带进来，
   * 但本机构曲率很小，1e-6 是标准的前向差分选择。 */
  const double h[kJointCount] = {1e-6, 1e-6, 1e-6, 1e-6};

  for (int c = 0; c < kJointCount; ++c) {
    double qp[kJointCount];
    double qm[kJointCount];
    for (int i = 0; i < kJointCount; ++i) {
      qp[i] = q[i];
      qm[i] = q[i];
    }
    qp[c] += h[c];
    qm[c] -= h[c];

    const Mat4 Tp = forward_kinematics(table, qp);
    const Mat4 Tm = forward_kinematics(table, qm);

    for (int r = 0; r < 3; ++r) {
      J[r][c] = (Tp.t[r] - Tm.t[r]) / (2.0 * h[c]);
    }
  }
}

}  // namespace arm_control
