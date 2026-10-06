// ============================================================================
//  scara_ik.cpp
// ============================================================================
#include "arm_control/scara_ik.hpp"

#include <cmath>

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

/// θ2 接近 0 或 ±π 时两臂共线，雅可比退化。判据取 sin(3°)，对应腕心距离
/// 最大伸展只差 0.12 mm —— 换句话说这个"禁区"窄到不影响正常取放，
/// 但足以挡住"目标正好落在边界上时解出 θ2=0 然后满力矩硬顶"这种情形。
constexpr double kSinSingular = 0.0523;

/// 一组解 + 它的分支标记，供优选使用
struct Candidate
{
  double q[kJointCount];
  bool   elbow_up;
  IkStatus status;
  bool   usable;
};

/// 构造平面 2R 的一组解（s2 的符号决定分支）
Candidate build_candidate(const ArmGeometry & geo, const JointLimits & limits,
                          double wx, double wy, double phi, double d4,
                          double c2, double s2, bool elbow_up)
{
  Candidate cd{};
  cd.elbow_up = elbow_up;
  cd.usable = false;

  const double theta2 = std::atan2(s2, c2);
  const double theta1 = wrap_pi(std::atan2(wy, wx) -
                                std::atan2(geo.a2 * s2, geo.a1 + geo.a2 * c2));
  const double theta3 = wrap_pi(phi - theta1 - theta2);

  cd.q[kJ1Base]     = theta1;
  cd.q[kJ2Shoulder] = theta2;
  cd.q[kJ3Elbow]    = theta3;
  cd.q[kJ4Lift]     = d4;

  /* 限位检查逐个关节做，并把"是哪一个关节越限"留给调用方通过 status 知道
   * 大类。这里不区分是哪个关节：对上层业务来说"这组解不能用"就够了，
   * 具体是哪个关节越限属于调试信息（日志里会打出来）。 */
  for (int i = 0; i < kJointCount; ++i) {
    if (!limits.within(i, cd.q[i])) {
      cd.status = IkStatus::JointLimit;
      return cd;
    }
  }

  cd.status = (std::fabs(s2) < kSinSingular) ? IkStatus::Singular : IkStatus::Ok;
  cd.usable = true;
  return cd;
}

/// 两组解之间的关节空间距离（只算三个回转轴；J4 在两组解里相同）
double joint_distance(const double a[kJointCount], const double b[kJointCount])
{
  double s = 0.0;
  for (int i = 0; i <= kJ3Elbow; ++i) {
    const double d = wrap_pi(a[i] - b[i]);
    // 权重的含义：J2/J3 的惯量比 J1 小得多（谐波减速器 1:50 + 臂重集中在
    // 大臂根部），同样的角度跳变，J1 转起来对末端的影响更大。
    // 这里给 J1 一个 1.4 的权重，让优选更倾向于"少转基座"。
    const double w = (i == kJ1Base) ? 1.4 : 1.0;
    s += w * d * d;
  }
  return std::sqrt(s);
}
}  // namespace

const char * to_string(IkStatus s)
{
  switch (s) {
    case IkStatus::Ok:            return "可达";
    case IkStatus::OutOfReach:    return "超出工作半径";
    case IkStatus::LiftOutOfRange:return "超出升降行程";
    case IkStatus::JointLimit:    return "撞关节限位";
    case IkStatus::Singular:      return "接近奇异";
  }
  return "未知";
}

bool near_singularity(const ArmGeometry & geo, const double q[kJointCount])
{
  (void)geo;
  return std::fabs(std::sin(q[kJ2Shoulder])) < kSinSingular;
}

bool scara_inverse(const ArmGeometry & geo, const JointLimits & limits,
                   double x, double y, double z, double phi,
                   const double * seed, IkSolution * out)
{
  if (out == nullptr) { return false; }
  *out = IkSolution{};

  // ---- 1. 直线轴：与平面解完全解耦 ----
  const double d4 = z - geo.d1;
  if (d4 < limits.lo[kJ4Lift] || d4 > limits.hi[kJ4Lift]) {
    out->status = IkStatus::LiftOutOfRange;
    return false;
  }

  // ---- 2. 腕心（J3 轴心的平面坐标）----
  // 末端吸盘中心在 J3 的 φ 方向上偏 a3，所以反推就是沿 -φ 退回 a3。
  const double wx = x - geo.a3 * std::cos(phi);
  const double wy = y - geo.a3 * std::sin(phi);

  // ---- 3. 平面 2R ----
  const double r2 = wx * wx + wy * wy;
  const double c2 = (r2 - geo.a1 * geo.a1 - geo.a2 * geo.a2) / (2.0 * geo.a1 * geo.a2);

  /* 这里判的是**腕心**的可达性，所以外边界是 a1 + a2（不含腕部偏移 a3），
   * 内边界是 |a1 - a2|（两连杆折到极限时的腕心半径）。
   * 用 reach_max()（含 a3）去比是错的 —— 那会把"末端够得着但腕心够不着"
   * 的一小段环带误判成可达，然后在实机上表现为"逆解成功但末端到不了"。
   *
   * 本机构 a1 == a2，内边界为 0，所以 c2 < -1 这一支实际不可达；保留它是
   * 因为换连杆（比如把大臂加长到 0.24）之后它立刻会变成有效分支。 */
  const double wrist_r_max = geo.a1 + geo.a2;
  const double wrist_r_min = std::fabs(geo.a1 - geo.a2);
  const double wrist_r     = std::sqrt(r2);

  if (c2 > 1.0 || c2 < -1.0) {
    out->status = IkStatus::OutOfReach;
    // reach_error 在这里承载"差多远"：正数 = 够不到，负数 = 太靠里
    out->reach_error = (c2 > 1.0) ? (wrist_r - wrist_r_max)
                                  : (wrist_r_min - wrist_r);
    return false;
  }

  const double s2_abs = std::sqrt(std::max(0.0, 1.0 - c2 * c2));

  Candidate up   = build_candidate(geo, limits, wx, wy, phi, d4, c2,  s2_abs, true);
  Candidate down = build_candidate(geo, limits, wx, wy, phi, d4, c2, -s2_abs, false);

  // ---- 4. 优选 ----
  const Candidate * chosen = nullptr;
  if (up.usable && down.usable) {
    if (seed == nullptr) {
      // 没有当前位姿可参考时取肘上解。这是 SCARA 的常规工作姿态
      // （小臂在外侧），能避开基座法兰的干涉。
      chosen = &up;
    } else {
      chosen = (joint_distance(up.q, seed) <= joint_distance(down.q, seed))
                 ? &up : &down;
    }
  } else if (up.usable) {
    chosen = &up;
  } else if (down.usable) {
    chosen = &down;
  } else {
    /* 两组解都不可用。此时最可能的原因是"几何上有解但被关节限位挡住"
     * （比如目标在正后方，θ1 要转 190°，而限位是 ±180°）—— 报 JointLimit
     * 比报 OutOfReach 更准确，因为后者会让现场去查机械尺寸，而问题其实
     * 出在工作区摆放上。只有当两组都不是越限时才沿用几何上的原因。 */
    const bool limit_blocked = (up.status == IkStatus::JointLimit) ||
                               (down.status == IkStatus::JointLimit);
    out->status = limit_blocked ? IkStatus::JointLimit : up.status;
    out->elbow_up = true;
    return false;
  }

  for (int i = 0; i < kJointCount; ++i) { out->q[i] = chosen->q[i]; }
  out->elbow_up = chosen->elbow_up;
  out->status = chosen->status;

  // ---- 5. 回代校验 ----
  /* 用正解算一遍实际末端位置，把误差一起返回。
   * 这一步不是"算法需要"，而是为了在标定时能拿到一个可量化的指标：
   * 如果 reach_error 稳定在几毫米，说明 D-H 表里的某个长度写错了；
   * 如果它在 1e-12 量级，说明运动学自洽，剩下的误差全在机械装配上。 */
  const ToolPose fk = forward_tool_pose(geo, out->q);
  out->reach_error = std::sqrt((fk.x - x) * (fk.x - x) +
                               (fk.y - y) * (fk.y - y) +
                               (fk.z - z) * (fk.z - z));
  return true;
}

}  // namespace arm_control
