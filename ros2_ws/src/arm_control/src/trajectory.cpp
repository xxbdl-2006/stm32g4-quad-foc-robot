// ============================================================================
//  trajectory.cpp
// ============================================================================
#include "arm_control/trajectory.hpp"

#include <algorithm>
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

/**
 * 五次多项式（零边界速度/加速度，即 s(τ) = 10τ³ - 15τ⁴ + 6τ⁵）的两个峰值常数。
 *
 *   s'(τ)  = 30τ² - 60τ³ + 30τ⁴  →  τ = 0.5 处取最大值 1.875
 *   s''(τ) = 60τ - 180τ² + 120τ³ →  τ = (3±√3)/6 处 |s''| = 10/√3 ≈ 5.7735
 *
 * 这两个数决定了"给定位移量、求最短可行时间"的解析式：
 *   峰值速度 = 1.875 · |Δq| / T   ≤ vmax  →  T ≥ 1.875·|Δq|/vmax
 *   峰值加速度 = 5.7735 · |Δq| / T² ≤ amax →  T ≥ √(5.7735·|Δq|/amax)
 * 用解析式而不是"试一个 T、采样看看超没超"，是因为这个函数在每个控制周期
 * 都会被调用，不能在里面做迭代。
 */
constexpr double kQuinticPeakVel = 1.875;
constexpr double kQuinticPeakAcc = 5.773522373042444;
}  // namespace

// ---------------------------------------------------------------------------
//  QuinticSegment
// ---------------------------------------------------------------------------
QuinticSegment::QuinticSegment(double q0, double v0, double a0,
                               double q1, double v1, double a1, double T)
: T_(T)
{
  if (T_ <= 0.0) {
    // 退化成"保持不动"。不给 T<=0 时抛异常是因为调用方（多关节同步那段）
    // 在"所有关节位移都是 0"时确实会传 0 进来，那种情况本就该原地不动。
    c_[0] = q0;
    c_[1] = 0.0;
    c_[2] = 0.0;
    c_[3] = 0.0;
    c_[4] = 0.0;
    c_[5] = 0.0;
    T_ = 0.0;
    return;
  }

  const double dq = q1 - q0;
  const double T2 = T_ * T_;
  const double T3 = T2 * T_;
  const double T4 = T3 * T_;
  const double T5 = T4 * T_;

  c_[0] = q0;
  c_[1] = v0;
  c_[2] = 0.5 * a0;
  c_[3] = (20.0 * dq - (8.0 * v1 + 12.0 * v0) * T_ - (3.0 * a0 - a1) * T2) / (2.0 * T3);
  c_[4] = (-30.0 * dq + (14.0 * v1 + 16.0 * v0) * T_ + (3.0 * a0 - 2.0 * a1) * T2) / (2.0 * T4);
  c_[5] = (12.0 * dq - 6.0 * (v1 + v0) * T_ + (a1 - a0) * T2) / (2.0 * T5);
}

double QuinticSegment::position(double t) const
{
  if (T_ <= 0.0) { return c_[0]; }
  const double x = std::clamp(t, 0.0, T_);
  return c_[0] + x * (c_[1] + x * (c_[2] + x * (c_[3] + x * (c_[4] + x * c_[5]))));
}

double QuinticSegment::velocity(double t) const
{
  if (T_ <= 0.0) { return 0.0; }
  const double x = std::clamp(t, 0.0, T_);
  return c_[1] + x * (2.0 * c_[2] + x * (3.0 * c_[3] + x * (4.0 * c_[4] + x * 5.0 * c_[5])));
}

double QuinticSegment::acceleration(double t) const
{
  if (T_ <= 0.0) { return 0.0; }
  const double x = std::clamp(t, 0.0, T_);
  return 2.0 * c_[2] + x * (6.0 * c_[3] + x * (12.0 * c_[4] + x * 20.0 * c_[5]));
}

double QuinticSegment::peak_velocity() const
{
  if (T_ <= 0.0) { return 0.0; }
  /* 速度是四次多项式，它的极值点最多 3 个。这里用 257 点均匀采样 + 端点
   * 取最大：对次数这么低的函数，采样点必然落在每个峰的附近，误差是二阶的
   * （峰附近 v(t) ≈ v_max - k(t-t*)²，最坏落在半个间隔处，相对误差 ~1e-5 量级），
   * 对"取一个不太激进的规划初值"完全够用。
   * 之所以不做解析求根：通用边界条件下 v'(t)=0 是一个三次方程，写它的
   * 闭式解（卡尔达诺公式）会让这段代码比它解决的问题长得多。 */
  constexpr int kN = 256;
  double m = 0.0;
  for (int i = 0; i <= kN; ++i) {
    m = std::max(m, std::fabs(velocity(T_ * static_cast<double>(i) / kN)));
  }
  return m;
}

double QuinticSegment::peak_acceleration() const
{
  if (T_ <= 0.0) { return 0.0; }
  constexpr int kN = 256;
  double m = 0.0;
  for (int i = 0; i <= kN; ++i) {
    m = std::max(m, std::fabs(acceleration(T_ * static_cast<double>(i) / kN)));
  }
  return m;
}

// ---------------------------------------------------------------------------
//  同步持续时间
// ---------------------------------------------------------------------------
double synchronized_duration(const double dq[kJointCount], const TrajectoryLimits & lim)
{
  double T = 0.0;
  for (int i = 0; i < kJointCount; ++i) {
    const double d = std::fabs(dq[i]);
    if (d < 1e-9) { continue; }

    const double t_v = kQuinticPeakVel * d / std::max(1e-9, lim.vmax[i]);
    const double t_a = std::sqrt(kQuinticPeakAcc * d / std::max(1e-9, lim.amax[i]));
    T = std::max(T, std::max(t_v, t_a));
  }
  return T;
}

// ---------------------------------------------------------------------------
//  JointQuinticMove
// ---------------------------------------------------------------------------
void JointQuinticMove::reset(const double q_from[kJointCount],
                             const double q_to[kJointCount],
                             const TrajectoryLimits & lim)
{
  double dq[kJointCount];
  for (int i = 0; i < kJointCount; ++i) { dq[i] = q_to[i] - q_from[i]; }

  T_ = synchronized_duration(dq, lim);
  for (int i = 0; i < kJointCount; ++i) {
    // 边界速度/加速度全 0：要求"到位即停"，不带着速度冲进下一个动作。
    // 这一点对取放任务很关键 —— 下压段结束时的残余速度会变成吸盘的冲击。
    seg_[i] = QuinticSegment(q_from[i], 0.0, 0.0, q_to[i], 0.0, 0.0, T_);
  }
}

void JointQuinticMove::sample(double t, double q_out[kJointCount],
                              double qd_out[kJointCount],
                              double qdd_out[kJointCount]) const
{
  for (int i = 0; i < kJointCount; ++i) {
    q_out[i] = seg_[i].position(t);
    if (qd_out != nullptr) { qd_out[i] = seg_[i].velocity(t); }
    if (qdd_out != nullptr) { qdd_out[i] = seg_[i].acceleration(t); }
  }
}

// ---------------------------------------------------------------------------
//  CartesianMove
// ---------------------------------------------------------------------------
bool CartesianMove::plan(const ArmGeometry & geo, const JointLimits & lim,
                         const TrajectoryLimits & tlim,
                         const double q_seed[kJointCount],
                         const Pose3 & from, const Pose3 & to)
{
  geo_ = &geo;
  lim_ = &lim;
  from_ = from;
  to_ = to;
  valid_ = false;
  T_ = 0.0;
  min_sin_t2_ = 1.0;

  // ---- 1. 先确认两端与若干中间点都有解 ----
  // 先做这一步而不是直接展开采样，是为了尽早失败：一条穿过工作空间外面
  // 的直线要走到一半才发现无解，那时末端已经出发了。
  constexpr int kProbe = 16;
  double q_prev[kJointCount];
  for (int i = 0; i < kJointCount; ++i) { q_prev[i] = q_seed[i]; }

  double probe_q[kProbe + 1][kJointCount];
  for (int i = 0; i <= kProbe; ++i) {
    const double s = static_cast<double>(i) / kProbe;
    const Pose3 p = sample_pose_plane(from, to, s);

    IkSolution sol;
    // 用上一个探测点的解做种子 → 探测过程中不会出现分支跳变，
    // 于是"路径是否连续"这件事在探测阶段就一起验证了。
    if (!scara_inverse(geo, lim, p.x, p.y, p.z, p.yaw, q_prev, &sol)) {
      return false;
    }
    for (int k = 0; k < kJointCount; ++k) {
      probe_q[i][k] = sol.q[k];
      q_prev[k] = sol.q[k];
    }
    min_sin_t2_ = std::min(min_sin_t2_, std::fabs(std::sin(sol.q[kJ2Shoulder])));
  }

  // 终点解留档：多段直线首尾相接时，下一段用它做种子
  for (int k = 0; k < kJointCount; ++k) { end_q_[k] = probe_q[kProbe][k]; }

  // ---- 2. 时间初值：按各关节在整条路径上的最大行程估 ----
  double dq[kJointCount];
  for (int k = 0; k < kJointCount; ++k) {
    double mn = probe_q[0][k];
    double mx = probe_q[0][k];
    for (int i = 1; i <= kProbe; ++i) {
      mn = std::min(mn, probe_q[i][k]);
      mx = std::max(mx, probe_q[i][k]);
    }
    dq[k] = mx - mn;
  }
  T_ = synchronized_duration(dq, tlim);
  if (T_ <= 0.0) { T_ = 1e-3; }   // 纯零位移的退化路径，给一个极小时间即可

  /* ---- 3. 迭代校验关节速度/加速度 ----
   * 直线路径上的关节轨迹是"五次时间标定 ∘ 逆解"的复合，它的速度/加速度
   * 不再满足前面那两条解析界（解析界只对"关节本身走五次多项式"成立）。
   * 尤其是路径经过接近奇异的位置时，关节速度会被放大很多倍。
   * 所以这里真的采样算一遍，超限就把总时间按超限比例放长重来。
   * 收敛很快：关节速度大致与 T 成反比，一次放大就能压到界内，
   * 留 5 轮上限只是为了防住"奇异点附近怎么放长都不够"的病态路径。 */
  for (int iter = 0; iter < 5; ++iter) {
    constexpr int kSample = 200;
    const double dt = T_ / kSample;
    double q_m1[kJointCount];   // t_{i-1}
    double q_0[kJointCount];    // t_i
    double q_p1[kJointCount];   // t_{i+1}

    if (!sample_joint_internal(0.0, geo, q_seed, q_m1)) { return false; }
    if (!sample_joint_internal(dt, geo, q_m1, q_0)) { return false; }

    double worst = 0.0;   // 需要的放长倍率
    for (int i = 1; i < kSample; ++i) {
      const double t = dt * static_cast<double>(i + 1);
      if (!sample_joint_internal(t, geo, q_0, q_p1)) { return false; }

      for (int k = 0; k < kJointCount; ++k) {
        /* 速度用中心一阶差分，加速度用中心二阶差分。
         * 一开始这里写的是 a ≈ 2·Δq/dt²（把相邻两点的差当作加速度上界），
         * 这个式子是错的：它随 dt → 0 发散，采样越密算出来的"加速度"越大。
         * 后果是一条最简单的竖直下压（只有 J4 在动，峰值加速度恰好 0.6 m/s²，
         * 正好压在上限上）被判成超限一百多倍，规划直接被拒 —— 表现为
         * "机械臂拒绝做最普通的抬升动作"。
         * 中心二阶差分才会随 dt→0 收敛。 */
        const double v = std::fabs(q_p1[k] - q_m1[k]) / (2.0 * dt);
        const double a = std::fabs(q_m1[k] - 2.0 * q_0[k] + q_p1[k]) / (dt * dt);
        if (v > tlim.vmax[k]) { worst = std::max(worst, v / tlim.vmax[k]); }
        if (a > tlim.amax[k]) { worst = std::max(worst, std::sqrt(a / tlim.amax[k])); }
        q_m1[k] = q_0[k];
        q_0[k] = q_p1[k];
      }
    }

    if (worst <= 1.02) {
      valid_ = true;
      break;
    }
    T_ *= std::min(4.0, worst * 1.05);
  }

  // 采样种子复位到规划起点，让真正运行时从路径开头连续解下去
  for (int i = 0; i < kJointCount; ++i) { last_q_[i] = q_seed[i]; }
  return valid_;
}

Pose3 CartesianMove::sample_pose(double t) const
{
  if (T_ <= 0.0) { return from_; }
  // 归一到 [0,1]，由 s_move_ 给出五次时间标定下的路径参数
  return sample_pose_plane(from_, to_, s_move_.position(t / T_));
}

Pose3 CartesianMove::sample_pose_plane(const Pose3 & a, const Pose3 & b, double s)
{
  Pose3 p;
  p.x = a.x + (b.x - a.x) * s;
  p.y = a.y + (b.y - a.y) * s;
  p.z = a.z + (b.z - a.z) * s;
  // yaw 沿最短方向线性插值：从一个姿态绕远路转 350° 会撞到自己
  p.yaw = a.yaw + wrap_pi(b.yaw - a.yaw) * s;
  return p;
}

bool CartesianMove::sample_joint_internal(double t, const ArmGeometry & geo,
                                          const double seed[kJointCount],
                                          double q_out[kJointCount]) const
{
  const Pose3 p = sample_pose(t);
  IkSolution sol;
  if (!scara_inverse(geo, *lim_, p.x, p.y, p.z, p.yaw, seed, &sol)) {
    return false;
  }
  for (int i = 0; i < kJointCount; ++i) { q_out[i] = sol.q[i]; }
  return true;
}

bool CartesianMove::sample_joint(double t, double q_out[kJointCount]) const
{
  if (!valid_ || geo_ == nullptr || lim_ == nullptr) { return false; }
  const bool ok = sample_joint_internal(t, *geo_, last_q_, q_out);
  if (ok) {
    for (int i = 0; i < kJointCount; ++i) { last_q_[i] = q_out[i]; }
  }
  return ok;
}

}  // namespace arm_control
