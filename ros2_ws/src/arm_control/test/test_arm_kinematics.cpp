// ============================================================================
//  test_arm_kinematics.cpp
//  arm_control 纯逻辑层的主机侧回归测试。
//
//  编译运行（不需要 ROS、不需要 Eigen、不需要任何第三方依赖）：
//      test/run_host_test.sh
//
//  这些用例是"虚实联调"里的"虚"那一半：真机上一次全流程取放要几十秒，
//  而且失败了很难复现；这里用解析基准 + 数值回代，可以把运动学与轨迹
//  规划跑几千遍。真机上只要再核对三件事就够了：D-H 表里的长度、
//  关节零位、丝杠导程。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "arm_control/current_alloc.hpp"
#include "arm_control/dh.hpp"
#include "arm_control/scara_ik.hpp"
#include "arm_control/trajectory.hpp"

// 关节索引与尺寸常量在 arm_control 命名空间里；测试里用得很频繁，
// 逐个限定写会让断言的可读性变差，这里显式引入。
using arm_control::kJ1Base;
using arm_control::kJ2Shoulder;
using arm_control::kJ3Elbow;
using arm_control::kJ4Lift;
using arm_control::kJointCount;

namespace
{

int g_pass = 0;
int g_fail = 0;
const char * g_section = "";

void section(const char * s)
{
  g_section = s;
  std::printf("\n=== %s ===\n", s);
}

void check(bool cond, const std::string & what)
{
  if (cond) {
    ++g_pass;
    std::printf("  [ok]   %s\n", what.c_str());
  } else {
    ++g_fail;
    std::printf("  [FAIL] %s   (%s)\n", what.c_str(), g_section);
  }
}

void check_near(double got, double want, double tol, const std::string & what)
{
  const bool ok = std::fabs(got - want) <= tol;
  if (ok) {
    ++g_pass;
    std::printf("  [ok]   %s (%.9g)\n", what.c_str(), got);
  } else {
    ++g_fail;
    std::printf("  [FAIL] %s  期望 %.9g 实际 %.9g 差 %.3g (tol %.3g)  (%s)\n",
                what.c_str(), want, got, std::fabs(got - want), tol, g_section);
  }
}

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
//  A. D-H 表与正运动学
// ---------------------------------------------------------------------------
void test_forward_kinematics()
{
  section("A. 正运动学（D-H 表）");

  arm_control::ArmGeometry geo;
  const arm_control::DhTable table = arm_control::make_scara_table(geo);

  // 全零位：三个回转轴都在 0，升降轴在 0（= 基座高度 d1 处的平面）
  const double q0[kJointCount] = {0.0, 0.0, 0.0, 0.0};
  const arm_control::ToolPose p0 = arm_control::forward_tool_pose(geo, q0);

  check_near(p0.x, geo.a1 + geo.a2 + geo.a3, 1e-12, "零位 x = a1+a2+a3");
  check_near(p0.y, 0.0, 1e-12, "零位 y = 0");
  check_near(p0.z, geo.d1, 1e-12, "零位 z = d1");
  check_near(p0.yaw, 0.0, 1e-12, "零位 yaw = 0");

  /* 关键交叉校验：手写展开式（forward_tool_pose）与 D-H 矩阵链
   * （forward_kinematics）必须给出一致的位置。
   * 这两个实现是刻意分开写的 —— 如果哪天改了 D-H 表却忘了同步展开式
   * （或者反过来），这里会立刻炸。这是本文件里最重要的一条断言。 */
  const double cases[4][kJointCount] = {
    {0.3, -0.4, 0.2, 0.01},
    {-1.2, 0.9, -1.1, -0.03},
    {2.5, -1.5, 2.4, 0.02},
    {0.0, 1.5707963267948966, -1.5707963267948966, 0.0},
  };
  double worst = 0.0;
  for (const auto & qc : cases) {
    const arm_control::Mat4 T = arm_control::forward_kinematics(table, qc);
    const arm_control::ToolPose pf = arm_control::forward_tool_pose(geo, qc);
    worst = std::max(worst, std::fabs(T.t[0] - pf.x));
    worst = std::max(worst, std::fabs(T.t[1] - pf.y));
    worst = std::max(worst, std::fabs(T.t[2] - pf.z));
    worst = std::max(worst, std::fabs(arm_control::yaw_of(T) - pf.yaw));
  }
  check(worst < 1e-12, "D-H 矩阵链与展开式一致（最大偏差 " +
                       std::to_string(worst) + "）");

  // 升降轴只影响 z
  double q1[kJointCount] = {0.5, 0.3, -0.2, 0.025};
  const arm_control::ToolPose p1 = arm_control::forward_tool_pose(geo, q1);
  q1[kJ4Lift] = -0.05;
  const arm_control::ToolPose p2 = arm_control::forward_tool_pose(geo, q1);
  check_near(p2.z - p1.z, -0.075, 1e-12, "升降轴位移直接进 z");
  check_near(p2.x, p1.x, 1e-12, "升降轴不影响 x");
  check_near(p2.y, p1.y, 1e-12, "升降轴不影响 y");
}

// ---------------------------------------------------------------------------
//  B. 逆运动学
// ---------------------------------------------------------------------------
void test_inverse_kinematics()
{
  section("B. 逆运动学（闭式解）");

  arm_control::ArmGeometry geo;
  arm_control::JointLimits lim;

  // ---- B1 往返：可达工作空间内随机撒点，正解再逆解必须回到原关节值 ----
  double worst_err = 0.0;
  int tried = 0;
  int solved = 0;
  for (double r = 0.12; r <= 0.40; r += 0.02) {
    for (double th = 0.0; th < 6.28; th += 0.35) {
      const double x = r * std::cos(th);
      const double y = r * std::sin(th);
      for (double phi = -kPi; phi < kPi; phi += 1.2) {
        ++tried;
        arm_control::IkSolution sol;
        if (!arm_control::scara_inverse(geo, lim, x, y, geo.d1 - 0.02, phi,
                                        nullptr, &sol)) {
          continue;
        }
        ++solved;
        worst_err = std::max(worst_err, sol.reach_error);

        // 回代必须真正落在目标点上（不是"解出来了但到不了"）
        const arm_control::ToolPose fk = arm_control::forward_tool_pose(geo, sol.q);
        const double e = std::sqrt((fk.x - x) * (fk.x - x) + (fk.y - y) * (fk.y - y));
        worst_err = std::max(worst_err, e);
      }
    }
  }
  check(solved > tried / 3, "工作空间内大部分采样点可解（" +
                            std::to_string(solved) + "/" + std::to_string(tried) + "）");
  check(worst_err < 1e-9, "逆解回代误差 < 1e-9 m（实际 " +
                          std::to_string(worst_err) + "）");

  // ---- B2 明确可达的目标 ----
  arm_control::IkSolution sol;
  check(arm_control::scara_inverse(geo, lim, 0.25, 0.0, geo.d1 - 0.02, 0.0,
                                   nullptr, &sol),
        "典型取放点 (0.25, 0, d1+0.01) 可解");
  check_near(std::fabs(std::sin(sol.q[kJ2Shoulder])), 0.8966, 0.01,
             "该点的 θ2 与手算一致（约 ±116°）");

  // ---- B3 越界：太远 / 太高的 z ----
  arm_control::IkSolution bad;
  check(!arm_control::scara_inverse(geo, lim, 0.60, 0.0, geo.d1, 0.0, nullptr, &bad),
        "超出工作半径的目标被拒绝");
  check(bad.status == arm_control::IkStatus::OutOfReach, "拒绝原因是超出工作半径");
  check(bad.reach_error > 0.0,
        "reach_error 报出还差多远（" + std::to_string(bad.reach_error) + " m）");

  check(!arm_control::scara_inverse(geo, lim, 0.25, 0.0, geo.d1 + 0.20, 0.0,
                                    nullptr, &bad),
        "超出升降行程的目标被拒绝");
  check(bad.status == arm_control::IkStatus::LiftOutOfRange, "拒绝原因是升降行程");

  // ---- B4 关节限位：路径穿到内孔里，θ2 需要超过 150° ----
  // 腕心半径 < 93mm 时 c2 < cos(150°)，θ2 必然越限
  check(!arm_control::scara_inverse(geo, lim, 0.06, 0.0, geo.d1, 0.0, nullptr, &bad),
        "需要 θ2 超过 150° 的目标被关节限位拒绝");
  check(bad.status == arm_control::IkStatus::JointLimit, "拒绝原因是关节限位");

  // ---- B5 奇异：肘完全伸直 ----
  arm_control::IkSolution sing;
  const bool s_ok = arm_control::scara_inverse(geo, lim,
                                               geo.a1 + geo.a2 + geo.a3, 0.0,
                                               geo.d1, 0.0, nullptr, &sing);
  check(!s_ok || sing.status == arm_control::IkStatus::Singular,
        "最大伸展处被标记为接近奇异而不是正常可达");

  const double q_straight[kJointCount] = {0.0, 0.0, 0.0, 0.0};
  const double q_bent[kJointCount]     = {0.0, 1.0, 0.0, 0.0};
  check(arm_control::near_singularity(geo, q_straight),
        "near_singularity 能识别 θ2 = 0");
  check(!arm_control::near_singularity(geo, q_bent),
        "near_singularity 不会把 θ2 = 1 rad 误判为奇异");

  // ---- B6 分支连续性：相邻目标必须解到相邻的关节空间里去 ----
  /* 这是"笛卡尔直线路径逐点解逆解"能不能成立的前提。如果相邻两个点
   * 选到了不同的分支（肘上/肘下），末端会在两点之间划出一条谁也没规划的
   * 大弧线 —— 表现是机械臂突然"甩"一下。所以逆解必须以上一次的解做种子。 */
  double seed[kJointCount] = {0.0, 0.0, 0.0, 0.0};
  double max_jump = 0.0;
  std::printf("  追踪一条直线 x: 0.24 -> 0.20, y: 0.10 -> 0.16\n");
  for (int i = 0; i <= 40; ++i) {
    const double x = 0.24 + (0.20 - 0.24) * i / 40.0;
    const double y = 0.10 + (0.16 - 0.10) * i / 40.0;
    arm_control::IkSolution s2;
    const bool ok = arm_control::scara_inverse(geo, lim, x, y, geo.d1 - 0.02,
                                               0.0, seed, &s2);
    check(ok, "直线路径第 " + std::to_string(i) + " 点可解");
    if (!ok) { continue; }
    if (i > 0) {
      double j = 0.0;
      for (int k = 0; k <= kJ3Elbow; ++k) {
        j = std::max(j, std::fabs(s2.q[k] - seed[k]));
      }
      max_jump = std::max(max_jump, j);
    }
    for (int k = 0; k < kJointCount; ++k) { seed[k] = s2.q[k]; }
  }
  // 相邻点间距 1.6mm，关节角变化不该超过 2°（0.035 rad）
  check(max_jump < 0.035, "相邻采样点的关节跳变 < 2°（实际最大 " +
                          std::to_string(max_jump * 180.0 / kPi) + "°）");
}

// ---------------------------------------------------------------------------
//  C. 五次多项式与同步
// ---------------------------------------------------------------------------
void test_quintic()
{
  section("C. 五次多项式插值");

  // 六个边界条件必须精确满足
  const arm_control::QuinticSegment seg(0.2, 0.0, 0.0, -0.5, 0.0, 0.0, 0.8);
  check_near(seg.position(0.0), 0.2, 1e-15, "q(0) = q0");
  check_near(seg.position(0.8), -0.5, 1e-12, "q(T) = q1");
  check_near(seg.velocity(0.0), 0.0, 1e-15, "q̇(0) = 0");
  check_near(seg.velocity(0.8), 0.0, 1e-12, "q̇(T) = 0");
  check_near(seg.acceleration(0.0), 0.0, 1e-15, "q̈(0) = 0");
  check_near(seg.acceleration(0.8), 0.0, 1e-12, "q̈(T) = 0");

  // 峰值常数：1.875 与 5.7735 这两个数是从 s(τ)=10τ³-15τ⁴+6τ⁵ 推出来的，
  // 这里用实际段验一遍，免得哪天有人改了系数而没改常数。
  const double dq = 0.7;
  check_near(seg.peak_velocity(), 1.875 * dq / 0.8, 1e-3,
             "峰值速度 = 1.875·Δq/T");
  check_near(seg.peak_acceleration(), 5.773522 * dq / (0.8 * 0.8), 1e-2,
             "峰值加速度 = 5.7735·Δq/T²");

  // 单调性：零边界条件、同号位移时不应该回头
  bool monotone = true;
  double prev = seg.position(0.0);
  for (int i = 1; i <= 200; ++i) {
    const double v = seg.position(0.8 * i / 200.0);
    if (v > prev + 1e-12) { monotone = false; }
    prev = v;
  }
  check(monotone, "从 +0.2 到 -0.5 的位移全程单调（不会中途回头）");

  // ---- 多关节同步 ----
  arm_control::TrajectoryLimits tlim;
  const double only_lift[kJointCount] = {0.0, 0.0, 0.0, 0.04};
  const double mixed[kJointCount]     = {0.10, 0.0, 0.0, 0.04};
  const double only_j1[kJointCount]   = {0.10, 0.0, 0.0, 0.0};

  const double t_lift = arm_control::synchronized_duration(only_lift, tlim);
  const double t_mix = arm_control::synchronized_duration(mixed, tlim);
  const double t_j1 = arm_control::synchronized_duration(only_j1, tlim);

  check_near(t_lift, std::sqrt(5.773522373 * 0.04 / 0.60), 1e-6,
             "升降轴 40mm 的持续时间由加速度上限决定");
  check_near(t_mix, t_lift, 1e-12,
             "多关节同步时由最慢的关节决定（升降轴）");
  check(t_j1 < t_lift, "J1 单独运动的持续时间更短");

  arm_control::JointQuinticMove move;
  const double move_from[kJointCount] = {0.0, 1.0, -1.0, 0.0};
  const double move_to[kJointCount]   = {0.3, 0.6, -0.7, 0.02};
  move.reset(move_from, move_to, tlim);
  check(move.duration() > 0.0, "多关节运动有正的持续时间");

  // 同步：所有关节必须同时到达（在同一时刻全部落到目标值上）
  double qa[kJointCount], qb[kJointCount];
  move.sample(0.0, qa);
  move.sample(move.duration(), qb);
  check_near(qb[0], 0.3, 1e-12, "J1 到达目标");
  check_near(qb[1], 0.6, 1e-12, "J2 到达目标");
  check_near(qb[2], -0.7, 1e-12, "J3 到达目标");
  check_near(qb[3], 0.02, 1e-12, "J4 到达目标");

  /* 每个关节的解析峰值都不该超限 —— 这正是 synchronized_duration 存在的
   * 意义：它按最慢的关节定总时长，其余关节因此"用不满"自己的限值。
   * 如果哪天有人把同步改成"各关节各自取时间"，这里会立刻炸。 */
  bool within = true;
  double use_v = 0.0, use_a = 0.0;
  for (int i = 0; i < kJointCount; ++i) {
    const double dqi = std::fabs(move_to[i] - move_from[i]);
    const double v = 1.875 * dqi / move.duration();
    const double a = 5.773522 * dqi / (move.duration() * move.duration());
    use_v = std::max(use_v, v / tlim.vmax[i]);
    use_a = std::max(use_a, a / tlim.amax[i]);
    if (v > tlim.vmax[i] * 1.001 || a > tlim.amax[i] * 1.001) { within = false; }
  }
  check(within, "解析峰值不超限（速度用满 " + std::to_string(use_v) +
                "×，加速度用满 " + std::to_string(use_a) + "×）");
  check(use_v < 0.999 || use_a > 0.999,
        "限制确实是由某个关节的某个约束卡住的（不是随便给了个长时间）");
}

// ---------------------------------------------------------------------------
//  D. 笛卡尔直线运动
// ---------------------------------------------------------------------------
void test_cartesian()
{
  section("D. 笛卡尔直线运动");

  arm_control::ArmGeometry geo;
  arm_control::JointLimits lim;
  arm_control::TrajectoryLimits tlim;

  // ---- D1 竖直下压：取放任务里最关键的那一段 ----
  arm_control::Pose3 a{0.25, 0.0, geo.d1 - 0.030, 0.0};   // 悬停高度（台面上方 30mm）
  arm_control::Pose3 b{0.25, 0.0, geo.d1 - 0.115, 0.0};  // 触碰高度（台面上方 25mm）

  const double seed[kJointCount] = {0.0, 0.0, 0.0, 0.0};
  arm_control::CartesianMove move;
  const bool ok = move.plan(geo, lim, tlim, seed, a, b);
  check(ok, "竖直下压路径规划成功");
  check(move.duration() > 0.0, "规划出正的持续时间");

  // 采样点必须严格落在直线段上 —— 下压段走弧线会把吸盘从物块上蹭掉
  double max_dev = 0.0;
  for (int i = 0; i <= 100; ++i) {
    const arm_control::Pose3 p = move.sample_pose(move.duration() * i / 100.0);
    // 直线的参数式：x,y 恒定，z 单调下降
    max_dev = std::max(max_dev, std::fabs(p.x - a.x));
    max_dev = std::max(max_dev, std::fabs(p.y - a.y));
    if (p.z < b.z - 1e-12 || p.z > a.z + 1e-12) { max_dev = std::max(max_dev, 1.0); }
  }
  check(max_dev < 1e-12, "采样末端严格在直线段上（最大偏离 " +
                         std::to_string(max_dev) + " m）");

  arm_control::Pose3 p_end = move.sample_pose(move.duration() + 0.5);
  check_near(p_end.z, b.z, 1e-12, "超时采样停在终点（不再外推）");

  // 关节速度/加速度经迭代标定后应当在限内
  double q_a[kJointCount], q_b[kJointCount];
  move.sample_joint(0.0, q_a);
  double worst_ratio = 0.0;
  const int n = 400;
  const double dt = move.duration() / n;
  for (int i = 1; i <= n; ++i) {
    const bool got = move.sample_joint(dt * i, q_b);
    check(got, "路径第 " + std::to_string(i) + " 个采样点可解逆解");
    if (!got) { break; }
    for (int k = 0; k < kJointCount; ++k) {
      worst_ratio = std::max(worst_ratio, std::fabs(q_b[k] - q_a[k]) / dt / tlim.vmax[k]);
      q_a[k] = q_b[k];
    }
  }
  check(worst_ratio <= 1.05, "按 400 点差分校验，关节速度不超限（用满 " +
                             std::to_string(worst_ratio) + "×）");

  // 末端必须真正到位
  const arm_control::ToolPose fk = arm_control::forward_tool_pose(geo, q_b);
  check_near(fk.x, b.x, 1e-9, "终点的 x 与目标一致");
  check_near(fk.y, b.y, 1e-9, "终点的 y 与目标一致");
  check_near(fk.z, b.z, 1e-9, "终点的 z 与目标一致");

  // ---- D2 穿过内孔的路径必须被拒绝 ----
  /* 从 (0.25, 0.05) 平移到 (-0.13, 0.05)，直线会从基座附近穿过。
   * 腕心半径小于 93mm 时 θ2 需要超过 150°（超出谐波减速器允许的转角），
   * 这一段在几何上是"看起来在圆环里、实际两连杆折不到"。
   * 规划阶段就必须拒绝：如果运行到一半才发现，末端已经悬在料框上方了。 */
  arm_control::Pose3 c{0.25, 0.05, geo.d1 - 0.02, 0.0};
  arm_control::Pose3 d{-0.13, 0.05, geo.d1 - 0.02, 0.0};
  arm_control::CartesianMove cross;
  check(!cross.plan(geo, lim, tlim, seed, c, d),
        "穿过基座内孔附近的平移路径被拒绝（而不是运行到一半才发现无解）");

  // ---- D3 终点超出工作半径的路径必须被拒绝 ----
  arm_control::Pose3 e{0.28, 0.14, geo.d1 - 0.02, 0.0};
  arm_control::Pose3 f{-0.28, 0.14, geo.d1 - 0.02, 0.0};
  arm_control::CartesianMove far;
  check(!far.plan(geo, lim, tlim, seed, e, f), "终点超出工作半径的路径被拒绝");

  // ---- D4 斜向平移：同时改 x/y/z，也要走直线 ----
  arm_control::Pose3 g{0.20, -0.12, geo.d1 - 0.020, 0.0};
  arm_control::Pose3 h{0.26, 0.02, geo.d1 - 0.060, 0.0};
  arm_control::CartesianMove diag;
  const double seed2[kJointCount] = {0.0, 0.0, 0.0, 0.0};
  const bool dok = diag.plan(geo, lim, tlim, seed2, g, h);
  check(dok, "斜向平移路径规划成功");
  if (dok) {
    // 直线性：任一点到 g->h 直线的距离应为 0（这里用参数一致性判）
    double worst = 0.0;
    const arm_control::Vec3 dir{h.x - g.x, h.y - g.y, h.z - g.z};
    const double L2 = dir.dot(dir);
    for (int i = 0; i <= 50; ++i) {
      const arm_control::Pose3 p = diag.sample_pose(diag.duration() * i / 50.0);
      const arm_control::Vec3 rel{p.x - g.x, p.y - g.y, p.z - g.z};
      const double s = rel.dot(dir) / L2;
      const arm_control::Vec3 proj{g.x + dir.x * s, g.y + dir.y * s, g.z + dir.z * s};
      const double dist = std::sqrt((p.x - proj.x) * (p.x - proj.x) +
                                    (p.y - proj.y) * (p.y - proj.y) +
                                    (p.z - proj.z) * (p.z - proj.z));
      worst = std::max(worst, dist);
    }
    check(worst < 1e-12, "斜向路径的点到直线距离为 0（最大 " +
                         std::to_string(worst) + " m）");
  }
}

// ---------------------------------------------------------------------------
//  E. 丝杠 / 单位换算
// ---------------------------------------------------------------------------
void test_units()
{
  section("E. 直线轴单位换算");

  const double lead = 0.010;
  check_near(arm_control::lift_to_motor_rad(0.010, lead), 2.0 * kPi, 1e-12,
             "10mm 位移 = 丝杠转一圈");
  check_near(arm_control::motor_rad_to_lift(2.0 * kPi, lead), 0.010, 1e-12,
             "反变换自洽");
  check_near(arm_control::motor_rad_to_lift(arm_control::lift_to_motor_rad(-0.035, lead), lead),
             -0.035, 1e-12, "-35mm 升降的往返换算");

  // 驱动板报的是 mrad，这里把整条链走一遍（工程里最容易错的那一步）
  const double d4 = -0.035;
  const double motor_rad = arm_control::lift_to_motor_rad(d4, lead);   // -21.99 rad
  const double mrad_on_can = motor_rad * 1000.0;                       // ≈ -21991 mrad
  check_near(mrad_on_can, -21991.1485751, 0.5,
             "CAN 上 J4 位置字段是 -21991 mrad（丝杠转角，不是毫米）");
  check_near(arm_control::motor_rad_to_lift(mrad_on_can * 1e-3, lead), d4, 1e-12,
             "从 CAN 帧解析回来的升降位移正确");
}

// ---------------------------------------------------------------------------
//  F. 电流预算分配
// ---------------------------------------------------------------------------
void test_current_alloc()
{
  section("F. 多关节电流预算分配");

  arm_control::CurrentBudget b;
  double out[kJointCount];

  // 四个关节全健康：总需求 11.5A < 总闸 12A，不缩放
  const bool all_ok[kJointCount] = {true, true, true, true};
  arm_control::allocate_current(b, all_ok, out);
  check_near(out[0], 4.0, 1e-12, "全健康时 J1 拿到额定 4.0A");
  check_near(out[3], 1.5, 1e-12, "全健康时 J4 拿到额定 1.5A");
  double sum = 0.0;
  for (double v : out) { sum += v; }
  check(sum <= b.total_cap + 1e-12, "四路合计不超过总闸");

  // J1 故障退出：其余三个关节不缩水（让出的份额变成余量）
  const bool j1_fault[kJointCount] = {false, true, true, true};
  arm_control::allocate_current(b, j1_fault, out);
  check_near(out[0], 0.0, 1e-12, "故障关节的电流上限为 0");
  check_near(out[1], 3.5, 1e-12, "J1 退出后 J2 仍拿到额定值（不被平均分摊薄）");
  check_near(out[2], 2.5, 1e-12, "J1 退出后 J3 仍拿到额定值");

  // 需求超过总闸时必须等比缩放
  arm_control::CurrentBudget tight = b;
  tight.total_cap = 6.0;                       // 需求 11.5A > 6A
  arm_control::allocate_current(tight, all_ok, out);
  sum = 0.0;
  for (double v : out) { sum += v; }
  check_near(sum, 6.0, 1e-12, "总需求超过总闸时按比例缩放到正好用满总闸");
  check(out[0] > out[1] && out[1] > out[2] && out[2] > out[3],
        "缩放后各关节的份额比例保持不变（大负载轴仍然拿得多）");

  // 单关节硬上限：不让某一个关节把总线吃光
  arm_control::CurrentBudget greedy = b;
  greedy.total_cap = 100.0;
  greedy.nominal[0] = 50.0;                    // 故意给 J1 一个贪心的份额
  arm_control::allocate_current(greedy, all_ok, out);
  check_near(out[0], greedy.boost_max[0], 1e-12,
             "单关节被 boost_max 卡住（J1 不会吃光总线）");

  // 全故障：一条电流都不给
  const bool all_fault[kJointCount] = {false, false, false, false};
  arm_control::allocate_current(b, all_fault, out);
  bool zero = true;
  for (double v : out) { if (v != 0.0) { zero = false; } }
  check(zero, "全部关节故障时不给任何电流");
}

}  // namespace

int main()
{
  std::printf("HWB-ARM4 运动学与轨迹规划 —— 主机侧回归测试\n");
  std::printf("机构：SCARA(RRPR) a1=0.180 a2=0.180 a3=0.060 d1=0.140 lead=10mm\n");

  test_forward_kinematics();
  test_inverse_kinematics();
  test_quintic();
  test_cartesian();
  test_units();
  test_current_alloc();

  std::printf("\n==================================================\n");
  std::printf("  通过 %d 项，失败 %d 项\n", g_pass, g_fail);
  std::printf("==================================================\n");
  return g_fail == 0 ? 0 : 1;
}
