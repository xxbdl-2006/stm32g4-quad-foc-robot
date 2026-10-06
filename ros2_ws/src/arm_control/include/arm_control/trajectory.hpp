// ============================================================================
//  trajectory.hpp
//  轨迹规划：五次多项式（关节空间）与笛卡尔直线（末端空间）。
//
//  为什么是五次而不是梯形速度 / S 曲线
//  ----------------------------------
//  梯形速度只有位移和速度连续，加速度在起停点是阶跃 —— 阶跃加速度意味着
//  阶跃力矩，而谐波减速器的柔性和齿隙会把力矩阶跃激励成末端抖动。
//  三次多项式解决了加速度不连续，但落点加速度不可控，多段拼接时每段
//  接缝处加速度都对不上，末端会"点头"。
//  五次多项式能同时满足**位置、速度、加速度**六个边界条件（6 个系数正好
//  对应 6 个约束），所以多段轨迹拼接起来是 C² 连续的。对"用真空吸盘吸起
//  一个 80mm 的物块"这种任务，末端在抓取点的残余抖动直接决定吸不吸得住，
//  这一点比"用时最短"重要得多。
//
//  两种轨迹的分工
//  --------------
//  · JointQuintic —— 关节空间，用于"从当前姿态到某个中间姿态"这种不关心
//    末端路径形状的运动（回原点、换姿态）。计算量最小，绝对不会撞奇异。
//  · CartesianMove —— 末端走**直线**，逐点解逆解。用于接近/下压/提升这类
//    需要末端走确定路径的段：下压时如果末端走的是弧线，吸盘会沿物块表面
//    滑动，压在边缘上就吸空了 —— 这是取放任务最常见的失败模式。
// ============================================================================
#ifndef ARM_CONTROL__TRAJECTORY_HPP_
#define ARM_CONTROL__TRAJECTORY_HPP_

#include "arm_control/arm_model.hpp"
#include "arm_control/scara_ik.hpp"

namespace arm_control
{

// ---------------------------------------------------------------------------
//  位置 + 末端朝向（末端只有绕 Z 的自由度，所以姿态用一个 yaw 表达）
// ---------------------------------------------------------------------------
struct Pose3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

// ---------------------------------------------------------------------------
//  关节速度/加速度上限
//
//  数值来源：
//   · J1 回转：谐波减速器 1:50 + 电机额定 3000 rpm → 输出侧 6.3 rad/s；
//     但 J1 要带动整条手臂，取 1.5 rad/s（≈86°/s）作为规划上限，留出
//     力矩余量。
//   · J2/J3：惯量小，取 2.0 / 2.5 rad/s。
//   · J4 升降：丝杠导程 10 mm，电机额定 3000 rpm 对应 0.5 m/s；实际取
//     0.15 m/s —— 升降轴带着吸盘与物块垂直运动，冲击直接作用在物块上。
//  加速度上限按"能在 ~0.15 s 内加到满速"估，是这几个关节在实机上
//  不打滑（J2/J3 的谐波减速器不会发闷响）的实测值。
// ---------------------------------------------------------------------------
struct TrajectoryLimits
{
  double vmax[kJointCount]{1.5, 2.0, 2.5, 0.15};
  double amax[kJointCount]{3.0, 5.0, 6.0, 0.60};
};

// ---------------------------------------------------------------------------
//  五次多项式单段
// ---------------------------------------------------------------------------
class QuinticSegment
{
public:
  /// 用六个边界条件构造：q(0)=q0, q'(0)=v0, q''(0)=a0, q(T)=q1, q'(T)=v1, q''(T)=a1
  QuinticSegment() = default;
  QuinticSegment(double q0, double v0, double a0,
                 double q1, double v1, double a1, double T);

  double duration() const { return T_; }
  double position(double t) const;
  double velocity(double t) const;
  double acceleration(double t) const;

  /// 本段内的最大速度 / 最大加速度（求导取极值，不靠采样）
  double peak_velocity() const;
  double peak_acceleration() const;

private:
  double c_[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double T_{0.0};
};

/**
 * @brief 一次多关节运动的**同步**持续时间。
 *
 * 关键点：所有关节用同一个 duration。
 * 如果每个关节各自按自己的极限取时间，末端在运动过程中会走出一条谁也没
 * 规划过的曲线（走得慢的关节"拖着"末端），在狭窄的取放空间里会撞到料框。
 * 取各关节所需时间的最大值，等于"最慢的那个关节决定节奏"，其余关节降速
 * 配合 —— 这正是工业控制器里常见的 time-synchronized 插补。
 *
 * @return 持续时间（秒）。所有关节位移都是 0 时返回 0。
 */
double synchronized_duration(const double dq[kJointCount],
                             const TrajectoryLimits & lim);

/// 单段多关节五次插值（各关节同 T，边界速度/加速度全 0）
class JointQuinticMove
{
public:
  void reset(const double q_from[kJointCount], const double q_to[kJointCount],
             const TrajectoryLimits & lim);

  double duration() const { return T_; }
  bool   done(double t) const { return t >= T_; }

  /// 采样：位置（必填），速度/加速度可传 nullptr
  void sample(double t, double q_out[kJointCount],
              double qd_out[kJointCount] = nullptr,
              double qdd_out[kJointCount] = nullptr) const;

private:
  QuinticSegment seg_[kJointCount];
  double T_{0.0};
};

/**
 * @brief 末端走直线的笛卡尔运动。
 *
 * 时间标定用五次多项式作用在路径参数 s ∈ [0,1] 上，也就是末端沿直线按
 * 五次速度曲线运动。路径中每个采样点都要解一次逆解 —— 逆解失败（走出
 * 工作空间或撞限位）时 plan() 会直接拒绝这条路径，而不是运行到一半才发现，
 * 这一点对"下压"这种末端在物块上方的短运动尤其重要。
 */
class CartesianMove
{
public:
  /**
   * @brief 规划一条从 from 到 to 的直线。
   * @param q_seed 规划时刻的真实关节值，用于选逆解分支与保持连续性
   * @return false 表示路径上有采样点无解，调用方应放弃这次运动
   */
  bool plan(const ArmGeometry & geo, const JointLimits & lim,
            const TrajectoryLimits & tlim,
            const double q_seed[kJointCount],
            const Pose3 & from, const Pose3 & to);

  double duration() const { return T_; }
  bool   valid() const { return valid_; }
  bool   done(double t) const { return t >= T_; }

  /// 路径上的最大 |sin θ2| 最小值（越小越接近奇异，用于日志与拒绝判据）
  double min_sin_theta2() const { return min_sin_t2_; }

  /// 路径终点对应的关节值。用于把多段直线首尾相接时给下一段做种子 ——
  /// 用"上一段的终点解"而不是实测值，段与段之间才不会跳分支。
  void end_q(double q_out[kJointCount]) const
  {
    for (int i = 0; i < kJointCount; ++i) { q_out[i] = end_q_[i]; }
  }

  /// 采样：返回该时刻的关节值。内部用上一次的解做种子，保证分支不跳变。
  bool sample_joint(double t, double q_out[kJointCount]) const;

  /// 采样：返回该时刻的末端位姿（不需要解逆解，调试/RViz 用）
  Pose3 sample_pose(double t) const;

private:
  /**
   * @brief 在 from->to 的直线上按参数 s∈[0,1] 取一个位姿。
   *
   * 只做直线插值，**不做** 二次曲线/避障 —— 需要绕开障碍时，正确做法是
   * 由上层把路径拆成多个 CartesianMove（例如"抬到安全高度 -> 平移到目标
   * 上方 -> 再下压"），而不是在这里偷偷走弧线。走弧线的代价是末端路径
   * 不可预测，而取放任务恰恰要求末端在接近/下压时走确定的路径。
   */
  static Pose3 sample_pose_plane(const Pose3 & a, const Pose3 & b, double s);

  /// 内部采样：调用方传入种子（规划期用探测序列，运行期用上一次的解）
  bool sample_joint_internal(double t, const ArmGeometry & geo,
                             const double seed[kJointCount],
                             double q_out[kJointCount]) const;

  Pose3 from_{}, to_{};
  double T_{0.0};
  bool   valid_{false};
  double min_sin_t2_{1.0};

  /**
   * 路径参数 s(t) 的五次时间标定。s 从 0 到 1、历时 1（真正的时长由
   * 调用时用 t/T_ 归一化体现），因此这个对象与具体路径无关，可以直接
   * 在成员里构造好，不必每次采样都算一遍系数。
   */
  QuinticSegment s_move_{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0};

  /**
   * 上一次采样解出的关节值，作为下一次逆解的种子。
   *
   * 标成 mutable 是为了让 sample_joint 可以是 const：采样在逻辑上是"读"，
   * 但为了保持逆解分支连续，它必须记住上一次的解。把它藏起来（而不是
   * 让调用方自己传种子）能避免一个很容易犯的错 —— 调用方忘了传上一次的
   * 解，结果在路径中间某个点跳到了另一组解，末端"啪"地划一下大弧线。
   */
  mutable double last_q_[kJointCount]{0.0};

  /// 路径终点（s = 1）的关节解，规划阶段一并算好
  double end_q_[kJointCount]{0.0};

  /// 规划时传入的机构与限位。取样期间一直要用，因此持有指针 —— 调用方
  /// 必须保证它们的生命周期覆盖本对象（节点里都是成员，天然满足）。
  const ArmGeometry * geo_{nullptr};
  const JointLimits * lim_{nullptr};
};

}  // namespace arm_control

#endif  // ARM_CONTROL__TRAJECTORY_HPP_
