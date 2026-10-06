// ============================================================================
//  scara_ik.hpp
//  四自由度 SCARA(RRPR) 的**闭式**逆运动学。
//
//  为什么是闭式而不是数值迭代
//  --------------------------
//  本机构的最后两级（J4 升降、末端吸盘）与 XY 平面解耦，前两级构成一个
//  标准的平面 2R 臂，因此存在解析解。闭式解相比牛顿迭代：
//    · 不会发散，不需要初值猜测，迭代次数为 0；
//    · 能明确区分"无解"与"迭代没收敛"—— 这对"目标不可达就跳过该物块"
//      这个业务逻辑很重要，数值法给不出干净的布尔答案；
//    · 在 100 Hz 的规划周期里可以放心地在同一周期解很多次（笛卡尔直线
//      路径是逐点求解的）。
//
//  解的结构
//  --------
//  给定末端位姿 (x, y, z, φ)，其中 φ 是末端绕 Z 的朝向：
//    1. 直线轴：      d4 = z - d1            （与平面解完全解耦）
//    2. 腕心：        从末端沿 -φ 方向退回 a3，得到 J3 轴心的平面坐标
//    3. 平面 2R：     由 (a1, a2) 到腕心的两解（肘上/肘下）
//    4. 第三回转轴：  θ3 = φ - θ1 - θ2       （把末端朝向补齐）
//
//  于是同一个末端位姿有两组解。选哪一组由**与当前关节值的距离**决定 ——
//  原因见 choose_solution 的注释（关节空间跳变会让末端在两点之间划出一条
//  谁也没规划的弧线，比"只差一点点位置"危险得多）。
// ============================================================================
#ifndef ARM_CONTROL__SCARA_IK_HPP_
#define ARM_CONTROL__SCARA_IK_HPP_

#include "arm_control/arm_model.hpp"
#include "arm_control/dh.hpp"

namespace arm_control
{

enum class IkStatus
{
  Ok = 0,
  OutOfReach,        ///< 腕心超出 a1+a2 能覆盖的环带
  LiftOutOfRange,    ///< 目标 z 超出升降轴行程
  JointLimit,        ///< 解存在但撞关节限位
  Singular,          ///< 接近奇异（腕心落在最大/最小伸展处，雅可比退化）
};

const char * to_string(IkStatus s);

struct IkSolution
{
  IkStatus status{IkStatus::OutOfReach};
  double q[kJointCount]{0.0};   ///< q[0..2] rad，q[3] m
  double reach_error{0.0};      ///< 用正解回代后与目标位置的欧氏距离（m）
  bool   elbow_up{true};

  bool ok() const { return status == IkStatus::Ok; }
};

/**
 * @brief 求逆解。
 * @param geo    机构尺寸
 * @param limits 关节限位（用于拒绝越限解，以及判断哪一组更"省力"）
 * @param x,y,z  目标末端位置（基座系，m）
 * @param phi    目标末端朝向（rad，绕 Z）
 * @param seed   当前关节值，用于在两组解里选跳变小的那一组；传 nullptr 时取肘上解
 * @param out    输出
 * @return 是否有可行解
 */
bool scara_inverse(const ArmGeometry & geo, const JointLimits & limits,
                   double x, double y, double z, double phi,
                   const double * seed, IkSolution * out);

/**
 * @brief 判断某一组关节值下的雅可比是否接近奇异。
 *
 * 判据用的是平面 2R 的解析条件 |sin θ2| < eps —— 这比去看 3x4 数值雅可比的
 * 条件数便宜得多，而且物理含义清楚：θ2 = 0 或 ±π 时两臂共线，末端沿径向的
 * 运动能力退化（这个方向上只能靠 a2 的"折角"提供速度，力矩需求趋于无穷）。
 * 取 eps = sin(3°) ≈ 0.052。
 */
bool near_singularity(const ArmGeometry & geo, const double q[kJointCount]);

}  // namespace arm_control

#endif  // ARM_CONTROL__SCARA_IK_HPP_
