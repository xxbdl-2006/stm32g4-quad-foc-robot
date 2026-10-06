// ============================================================================
//  dh.hpp
//  标准 D-H（Denavit–Hartenberg）参数表与正运动学。
//
//  为什么用 D-H 而不是直接手写三角函数
//  ----------------------------------
//  本机构的三个回转轴都平行于 Z，手写展开式当然也能得到正确的 FK
//  （而且更快）。但那样写死以后，任何一个连杆长度或轴序的改动都要重新
//  推导一遍闭式解 —— 而这类机械臂恰恰是最容易被改的：换一根长一点的大臂、
//  把升降轴从末端挪到中间，都会发生。
//
//  用 D-H 表的意义是：**机构描述是可替换的数据，正解是通用的算法**。
//  改机构 = 改表，正解代码一行不动；逆解那边同样是查表得到的几何关系。
//
//  标准 D-H 的单级变换（Craig 约定）
//  --------------------------------
//      i-1_T_i = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)
//
//  本机构的表（α 全为 0，因为三个回转轴互相平行 —— 这正是 SCARA 的定义）
//  ------------------------------------------------------------------
//    i │ θ_i    │ d_i │ a_i │ α_i
//   ---+--------+-----+-----+-----
//    1 │ θ1     │ d1  │ a1  │  0
//    2 │ θ2     │  0  │ a2  │  0
//    3 │ θ3     │  0  │ a3  │  0
//    4 │  0     │ d4  │  0  │  0     ← d4 是变量（直线轴），θ4 固定为 0
//
//  α = 0 的后果值得记住：末端姿态只有绕 Z 的旋转 θ1+θ2+θ3，俯仰/横滚
//  恒为 0。所以"把吸盘垂直压到物块上"这件事是机构自带的，不需要姿态规划 ——
//  这是选 SCARA 做取放任务的核心原因。
// ============================================================================
#ifndef ARM_CONTROL__DH_HPP_
#define ARM_CONTROL__DH_HPP_

#include <cstdint>

#include "arm_control/arm_model.hpp"
#include "arm_control/linalg.hpp"

namespace arm_control
{

/// 标准 D-H 表的一行。关节变量按 joint_type 取 theta（回转）或 d（直线）。
struct DhRow
{
  double theta{0.0};        ///< 关节变量（回转轴）或固定偏置
  double d{0.0};            ///< 关节变量（直线轴）或固定偏置
  double a{0.0};            ///< 连杆长度
  double alpha{0.0};        ///< 连杆扭角
  bool   prismatic{false};  ///< true 时变量是 d 而不是 theta
};

struct DhTable
{
  static constexpr int kRows = kJointCount;
  DhRow row[kRows];

  /// 相邻两级（基座 -> 末端法兰）的固定变换
  Mat4 base_to_shoulder{Mat4::identity()};
};

/**
 * @brief 由机构尺寸生成 D-H 表。
 *
 * 单独抽成函数而不是写死在成员初始化里，是为了让"改机构"有一个明确的
 * 入口：想试一根更长的大臂，改这里或直接构造 DhTable 即可。
 */
DhTable make_scara_table(const ArmGeometry & geo);

/**
 * @brief 正运动学：关节值 -> 末端（吸盘中心）在基座系下的位姿。
 * @param q 关节值数组：q[0..2] 单位 rad，q[3] 单位 **m**（直线轴）
 */
Mat4 forward_kinematics(const DhTable & table, const double q[kJointCount]);

/**
 * @brief 正运动学，但只关心末端位置与 yaw（省掉矩阵乘法，热路径上用）。
 */
struct ToolPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};    ///< 末端绕 Z 的朝向，rad
};

ToolPose forward_tool_pose(const ArmGeometry & geo, const double q[kJointCount]);

/**
 * @brief 数值雅可比（3x4，只取位置。用于判断奇异与限速估算）。
 *
 * 解析雅可比对 RRPR 来说也不难写，但数值差分只有 8 次 FK，而 FK 是几十次
 * 乘加 —— 在 100 Hz 的规划周期里完全负担得起，换来的是"改 D-H 表之后
 * 雅可比自动跟着对"，不会出现解析式与表不一致这种最阴的 bug。
 */
void numerical_jacobian(const DhTable & table, const double q[kJointCount],
                        double J[3][kJointCount]);

}  // namespace arm_control

#endif  // ARM_CONTROL__DH_HPP_
