// ============================================================================
//  current_alloc.hpp
//  多关节共用一份电流预算时的分配。
//
//  为什么需要它
//  ------------
//  四路驱动的电流不是独立的：它们共用同一个 24V 母线、同一块散热基板、
//  同一路总闸。所以"给每个关节各自一个固定上限"是不成立的 —— 四个关节
//  同时要满力矩时，总电流会超出电源与散热的能力。
//
//  这里的做法是**总额恒定**：给定一个总线电流上限和四个关节的额定份额，
//  按份额分配；某个关节因为故障退出时，它让出的那一份自动被其余关节吸收
//  （受各自的 boost 上限约束，不让某一个关节独自吃满）。
//
//  为什么不用"平均分"
//  ----------------
//  四个关节的负载完全不同：J1 要带动整条手臂的转动惯量，J4 只推一个
//  吸盘加物块。平均分会让 J1 在需要力矩的时候拿不到，而 J4 空着一半额度。
//  份额按实际负载比例给（见下面的默认值），并且可以被标定覆盖。
// ============================================================================
#ifndef ARM_CONTROL__CURRENT_ALLOC_HPP_
#define ARM_CONTROL__CURRENT_ALLOC_HPP_

#include <algorithm>

#include "arm_control/arm_model.hpp"

namespace arm_control
{

/**
 * @brief 每关节的电流预算份额（A）。
 *
 * 默认值来源：24V 母线在四路同时工作时的实测峰值分配。J1 是大臂+小臂+
 * 腕部的整体回转，惯量最大；J2 承力最大但行程短；J3 只带腕部；J4 是丝杠，
 * 静态保持几乎不耗电流，只在升降加速时取一点。
 * total_cap 取 12A 是按 750W/24V 电源（31A）留出一半余量给气泵与浪涌定的 ——
 * 气泵启动瞬时能吃到 10A 以上，那一瞬间必须给电机留下至少 12A 的通路，
 * 否则会触发板卡的欠压保护。
 */
struct CurrentBudget
{
  double nominal[kJointCount]{4.0, 3.5, 2.5, 1.5};   ///< 各关节额定份额（A）
  double boost_max[kJointCount]{6.0, 6.0, 6.0, 4.0}; ///< 单关节绝对上限（A）
  double total_cap{12.0};                            ///< 四路合计上限（A）
};

/**
 * @brief 按故障状态分配电流上限。
 *
 * @param b        预算配置
 * @param healthy  各关节是否健康（false 的关节输出 0）
 * @param out      输出：每关节的电流上限（A）
 */
inline void allocate_current(const CurrentBudget & b,
                             const bool healthy[kJointCount],
                             double out[kJointCount])
{
  for (int i = 0; i < kJointCount; ++i) { out[i] = 0.0; }

  double demand = 0.0;
  for (int i = 0; i < kJointCount; ++i) {
    if (healthy[i]) { demand += b.nominal[i]; }
  }
  if (demand <= 0.0) {
    return;   // 全故障：一条电流都不给
  }

  /* 只有"四个关节的总需求超过总闸"时才按比例缩。
   * 有故障关节退出时需求变小，通常就不会触发缩放 —— 这正是我们要的：
   * 它让出的那一份自动变成了其余关节的余量，不需要额外的"补偿"逻辑。 */
  const double scale = (demand > b.total_cap) ? (b.total_cap / demand) : 1.0;

  for (int i = 0; i < kJointCount; ++i) {
    if (!healthy[i]) { continue; }
    // 单关节硬上限：不让某一个关节把总线额度吃光（其余关节会因此失力矩）
    out[i] = std::min(b.nominal[i] * scale, b.boost_max[i]);
  }
}

}  // namespace arm_control

#endif  // ARM_CONTROL__CURRENT_ALLOC_HPP_
