// ============================================================================
//  block_tracker.hpp
//  多帧一致性跟踪 + 目标排序。替代 demo 里的 ThetaList 判据。
//
//  demo 的判据是"同一个角度值在列表里出现两次就算稳定"：
//      count = ThetaList[ThetaList == Theta1].size
//      if count == 2: 抓它
//  在"物块绝对静止 + 图像已裁到物块所在区域"时勉强能用，但有两个问题：
//    1. 按**角度**匹配，而不同物块可能碰巧同角度（方块的对称性让它只有
//       两个可能值），会把两个物块混成一个；
//    2. 判据里没有任何空间信息，视觉一旦抖出第二个假目标就会被当成
//       "稳定"，而假目标的坐标是没意义的。
//  因此这里改成在**工作台坐标系**里匹配：把每帧观测投影到基座系，
//  用空间邻近度做数据关联（最近邻 + 门限），再累计命中帧数。
//  投影之后"同一个物块在不同帧里的抖动"变成了毫米级的位置噪声，
//  而不同物块之间至少隔着几十毫米 —— 这个对比度足以做可靠的关联。
// ============================================================================
#ifndef HW_TASK__BLOCK_TRACKER_HPP_
#define HW_TASK__BLOCK_TRACKER_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

#include "hw_task/task_types.hpp"

namespace hw_task
{

/// 一帧里的一个观测（已投影到基座系的工作台面）
struct BlockObservation
{
  Eigen::Vector2d pos{0.0, 0.0};
  BlockColor color{BlockColor::Unknown};
  double yaw{0.0};               ///< 物块长边方向。仅用于显示：吸盘是圆形的，不影响抓取
  double area_px{0.0};
  double confidence_px{0.0};      ///< 视觉侧给的原始置信度（可选的独立信息源）
  float  pixel_x{0.0f};
  float  pixel_y{0.0f};
};

/// 一条跟踪轨迹
struct BlockTrack
{
  uint32_t id{0};
  Eigen::Vector2d pos{0.0, 0.0};
  BlockColor color{BlockColor::Unknown};
  double yaw{0.0};
  double area_px{0.0};
  double confidence{0.0};         ///< 0..1，= min(1, hits / min_stable)
  uint32_t hits{0};               ///< 连续命中帧数
  uint32_t misses{0};             ///< 连续丢失帧数
  uint32_t total_hits{0};
  double first_seen{0.0};
  double last_seen{0.0};
  bool  stable{false};

  /// 最近一次观测的像素坐标（仅用于调试/可视化，不要用来做判据）
  float last_pixel_x{0.0f};
  float last_pixel_y{0.0f};
};

struct TrackerConfig
{
  /// 数据关联门限：两帧之间同一个物块的位移不超过这个值（米）。
  /// 取值依据：机械臂在最大关节速度下末端最快约 0.35 m/s，视觉周期 33ms
  /// 内位移 12mm；加上投影噪声，取 40mm 有足够余量，又小于物块尺寸
  /// （80mm）所以不会串到隔壁物块。
  double match_radius{0.040};

  /// 至少连续命中多少帧才算稳定
  uint32_t min_stable_frames{3};

  /// 连续丢失多少帧后删除轨迹
  uint32_t max_missing_frames{5};

  /// 单个物块的最大存活时间（秒）。超过就丢弃 —— 防止把已经被搬走的
  /// 物块当成"还在原地"，也防止轨迹 id 无限增长。
  double track_ttl{20.0};

  /// 两个不同颜色的轨迹靠得太近时，认为是同一个物块的颜色抖动，
  /// 取命中多的那个颜色。门限比 match_radius 小，只处理真正的重叠。
  double color_conflict_radius{0.025};
};

class BlockTracker
{
public:
  void configure(const TrackerConfig & cfg) { cfg_ = cfg; }
  const TrackerConfig & config() const { return cfg_; }

  void reset();

  /**
   * @brief 喂入一帧观测。
   * @param obs 本帧全部观测（已投影到基座系工作台面）
   * @param t   本帧时间戳（秒，单调）
   */
  void update(const std::vector<BlockObservation> & obs, double t);

  const std::vector<BlockTrack> & tracks() const { return tracks_; }

  /// 只返回已稳定的轨迹
  std::vector<BlockTrack> stable_tracks() const;

  /**
   * @brief 按"颜色优先顺序 + 距离"给可抓目标排序。
   *
   * 排序规则（依次比较）：
   *   1. 若指定了 color_order，则严格按该顺序分组——比赛里经常要求
   *      "先放红的再放蓝的"，跳过颜色不对的物块比抓错再放回去快得多；
   *   2. 同组内按到基座回转轴的距离升序——先抓近的，关节行程更短、
   *      末端刚度也更好（越远的点越接近奇异）；
   *   3. 距离几乎相同（< 1cm）时按轨迹 id——只为了次序确定。
   *
   * @param order     颜色优先顺序，空表示不分组
   * @param reference 用于计算距离的参考点。机械臂用基座原点（0, 0）
   * @param excluded_ids 本次任务已经处理过的轨迹 id（会被排除）
   */
  std::vector<BlockTrack> rank_targets(const std::vector<BlockColor> & order,
                                       const Eigen::Vector2d & reference,
                                       const std::vector<uint32_t> & excluded_ids) const;

  /**
   * @brief 选出最适合作为当前抓取目标的一个物块。
   * @param cfg 任务配置（用的是 reach_radius_min / reach_radius_max）
   * @param reference 基座回转轴在台面上的投影（机械臂用 (0, 0)）
   * @return 命中则返回轨迹；没有可抓目标返回 stable=false 的空轨迹
   */
  BlockTrack pick(const TaskConfig & cfg,
                  const Eigen::Vector2d & reference,
                  const std::vector<uint32_t> & excluded_ids) const;

  /// 生成一行人类可读的跟踪摘要，用于 TaskStatus.message
  std::string summary() const;

private:
  TrackerConfig cfg_{};
  std::vector<BlockTrack> tracks_{};
  uint32_t next_id_{1};
  double   t_{0.0};
};

}  // namespace hw_task

#endif  // HW_TASK__BLOCK_TRACKER_HPP_
