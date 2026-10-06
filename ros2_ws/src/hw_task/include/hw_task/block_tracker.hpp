// ============================================================================
//  block_tracker.hpp
//  多帧一致性跟踪 + 目标排序。替代 demo 里的 ThetaList 判据。
//
//  demo 的判据是"同一个角度值在列表里出现两次就算稳定"：
//      count = ThetaList[ThetaList == Theta1].size
//      if count == 2: 抓它
//  在固定相机 + 静止物块的场景下勉强能用，但有两个问题：
//    1. 按**角度**匹配，而不同物块可能碰巧同角度（方块的对称性让它只有
//       两个可能值），会把两个物块混成一个；
//    2. 相机装在移动底盘上之后，同一个静止物块在连续两帧里的像素坐标
//       完全不同，任何基于像素量的匹配都会失败。
//  因此这里改成在**地面坐标系**里匹配：把每帧观测都投影到 odom 系，
//  用空间邻近度做数据关联（最近邻 + 门限），再累计命中帧数。
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

/// 一帧里的一个观测（已投影到 odom 系地面）
struct BlockObservation
{
  Eigen::Vector2d pos{0.0, 0.0};
  BlockColor color{BlockColor::Unknown};
  double yaw{0.0};
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
  /// 取值依据：车最快 0.85 m/s，控制周期 20 ms 时位移 17 mm；
  /// 加上投影噪声与里程计漂移，取 60 mm 有足够余量，
  /// 又小于物块尺寸（80 mm）所以不会串到隔壁物块。
  double match_radius{0.060};

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
   * @param obs 本帧全部观测（已投影到 odom 系）
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
   *   2. 同组内按到车体的距离升序——先抓近的，减少行驶里程；
   *   3. 距离相同（< 1cm）时按方位角绝对值升序——少转弯。
   *
   * @param order     颜色优先顺序，空表示不分组
   * @param vehicle   车体当前位置
   * @param vehicle_yaw 车体朝向
   * @param keep_ids  本次任务已经处理过的轨迹 id（会被排除）
   */
  std::vector<BlockTrack> rank_targets(const std::vector<BlockColor> & order,
                                       const Eigen::Vector2d & vehicle,
                                       double vehicle_yaw,
                                       const std::vector<uint32_t> & excluded_ids) const;

  /**
   * @brief 选出最适合作为当前抓取目标的一个物块。
   * @param cfg 任务配置里的可抓范围（grasp_radius_*、grasp_bearing_max）
   * @return 命中则返回轨迹；没有可抓目标返回 std::nullopt 语义的 stable=false 轨迹
   */
  BlockTrack pick(const TaskConfig & cfg,
                  const Eigen::Vector2d & vehicle,
                  double vehicle_yaw,
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
