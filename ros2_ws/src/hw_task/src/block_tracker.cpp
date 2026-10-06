// ============================================================================
//  block_tracker.cpp
// ============================================================================
#include "hw_task/block_tracker.hpp"
#include "hw_task/task_types.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace hw_task
{

namespace
{
/// 本文件里所有三角运算统一用这个常量。
/// 不用 <cmath> 里的 M_PI：glibc 只在 __USE_MISC 下导出它，
/// 用 `-std=c++17`（严格模式）编译时它会消失 —— 而 gnu++17 又有。
/// 自带的常量不会有这个坑，也能保证主机侧测试与目标机行为一致。
constexpr double kPi = 3.14159265358979323846;

constexpr double kColorConflictScale = 0.5;
}

void BlockTracker::reset()
{
  tracks_.clear();
  next_id_ = 1;
  t_ = 0.0;
}

void BlockTracker::update(const std::vector<BlockObservation> & obs, double t)
{
  t_ = t;

  // ---- 1. 全部轨迹先记一次"本帧没被命中" ----
  for (auto & tr : tracks_) {
    tr.misses++;
    tr.stable = false;
  }

  // ---- 2. 贪心最近邻关联 ----
  // 本帧观测数与轨迹数都很小（<10），O(n*m) 的贪心足够，不必上匈牙利算法。
  // 贪心的前提是"每个物块只会被一条轨迹匹配" —— 这在物块间距 > 2*match_radius
  // 时成立，而比赛场地里物块不可能挨那么近。
  std::vector<bool> obs_used(obs.size(), false);
  for (auto & tr : tracks_) {
    double best_d2 = cfg_.match_radius * cfg_.match_radius;
    int best = -1;
    for (std::size_t i = 0; i < obs.size(); ++i) {
      if (obs_used[i]) { continue; }
      const double d2 = (obs[i].pos - tr.pos).squaredNorm();
      if (d2 < best_d2) {
        best_d2 = d2;
        best = static_cast<int>(i);
      }
    }

    if (best >= 0) {
      const auto & o = obs[static_cast<std::size_t>(best)];
      obs_used[static_cast<std::size_t>(best)] = true;

      // 一阶低通平滑位置：投影噪声主要来自像素量化与标定残差，
      // 单帧跳变能到 2~3 cm，而物块是静止的，所以平滑不会引入滞后。
      const double alpha = (tr.hits == 0) ? 1.0 : 0.55;
      tr.pos = alpha * o.pos + (1.0 - alpha) * tr.pos;

      // 颜色冲突：同一条轨迹被两个颜色的观测命中过，取本次命中更可信的那个。
      // 只在空间上确实重叠时才算冲突（见 color_conflict_radius）。
      if (o.color != tr.color && o.color != BlockColor::Unknown) {
        if (tr.color == BlockColor::Unknown) {
          tr.color = o.color;
        } else if (tr.hits < 2U) {
          // 轨迹刚建立，允许改判
          tr.color = o.color;
        } else if ((o.pos - tr.pos).norm() < cfg_.color_conflict_radius) {
          // 已经稳定了还颜色不一致，说明是识别抖动；保留原颜色，
          // 但不要因为这一帧就重置 hits —— 否则方块永远不会稳定。
        }
      }

      tr.yaw = o.yaw;
      tr.area_px = o.area_px;
      tr.hits++;
      tr.total_hits++;
      tr.misses = 0;
      tr.last_seen = t;
      tr.last_pixel_x = o.pixel_x;
      tr.last_pixel_y = o.pixel_y;
    }
  }

  // ---- 3. 本帧没匹配上的观测 -> 新建轨迹 ----
  for (std::size_t i = 0; i < obs.size(); ++i) {
    if (obs_used[i]) { continue; }
    // 与已有轨迹太近的观测不新建（那是关联阶段被抢走的，属于正常竞争）
    bool too_close = false;
    for (const auto & tr : tracks_) {
      if ((obs[i].pos - tr.pos).norm() < cfg_.match_radius * kColorConflictScale) {
        too_close = true;
        break;
      }
    }
    if (too_close) { continue; }

    BlockTrack tr;
    tr.id = next_id_++;
    tr.pos = obs[i].pos;
    tr.color = obs[i].color;
    tr.yaw = obs[i].yaw;
    tr.area_px = obs[i].area_px;
    tr.hits = 1;
    tr.total_hits = 1;
    tr.misses = 0;
    tr.first_seen = t;
    tr.last_seen = t;
    tr.last_pixel_x = obs[i].pixel_x;
    tr.last_pixel_y = obs[i].pixel_y;
    tracks_.push_back(tr);
  }

  // ---- 4. 刷新置信度与 stable 标志 ----
  const uint32_t need = std::max(1U, cfg_.min_stable_frames);
  for (auto & tr : tracks_) {
    tr.confidence = std::min(1.0, static_cast<double>(tr.hits) /
                                   static_cast<double>(need));
    tr.stable = (tr.hits >= need);
  }

  // ---- 5. 淘汰 ----
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
                   [&](const BlockTrack & tr) {
                     if (tr.misses > cfg_.max_missing_frames) { return true; }
                     if ((t - tr.last_seen) > cfg_.track_ttl) { return true; }
                     return false;
                   }),
    tracks_.end());
}

std::vector<BlockTrack> BlockTracker::stable_tracks() const
{
  std::vector<BlockTrack> out;
  for (const auto & tr : tracks_) {
    if (tr.stable) { out.push_back(tr); }
  }
  return out;
}

std::vector<BlockTrack> BlockTracker::rank_targets(
  const std::vector<BlockColor> & order,
  const Eigen::Vector2d & reference,
  const std::vector<uint32_t> & excluded_ids) const
{
  std::vector<BlockTrack> cand;
  for (const auto & tr : tracks_) {
    if (!tr.stable) { continue; }
    if (std::find(excluded_ids.begin(), excluded_ids.end(), tr.id) != excluded_ids.end()) {
      continue;
    }
    cand.push_back(tr);
  }

  // 颜色优先级：没在 order 里出现的颜色排在最后
  auto color_rank = [&](BlockColor c) -> int {
      if (order.empty()) { return 0; }
      for (std::size_t i = 0; i < order.size(); ++i) {
        if (order[i] == c) { return static_cast<int>(i); }
      }
      return static_cast<int>(order.size());
    };

  std::sort(cand.begin(), cand.end(),
            [&](const BlockTrack & a, const BlockTrack & b) {
              const int ra = color_rank(a.color);
              const int rb = color_rank(b.color);
              if (ra != rb) { return ra < rb; }

              const double da = (a.pos - reference).norm();
              const double db = (b.pos - reference).norm();
              if (std::abs(da - db) > 0.01) { return da < db; }

              /* 距离几乎相同时的次序：按轨迹 id。
               * 底盘方案里这里比的是方位角（"少转弯"）；机械臂没有"车头
               * 朝向"这个约束，各方向的运动代价只与关节行程有关，没有
               * 便宜的平局判据。用 id 保证次序稳定即可 —— 关键是**确定性**：
               * 排序不稳定会让同一个目标在两个周期里排出不同顺序，
               * 状态机就会反复切换目标，永远抓不完一个。 */
              return a.id < b.id;
            });

  return cand;
}

BlockTrack BlockTracker::pick(const TaskConfig & cfg,
                              const Eigen::Vector2d & reference,
                              const std::vector<uint32_t> & excluded_ids) const
{
  BlockTrack best;
  bool found = false;
  double best_dist = 1e9;

  for (const auto & tr : tracks_) {
    if (!tr.stable) { continue; }
    if (tr.confidence < cfg.min_confidence) { continue; }
    if (tr.area_px < cfg.min_area_px || tr.area_px > cfg.max_area_px) { continue; }
    if (std::find(excluded_ids.begin(), excluded_ids.end(), tr.id) != excluded_ids.end()) {
      continue;
    }

    /* 只判"到基座回转轴的距离是否落在可达环带里"。
     * 底盘方案里还有一条"方位角不超过 ±34°"—— 那是"车头正前方"的约束，
     * 机械臂没有这个概念：它能转到任意方位。换成环带判据之后，落在
     * 内孔（太近会撞基座）或外缘（接近奇异）的物块会被自然排除。 */
    const double r = (tr.pos - reference).norm();
    if (r < cfg.reach_radius_min || r > cfg.reach_radius_max) { continue; }

    if (r < best_dist) {
      best_dist = r;
      best = tr;
      found = true;
    }
  }

  if (!found) {
    // 返回一个 stable=false 的空轨迹，调用方用它判断"没有可抓目标"
    BlockTrack none;
    none.stable = false;
    return none;
  }
  return best;
}

std::string BlockTracker::summary() const
{
  std::ostringstream os;
  std::size_t stable = 0;
  for (const auto & tr : tracks_) {
    if (tr.stable) { ++stable; }
  }
  os << "轨迹 " << tracks_.size() << " 条（稳定 " << stable << "）：";
  for (const auto & tr : tracks_) {
    os << " #" << tr.id << "[";
    switch (tr.color) {
      case BlockColor::Red:   os << "红"; break;
      case BlockColor::Blue:  os << "蓝"; break;
      case BlockColor::Green: os << "绿"; break;
      default:                os << "?";  break;
    }
    os << " " << tr.hits << "帧 " << (tr.stable ? "稳定" : "未稳") << "]";
  }
  return os.str();
}

}  // namespace hw_task
