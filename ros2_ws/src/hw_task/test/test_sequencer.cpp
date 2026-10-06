// ============================================================================
//  test_sequencer.cpp
//  hw_task 纯逻辑层的主机侧验证。不依赖 ROS、不依赖 Eigen（用自带的垫片）。
//
//  跑法（在装了 ROS 的目标机上）：
//      colcon test --packages-select hw_task
//  跑法（没有 ROS 的开发机上，直接编译）：
//      g++ -std=c++17 -I include -I test/host_shim
//          test/test_sequencer.cpp src/task_sequencer.cpp
//          src/block_tracker.cpp src/ground_projection.cpp -o /tmp/t && /tmp/t
//
//  为什么值得写这个
//  --------------
//  比赛现场最贵的成本是时间。状态机里"某个状态超时后没清标志，于是任务卡死"
//  这类问题，在真车上复现一次要几分钟，而在仿真里只要毫秒。这里把状态机
//  接一个运动学积分器和假的传感器，就能在 PC 上把完整流程跑几百遍。
//
//  注意：这里假的是**传感器与底盘**，状态机本身跑的是与真车完全相同的那份代码。
// ============================================================================
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "hw_task/block_tracker.hpp"
#include "hw_task/ground_projection.hpp"
#include "hw_task/task_sequencer.hpp"

namespace
{

int g_failures = 0;
int g_checks = 0;

void check(bool cond, const std::string & what)
{
  ++g_checks;
  if (!cond) {
    ++g_failures;
    std::printf("  \033[31m[FAIL]\033[0m %s\n", what.c_str());
  }
}

void check_near(double a, double b, double tol, const std::string & what)
{
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    ++g_failures;
    std::printf("  \033[31m[FAIL]\033[0m %s：得到 %.6f，期望 %.6f（容差 %.6f，差 %.6f）\n",
                what.c_str(), a, b, tol, std::fabs(a - b));
  }
}

void section(const char * name)
{
  std::printf("\n\033[1m== %s ==\033[0m\n", name);
}

// ===========================================================================
//  底盘运动学积分器（差速/滑移转向：只能前进后退 + 转向）
// ===========================================================================
struct VehicleSim
{
  double x{0.0}, y{0.0}, yaw{0.0};
  double vx{0.0}, wz{0.0};
  double t{0.0};

  void step(double cmd_vx, double cmd_wz, double dt)
  {
    // 简单一阶响应：速度环是 1kHz 的，20ms 内基本到位，直接用命令值即可。
    vx = cmd_vx;
    wz = cmd_wz;
    x += vx * std::cos(yaw) * dt;
    y += vx * std::sin(yaw) * dt;
    yaw += wz * dt;
    t += dt;
  }
};

struct FakeWorld
{
  // 物块（可多个）
  struct Block
  {
    double x, y, yaw;
    hw_task::BlockColor color;
    bool on_vehicle{false};
    bool removed{false};
  };
  std::vector<Block> blocks;

  // 气泵
  bool   pump_run{false};
  double pump_duty{0.0};
  double pressure{0.0};      ///< 表压 kPa
  bool   vacuum_capable{true};  ///< false 模拟"吸盘漏气，永远吸不住"
  bool   drop_after_grasp{false}; ///< true 模拟"吸起来又掉件"
  bool   visible{true};

  /// 气泵动力学：开泵后约 0.15s 建立真空
  void pump_step(double dt)
  {
    const double target = pump_run ? (vacuum_capable ? -28.0 : -2.0) : 0.0;
    const double tau = 0.05;
    pressure += (target - pressure) * (dt / tau);
    if (drop_after_grasp && pump_run) {
      // 模拟搬运途中掉件：吸住之后压力突然消失
      pressure += (0.0 - pressure) * (dt / tau) * 3.0;
    }
  }
};

/// 把世界状态打包成 SensorSnapshot
hw_task::SensorSnapshot make_snapshot(const VehicleSim & v, const FakeWorld & w,
                                      bool target_valid, const FakeWorld::Block * tgt)
{
  hw_task::SensorSnapshot s;
  s.t = v.t;
  s.vehicle_x = v.x;
  s.vehicle_y = v.y;
  s.vehicle_yaw = v.yaw;
  s.vehicle_vx = v.vx;
  s.vehicle_wz = v.wz;
  s.vision_ok = w.visible;
  s.blocks_visible = static_cast<uint32_t>(w.blocks.size());
  s.pump_online = true;
  s.pump_pressure_kpa = w.pressure;
  s.target_valid = target_valid;
  if (target_valid && tgt) {
    s.target_x = tgt->x;
    s.target_y = tgt->y;
    s.target_yaw = tgt->yaw;
    s.target_color = tgt->color;
    s.target_confidence = 1.0f;
  }
  return s;
}

// ===========================================================================
//  A. 地面投影的解析验证
// ===========================================================================
void test_ground_projection()
{
  section("A. 地面投影（对照解析解）");

  hw_task::CameraIntrinsics K;
  K.fx = 600.0;
  K.fy = 600.0;
  K.cx = 320.0;
  K.cy = 240.0;
  K.width = 640;
  K.height = 480;

  hw_task::GroundProjection gp;
  gp.set_intrinsics(K);

  /* 构造一个"正对下方"的相机，这样有解析解可比：
   * 相机光轴 z 指向世界 -z（正下方），绕世界 x 轴转 π 即可。
   *   R = diag(1, -1, -1)   （绕 x 转 π）
   *       cam x -> world  x
   *       cam y -> world -y
   *       cam z -> world -z   ← 朝下
   * 相机装在 (0, 0.5, 0.3)。 */
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = 1.0;
  T(1, 1) = -1.0;
  T(2, 2) = -1.0;
  T(0, 3) = 0.0;
  T(1, 3) = 0.5;
  T(2, 3) = 0.3;

  // ① 主点像素 -> 相机正下方的地面点
  {
    const auto p = gp.project(320.0, 240.0, T);
    check(p.valid, "主点像素应该能投到地面");
    check_near(p.x, 0.0, 1e-6, "主点投影 x");
    check_near(p.y, 0.5, 1e-6, "主点投影 y");
    check_near(p.range, 0.3, 1e-6, "主点投影距离应等于相机高度");
  }

  // ② 像素右偏 fx（xd = 1）-> 地面点应偏离 0.3m
  //    射线 (1,0,1)/√2 -> 世界 (1,0,-1)/√2；t = 0.3/(1/√2)；落点 x = 0.3
  {
    const auto p = gp.project(320.0 + 600.0, 240.0, T);
    check(p.valid, "右偏像素应能投到地面");
    check_near(p.x, 0.3, 1e-6, "xd=1 时地面偏移量应等于相机高度");
    check_near(p.y, 0.5, 1e-6, "该点 y 不应变化");
  }

  // ③ 像素下偏 fy（yd = 1）-> 世界 -y 方向 0.3m（因为 cam y 映射到 world -y）
  {
    const auto p = gp.project(320.0, 240.0 + 600.0, T);
    check(p.valid, "下偏像素应能投到地面");
    check_near(p.x, 0.0, 1e-6, "该点 x 不应变化");
    check_near(p.y, 0.2, 1e-6, "yd=1 时地面偏移量应等于相机高度");
  }

  // ④ 射线朝上（相机朝天）-> 必须判无效，不能返回一个荒唐的远点
  {
    Eigen::Matrix4d Tup = Eigen::Matrix4d::Identity();
    Tup(0, 1) = 1.0;
    Tup(1, 0) = 1.0;
    Tup(0, 0) = 0.0;
    Tup(1, 1) = 0.0;
    // 上面构造的是绕 z 转 90°，光轴仍朝下；改成真的朝上：
    Eigen::Matrix4d Tsky = Eigen::Matrix4d::Identity();
    Tsky(2, 2) = 1.0;   // cam z -> world +z（朝上）
    Tsky(2, 3) = 0.3;
    const auto p = gp.project(320.0, 240.0, Tsky);
    check(!p.valid, "射线朝上时必须判无效");
  }

  // 相机的解析理想投影（相机在 (0, 0.5, cam_h)、光轴朝下）
  const double cam_h = 0.3;
  auto ideal = [&](double u, double v) {
      return Eigen::Vector2d((u - 320.0) / 600.0 * cam_h,
                             0.5 - (v - 240.0) / 600.0 * cam_h);
    };

  // ⑤ 残差校正：给一个已知的整体平移，看能不能吸收掉
  {
    hw_task::GroundProjection gp2;
    gp2.set_intrinsics(K);

    /* 三个标定点：像素 -> 真实世界。
     * 真实世界坐标 = **解析**理想投影 + (ox, oy)，模拟"相机实际装歪了 2cm"。
     *
     * 理想投影自己算，不能拿 gp2.project() 去求 —— 因为 gp2 身上已经带着
     * 前面标定点解出来的残差，第二次调用开始它的输出就是"已经校正过"的了，
     * 再加一次偏移会变成 2*offset。第一版就是这么错的，结果校正后误差
     * 反而翻倍，看着像投影算法有 bug。 */
    const double ox = 0.02, oy = -0.015;
    /* 标定点必须**铺开**。第一版用的是三个挨在一起的像素
     * （(300,230)(350,250)(320,240)，地面跨度只有 2.5cm），
     * 结果设计矩阵近乎共线，正规方程解出来的仿射矩阵线性部分乱飞，
     * 靠平移去凑 —— 在标定点附近凑得很准，出了那个小区域就完全失效。
     * 这正是 ground_projection.cpp 里 kMinCalibSpread 那道安全阀要拦的情况。
     * 这里取覆盖整幅画面的三个点，地面跨度约 0.37m。 */
    const double cal_px[3][2] = {{20.0, 20.0}, {620.0, 20.0}, {320.0, 460.0}};
    double last_err = -1.0;
    for (const auto & c : cal_px) {
      const Eigen::Vector2d g = ideal(c[0], c[1]);
      last_err = gp2.add_calibration_point(c[0], c[1], g.x() + ox, g.y() + oy, T);
    }
    check(last_err >= 0.0, "标定点应被接受");
    check(gp2.affine_solved(), "点铺得够开时应该解出完整的 2D 仿射");
    check(gp2.calibration_spread() > hw_task::GroundProjection::kMinCalibSpread,
          "标定点跨度应超过安全阀门槛");
    check(last_err < 1e-3,
          "残差应降到亚 mm 级（实际 " + std::to_string(last_err * 1000.0) + " mm）");

    // 校正后再投一个点，应带上这个偏移
    const auto p = gp2.project(320.0, 240.0, T);
    check_near(p.x, ox, 2e-3, "校正后 x 应带上平移量");
    check_near(p.y, 0.5 + oy, 2e-3, "校正后 y 应带上平移量");
  }

  // ⑥ 标定点挤在一起时必须退化成"只解平移"，而不是硬解出一个坏矩阵
  {
    hw_task::GroundProjection gp3;
    gp3.set_intrinsics(K);

    const double ox = 0.03, oy = 0.012;
    // 三个像素挨得极近（地面跨度约 1cm），远小于 kMinCalibSpread
    const double cal_px[3][2] = {{318.0, 239.0}, {322.0, 241.0}, {320.0, 240.0}};
    for (const auto & c : cal_px) {
      const Eigen::Vector2d g = ideal(c[0], c[1]);
      gp3.add_calibration_point(c[0], c[1], g.x() + ox, g.y() + oy, T);
    }

    check(!gp3.affine_solved(), "点太集中时不应解完整仿射");
    check(gp3.calibration_spread() < hw_task::GroundProjection::kMinCalibSpread,
          "跨度判定应识别出点太集中");
    // 线性部分必须仍是单位阵（只解了平移）
    const Eigen::Matrix3d r = gp3.residual_matrix();
    check_near(r(0, 0), 1.0, 1e-9, "退化情况下线性部分 x 应为 1");
    check_near(r(0, 1), 0.0, 1e-9, "退化情况下交叉项应为 0");
    check_near(r(1, 1), 1.0, 1e-9, "退化情况下线性部分 y 应为 1");
    const auto p3 = gp3.project(320.0, 240.0, T);
    check_near(p3.x, ox, 3e-3, "退化情况下平移仍应被校正");
    check_near(p3.y, 0.5 + oy, 3e-3, "退化情况下平移仍应被校正");
  }
}

// ===========================================================================
//  B. 多帧跟踪与排序
// ===========================================================================
void test_block_tracker()
{
  section("B. 多帧跟踪与目标排序");

  hw_task::TrackerConfig tc;
  tc.match_radius = 0.06;
  tc.min_stable_frames = 3;
  tc.max_missing_frames = 2;
  tc.track_ttl = 20.0;

  hw_task::BlockTracker tr;
  tr.configure(tc);

  auto obs = [](double x, double y, hw_task::BlockColor c) {
      hw_task::BlockObservation o;
      o.pos = Eigen::Vector2d(x, y);
      o.color = c;
      // 面积必须给：pick() 会用 min_area_px/max_area_px 过滤，
      // 留 0 的话所有观测都会被当成"太小"而滤掉（第一版就踩了这个坑）。
      o.area_px = 5000.0;
      return o;
    };

  // ---- ① 连续 3 帧才算稳定 ----
  for (int i = 0; i < 3; ++i) {
    tr.update({obs(0.5, 0.1, hw_task::BlockColor::Red)}, i * 0.033);
    if (i < 2) {
      check(tr.stable_tracks().empty(),
            "第 " + std::to_string(i + 1) + " 帧不应判为稳定");
    }
  }
  check(tr.stable_tracks().size() == 1, "第 3 帧应出现 1 条稳定轨迹");
  check_near(tr.tracks()[0].confidence, 1.0, 1e-9, "稳定后置信度应为 1");

  // ---- ② 位置抖动小于 match_radius 时不应新建轨迹 ----
  tr.update({obs(0.52, 0.11, hw_task::BlockColor::Red)}, 0.1);
  check(tr.tracks().size() == 1, "小抖动仍应是同一条轨迹");

  // ---- ③ 远离超过 match_radius -> 新轨迹 ----
  tr.update({obs(0.52, 0.11, hw_task::BlockColor::Red),
             obs(0.90, -0.30, hw_task::BlockColor::Blue)}, 0.13);
  check(tr.tracks().size() == 2, "出现第二个物块时应新建轨迹");

  // ---- ④ 连续丢失超过 max_missing_frames -> 删除轨迹 ----
  for (int i = 0; i < 4; ++i) {
    tr.update({}, 0.16 + i * 0.033);
  }
  check(tr.tracks().empty(), "连续丢失后轨迹应被删除");

  // ---- ⑤ 颜色优先 + 距离排序 ----
  {
    hw_task::BlockTracker t2;
    t2.configure(tc);
    for (int i = 0; i < 4; ++i) {
      t2.update({obs(0.80, 0.0, hw_task::BlockColor::Blue),
                 obs(0.30, 0.0, hw_task::BlockColor::Red),
                 obs(0.55, 0.0, hw_task::BlockColor::Blue)},
                i * 0.033);
    }
    check(t2.stable_tracks().size() == 3, "应有 3 条稳定轨迹");

    const Eigen::Vector2d veh(0.0, 0.0);
    const auto ranked = t2.rank_targets({hw_task::BlockColor::Red}, veh, 0.0, {});
    check(ranked.size() == 3, "排序后仍应有 3 个");
    check(ranked[0].color == hw_task::BlockColor::Red,
          "指定红色优先时，红色必须排第一");
    check_near(ranked[1].pos.x(), 0.55, 1e-9, "同色内应按距离升序（近的在前）");
    check_near(ranked[2].pos.x(), 0.80, 1e-9, "同色内第二近的在后");

    // 不指定颜色顺序时，纯按距离
    const auto plain = t2.rank_targets({}, veh, 0.0, {});
    check_near(plain[0].pos.x(), 0.30, 1e-9, "不分组时最近的排第一");

    // 排除列表生效
    const auto excl = t2.rank_targets({}, veh, 0.0, {plain[0].id});
    check(excl.size() == 2, "排除一个后应剩 2 个");
    check_near(excl[0].pos.x(), 0.55, 1e-9, "排除后最近的变成 0.55");
  }

  // ---- ⑥ 可抓范围筛选 ----
  {
    hw_task::TaskConfig cfg;
    cfg.min_area_px = 100.0f;
    cfg.max_area_px = 1e9f;
    cfg.min_confidence = 0.5f;
    cfg.grasp_radius_min = 0.18f;
    cfg.grasp_radius_max = 0.85f;
    cfg.grasp_bearing_max = 0.60f;

    hw_task::BlockTracker t3;
    t3.configure(tc);
    for (int i = 0; i < 4; ++i) {
      t3.update({obs(0.10, 0.0, hw_task::BlockColor::Red),   // 太近
                 obs(1.50, 0.0, hw_task::BlockColor::Red),   // 太远
                 obs(0.50, 0.50, hw_task::BlockColor::Red),  // 方位角 45° 超限
                 obs(0.50, 0.05, hw_task::BlockColor::Red)}, // 合法
                i * 0.033);
    }
    const auto pick = t3.pick(cfg, Eigen::Vector2d(0.0, 0.0), 0.0, {});
    check(pick.stable, "应能挑出一个合法目标");
    check_near(pick.pos.x(), 0.50, 1e-9, "被选中的应是那个合法物块");
    check_near(pick.pos.y(), 0.05, 1e-9, "被选中的应是那个合法物块");
  }
}

// ===========================================================================
//  C. 完整取放流程
// ===========================================================================
struct CycleResult
{
  hw_task::TaskState final_state{hw_task::TaskState::Idle};
  uint32_t placed{0};
  uint32_t skipped{0};
  int steps{0};
  double end_x{0.0}, end_y{0.0}, end_yaw{0.0};
  std::string last_message;
};

/// 跑一次完整任务直到进入终态
CycleResult run_cycle(hw_task::TaskSequencer & seq, VehicleSim & veh, FakeWorld & world,
                      const hw_task::StartRequest & req, double block_x, double block_y,
                      double block_yaw, int max_steps = 8000)
{
  CycleResult r;

  hw_task::SensorSnapshot s0 = make_snapshot(veh, world, false, nullptr);
  seq.start(req, s0);

  const double dt = 0.02;
  bool started = true;

  for (int i = 0; i < max_steps; ++i) {
    // --- 视觉：把当前"还没被处理掉"的物块喂给状态机 ---
    FakeWorld::Block * tgt = nullptr;
    for (auto & b : world.blocks) {
      if (!b.removed && !b.on_vehicle) { tgt = &b; break; }
    }

    const auto st = seq.state();
    const bool need_target = (st == hw_task::TaskState::Scan) ||
                             (st == hw_task::TaskState::Approach) ||
                             (st == hw_task::TaskState::Align);

    // 模拟 ROS 节点：处于需要目标的阶段时，持续把目标重新绑定给状态机
    if (need_target && tgt) {
      seq.bind_target(1, Eigen::Vector2d(tgt->x, tgt->y), tgt->color, tgt->yaw, 1.0f);
    }

    hw_task::SensorSnapshot s = make_snapshot(veh, world, tgt != nullptr, tgt);
    const hw_task::Actuation a = seq.update(s, dt);

    // --- 执行动作 ---
    veh.step(a.cmd_vx, a.cmd_wz, dt);
    world.pump_run = a.pump_run;
    world.pump_duty = a.pump_duty;
    world.pump_step(dt);

    // --- 物块状态机：吸住了就跟着车走，放下了就落在车前面 ---
    if (tgt && !tgt->on_vehicle && a.pump_run &&
        std::fabs(world.pressure) > 10.0) {
      // 判定吸住后由状态机自己管，这里只在"释放"时处理
    }
    if (tgt && !tgt->on_vehicle && world.pressure < -10.0 && a.pump_run) {
      tgt->on_vehicle = true;
    }
    if (tgt && tgt->on_vehicle && !a.pump_run) {
      tgt->on_vehicle = false;
      tgt->removed = true;
      // 放到车前方 standoff 处
      tgt->x = veh.x + 0.34 * std::cos(veh.yaw);
      tgt->y = veh.y + 0.34 * std::sin(veh.yaw);
      tgt->yaw = veh.yaw;
    }

    r.steps = i;
    r.last_message = seq.status().message;

    if (a.finished || a.aborted) { break; }
    if (!started) { break; }
  }

  const auto & stt = seq.status();
  r.final_state = stt.state;
  r.placed = stt.grasped_count;
  r.skipped = stt.skipped_count;
  r.end_x = veh.x;
  r.end_y = veh.y;
  r.end_yaw = veh.yaw;
  return r;
}

hw_task::TaskConfig make_task_config()
{
  hw_task::TaskConfig cfg;
  // 判据放宽到测试用（真车参数见 config/task.yaml）
  cfg.min_confidence = 0.5f;
  cfg.min_stable_frames = 1;
  cfg.min_area_px = 100.0f;
  cfg.max_area_px = 1e9f;
  cfg.grasp_radius_min = 0.18f;
  cfg.grasp_radius_max = 0.85f;
  cfg.grasp_bearing_max = 0.70f;

  cfg.approach_speed = 0.35f;
  cfg.align_speed = 0.09f;
  cfg.standoff = 0.34f;
  cfg.standoff_tol = 0.030f;
  cfg.bearing_tol = 0.045f;

  cfg.scan_timeout = 2.0f;      // 测试里缩短，不然一个失败用例要跑 12 秒
  cfg.approach_timeout = 20.0f;
  cfg.align_timeout = 12.0f;
  cfg.settle_time = 0.20f;
  cfg.descend_time = 0.15f;
  cfg.grasp_timeout = 0.8f;
  cfg.lift_time = 0.20f;
  cfg.haul_timeout = 60.0f;
  cfg.place_timeout = 25.0f;
  cfg.release_time = 0.20f;
  cfg.retreat_time = 0.40f;
  cfg.vision_lost_timeout = 1.0f;

  cfg.max_scan_retry = 3;
  cfg.max_align_retry = 2;
  cfg.max_grasp_retry = 2;

  cfg.use_pressure_check = true;
  cfg.use_pump_pressure_mode = false;
  cfg.grasp_duty = 1.0f;
  cfg.hold_duty = 0.75f;
  cfg.vacuum_threshold_kpa = 6.0f;

  cfg.place_x = 0.0;
  cfg.place_y = 1.20;
  cfg.place_yaw = 1.5707963267948966;
  cfg.place_pitch = 0.12;
  cfg.place_columns = 4;
  cfg.place_row_pitch = 0.12;

  cfg.home_x = 0.0;
  cfg.home_y = 0.0;
  cfg.home_yaw = 0.0;
  return cfg;
}

void test_nominal_cycle()
{
  section("C. 完整取放流程（正前方物块）");

  hw_task::TaskSequencer seq;
  seq.configure(make_task_config());

  VehicleSim veh;
  FakeWorld world;
  world.blocks.push_back({0.55, 0.0, 0.0, hw_task::BlockColor::Red, false, false});

  hw_task::StartRequest req;
  req.max_blocks = 1;

  const auto r = run_cycle(seq, veh, world, req, 0.55, 0.0, 0.0);

  std::printf("  终态=%s 放置=%u 跳过=%u 步数=%d 位置=(%.3f, %.3f, %.3f)\n",
              hw_task::to_string(r.final_state), r.placed, r.skipped, r.steps,
              r.end_x, r.end_y, r.end_yaw);

  check(r.final_state == hw_task::TaskState::Done, "任务应正常结束");
  check(r.placed == 1, "应放置 1 个物块");
  check(r.skipped == 0, "不应有跳过");

  // 结束位置应靠近投放槽位 0：(0.0, 1.20)，朝向 π/2
  check_near(r.end_x, 0.0, 0.12, "结束后 x 应落在投放槽位附近");
  check_near(r.end_y, 1.20, 0.12, "结束后 y 应落在投放槽位附近");
  check_near(r.end_yaw, 1.5707963267948966, 0.15, "结束后朝向应对上投放朝向");
}

void test_side_block()
{
  section("D. 侧前方物块（考验转向收敛）");

  hw_task::TaskSequencer seq;
  seq.configure(make_task_config());

  VehicleSim veh;
  FakeWorld world;
  // 方位角 atan2(0.28, 0.50) ≈ 29°，在 grasp_bearing_max 内
  world.blocks.push_back({0.50, 0.28, 0.0, hw_task::BlockColor::Blue, false, false});

  hw_task::StartRequest req;
  req.max_blocks = 1;

  const auto r = run_cycle(seq, veh, world, req, 0.50, 0.28, 0.0);

  std::printf("  终态=%s 放置=%u 跳过=%u 步数=%d 位置=(%.3f, %.3f)\n",
              hw_task::to_string(r.final_state), r.placed, r.skipped, r.steps,
              r.end_x, r.end_y);

  check(r.final_state == hw_task::TaskState::Done, "侧前方物块也应能完成");
  check(r.placed == 1, "应放置 1 个物块");
}

void test_grasp_failure_skips()
{
  section("E. 吸不住时跳过并最终失败（不卡死）");

  hw_task::TaskSequencer seq;
  auto cfg = make_task_config();
  cfg.max_grasp_retry = 2;
  seq.configure(cfg);

  VehicleSim veh;
  FakeWorld world;
  world.vacuum_capable = false;      // 永远吸不住
  world.blocks.push_back({0.55, 0.0, 0.0, hw_task::BlockColor::Red, false, false});

  hw_task::StartRequest req;
  req.max_blocks = 1;

  const auto r = run_cycle(seq, veh, world, req, 0.55, 0.0, 0.0);

  std::printf("  终态=%s 放置=%u 跳过=%u 步数=%d\n",
              hw_task::to_string(r.final_state), r.placed, r.skipped, r.steps);

  // 关键：必须走到终态，不能永远卡在重试循环里
  check(r.final_state == hw_task::TaskState::Failed,
        "一直吸不住时应以 FAILED 结束（而不是永远重试）");
  check(r.placed == 0, "一个都不该放置");
  check(r.skipped > 0, "应有跳过计数");
  check(r.steps < 7900, "不应该在重试里打转太久");
}

void test_estop()
{
  section("F. 急停中断");

  hw_task::TaskSequencer seq;
  seq.configure(make_task_config());

  VehicleSim veh;
  FakeWorld world;
  world.blocks.push_back({0.55, 0.0, 0.0, hw_task::BlockColor::Red, false, false});

  hw_task::StartRequest req;
  req.max_blocks = 1;
  seq.start(req, make_snapshot(veh, world, false, nullptr));

  // 推进到 ALIGN 附近，然后急停
  bool reached_align = false;
  for (int i = 0; i < 2000 && !reached_align; ++i) {
    seq.bind_target(1, Eigen::Vector2d(0.55, 0.0), hw_task::BlockColor::Red, 0.0, 1.0f);
    hw_task::SensorSnapshot s = make_snapshot(veh, world, true, &world.blocks[0]);
    const auto a = seq.update(s, 0.02);
    veh.step(a.cmd_vx, a.cmd_wz, 0.02);
    world.pump_run = a.pump_run;
    world.pump_step(0.02);
    if (seq.state() == hw_task::TaskState::Align ||
        seq.state() == hw_task::TaskState::Settle) {
      reached_align = true;
    }
  }
  check(reached_align, "应能推进到 ALIGN/SETTLE");

  hw_task::SensorSnapshot s = make_snapshot(veh, world, true, &world.blocks[0]);
  s.estop = true;
  const auto a = seq.update(s, 0.02);

  check(seq.state() == hw_task::TaskState::Failed, "急停后应进入 FAILED");
  check(!a.pump_run, "急停后气泵应停止");
  check(a.cmd_vx == 0.0 && a.cmd_wz == 0.0, "急停后底盘指令应为零");
}

void test_abort_return_home()
{
  section("G. 中止并返回起点");

  hw_task::TaskSequencer seq;
  seq.configure(make_task_config());

  VehicleSim veh;
  FakeWorld world;
  world.blocks.push_back({0.55, 0.0, 0.0, hw_task::BlockColor::Red, false, false});

  hw_task::StartRequest req;
  req.max_blocks = 5;
  seq.start(req, make_snapshot(veh, world, false, nullptr));

  for (int i = 0; i < 300; ++i) {
    seq.bind_target(1, Eigen::Vector2d(0.55, 0.0), hw_task::BlockColor::Red, 0.0, 1.0f);
    hw_task::SensorSnapshot s = make_snapshot(veh, world, true, &world.blocks[0]);
    const auto a = seq.update(s, 0.02);
    veh.step(a.cmd_vx, a.cmd_wz, 0.02);
    world.pump_step(0.02);
  }
  const double moved = std::hypot(veh.x, veh.y);
  check(moved > 0.10, "中止前应该确实移动过（否则测试没有意义）");

  seq.abort(true, true, make_snapshot(veh, world, true, &world.blocks[0]));

  // 跑到终态
  bool done = false;
  for (int i = 0; i < 3000 && !done; ++i) {
    hw_task::SensorSnapshot s = make_snapshot(veh, world, true, &world.blocks[0]);
    const auto a = seq.update(s, 0.02);
    veh.step(a.cmd_vx, a.cmd_wz, 0.02);
    world.pump_step(0.02);
    if (a.aborted || seq.state() == hw_task::TaskState::Failed) { done = true; }
  }

  std::printf("  终态=%s 回到 (%.3f, %.3f)\n",
              hw_task::to_string(seq.state()), veh.x, veh.y);

  check(seq.state() == hw_task::TaskState::Failed, "中止后应进入 FAILED");
  // 关键：中止+回起点之后**不能**继续扫描找物块（这是修过的一个 bug）
  check_near(veh.x, 0.0, 0.15, "应回到起点 x");
  check_near(veh.y, 0.0, 0.15, "应回到起点 y");
}

void test_payload_drop()
{
  section("H. 搬运途中掉件");

  /* 这个用例要模拟的是"吸住了 -> 运到半路掉了"，不是"从头就吸不住"
   * （后者是 C 用例的吸不住场景）。所以先用正常的假世界跑到 HAUL，
   * 再从那一刻起把 drop_after_grasp 打开，压力随即跌回大气压。 */
  hw_task::TaskSequencer seq;
  seq.configure(make_task_config());

  VehicleSim veh;
  FakeWorld world;
  world.blocks.push_back({0.55, 0.0, 0.0, hw_task::BlockColor::Red, false, false});

  hw_task::StartRequest req;
  req.max_blocks = 1;
  seq.start(req, make_snapshot(veh, world, false, nullptr));

  const double dt = 0.02;
  bool saw_haul = false;
  bool saw_drop_reaction = false;
  hw_task::TaskState final_state = hw_task::TaskState::Idle;

  for (int i = 0; i < 8000; ++i) {
    FakeWorld::Block * tgt = nullptr;
    for (auto & b : world.blocks) {
      if (!b.removed && !b.on_vehicle) { tgt = &b; break; }
    }
    const auto st = seq.state();
    const bool need = (st == hw_task::TaskState::Scan) ||
                      (st == hw_task::TaskState::Approach) ||
                      (st == hw_task::TaskState::Align);
    if (need && tgt) {
      seq.bind_target(1, Eigen::Vector2d(tgt->x, tgt->y), tgt->color, tgt->yaw, 1.0f);
    }

    hw_task::SensorSnapshot s = make_snapshot(veh, world, tgt != nullptr, tgt);
    const auto a = seq.update(s, dt);

    veh.step(a.cmd_vx, a.cmd_wz, dt);
    world.pump_run = a.pump_run;
    world.pump_duty = a.pump_duty;

    // 一旦进入 HAUL 就打开"掉件"，模拟吸盘在颠簸中泄压
    if (seq.state() == hw_task::TaskState::Haul && !saw_haul) {
      saw_haul = true;
      world.drop_after_grasp = true;
    }
    world.pump_step(dt);

    if (tgt && !tgt->on_vehicle && world.pressure < -10.0 && a.pump_run) {
      tgt->on_vehicle = true;
    }

    if (saw_haul && seq.status().skipped_count > 0) {
      saw_drop_reaction = true;
    }
    if (a.finished || a.aborted) { final_state = seq.state(); break; }
    final_state = seq.state();
  }

  std::printf("  终态=%s 放置=%u 跳过=%u 进入过HAUL=%s\n",
              hw_task::to_string(final_state), seq.status().grasped_count,
              seq.status().skipped_count, saw_haul ? "是" : "否");

  check(saw_haul, "应该先成功吸住并进入搬运阶段（否则这个用例没测到东西）");
  check(saw_drop_reaction, "掉件后应触发跳过处理");
  check(seq.status().grasped_count == 0, "掉件了就不该计入放置成功");
  check(final_state != hw_task::TaskState::Idle, "应走到终态而不是卡住");
}

void test_pause_resume()
{
  section("I. 暂停与恢复（计时补偿）");

  hw_task::TaskSequencer seq;
  seq.configure(make_task_config());

  VehicleSim veh;
  FakeWorld world;
  world.blocks.push_back({0.55, 0.0, 0.0, hw_task::BlockColor::Red, false, false});

  hw_task::StartRequest req;
  req.max_blocks = 1;
  seq.start(req, make_snapshot(veh, world, false, nullptr));

  // 推进到 APPROACH/ALIGN。60 步 = 1.2 秒：物块在正前方 0.55m，
  // 按 0.35 m/s 大约 1 秒走完接近段，此时正好在 APPROACH 或 ALIGN。
  // 原来用 200 步（4 秒）会一路跑到 HAUL，测不到想测的东西。
  for (int i = 0; i < 60; ++i) {
    seq.bind_target(1, Eigen::Vector2d(0.55, 0.0), hw_task::BlockColor::Red, 0.0, 1.0f);
    hw_task::SensorSnapshot s = make_snapshot(veh, world, true, &world.blocks[0]);
    const auto a = seq.update(s, 0.02);
    veh.step(a.cmd_vx, a.cmd_wz, 0.02);
    world.pump_step(0.02);
  }

  const auto before = seq.state();
  check(before == hw_task::TaskState::Approach ||
        before == hw_task::TaskState::Align, "应处于接近/对准阶段");

  check(seq.pause(make_snapshot(veh, world, true, &world.blocks[0])), "暂停应成功");
  check(seq.state() == hw_task::TaskState::Paused, "状态应为 PAUSED");

  // 暂停期间推进 10 秒（模拟操作员去处理别的事）
  for (int i = 0; i < 500; ++i) {
    veh.t += 0.02;
    hw_task::SensorSnapshot s = make_snapshot(veh, world, true, &world.blocks[0]);
    const auto a = seq.update(s, 0.02);
    check(a.cmd_vx == 0.0 && a.cmd_wz == 0.0, "暂停期间底盘不应动");
    break;   // 只检查一次就够，避免刷屏
  }
  veh.t += 10.0;

  check(seq.resume(make_snapshot(veh, world, true, &world.blocks[0])), "恢复应成功");
  check(seq.state() == before, "恢复后应回到原来的状态");

  // 恢复后不能立刻超时。原实现里 t_state_ 被重置成当前时间之后再减去它自己，
  // 等于没补偿，恢复的瞬间就会因为"状态已持续 10 秒"而超时。
  hw_task::SensorSnapshot s2 = make_snapshot(veh, world, true, &world.blocks[0]);
  const auto a2 = seq.update(s2, 0.02);
  (void)a2;
  check(seq.state() == before,
        "恢复后的第一个周期不应立刻超时跳走（实际：" +
        std::string(hw_task::to_string(seq.state())) + "）");
}

}  // namespace

int main()
{
  std::printf("\n\033[1mhw_task 纯逻辑层测试\033[0m\n");
  std::printf("（假的是传感器与底盘，状态机跑的是与真车完全相同的那份代码）\n");

  test_ground_projection();
  test_block_tracker();
  test_nominal_cycle();
  test_side_block();
  test_grasp_failure_skips();
  test_estop();
  test_abort_return_home();
  test_payload_drop();
  test_pause_resume();

  std::printf("\n\033[1m结果：%d 项检查，%d 项失败\033[0m\n\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
