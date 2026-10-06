// ============================================================================
//  test_sequencer.cpp
//  hw_task 纯逻辑层的主机侧回归测试。
//
//  编译运行（不需要 ROS）：
//      test/run_host_test.sh
//
//  这个测试用 **arm_control 的真实运动学** 做仿真：状态机下达的是末端目标位姿，
//  仿真器用同一份 D-H 表与闭式逆解把它变成关节角，再按关节限速走过去，
//  然后用正解算出"实测"末端位姿回喂给状态机。
//  也就是简历里说的"虚实联调" —— 控制器和仿真在同一个闭环里，而不是各写一套。
//
//  真机上一次完整取放要几十秒，失败了很难复现；这里可以把全部状态、
//  全部失败分支跑几百遍。真机上还要核对的是：D-H 表的长度、关节零位、
//  丝杠导程、相机内参与安装位姿。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "arm_control/arm_model.hpp"
#include "arm_control/dh.hpp"
#include "arm_control/scara_ik.hpp"
#include "arm_control/trajectory.hpp"

#include "hw_task/block_tracker.hpp"
#include "hw_task/table_projection.hpp"
#include "hw_task/task_sequencer.hpp"

// 运动学的关节索引常量在 arm_control 命名空间里；仿真器里用得频繁，
// 逐个限定会淹没断言本身的可读性。
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
    std::printf("  [ok]   %s (%.6g)\n", what.c_str(), got);
  } else {
    ++g_fail;
    std::printf("  [FAIL] %s  期望 %.6g 实际 %.6g 差 %.3g  (%s)\n",
                what.c_str(), want, got, std::fabs(got - want), g_section);
  }
}

// ===========================================================================
//  相机模型（理想针孔 + 固定安装在工作台上方）
//
//  相机在基座系 (0, 0.50, cam_h) 处垂直向下看。图像 x 向右 = 基座 +x。
//
//  图像 y 的方向必须想清楚：相机光学系是右手系（z 沿光轴向前），而光轴朝下，
//  也就是 z_cam = -z_base。若再要求 x_cam = +x_base，那么
//      z_cam = x_cam × y_cam  ->  -z_base = x_base × y_cam  ->  y_cam = -y_base
//  所以图像里"向下"对应基座的 **-y** 方向。写成矩阵是绕基座 x 轴转 180°：
//      R = diag(1, -1, -1)     行列式 +1，是合法旋转
//  这里有个很容易踩的坑：如果图省事把外参写成 diag(1, 1, -1)，行列式是 -1，
//  它**不是旋转**，而是一个镜像。自己生成像素、自己投影，往返照样对得上，
//  测试会全绿 —— 但 ROS 里的 TF 必须来自四元数，根本表达不出这种矩阵，
//  换成真相机时整套投影会静默地左右镜像。
//
//  于是理想投影是解析的：
//      u = cx + fx * x / cam_h
//      v = cy - fy * (y - 0.50) / cam_h
//  用这个模型生成像素、再用 TableProjection 投回去，可以完整验证
//  "内参 + 外参 + 台面求交"这条链路 —— 而且不依赖任何标定数据。
// ===========================================================================
constexpr double kCamH = 0.600;      // 相机离台面 600mm，俯视整个工作区
constexpr double kCamY = 0.500;      // 光轴在基座系 y 方向的偏移

Eigen::Matrix4d camera_to_base()
{
  /* 相机光学系 -> 基座系：绕基座 x 轴转 180° 再平移。
   *   基座 x =  相机 x
   *   基座 y = -相机 y + 偏移
   *   基座 z = -相机 z + cam_h
   * 对应 URDF 里 camera_link 在 (0, 0.5, 0.6)、camera_optical_joint
   * 的 rpy = (π, 0, 0)。 */
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(1, 1) = -1.0;
  T(2, 2) = -1.0;
  T(1, 3) = kCamY;
  T(2, 3) = kCamH;
  return T;
}

hw_task::CameraIntrinsics make_intrinsics()
{
  hw_task::CameraIntrinsics K;
  K.fx = 600.0;
  K.fy = 600.0;
  K.cx = 320.0;
  K.cy = 240.0;
  K.width = 640;
  K.height = 480;
  K.k1 = 0.0;
  K.k2 = 0.0;
  return K;
}

/// 世界坐标 -> 理想像素坐标
void world_to_pixel(double x, double y, double * u, double * v)
{
  const hw_task::CameraIntrinsics K = make_intrinsics();
  *u = K.cx + K.fx * x / kCamH;
  *v = K.cy - K.fy * (y - kCamY) / kCamH;   // 注意负号，见上面的推导
}

// ===========================================================================
//  机械臂仿真器：真实 D-H 正逆解 + 关节限速
// ===========================================================================
class ArmSim
{
public:
  ArmSim()
  {
    // 起始姿态取 home 位的逆解，避免"开机就停在工作区中间挡住相机"
    arm_control::IkSolution sol;
    if (arm_control::scara_inverse(geo_, lim_, 0.28, 0.0, 0.100, 0.0, nullptr, &sol)) {
      for (int i = 0; i < kJointCount; ++i) { q_[i] = sol.q[i]; cmd_[i] = sol.q[i]; }
    }
  }

  /// 接受状态机下达的新目标
  void command(const hw_task::Actuation & a)
  {
    if (!a.set_pose) { return; }
    arm_control::IkSolution sol;
    if (!arm_control::scara_inverse(geo_, lim_, a.pose_x, a.pose_y, a.pose_z,
                                    a.pose_yaw, q_, &sol)) {
      /* 逆解失败 —— 真实系统里表现为 arm_control 拒绝目标并在 /arm/status
       * 上置 ik_ok=false。仿真器把这件事记下来，由 sample() 传给状态机。 */
      ik_failed_ = true;
      ++rejected_goals_;
      return;
    }
    ik_failed_ = false;
    for (int i = 0; i < kJointCount; ++i) { cmd_[i] = sol.q[i]; }
  }

  /// 按关节限速把关节值推向目标
  void step(double dt)
  {
    const arm_control::TrajectoryLimits tl;
    for (int i = 0; i < kJointCount; ++i) {
      /* 仿真里只用 60% 的关节限速。真实系统里 arm_control 走的是五次
       * 多项式（起停处速度为零），不会一直贴着限速跑；用满限速会让
       * 仿真比真实系统"灵活"，掩盖掉超时类的问题。 */
      const double v = tl.vmax[i] * 0.6;
      const double d = cmd_[i] - q_[i];
      const double step_max = v * dt;
      if (std::fabs(d) <= step_max) {
        q_[i] = cmd_[i];
      } else {
        q_[i] += (d > 0.0 ? step_max : -step_max);
      }
    }
  }

  /// 正解 + 有限差分求末端速度
  void sample(double dt, hw_task::SensorSnapshot & s)
  {
    const arm_control::ToolPose p = arm_control::forward_tool_pose(geo_, q_);
    if (have_prev_ && dt > 0.0) {
      s.tool_vx = (p.x - prev_.x) / dt;
      s.tool_vy = (p.y - prev_.y) / dt;
      s.tool_vz = (p.z - prev_.z) / dt;
    }
    prev_ = p;
    have_prev_ = true;
    s.tool_x = p.x;
    s.tool_y = p.y;
    s.tool_z = p.z;
    s.ik_ok = !ik_failed_;
  }

  uint32_t rejected_goals() const { return rejected_goals_; }

private:
  arm_control::ArmGeometry geo_{};
  arm_control::JointLimits lim_{};
  double q_[kJointCount]{0.0, 0.0, 0.0, 0.0};
  double cmd_[kJointCount]{0.0, 0.0, 0.0, 0.0};
  arm_control::ToolPose prev_{};
  bool   have_prev_{false};
  bool   ik_failed_{false};
  uint32_t rejected_goals_{0};
};

// ===========================================================================
//  假世界：物块、气泵、真空压力
// ===========================================================================
struct FakeBlock
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  hw_task::BlockColor color{hw_task::BlockColor::Red};
  double area_px{5000.0};
};

class FakeWorld
{
public:
  std::vector<FakeBlock> blocks;

  // ---- 气泵 ----
  bool   pump_dead{false};        ///< 泵坏了：抽不出真空（模拟吸不住）
  double pressure{0.0};           ///< kPa，表压
  double target_vacuum{-30.0};    ///< 泵能建立的最大真空度
  double rise_tau{0.35};          ///< 抽气时间常数
  double fall_tau{0.25};          ///< 泄气时间常数

  /// 抬起超过 drop_trigger_z 就漏气（模拟吸盘在颠簸中泄压）
  bool   drops_on_lift{false};
  double drop_trigger_z{0.075};
  /// 密封一旦破坏就锁存：真空再也建立不起来
  bool   seal_broken{false};

  /// 视觉位置噪声幅度（m）
  double noise{0.0};

  int attached{-1};               ///< 被吸住的物块下标

  void apply(const hw_task::Actuation & a)
  {
    pump_on_ = a.pump_run && a.pump_duty > 0.001;
  }

  void step(double dt, double tool_x, double tool_y, double tool_z)
  {
    /* 密封失效：抬起时如果吸盘漏气，就锁存"吸盘坏了"这个事实。
     * 必须锁存而不是"只在高处漏"—— 否则末端一降回物块上，压力又恢复成
     * 正常真空，仿真就变成了"掉了又自己吸回来"，测不出掉件处理逻辑。
     * 现实里也确实如此：密封面一旦被破坏（吸进灰尘、物块表面不平），
     * 在原位重新抽气通常也抽不起来了。 */
    if (drops_on_lift && attached >= 0 && tool_z > drop_trigger_z) {
      seal_broken = true;
    }

    // ---- 压力一阶动态 ----
    const bool pumping = pump_on_ && !pump_dead && !seal_broken;
    const double goal = pumping ? target_vacuum : 0.0;
    const double tau = pumping ? rise_tau : 0.05;
    pressure += (goal - pressure) * std::min(1.0, dt / tau);

    // ---- 吸附判定 ----
    const bool vacuum = std::fabs(pressure) > 5.0;
    if (attached < 0) {
      if (vacuum) {
        for (std::size_t i = 0; i < blocks.size(); ++i) {
          const double d = std::hypot(blocks[i].x - tool_x, blocks[i].y - tool_y);
          // 吸盘中心与物块中心的偏差在 20mm 内才算吸上（圆吸盘的实际接触面）
          if (d < 0.020) { attached = static_cast<int>(i); break; }
        }
      }
    } else if (!vacuum) {
      attached = -1;   // 泄气或漏气 -> 松脱，物块留在原地
    }

    // ---- 被吸住的物块跟着末端走 ----
    if (attached >= 0) {
      blocks[static_cast<std::size_t>(attached)].x = tool_x;
      blocks[static_cast<std::size_t>(attached)].y = tool_y;
    }
  }

  /// 生成一帧观测
  std::vector<hw_task::BlockObservation> observe() const
  {
    std::vector<hw_task::BlockObservation> out;
    for (std::size_t i = 0; i < blocks.size(); ++i) {
      /* 被吸在吸盘上的物块不参与视觉：相机当然看得见它，但投影链路按
       * "物块在台面上"求交，会给出一个完全错误的位置（它在空中）。
       * 真实系统里这是个已知干扰源，处理办法是任务层在搬运阶段不重新挑
       * 目标 —— 由 TaskSequencer 的状态机保证（只有 SCAN/APPROACH/ALIGN
       * 会重新绑目标）。 */
      if (static_cast<int>(i) == attached) { continue; }

      const double dx = (noise > 0.0)
        ? (noise * std::sin(static_cast<double>(i) * 7.3 + t_)) : 0.0;
      const double dy = (noise > 0.0)
        ? (noise * std::cos(static_cast<double>(i) * 3.1 + t_)) : 0.0;

      hw_task::BlockObservation o;
      o.pos = Eigen::Vector2d(blocks[i].x + dx, blocks[i].y + dy);
      o.color = blocks[i].color;
      o.yaw = blocks[i].yaw;
      o.area_px = blocks[i].area_px;
      o.confidence_px = 0.8;
      double pu = 0.0, pv = 0.0;
      world_to_pixel(blocks[i].x, blocks[i].y, &pu, &pv);
      o.pixel_x = static_cast<float>(pu);
      o.pixel_y = static_cast<float>(pv);
      out.push_back(o);
    }
    return out;
  }

  void advance_time(double dt) { t_ += dt; }

private:
  bool   pump_on_{false};
  double t_{0.0};
};

// ===========================================================================
//  测试台：把状态机接上仿真器
// ===========================================================================
struct RunResult
{
  hw_task::TaskState final_state{hw_task::TaskState::Idle};
  uint32_t placed{0};
  uint32_t skipped{0};
  int      steps{0};
  bool     finished{false};
  bool     aborted{false};
  uint8_t  max_layer{0};
  double   final_t{0.0};
  std::string last_message;
};

struct Harness
{
  hw_task::TaskConfig    cfg;
  hw_task::TaskSequencer seq;
  hw_task::BlockTracker  tracker;
  ArmSim    sim;
  FakeWorld world;

  double dt{0.02};                 ///< 50 Hz，与 task.yaml 的 control_rate_hz 一致

  // ---- 可注入的故障 ----
  bool vision_down{false};
  bool arm_faulted{false};
  bool estop{false};

  Harness()
  {
    cfg.place_x = 0.28;
    cfg.place_y = -0.14;
    cfg.place_yaw = 0.0;

    hw_task::TrackerConfig tc;
    tc.match_radius = 0.040;
    tc.min_stable_frames = cfg.min_stable_frames;
    tracker.configure(tc);
    seq.configure(cfg);
  }

  hw_task::SensorSnapshot snapshot(double t)
  {
    hw_task::SensorSnapshot s;
    s.t = t;
    sim.sample(dt, s);

    s.vision_ok = vision_ok_;
    s.blocks_visible = visible_;

    s.pump_online = true;
    s.pump_pressure_kpa = world.pressure;
    s.pump_fault = 0;

    s.arm_faulted = arm_faulted;
    s.estop = estop;

    if (have_bound_) {
      s.target_valid = true;
      s.target_x = bound_x;
      s.target_y = bound_y;
      s.target_yaw = bound_yaw;
      s.target_color = bound_color;
      s.target_confidence = bound_conf;
    }
    return s;
  }

  void feed_vision(double t)
  {
    const auto obs = world.observe();
    visible_ = static_cast<uint32_t>(obs.size());
    vision_ok_ = true;
    if (vision_down) { vision_ok_ = false; return; }   // 模拟相机掉线
    tracker.update(obs, t);
  }

  /// 与 task_executor_node 的 pick_target 一致：只在需要目标的状态挑
  void pick()
  {
    const hw_task::TaskState st = seq.state();
    const bool need = (st == hw_task::TaskState::Scan) ||
                      (st == hw_task::TaskState::Approach) ||
                      (st == hw_task::TaskState::Align);
    if (!need || !vision_ok_) { return; }

    const auto excluded = seq.excluded_ids();
    const hw_task::BlockTrack best =
      tracker.pick(cfg, Eigen::Vector2d(0.0, 0.0), excluded);
    if (!best.stable) { return; }

    have_bound_ = true;
    bound_x = best.pos.x();
    bound_y = best.pos.y();
    bound_yaw = best.yaw;
    bound_color = best.color;
    bound_conf = static_cast<float>(best.confidence);
    seq.bind_target(best.id, best.pos.x(), best.pos.y(), best.color, best.yaw,
                    static_cast<float>(best.confidence));
  }

  /// 推进一个控制周期，返回本周期状态机给出的动作
  hw_task::Actuation tick(double & t)
  {
    world.advance_time(dt);
    feed_vision(t);
    pick();

    const hw_task::SensorSnapshot s = snapshot(t);
    const hw_task::Actuation a = seq.update(s, dt);

    sim.command(a);
    world.apply(a);
    sim.step(dt);
    world.step(dt, s.tool_x, s.tool_y, s.tool_z);

    t += dt;
    return a;
  }

  RunResult run(const hw_task::StartRequest & req, int max_steps = 6000)
  {
    double t = 0.0;
    seq.start(req, snapshot(t));

    RunResult r;
    for (int i = 0; i < max_steps; ++i) {
      const hw_task::Actuation a = tick(t);
      ++r.steps;
      r.max_layer = std::max<uint8_t>(r.max_layer, seq.status().place_layer);
      r.last_message = seq.status().message;
      if (a.finished || a.aborted) {
        r.finished = a.finished;
        r.aborted = a.aborted;
        break;
      }
    }
    r.final_state = seq.state();
    r.placed = seq.status().grasped_count;
    r.skipped = seq.status().skipped_count;
    r.final_t = t;
    return r;
  }

  void add_block(double x, double y, hw_task::BlockColor c, double yaw = 0.0)
  {
    FakeBlock b;
    b.x = x; b.y = y; b.color = c; b.yaw = yaw;
    world.blocks.push_back(b);
  }

private:
  bool     vision_ok_{true};
  uint32_t visible_{0};
  bool     have_bound_{false};
  double   bound_x{0.0}, bound_y{0.0}, bound_yaw{0.0};
  hw_task::BlockColor bound_color{hw_task::BlockColor::Red};
  float    bound_conf{0.0f};
};

const char * st_name(hw_task::TaskState s) { return hw_task::to_string(s); }

// ===========================================================================
//  A. 工作台投影
// ===========================================================================
void test_projection()
{
  section("A. 工作台投影（像素 -> 基座系台面坐标）");

  hw_task::TableProjection tp;
  tp.set_intrinsics(make_intrinsics());
  const Eigen::Matrix4d T = camera_to_base();

  double worst = 0.0;
  int valid = 0;
  for (double x = -0.30; x <= 0.30; x += 0.05) {
    for (double y = 0.20; y <= 0.80; y += 0.05) {
      double u = 0.0, v = 0.0;
      world_to_pixel(x, y, &u, &v);
      const hw_task::TablePoint p = tp.project(u, v, T);
      if (!p.valid) { continue; }
      ++valid;
      worst = std::max(worst, std::hypot(p.x - x, p.y - y));
    }
  }
  check(valid > 100, "工作区采样点全部可投影（" + std::to_string(valid) + " 个）");

  /* 外参必须是**合法旋转**（行列式 +1）。
   * 手算而不是调 Eigen 的 determinant：主机侧用的是极简垫片，没有那个成员；
   * 而这条断言恰恰是"相机俯视时图像 y 轴必须反向"的直接体现，值得写清楚。 */
  const double det_rc =
      T(0, 0) * (T(1, 1) * T(2, 2) - T(1, 2) * T(2, 1))
    - T(0, 1) * (T(1, 0) * T(2, 2) - T(1, 2) * T(2, 0))
    + T(0, 2) * (T(1, 0) * T(2, 1) - T(1, 1) * T(2, 0));
  check_near(det_rc, 1.0, 1e-12,
             "相机外参是合法旋转（行列式 +1，能被四元数表达）");
  check(worst < 1e-9, "像素->台面 与 台面->像素 解析往返一致（最大误差 " +
                      std::to_string(worst) + " m）");

  // ---- 标定：点铺得够开 -> 完整 2D 仿射 ----
  {
    hw_task::TableProjection g;
    g.set_intrinsics(make_intrinsics());
    const double ox = 0.012, oy = -0.008;   // 模拟"相机重新夹装后偏了 1cm"

    const double px[3][2] = {{60.0, 60.0}, {580.0, 60.0}, {320.0, 420.0}};
    double last_err = -1.0;
    for (const auto & c : px) {
      /* 真实世界坐标用**解析**理想投影算，不能拿 g.project() 去求 ——
       * 那上面已经叠了残差，第二次调用开始输出就是"已校正"的了，
       * 再加一次偏移会变成 2×offset（第一版正是这么错的，表现为
       * "标定之后误差反而翻倍"，看着像投影算法有 bug）。 */
      const double wx = (c[0] - 320.0) / 600.0 * kCamH + ox;
      const double wy = kCamY - (c[1] - 240.0) / 600.0 * kCamH + oy;
      last_err = g.add_calibration_point(c[0], c[1], wx, wy, T);
    }
    check(g.affine_solved(), "标定点铺开时解出完整 2D 仿射");
    check(last_err >= 0.0 && last_err < 1e-6,
          "残差降到 1e-6 m 以下（实际 " + std::to_string(last_err) + "）");
    const hw_task::TablePoint p = g.project(320.0, 240.0, T);
    check_near(p.x, ox, 1e-3, "校正后带上 x 向平移量");
    check_near(p.y, kCamY + oy, 1e-3, "校正后带上 y 向平移量");
  }

  // ---- 标定点挤在一起 -> 必须退化成只解平移 ----
  /* 三个点挤在 1cm 内时，完整 2D 仿射是**病态问题**：设计矩阵近似共线，
   * 正规方程把条件数平方，双精度也压不住 —— 会解出一个线性部分乱飞、
   * 靠平移去凑的矩阵。它在标定点附近很准，出了那个小区域完全失效，
   * 但看起来"标定成功了"。所以必须有一道安全阀。 */
  {
    hw_task::TableProjection g;
    g.set_intrinsics(make_intrinsics());
    const double ox = 0.02, oy = 0.015;
    const double px[3][2] = {{318.0, 239.0}, {322.0, 241.0}, {320.0, 240.0}};
    for (const auto & c : px) {
      const double wx = (c[0] - 320.0) / 600.0 * kCamH + ox;
      const double wy = kCamY - (c[1] - 240.0) / 600.0 * kCamH + oy;
      g.add_calibration_point(c[0], c[1], wx, wy, T);
    }
    check(!g.affine_solved(), "标定点太集中时不硬解完整仿射");
    check(g.calibration_spread() < hw_task::TableProjection::kMinCalibSpread,
          "跨度判定识别出点太集中");
    const Eigen::Matrix3d r = g.residual_matrix();
    check_near(r(0, 0), 1.0, 1e-9, "退化情况下线性部分保持单位阵");
    check_near(r(0, 1), 0.0, 1e-9, "退化情况下交叉项为 0");
    const hw_task::TablePoint p = g.project(320.0, 240.0, T);
    check_near(p.x, ox, 3e-3, "退化情况下平移仍被校正");
  }
}

// ===========================================================================
//  B. 数据关联
// ===========================================================================
void test_tracker()
{
  section("B. 视觉数据关联");

  hw_task::BlockTracker tk;
  hw_task::TaskConfig cfg;
  hw_task::TrackerConfig tc;
  tc.min_stable_frames = 3;
  tk.configure(tc);

  auto obs = [](double x, double y, hw_task::BlockColor c, double area) {
      hw_task::BlockObservation o;
      o.pos = Eigen::Vector2d(x, y);
      o.color = c;
      o.area_px = area;      // 面积必须给：pick() 会用它过滤
      o.confidence_px = 0.8;
      return o;
    };

  for (int i = 0; i < 3; ++i) {
    tk.update({obs(0.25, 0.10, hw_task::BlockColor::Red, 5000.0)}, 0.1 * i);
  }
  check(tk.stable_tracks().size() == 1, "连续 3 帧同类观测形成一条稳定轨迹");
  check(tk.pick(cfg, Eigen::Vector2d(0, 0), {}).stable, "默认面积下可以选出目标");

  {
    hw_task::BlockTracker small;
    small.configure(tc);
    for (int i = 0; i < 3; ++i) {
      small.update({obs(0.25, 0.10, hw_task::BlockColor::Red, 100.0)}, 0.1 * i);
    }
    check(!small.pick(cfg, Eigen::Vector2d(0, 0), {}).stable,
          "面积小于 min_area_px 的观测不会被选为目标");
  }

  {
    hw_task::BlockTracker far;
    far.configure(tc);
    for (int i = 0; i < 3; ++i) {
      far.update({obs(0.55, 0.0, hw_task::BlockColor::Red, 5000.0)}, 0.1 * i);
    }
    check(!far.pick(cfg, Eigen::Vector2d(0, 0), {}).stable,
          "超出工作半径上限的物块被排除");

    hw_task::BlockTracker near;
    near.configure(tc);
    for (int i = 0; i < 3; ++i) {
      near.update({obs(0.04, 0.0, hw_task::BlockColor::Red, 5000.0)}, 0.1 * i);
    }
    check(!near.pick(cfg, Eigen::Vector2d(0, 0), {}).stable,
          "落在基座内孔里的物块被排除");
  }

  {
    hw_task::BlockTracker multi;
    multi.configure(tc);
    std::vector<hw_task::BlockObservation> frame;
    frame.push_back(obs(0.34, 0.0, hw_task::BlockColor::Blue, 5000.0));
    frame.push_back(obs(0.22, 0.0, hw_task::BlockColor::Red, 5000.0));
    frame.push_back(obs(0.30, 0.0, hw_task::BlockColor::Red, 5000.0));
    for (int i = 0; i < 3; ++i) { multi.update(frame, 0.1 * i); }

    const auto ranked = multi.rank_targets({hw_task::BlockColor::Red},
                                           Eigen::Vector2d(0, 0), {});
    check(ranked.size() >= 2, "排序列出多个可用目标");
    if (ranked.size() >= 2) {
      const bool same = (ranked[0].color == hw_task::BlockColor::Red) &&
                        (ranked[1].color == hw_task::BlockColor::Red);
      check(same, "指定颜色优先时先排同色的目标");
      check((ranked[0].pos - Eigen::Vector2d(0, 0)).norm() <
            (ranked[1].pos - Eigen::Vector2d(0, 0)).norm(),
            "同色内部按到基座的距离升序");
    }
  }
}

// ===========================================================================
//  C. 完整取放流程
// ===========================================================================
void test_full_cycle()
{
  section("C. 完整取放流程（1 个物块）");

  Harness h;
  h.add_block(0.26, 0.06, hw_task::BlockColor::Red, 0.3);

  hw_task::StartRequest req;
  req.max_blocks = 1;

  const RunResult r = h.run(req);

  std::printf("  终态=%s 放置=%u 跳过=%u 步数=%d 用时=%.1fs\n",
              st_name(r.final_state), r.placed, r.skipped, r.steps, r.final_t);
  std::printf("  末条消息: %s\n", r.last_message.c_str());

  check(r.final_state == hw_task::TaskState::Done, "任务正常结束");
  check(r.finished, "上报 finished=true");
  check(r.placed == 1, "放置计数为 1");
  check(r.skipped == 0, "没有跳过任何物块");
  check(r.max_layer == 0, "码垛模式下第一个物块放在第 0 层");
}

// ===========================================================================
//  D. 吸不住
// ===========================================================================
void test_grasp_failure()
{
  section("D. 吸不住（气路故障）");

  Harness h;
  h.world.pump_dead = true;          // 泵抽不出真空
  h.add_block(0.26, 0.05, hw_task::BlockColor::Blue);

  hw_task::StartRequest req;
  req.max_blocks = 1;

  const RunResult r = h.run(req, 9000);

  std::printf("  终态=%s 放置=%u 跳过=%u 步数=%d\n",
              st_name(r.final_state), r.placed, r.skipped, r.steps);
  std::printf("  末条消息: %s\n", r.last_message.c_str());

  check(r.placed == 0, "一个都没放上");
  check(r.skipped > 0, "吸不住的物块被计入跳过");
  check(r.aborted || r.finished, "走到了终态而不是卡在重试循环里");
  /* 关键回归：连续吸不住进了 FAILED，必须把 aborted 报给上层。
   * 曾经的写法是 a.aborted = true 之后 return stop_all(...)，
   * 而 stop_all 新建对象 —— 标志丢了，上层以为任务还在跑。 */
  check(r.aborted, "连续吸不住时把 aborted 上报给上层");
}

// ===========================================================================
//  E. 急停
// ===========================================================================
void test_estop()
{
  section("E. 急停");

  Harness h;
  h.add_block(0.26, 0.05, hw_task::BlockColor::Red);

  hw_task::StartRequest req;
  req.max_blocks = 1;
  double t = 0.0;
  h.seq.start(req, h.snapshot(t));

  for (int i = 0; i < 40; ++i) { h.tick(t); }   // 0.8s，此时应在接近/对准
  std::printf("  急停前状态=%s\n", st_name(h.seq.state()));

  h.estop = true;
  const hw_task::SensorSnapshot s = h.snapshot(t);
  const hw_task::Actuation a = h.seq.update(s, h.dt);

  check(h.seq.state() == hw_task::TaskState::Failed, "急停后进入 FAILED");
  check(!a.pump_run, "急停同时关掉气泵");
  check(!a.set_pose, "急停不再下达新的运动目标");
}

// ===========================================================================
//  F. 中止 + 回原点
// ===========================================================================
void test_abort_return_home()
{
  section("F. 中止后返回原点");

  Harness h;
  h.add_block(0.26, 0.05, hw_task::BlockColor::Red);
  h.add_block(0.34, -0.10, hw_task::BlockColor::Blue);

  hw_task::StartRequest req;
  req.max_blocks = 2;
  double t = 0.0;
  h.seq.start(req, h.snapshot(t));

  for (int i = 0; i < 40; ++i) { h.tick(t); }
  std::printf("  中止前状态=%s\n", st_name(h.seq.state()));

  const bool ok = h.seq.abort(/*release=*/true, /*return_home=*/true, h.snapshot(t));
  check(ok, "中止调用被接受");

  hw_task::Actuation a;
  bool aborted_reported = false;
  for (int i = 0; i < 2000; ++i) {
    a = h.tick(t);
    if (a.aborted) { aborted_reported = true; break; }
  }

  std::printf("  中止后状态=%s 消息=%s\n", st_name(h.seq.state()), a.note.c_str());
  check(h.seq.state() == hw_task::TaskState::Failed, "中止后进入 FAILED（不继续干活）");
  check(aborted_reported, "中止把 aborted 上报给上层");
  /* 最关键的一条：中止后不能自己继续下一个物块。
   * 曾经的实现借用 RETREAT 状态回原点，而 RETREAT 的职责是"退开后找下一个" ——
   * 结果按了中止，机械臂回到原点又接着抓。 */
  check(h.seq.status().grasped_count == 0, "中止后没有偷偷继续放物块");
}

// ===========================================================================
//  G. 搬运途中掉件
// ===========================================================================
void test_payload_drop()
{
  section("G. 搬运途中掉件");

  Harness h;
  h.add_block(0.26, 0.05, hw_task::BlockColor::Red);

  hw_task::StartRequest req;
  req.max_blocks = 1;
  double t = 0.0;
  h.seq.start(req, h.snapshot(t));

  bool saw_haul = false;
  bool saw_skip = false;
  hw_task::Actuation a;
  for (int i = 0; i < 9000; ++i) {
    a = h.tick(t);

    /* 一旦进入 HAUL 就把吸盘"弄漏气"。注意必须等到 HAUL 之后 ——
     * 从头就漏气测的是"吸不住"（D 用例），不是"搬着搬着掉了"。 */
    if (h.seq.state() == hw_task::TaskState::Haul && !saw_haul) {
      saw_haul = true;
      h.world.drops_on_lift = true;
      h.world.drop_trigger_z = 0.070;
    }
    if (h.seq.status().skipped_count > 0) { saw_skip = true; }

    if (a.finished || a.aborted) { break; }
  }

  std::printf("  进入过HAUL=%s 触发跳过=%s 终态=%s 放置=%u\n",
              saw_haul ? "是" : "否", saw_skip ? "是" : "否",
              st_name(h.seq.state()), h.seq.status().grasped_count);

  check(saw_haul, "先成功吸住并进入搬运阶段（否则这个用例什么也没测到）");
  check(saw_skip, "掉件后触发了跳过处理");
  check(h.seq.status().grasped_count == 0, "掉件了就不该计入放置成功");
  check(h.seq.state() != hw_task::TaskState::Idle, "走到了终态而不是卡住");
}

// ===========================================================================
//  H. 暂停 / 恢复
// ===========================================================================
void test_pause_resume()
{
  section("H. 暂停与恢复");

  Harness h;
  h.add_block(0.26, 0.05, hw_task::BlockColor::Red);

  hw_task::StartRequest req;
  req.max_blocks = 1;
  double t = 0.0;
  h.seq.start(req, h.snapshot(t));

  // 推进到 APPROACH/ALIGN。60 步 = 1.2s：物块在 0.26m 处，
  // 仿真关节限速 60% 下大约 1s 走完接近段。
  // 原先把这一步写成 200 步（4 秒），早就跑到 HAUL 去了，测不到想测的东西。
  for (int i = 0; i < 60; ++i) { h.tick(t); }
  const hw_task::TaskState before = h.seq.state();
  std::printf("  暂停前状态=%s\n", st_name(before));

  check(h.seq.pause(h.snapshot(t)), "暂停被接受");
  check(h.seq.state() == hw_task::TaskState::Paused, "进入 PAUSED");

  // 暂停期间推进 2 秒（真实时间在走，状态机不该动）
  for (int i = 0; i < 100; ++i) { t += h.dt; }

  check(h.seq.resume(h.snapshot(t)), "恢复被接受");
  check(h.seq.state() == before, "恢复到暂停前的状态");

  /* 恢复后不能立刻超时 —— 这验证的是"把暂停时长从两个计时器里扣掉"。
   * 曾经的写法先 t_state_ = s.t 再 t_task_ += (s.t - t_state_)，
   * 括号里恒为 0，等于没补偿；暂停 2 秒再恢复会立刻触发状态超时。 */
  h.seq.update(h.snapshot(t), h.dt);
  check(h.seq.state() == before, "恢复后的第一个周期不会因超时跳状态");

  bool done = false;
  for (int i = 0; i < 8000; ++i) {
    const hw_task::Actuation b = h.tick(t);
    if (b.finished || b.aborted) { done = true; break; }
  }
  check(done, "恢复后能继续跑完任务");
  check(h.seq.status().grasped_count == 1, "最终放置了 1 个");
}

// ===========================================================================
//  I. 不可达目标
// ===========================================================================
void test_unreachable_target()
{
  section("I. 不可达目标的处理");

  Harness h;
  h.add_block(0.55, 0.0, hw_task::BlockColor::Red);   // 超出 0.40 的工作半径上限

  hw_task::StartRequest req;
  req.max_blocks = 1;

  const RunResult r = h.run(req, 9000);

  std::printf("  终态=%s 放置=%u 跳过=%u 步数=%d\n",
              st_name(r.final_state), r.placed, r.skipped, r.steps);
  std::printf("  末条消息: %s\n", r.last_message.c_str());

  check(r.placed == 0, "够不到的物块不会被抓");
  check(r.final_state == hw_task::TaskState::Failed ||
        r.final_state == hw_task::TaskState::Done,
        "最终收敛到终态（DONE 或 FAILED），不会无限重试");
}

// ===========================================================================
//  J. 码垛
// ===========================================================================
void test_stacking()
{
  section("J. 码垛（同槽位逐层叠高）");

  Harness h;
  h.cfg.place_mode = hw_task::PlaceMode::Stack;
  h.cfg.place_max_layers = 3;
  h.seq.configure(h.cfg);

  h.add_block(0.24, 0.05, hw_task::BlockColor::Red);
  h.add_block(0.30, 0.02, hw_task::BlockColor::Blue);
  h.add_block(0.27, -0.06, hw_task::BlockColor::Green);

  hw_task::StartRequest req;
  req.max_blocks = 3;

  const RunResult r = h.run(req, 24000);

  std::printf("  终态=%s 放置=%u 跳过=%u 最高层=%u 步数=%d\n",
              st_name(r.final_state), r.placed, r.skipped, r.max_layer, r.steps);
  std::printf("  末条消息: %s\n", r.last_message.c_str());

  check(r.placed == 3, "三个物块都放上了");
  check(r.final_state == hw_task::TaskState::Done, "任务正常结束");
  /* 这是"有了升降轴才能做码垛"的直接体现：底盘方案里 Z 不可控，只能平铺。
   * 这里要求层号确实递增过 —— 否则就是"平铺"而不是"码垛"。 */
  check(r.max_layer >= 2, "层号递增到 2（第 3 层），确实在叠高而不是平铺");
}

// ===========================================================================
//  K. 视觉持续丢失
// ===========================================================================
void test_vision_lost()
{
  section("K. 视觉持续丢失");

  Harness h;
  h.vision_down = true;      // 相机掉线
  h.add_block(0.26, 0.05, hw_task::BlockColor::Red);

  hw_task::StartRequest req;
  req.max_blocks = 1;

  const RunResult r = h.run(req, 6000);

  std::printf("  终态=%s 步数=%d 消息=%s\n",
              st_name(r.final_state), r.steps, r.last_message.c_str());

  check(r.final_state == hw_task::TaskState::Failed, "一直看不到目标时判定失败");
  check(r.aborted, "失败时把 aborted 上报给上层");
  check(r.placed == 0, "没有目标就不会抓");
}

// ===========================================================================
//  L. 视觉噪声下的鲁棒性
// ===========================================================================
void test_noisy_vision()
{
  section("L. 视觉噪声下的鲁棒性");

  Harness h;
  h.world.noise = 0.0012;    // ±1.2mm 位置噪声，接近真实投影的重复精度
  h.add_block(0.26, 0.04, hw_task::BlockColor::Red);

  hw_task::StartRequest req;
  req.max_blocks = 1;

  const RunResult r = h.run(req, 9000);

  std::printf("  终态=%s 放置=%u 跳过=%u 步数=%d\n",
              st_name(r.final_state), r.placed, r.skipped, r.steps);

  /* 有 1.2mm 噪声时视觉伺服的目标每帧都在动，但 servo_tol 是 1.5mm，
   * 所以状态机不会每周期重发目标 —— 这正是 servo_tol 存在的意义：
   * 阈值必须比噪声大，否则收敛判据永远不满足，机械臂会在目标附近
   * 无限微调下去（"到不了"）。 */
  check(r.placed == 1, "有视觉噪声时仍然能完成取放");
  check(r.final_state == hw_task::TaskState::Done, "正常结束");
}

}  // namespace

int main()
{
  std::printf("HWB-ARM4 取放任务层 —— 主机侧回归测试\n");
  std::printf("仿真用的是 arm_control 的真实 D-H 正逆解，闭环里没有第二套运动学\n");

  test_projection();
  test_tracker();
  test_full_cycle();
  test_grasp_failure();
  test_estop();
  test_abort_return_home();
  test_payload_drop();
  test_pause_resume();
  test_unreachable_target();
  test_stacking();
  test_vision_lost();
  test_noisy_vision();

  std::printf("\n==================================================\n");
  std::printf("  通过 %d 项，失败 %d 项\n", g_pass, g_fail);
  std::printf("==================================================\n");
  return g_fail == 0 ? 0 : 1;
}
