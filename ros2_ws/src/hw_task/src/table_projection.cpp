// ============================================================================
//  table_projection.cpp
// ============================================================================
#include "hw_task/table_projection.hpp"

#include <algorithm>
#include <cmath>

namespace hw_task
{

namespace
{
/// 本文件里所有三角运算统一用这个常量。
/// 不用 <cmath> 里的 M_PI：glibc 只在 __USE_MISC 下导出它，
/// 用 `-std=c++17`（严格模式）编译时它会消失 —— 而 gnu++17 又有。
/// 自带的常量不会有这个坑，也能保证主机侧测试与目标机行为一致。
constexpr double kPi = 3.14159265358979323846;

constexpr double kEps = 1e-9;

/// 归一化角度到 (-pi, pi]
double wrap_pi(double a)
{
  while (a > kPi) { a -= 2.0 * kPi; }
  while (a <= -kPi) { a += 2.0 * kPi; }
  return a;
}

/// 3x3 线性方程组，列主元高斯消元。奇异时返回零向量（调用方会因残差过大而拒绝该标定）。
Eigen::Vector3d solve3(double m[3][3], const double rhs[3])
{
  double a[3][3];
  double b[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) { a[i][j] = m[i][j]; }
    b[i] = rhs[i];
  }

  for (int c = 0; c < 3; ++c) {
    int piv = c;
    for (int r = c + 1; r < 3; ++r) {
      if (std::abs(a[r][c]) > std::abs(a[piv][c])) { piv = r; }
    }
    if (std::abs(a[piv][c]) < 1e-12) { return Eigen::Vector3d(0.0, 0.0, 0.0); }
    if (piv != c) {
      for (int k = 0; k < 3; ++k) { std::swap(a[c][k], a[piv][k]); }
      std::swap(b[c], b[piv]);
    }
    for (int r = c + 1; r < 3; ++r) {
      const double f = a[r][c] / a[c][c];
      for (int k = c; k < 3; ++k) { a[r][k] -= f * a[c][k]; }
      b[r] -= f * b[c];
    }
  }

  Eigen::Vector3d x;
  for (int r = 2; r >= 0; --r) {
    double s = b[r];
    for (int k = r + 1; k < 3; ++k) { s -= a[r][k] * x(k); }
    x(r) = s / a[r][r];
  }
  return x;
}
}  // namespace

void TableProjection::set_intrinsics(const CameraIntrinsics & k)
{
  K_ = k;
}

void TableProjection::set_residual(double a00, double a01, double a10, double a11,
                                    double tx, double ty)
{
  resid_ << a00, a01, tx,
            a10, a11, ty,
            0.0, 0.0, 1.0;
  has_residual_ = true;
}

void TableProjection::clear_residual()
{
  resid_ = Eigen::Matrix3d::Identity();
  has_residual_ = false;
  calib_pts_ = 0;
  calib_spread_ = 0.0;
  affine_solved_ = false;
}

Eigen::Matrix3d TableProjection::residual_matrix() const
{
  return resid_;
}

TablePoint TableProjection::apply_residual(const TablePoint & p) const
{
  if (!has_residual_ || !p.valid) {
    return p;
  }
  const Eigen::Vector3d v = resid_ * Eigen::Vector3d(p.x, p.y, 1.0);
  TablePoint out = p;
  out.x = v.x();
  out.y = v.y();
  // range 保持原语义（相机原点到交点的距离），残差校正只动 x/y，不动它
  return out;
}

TablePoint TableProjection::project(double u, double v,
                                      const Eigen::Matrix4d & T_oc) const
{
  TablePoint out;
  if (!K_.valid()) {
    return out;
  }

  const double fx = K_.fx, fy = K_.fy, cx = K_.cx, cy = K_.cy;

  // ---- 1. 去畸变（Brown 径向模型，只做两级就够了：本项目用的 M12 镜头
  //         畸变很小，k1/k2 默认都是 0，标定后填进来即可）----
  double xd = (u - cx) / fx;
  double yd = (v - cy) / fy;
  if (K_.k1 != 0.0 || K_.k2 != 0.0) {
    const double r2 = xd * xd + yd * yd;
    const double r4 = r2 * r2;
    const double radial = 1.0 + K_.k1 * r2 + K_.k2 * r4;
    if (radial > 1e-6) {
      xd /= radial;
      yd /= radial;
    }
  }

  // ---- 2. 相机光学系下的射线方向 ----
  // 光学系约定（REP-103 / OpenCV）：x 向右、y 向下、z 向前
  const Eigen::Vector3d d_cam(xd, yd, 1.0);
  const Eigen::Vector3d d_cam_n = d_cam.normalized();

  // ---- 3. 旋转到基座系 ----
  const Eigen::Matrix3d R_oc = T_oc.block<3, 3>(0, 0);
  const Eigen::Vector3d origin(T_oc(0, 3), T_oc(1, 3), T_oc(2, 3));
  const Eigen::Vector3d d_base = R_oc * d_cam_n;

  // ---- 4. 与地面 z = 0 求交 ----
  // 相机是向下俯视的，所以 d_base.z() 应为负；若为正说明射线朝天，
  // 那这个像素根本不在工作面上（比如拍到了远处的墙），直接判无效。
  if (d_base.z() > -1e-6) {
    return out;
  }
  const double t = -origin.z() / d_base.z();
  if (t <= 0.0) {
    return out;   // 交点在相机背后
  }

  const Eigen::Vector3d hit = origin + t * d_base;

  TablePoint raw;
  raw.x = hit.x();
  raw.y = hit.y();
  raw.range = (hit - origin).norm();
  raw.valid = true;

  out = apply_residual(raw);
  out.valid = true;
  return out;
}

double TableProjection::project_yaw(double u, double v, double yaw_img_rad,
                                     double length_px,
                                     const Eigen::Matrix4d & T_oc) const
{
  // 物块长轴在图像里的两个端点。图像坐标系 y 向下，
  // 因此角度的正方向与地面系相反，但这里只关心最终地面角，符号由投影结果决定。
  const double half = std::max(2.0, length_px * 0.5);
  const double du = std::cos(yaw_img_rad) * half;
  const double dv = std::sin(yaw_img_rad) * half;

  const TablePoint p1 = project(u - du, v - dv, T_oc);
  const TablePoint p2 = project(u + du, v + dv, T_oc);
  if (!p1.valid || !p2.valid) {
    // 端点投不出来（贴太近或在视野边缘）时退回"图像角 + 相机安装偏航"的近似。
    // 这个近似只在前向俯视且物块离主点不远时成立，所以只在退化路径上用。
    const Eigen::Matrix3d R_oc = T_oc.block<3, 3>(0, 0);
    const double cam_yaw = std::atan2(R_oc(1, 2), R_oc(0, 2));
    return wrap_pi(-(yaw_img_rad) + cam_yaw);
  }

  const double dx = p2.x - p1.x;
  const double dy = p2.y - p1.y;
  if (std::hypot(dx, dy) < 1e-4) {
    return 0.0;
  }
  // 返回物块长轴与 基座系 系 x 轴的夹角
  return std::atan2(dy, dx);
}

double TableProjection::add_calibration_point(double u, double v,
                                               double world_x, double world_y,
                                               const Eigen::Matrix4d & T_oc)
{
  // 用未校正的投影作为源。临时关掉残差，避免"用已校正的投影去校正残差"的反馈。
  const bool saved = has_residual_;
  const Eigen::Matrix3d saved_m = resid_;
  has_residual_ = false;
  const TablePoint raw = project(u, v, T_oc);
  has_residual_ = saved;
  resid_ = saved_m;

  if (!raw.valid) {
    return -1.0;
  }

  // 环形缓冲：满了就丢掉最老的点。保留最近 8 个点是因为装配公差和
  // 镜头畸变在不同视野区域不一样，用太老的点反而拉低精度。
  if (calib_pts_ >= kMaxCalibPts) {
    for (std::size_t i = 1; i < kMaxCalibPts; ++i) {
      calib_src_[i - 1] = calib_src_[i];
      calib_dst_[i - 1] = calib_dst_[i];
    }
    calib_pts_ = kMaxCalibPts - 1;
  }
  calib_src_[calib_pts_] = Eigen::Vector2d(raw.x, raw.y);
  calib_dst_[calib_pts_] = Eigen::Vector2d(world_x, world_y);
  ++calib_pts_;

  solve_residual();

  // 返回校正后的重投影误差
  const TablePoint after = apply_residual(raw);
  return static_cast<double>(std::hypot(after.x - world_x, after.y - world_y));
}

void TableProjection::solve_residual()
{
  if (calib_pts_ == 0) {
    clear_residual();
    return;
  }

  /* 先算标定点的空间跨度。点挤在一起时（比如操作员在屏幕上反复点同一个
   * 位置附近的物块），完整的 2D 仿射是**病态问题**：设计矩阵几乎是共线的，
   * 正规方程又把条件数平方，双精度也压不住 —— 会解出一个线性部分乱飞、
   * 靠平移去凑的矩阵。那种矩阵在标定点附近误差很小，但离开标定区域
   * 两米就完全不可用，而它看起来"标定成功了"，非常危险。
   *
   * 所以：跨度不够就只解平移。平移永远有解，而且能吃掉绝大部分残差
   * （残差的主要来源就是相机安装公差）。要校正旋转/尺度，请把标定块
   * 分散摆到工作区的四个角上再采点。 */
  {
    double minx = 1e18, maxx = -1e18, miny = 1e18, maxy = -1e18;
    for (std::size_t i = 0; i < calib_pts_; ++i) {
      minx = std::min(minx, calib_src_[i].x());
      maxx = std::max(maxx, calib_src_[i].x());
      miny = std::min(miny, calib_src_[i].y());
      maxy = std::max(maxy, calib_src_[i].y());
    }
    calib_spread_ = std::hypot(maxx - minx, maxy - miny);
  }

  if (calib_pts_ < 3 || calib_spread_ < kMinCalibSpread) {
    affine_solved_ = false;
    // 点太少 或 点太集中 -> 只解平移（均值差）
    Eigen::Vector2d s(0.0, 0.0), d(0.0, 0.0);
    for (std::size_t i = 0; i < calib_pts_; ++i) {
      s += calib_src_[i];
      d += calib_dst_[i];
    }
    s /= static_cast<double>(calib_pts_);
    d /= static_cast<double>(calib_pts_);
    set_residual(1.0, 0.0, 0.0, 1.0, d.x() - s.x(), d.y() - s.y());
    return;
  }

  /* 最小二乘解 [a00 a01 tx; a10 a11 ty]，设计矩阵每行是 [x, y, 1]。
   *
   * 用正规方程 (AᵀA) c = Aᵀb 直接解 3x3 —— 不引入 Eigen 的动态矩阵与 QR，
   * 因为这个问题的未知量恒为 3 个，且标定点分布在工作区内（坐标量级 0.x 米、
   * 条件数很小）。这样写还有一个附加好处：这段代码不依赖 Eigen 的求解器
   * API，主机侧逻辑测试只需要一个极简的向量/矩阵垫片就能跑。 */
  double ata[3][3] = {{0.0}};
  double atbx[3] = {0.0};
  double atby[3] = {0.0};
  for (std::size_t i = 0; i < calib_pts_; ++i) {
    const double row[3] = {calib_src_[i].x(), calib_src_[i].y(), 1.0};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) { ata[r][c] += row[r] * row[c]; }
      atbx[r] += row[r] * calib_dst_[i].x();
      atby[r] += row[r] * calib_dst_[i].y();
    }
  }

  const Eigen::Vector3d cx = solve3(ata, atbx);
  const Eigen::Vector3d cy = solve3(ata, atby);

  // 安全检查：线性部分偏离单位阵超过 15% 说明标定点本身有问题
  // （比如把标定块放错位置、或者相机外参错得离谱）。
  // 这种情况下宁可不校正，也不要让一个坏矩阵把整个投影链路带偏。
  const double dev = std::max({std::abs(cx(0) - 1.0), std::abs(cx(1)),
                               std::abs(cy(0)),      std::abs(cy(1) - 1.0)});
  if (dev > 0.15) {
    return;
  }

  affine_solved_ = true;
  set_residual(cx(0), cx(1), cy(0), cy(1), cx(2), cy(2));
}

}  // namespace hw_task
