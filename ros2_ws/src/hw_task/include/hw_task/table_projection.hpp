// ============================================================================
//  table_projection.hpp
//  像素 -> 基座系工作台面坐标的投影。这是替代 demo 里那两行线性拟合的东西。
//
//  为什么不用 demo 的线性拟合
//  --------------------------
//  demo 里相机是固定安装的，把像素直接线性映射到机械臂基座坐标：
//      robot_x = 0.0008*object_y + 0.2563
//      robot_y = -0.0010*object_x + 0.1450
//  这在"相机不动 + 工作平面固定"时**能用**，但它把内参、外参、台面高度
//  混进了两个碰运气的系数里：重新夹一次相机或换一块厚一点的台面，
//
//  正确做法是走完整投影链路：
//      像素 (u,v)
//        -> 归一化射线 d_cam = K^-1 [u,v,1]
//  系数就得重猜，而且没有任何指标告诉你猜得对不对。
//        -> 与地面平面 z = 0 求交
//  其中 T_oc 用**图像时间戳**去 TF 里查历史值 —— 因为驱动板在收到相机
//  这里把映射拆成有物理意义的链路，外参由 TF 提供，残余误差用
//  2D 仿射校正吃掉 —— 见下。
//
//  剩下的装配公差 / 镜头畸变残差用一个 2D 仿射校正补掉（见 set_residual）。
// ============================================================================
// ============================================================================
//  table_projection.hpp
//  像素 -> 工作台面坐标的投影。这是替代 demo 里那两行线性拟合的东西。
//
//  demo 的做法
//  ----------
//  demo 的相机是固定安装的，于是它把像素直接线性映射到机械臂基座坐标：
//      robot_x = 0.0008*object_y + 0.2563
//      robot_y = -0.0010*object_x + 0.1450
//  这在"相机不动 + 工作平面固定"时**能用**，但它把两件事混在了一起：
//  相机内参、相机安装外参、工作台高度。任何一项变了（重新夹一次相机、
//  换一块厚一点的工作台），系数就得重新试凑，而且没有任何指标告诉
//  你试得对不对。
//
//  这里保留 demo 的工作前提（相机固定安装、物块在工作台平面上），
//  但把映射拆成有物理意义的链路：
//      像素 (u,v)
//        -> 归一化射线 d_cam = K^-1 [u,v,1]
//        -> 用相机外参把射线转到基座系
//        -> 与工作台平面 z = 0 求交
//  其中相机外参由 TF 提供（固定安装时是静态变换，用图像时间戳查仍然
//  更稳妥：如果相机改成装在末端上，这条链路不用改）。
//
//  剩下的装配公差 / 镜头畸变残差用一个 2D 仿射校正补掉（见 set_residual）。
//  这样"重新夹一次相机"只需要重采几个标定点，而不用重猜那两个系数。
// ============================================================================
#ifndef HW_TASK__TABLE_PROJECTION_HPP_
#define HW_TASK__TABLE_PROJECTION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <optional>

namespace hw_task
{

struct CameraIntrinsics
{
  double fx{600.0};
  double fy{600.0};
  double cx{320.0};
  double cy{240.0};
  int    width{640};
  int    height{480};
  /// 径向畸变前两个系数。默认 0；用 ros2 camera_calibration 标出来再填。
  double k1{0.0};
  double k2{0.0};

  bool valid() const { return fx > 1.0 && fy > 1.0 && width > 0 && height > 0; }
};

/// 基座系下的工作台面点（z 恒为 0）
struct TablePoint
{
  double x{0.0};
  double y{0.0};
  double range{0.0};        ///< 相机原点到交点的距离，用于判远/近
  bool   valid{false};
};

class TableProjection
{
public:
  TableProjection() = default;

  void set_intrinsics(const CameraIntrinsics & k);
  const CameraIntrinsics & intrinsics() const { return K_; }

  /**
   * @brief 把一个像素投到地面上。
   * @param u,v  像素坐标（原始图像，未裁剪 ROI）
   * @param T_oc 相机光学系 -> 基座系的刚体变换（4x4）
   * @return 交点；射线与地面平行或交点在相机后方时 valid=false
   */
  TablePoint project(double u, double v, const Eigen::Matrix4d & T_oc) const;

  /**
   * @brief 由图像里的物块中心、图像角度、长边像素长度，求物块在地面上的真实偏航角。
   *
   * 直接把 minAreaRect 的角度当 yaw 用是错的：相机是斜俯视的，图像里
   * 一个 45° 的物块投影到地面并不是 45°。这里沿长轴在图像里取两个端点，
   * 分别投到地面，再用两端点的地面连线求角 —— 透视关系自然被吸收。
   */
  double project_yaw(double u, double v, double yaw_img_rad, double length_px,
                     const Eigen::Matrix4d & T_oc) const;

  /**
   * @brief 设置地面残差校正： [x';y'] = A * [x;y] + t
   * @param a00,a01,a10,a11 线性部分（默认单位矩阵）
   * @param tx,ty           平移部分
   *
   * 校正量由 CalibrateMapping 服务累积标定点后用最小二乘解出。
   * 只用来吃掉装配公差与畸变残差，正常情况下线性部分应该非常接近单位阵
   * （偏差 > 5% 说明相机外参或内参有问题，应该去改 URDF/标定，而不是在这里补）。
   */
  void set_residual(double a00, double a01, double a10, double a11,
                    double tx, double ty);
  void clear_residual();

  /**
   * @brief 用一对 (像素, 世界坐标) 标定点做增量标定。
   * @return 本次标定点的重投影误差（米）；输入非法时返回 -1
   *
   * 单点只能解平移；要解出完整的 2D 仿射至少需要 3 个不共线的点。
   * 因此内部维护一个最多 8 点的缓冲，每次来新点就用全部点重解一次最小二乘。
   */
  double add_calibration_point(double u, double v,
                               double world_x, double world_y,
                               const Eigen::Matrix4d & T_oc);

  /// 已采集的标定点数
  std::size_t calibration_points() const { return calib_pts_; }

  /// 标定点在源坐标系里的空间跨度（对角包围盒，米）
  double calibration_spread() const { return calib_spread_; }

  /// 是否真正解出了完整的 2D 仿射（false 表示点太集中，只校正了平移）
  bool affine_solved() const { return affine_solved_; }

  /// 判定"点够散"的门槛。低于它只解平移，见 solve_residual() 的注释。
  static constexpr double kMinCalibSpread = 0.08;

  /// 当前残差校正矩阵（调试用）
  Eigen::Matrix3d residual_matrix() const;

private:
  TablePoint apply_residual(const TablePoint & p) const;

  CameraIntrinsics K_{};
  bool   has_residual_{false};
  Eigen::Matrix3d resid_{Eigen::Matrix3d::Identity()};

  // ---- 标定点缓冲 ----
  static constexpr std::size_t kMaxCalibPts = 8;
  std::array<Eigen::Vector2d, kMaxCalibPts> calib_src_{};   ///< 未校正的投影
  std::array<Eigen::Vector2d, kMaxCalibPts> calib_dst_{};   ///< 真实世界坐标
  std::size_t calib_pts_{0};
  double calib_spread_{0.0};
  bool   affine_solved_{false};

  /// 用缓冲里的点重解 2D 仿射（点 < 3 时只解平移）
  void solve_residual();
};

}  // namespace hw_task

#endif  // HW_TASK__TABLE_PROJECTION_HPP_
