// ============================================================================
//  ground_projection.hpp
//  像素 -> odom 系地面坐标的投影。这是替代 demo 里那两行线性拟合的东西。
//
//  为什么不用 demo 的线性拟合
//  --------------------------
//  demo 里相机是固定安装的，把像素直接线性映射到机械臂基座坐标：
//      robot_x = 0.0008*object_y + 0.2563
//      robot_y = -0.0010*object_x + 0.1450
//  这在"相机不动 + 工作平面固定"时是对的。但本项目的相机装在**移动底盘**上，
//  车一动，同样的像素对应完全不同的世界坐标。线性拟合直接失效。
//
//  正确做法是走完整投影链路：
//      像素 (u,v)
//        -> 归一化射线 d_cam = K^-1 [u,v,1]
//        -> 由 TF 提供的 T_oc（相机光学系 -> odom）旋转到 odom 系
//        -> 与地面平面 z = 0 求交
//  其中 T_oc 用**图像时间戳**去 TF 里查历史值 —— 因为驱动板在收到相机
//  SYNC 上升沿的那一刻锁存了四轮位姿，用同一时刻的位姿才不会把图像和
//  位姿错开一个里程计周期。
//
//  剩下的装配公差 / 镜头畸变残差用一个 2D 仿射校正补掉（见 set_residual）。
// ============================================================================
#ifndef HW_TASK__GROUND_PROJECTION_HPP_
#define HW_TASK__GROUND_PROJECTION_HPP_

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

/// odom 系下的地面点（z 恒为 0）
struct GroundPoint
{
  double x{0.0};
  double y{0.0};
  double range{0.0};        ///< 相机原点到交点的距离，用于判远/近
  bool   valid{false};
};

class GroundProjection
{
public:
  GroundProjection() = default;

  void set_intrinsics(const CameraIntrinsics & k);
  const CameraIntrinsics & intrinsics() const { return K_; }

  /**
   * @brief 把一个像素投到地面上。
   * @param u,v  像素坐标（原始图像，未裁剪 ROI）
   * @param T_oc 相机光学系 -> odom 的刚体变换（4x4）
   * @return 交点；射线与地面平行或交点在相机后方时 valid=false
   */
  GroundPoint project(double u, double v, const Eigen::Matrix4d & T_oc) const;

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
  GroundPoint apply_residual(const GroundPoint & p) const;

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

// ---------------------------------------------------------------------------
//  工具：把地面点换算成车体极坐标
// ---------------------------------------------------------------------------
struct PolarTarget
{
  double range{0.0};     ///< 距离（m）
  double bearing{0.0};   ///< 方位角（rad），车头正前方为 0，左正右负
  double dx{0.0};        ///< 车体系前后偏差（正值=目标在车前方）
  double dy{0.0};        ///< 车体系左右偏差（正值=目标在车左侧）
};

PolarTarget to_polar(double target_x, double target_y,
                     double vehicle_x, double vehicle_y, double vehicle_yaw);

}  // namespace hw_task

#endif  // HW_TASK__GROUND_PROJECTION_HPP_
