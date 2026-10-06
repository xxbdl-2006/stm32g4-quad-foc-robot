// ============================================================================
//  block_detector_node.cpp
//  物块识别。把睿抗 demo 里那套 OpenCV 流水线 ROS2 化。
//
//  直接沿用 demo 的算法骨架（HSV 分割 -> 形态学 -> findContours -> minAreaRect），
//  但有四处必须改，不改会出错：
//
//  ① 取消水平镜像
//     demo 里有一句 cv2.flip(frame, 1)，因为那是个摆在桌上、镜头朝向操作者的
//     USB 摄像头，镜像后操作手感才对。本项目的相机固装在工作台上方朝前，镜像会把
//     像素 x 轴翻过来，而 task_executor 的投影链路是按标准 OpenCV 相机约定
//     （x 向右）写的 —— 镜像之后左右会反，抓取点会偏到物块的镜像位置。
//     所以默认 flip_horizontal = false，并保留开关以便换成后视相机时使用。
//
//  ② 不再"二选一"
//     demo 里 `if area_red > area_blue: Finall_area = cnts_red else cnts`
//     —— 每个周期只处理优势颜色，另一色的物块被整个丢掉。
//     这里改成逐颜色独立跑流水线，一帧里所有颜色的物块都报出来，
//     交给下游 tracker 按颜色优先级排序。
//
//  ③ ROI 坐标要换算回全图
//     demo 裁了 ROI 之后直接把 ROI 内的坐标当全图坐标用（因为它只做显示）。
//     本项目的投影用的是全图内参，所以发布前必须加回 (x0, y0) 偏移。
//
//  ④ 角度用 float 且从 boxPoints 算
//     demo 用 `np.int0(Theta[2])` 把角度截断成整数，还依赖 minAreaRect 的
//     角度约定（OpenCV 版本之间变过）。这里改为直接用四个角点求最长边的
//     方向角，与 OpenCV 版本无关，且保留 float 精度 —— 吸盘是长条形的，
//     0.5° 的角度误差在 80mm 的物块上就是 0.7mm 的对齐偏差。
// ============================================================================
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "hw_msgs/msg/detected_block.hpp"
#include "hw_msgs/msg/detected_block_array.hpp"

using namespace std::chrono_literals;

namespace hw_task
{

namespace
{

/// 一个颜色的 HSV 阈值区间（红色跨 0°，所以用两组区间）
struct HsvRange
{
  std::string name;
  uint8_t     color_id;
  int h_lo1, h_hi1, s_lo, s_hi, v_lo, v_hi;
  int h_lo2, h_hi2;      ///< 第二组；h_hi2 < 0 表示没有第二组
};

}  // namespace

class BlockDetectorNode : public rclcpp::Node
{
public:
  BlockDetectorNode()
  : rclcpp::Node("block_detector")
  {
    image_topic_    = declare_parameter("image_topic", std::string("/image_raw"));
    roi_x0_         = declare_parameter("roi.x0", 0);
    roi_x1_         = declare_parameter("roi.x1", 0);      // 0 = 用整幅
    roi_y0_         = declare_parameter("roi.y0", 0);
    roi_y1_         = declare_parameter("roi.y1", 0);
    flip_horizontal_ = declare_parameter("flip_horizontal", false);
    min_area_px_    = declare_parameter("min_area_px", 2000.0);
    max_area_px_    = declare_parameter("max_area_px", 30000.0);
    morph_kernel_   = declare_parameter("morph_kernel", 5);
    use_sharpen_    = declare_parameter("use_sharpen", true);
    prefilter_frames_ = declare_parameter("prefilter_stable_frames", 2);
    prefilter_radius_px_ = declare_parameter("prefilter_radius_px", 40.0);
    debug_view_     = declare_parameter("debug_view", false);
    max_blocks_     = declare_parameter("max_blocks_per_frame", 8);

    // ---- 颜色阈值 ----
    // 红色跨 HSV 的 0° 边界，必须拆成两段（[0,10] 与 [160,180]），
    // 这是 demo 里就踩过并写进注释的坑。
    // 阈值与 demo 保持一致，但把 S/V 下限从 100 调到 80 —— 现场灯光偏暗时
    // 100 会漏检，80 在正常灯光下也不会引进背景噪声（实测过）。
    colors_.push_back({"red",  hw_msgs::msg::DetectedBlock::COLOR_RED,
                       0, 10, 80, 255, 80, 255, 160, 180});
    colors_.push_back({"blue", hw_msgs::msg::DetectedBlock::COLOR_BLUE,
                       100, 130, 80, 255, 80, 255, -1, -1});
    colors_.push_back({"green", hw_msgs::msg::DetectedBlock::COLOR_GREEN,
                       45, 85, 80, 255, 80, 255, -1, -1});

    pub_ = create_publisher<hw_msgs::msg::DetectedBlockArray>("/detected_blocks", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr m) { on_image(m); });

    RCLCPP_INFO(get_logger(),
                "物块识别已启动：话题 %s，ROI [%d,%d,%d,%d]，面积窗口 %.0f~%.0f px²，"
                "镜像=%s，启用颜色 %zu 种",
                image_topic_.c_str(), roi_x0_, roi_x1_, roi_y0_, roi_y1_,
                min_area_px_, max_area_px_, flip_horizontal_ ? "是" : "否",
                colors_.size());
    RCLCPP_INFO(get_logger(),
                "注意：本节点只做像素级检测，稳定判据与地面投影在 "
                "hw_task/task_executor 里（它需要相机位姿才能把像素投到地面）");
  }

private:
  // -------------------------------------------------------------------------
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->encoding != "bgr8" && msg->encoding != "rgb8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "不支持的图像编码 %s（需要 bgr8/rgb8）",
                           msg->encoding.c_str());
      return;
    }

    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, msg->encoding)->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "cv_bridge 转换失败：%s", e.what());
      return;
    }

    if (msg->encoding == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    // ① 镜像（见文件头说明，默认关闭）
    if (flip_horizontal_) {
      cv::flip(frame, frame, 1);
    }

    const int full_w = frame.cols;
    const int full_h = frame.rows;

    // ---- ③ ROI：只在 ROI 内做检测，但坐标最终要加回偏移 ----
    int x0 = roi_x0_, y0 = roi_y0_, x1 = roi_x1_, y1 = roi_y1_;
    if (x1 <= x0 || y1 <= y0 || x1 > full_w || y1 > full_h) {
      x0 = 0; y0 = 0; x1 = full_w; y1 = full_h;
    }
    const cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
    const cv::Mat img = frame(roi);

    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    hw_msgs::msg::DetectedBlockArray out;
    out.header = msg->header;          // 时间戳原样透传，投影链路依赖它
    out.image_seq = seq_++;
    out.roi_clipped = (x0 != 0 || y0 != 0 || x1 != full_w || y1 != full_h);

    for (const auto & c : colors_) {
      cv::Mat mask = make_mask(hsv, c);
      detect(mask, c, out, x0, y0);
    }

    // 一帧里最多报 N 个，避免极端反光/曝光时刷屏把下游淹掉
    if (out.blocks.size() > static_cast<std::size_t>(max_blocks_)) {
      std::partial_sort(
        out.blocks.begin(),
        out.blocks.begin() + max_blocks_,
        out.blocks.end(),
        [](const hw_msgs::msg::DetectedBlock & a, const hw_msgs::msg::DetectedBlock & b) {
          return a.area > b.area;
        });
      out.blocks.resize(static_cast<std::size_t>(max_blocks_));
    }

    pub_->publish(out);

    if (debug_view_) {
      draw_debug(frame, out);
      cv::imshow("block_detector", frame);
      cv::waitKey(1);
    }
  }

  // -------------------------------------------------------------------------
  cv::Mat make_mask(const cv::Mat & hsv, const HsvRange & c)
  {
    cv::Mat m1, m2, mask;
    cv::inRange(hsv, cv::Scalar(c.h_lo1, c.s_lo, c.v_lo),
                cv::Scalar(c.h_hi1, c.s_hi, c.v_hi), m1);
    if (c.h_hi2 > 0) {
      cv::inRange(hsv, cv::Scalar(c.h_lo2, c.s_lo, c.v_lo),
                  cv::Scalar(c.h_hi2, c.s_hi, c.v_hi), m2);
      cv::bitwise_or(m1, m2, mask);
    } else {
      mask = m1;
    }

    // 形态学：先闭运算填内部空洞，再开运算去孤立噪点。
    // 顺序不能反 —— 反过来会先把物块的细边吃掉，再去填洞就晚了。
    if (morph_kernel_ >= 3) {
      const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel_, morph_kernel_));
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
    }

    if (use_sharpen_) {
      // demo 里的 3x3 锐化核，能让边缘收缩半像素，对 minAreaRect 的
      // 角度稳定性有帮助。
      static const cv::Mat sharpen_kernel =
        (cv::Mat_<float>(3, 3) << -1, -1, -1, -1, 9, -1, -1, -1, -1);
      cv::filter2D(mask, mask, -1, sharpen_kernel);
    }
    return mask;
  }

  // -------------------------------------------------------------------------
  void detect(const cv::Mat & mask, const HsvRange & c,
              hw_msgs::msg::DetectedBlockArray & out, int x_off, int y_off)
  {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto & cnt : contours) {
      const double area = cv::contourArea(cnt);
      // 面积过滤：对应 demo 的 min_area = 2000 / max_area = 30000。
      // 下限滤掉噪点，上限滤掉"两个物块粘在一起"的合并轮廓 ——
      // 那种情况下的 minAreaRect 会给一个巨大的错误角度。
      if (area < min_area_px_ || area > max_area_px_) {
        continue;
      }
      // 周长/面积比过大的轮廓是细长条（地面反光、线缆），排除
      const double perim = cv::arcLength(cnt, true);
      if (perim < 1.0 || (perim * perim / area) > 60.0) {
        continue;
      }

      const cv::RotatedRect rr = cv::minAreaRect(cnt);
      const cv::Point2f ctr = rr.center;

      hw_msgs::msg::DetectedBlock b;
      b.header = out.header;
      b.color = c.color_id;
      // ③ 换算回全图坐标
      b.pixel_x = ctr.x + static_cast<float>(x_off);
      b.pixel_y = ctr.y + static_cast<float>(y_off);
      b.area = static_cast<float>(area);
      b.yaw = long_axis_angle(rr);

      // 像素域的粗筛稳定度：同一颜色在连续帧里中心不跳、角度不跳才算数。
      // 这只是给下游减负的**预筛**，真正的判据在 tracker 里（地面坐标系）。
      update_prefilter(b);

      out.blocks.push_back(b);
    }
  }

  /**
   * ④ 从四角点求长轴方向角。
   *
   * 不用 rr.angle 的原因：OpenCV 4.5 前后这个字段的定义变过
   * （4.5 之前是 [-90,0)，之后是 (0,90]），而且它描述的是 width 边的方向，
   * 不是"长边"的方向。直接从 boxPoints 里挑距离最远的那对顶点求角，
   * 与版本无关、与 w/h 谁大无关。
   */
  static double long_axis_angle(const cv::RotatedRect & rr)
  {
    cv::Point2f pts[4];
    rr.points(pts);

    double best = -1.0;
    double ang = 0.0;
    for (int i = 0; i < 4; ++i) {
      for (int j = i + 1; j < 4; ++j) {
        const double dx = pts[j].x - pts[i].x;
        const double dy = pts[j].y - pts[i].y;
        const double d2 = dx * dx + dy * dy;
        if (d2 > best) {
          best = d2;
          ang = std::atan2(dy, dx);
        }
      }
    }

    // 归一化到 [-pi/2, pi/2)：正方形的对称性让 180° 的差别没有意义，
    // 而归一化之后 tracker 的角度平滑不会在 ±90° 边界上跳变。
    while (ang >= M_PI / 2.0) { ang -= M_PI; }
    while (ang < -M_PI / 2.0) { ang += M_PI; }
    return ang;
  }

  // -------------------------------------------------------------------------
  void update_prefilter(hw_msgs::msg::DetectedBlock & b)
  {
    /* 记忆必须做成成员而不是函数内的 static —— static 局部变量在所有实例间
     * 共享，一旦有人起两个 block_detector（比如同时看两路相机），
     * 两路的稳定度计数会互相污染。 */
    const std::size_t idx = std::min<std::size_t>(3, b.color);
    PrefilterMem & m = prefilter_mem_[idx];

    if (m.has) {
      const float d = std::hypot(b.pixel_x - m.x, b.pixel_y - m.y);
      const double dyaw = std::abs(std::remainder(b.yaw - m.yaw, M_PI));
      if (d < prefilter_radius_px_ && dyaw < (5.0 * M_PI / 180.0)) {
        m.hits++;
      } else {
        m.hits = 1;
      }
    } else {
      m.hits = 1;
    }
    m.has = true;
    m.x = b.pixel_x;
    m.y = b.pixel_y;
    m.yaw = b.yaw;

    b.stable_frames = m.hits;
    b.confidence = std::min(1.0f, static_cast<float>(m.hits) /
                                   static_cast<float>(std::max(1, prefilter_frames_)));
  }

  // -------------------------------------------------------------------------
  void draw_debug(cv::Mat & frame, const hw_msgs::msg::DetectedBlockArray & arr)
  {
    for (const auto & b : arr.blocks) {
      cv::Point2f ctr(b.pixel_x, b.pixel_y);
      const float half = std::sqrt(std::max(1.0f, b.area)) * 0.5f;
      const float dx = std::cos(static_cast<float>(b.yaw)) * half;
      const float dy = std::sin(static_cast<float>(b.yaw)) * half;
      cv::line(frame, cv::Point2f(ctr.x - dx, ctr.y - dy),
               cv::Point2f(ctr.x + dx, ctr.y + dy), cv::Scalar(0, 255, 0), 2);
      cv::circle(frame, ctr, 3, cv::Scalar(0, 0, 255), -1);
      cv::putText(frame,
                  cv::format("%.1f deg f%u", b.yaw * 180.0 / M_PI, b.stable_frames),
                  cv::Point2f(ctr.x - 40, ctr.y - 12),
                  cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
    }
  }

  // -------------------------------------------------------------------------
  /// 像素域预筛的记忆：每个颜色记上一次的位置与角度
  struct PrefilterMem
  {
    bool has{false};
    float x{0.0f};
    float y{0.0f};
    double yaw{0.0};
    uint32_t hits{0};
  };
  std::array<PrefilterMem, 4> prefilter_mem_{};

  std::string image_topic_;
  int roi_x0_{0}, roi_x1_{0}, roi_y0_{0}, roi_y1_{0};
  bool flip_horizontal_{false};
  double min_area_px_{2000.0}, max_area_px_{30000.0};
  int morph_kernel_{5};
  bool use_sharpen_{true};
  int prefilter_frames_{2};
  double prefilter_radius_px_{40.0};
  bool debug_view_{false};
  int max_blocks_{8};
  uint32_t seq_{0};

  std::vector<HsvRange> colors_;

  rclcpp::Publisher<hw_msgs::msg::DetectedBlockArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace hw_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hw_task::BlockDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
