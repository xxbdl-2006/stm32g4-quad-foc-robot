// ============================================================================
//  socketcan.hpp
//  Linux SocketCAN 的薄封装：打开/关闭、发送、超时接收、总线错误捕获。
//
//  设计要点
//  ---------
//  * 用 SOCK_RAW + CAN_RAW 而不是 slcan/字符设备：内核直接管理 CAN 控制器，
//    收发走 socket 缓冲，无需自己做串口帧解析，也不会因为串口阻塞丢帧。
//  * 打开 CAN_RAW_ERR_FILTER，让错误帧（CAN_ERR_FLAG）也进接收队列 ——
//    否则总线 off / ACK 错误这类问题在用户态完全看不见，只能靠"数据没来"猜。
//  * 关闭 CAN_RAW_LOOPBACK 和 CAN_RAW_RECV_OWN_MSGS：单机测试时不需要回环，
//    开着会让节点收到自己发的帧，造成反馈环。
//  * 时间戳用 SO_TIMESTAMP 由内核打，而不是 recv 返回后调 clock_gettime()。
//    两者差在几十微秒量级，做图像-位姿对齐时这几十微秒是有意义的。
// ============================================================================
#ifndef HW_CAN__SOCKETCAN_HPP_
#define HW_CAN__SOCKETCAN_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstdint>
#include <string>
#include <functional>
#include <stdexcept>

namespace hw_can
{

class CanBusError : public std::runtime_error
{
public:
  explicit CanBusError(const std::string & what) : std::runtime_error(what) {}
};

struct RxFrame
{
  can_frame frame{};
  /// 内核打的时间戳，单位秒（SEGMENT 形式已归一化）
  double timestamp{0.0};
  /// 是否为总线错误帧（CAN_ERR_FLAG）
  bool is_error{false};
};

class SocketCan
{
public:
  enum class Mode { Normal, ListenOnly, Loopback };

  explicit SocketCan(std::string interface_name);
  ~SocketCan();

  SocketCan(const SocketCan &) = delete;
  SocketCan & operator=(const SocketCan &) = delete;

  /// 打开并绑定接口。接口必须已由 `ip link set can0 up` 拉起。
  /// @throws CanBusError 接口不存在 / 权限不足 / 未 up
  void open(Mode mode = Mode::Normal);
  void close();
  bool is_open() const { return fd_ >= 0; }

  /// 阻塞发送一帧（带超时由内核 SO_SNDTIMEO 控制）。
  /// @return false 表示 TX 缓冲已满（正常现象，调用方应丢弃本帧而不是重试）
  bool send(const can_frame & f) const;

  /// 接收一帧，超时 timeout_ms 毫秒。
  /// @return true 收到帧（含错误帧，见 out.is_error）；false 超时
  bool recv(RxFrame & out, int timeout_ms) const;

  /// 批量接收，最多 max_frames 帧，超时 timeout_ms。
  /// 返回收到的帧数。用于桥节点的高吞吐循环。
  std::size_t recv_batch(RxFrame * out, std::size_t max_frames, int timeout_ms) const;

  /// 读取接口状态（bus-off / error-passive / 各类错误计数）。
  struct BusState
  {
    bool bus_off{false};
    bool error_passive{false};
    bool error_warning{false};
    uint32_t tx_error_counter{0};
    uint32_t rx_error_counter{0};
    uint32_t restarts{0};
  };
  BusState state() const;

  const std::string & interface_name() const { return iface_; }

private:
  void apply_filters() const;
  void apply_timestamping() const;

  std::string iface_;
  int fd_{-1};
};

}  // namespace hw_can

#endif  // HW_CAN__SOCKETCAN_HPP_
