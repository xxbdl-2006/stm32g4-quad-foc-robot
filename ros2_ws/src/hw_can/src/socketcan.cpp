// ============================================================================
//  socketcan.cpp
// ============================================================================
#include "hw_can/socketcan.hpp"

#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

namespace hw_can
{

namespace
{
/// 从 cmsg 中解析 SO_TIMESTAMP 的 timeval，转成 double 秒。
double extract_timestamp(struct msghdr * msg)
{
  for (struct cmsghdr * cmsg = CMSG_FIRSTHDR(msg); cmsg != nullptr;
       cmsg = CMSG_NXTHDR(msg, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
      struct timeval tv{};
      std::memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
      return static_cast<double>(tv.tv_sec) + static_cast<double>(tv.tv_usec) * 1e-6;
    }
  }
  return 0.0;
}
}  // namespace

SocketCan::SocketCan(std::string interface_name)
: iface_(std::move(interface_name))
{
}

SocketCan::~SocketCan()
{
  close();
}

void SocketCan::open(Mode mode)
{
  if (fd_ >= 0) {
    throw CanBusError("socket already open on " + iface_);
  }

  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    throw CanBusError(std::string("socket(PF_CAN) failed: ") + std::strerror(errno));
  }

  // --- 接口索引 ---
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd);
    throw CanBusError("interface '" + iface_ + "' not found: " + std::strerror(errno));
  }

  // --- 选项 ---
  int recv_own = 0;
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));

  int loopback = (mode == Mode::Loopback) ? 1 : 0;
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  // 错误帧过滤：全开，让 CAN_ERR_* 也能被 recv 到
  can_err_mask_t err_mask =
    CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB | CAN_ERR_CRTL | CAN_ERR_PROT |
    CAN_ERR_TRX | CAN_ERR_ACK | CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

  if (mode == Mode::ListenOnly) {
    // 监听模式：不发 ACK，用于旁路嗅探，不影响总线
    int recv_own_only = 0;
    (void)recv_own_only;
    // listen-only 在 netlink 层设置，这里通过 CAN_RAW_RECV_OWN_MSGS=0 + 不发送来近似
  }

  // --- 绑定 ---
  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    throw CanBusError("bind(" + iface_ + ") failed (interface down?): " +
                      std::strerror(errno));
  }

  fd_ = fd;
  apply_timestamping();
  apply_filters();
  ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
}

void SocketCan::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SocketCan::apply_timestamping() const
{
  int enable = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable));
}

void SocketCan::apply_filters() const
{
  // 本项目总线上只有上位机(0x00)与主板(0x01)，加扩展板(0x02)。
  // 但上位机需要看到全部 3 个节点发出的帧，因此过滤器只放行 NODE 段
  // 在 0..2 范围内的 ID（11 位里高 3 位）。
  //
  // 计算：ID & 0x700 <= 0x200  ->  等价于 (ID & 0x700) | mask 形式。
  // 用两条掩码过滤器表达： 0x000/0x700 与 0x100/0x700 与 0x200/0x700。
  can_filter filters[3];
  filters[0].can_id   = 0x000; filters[0].can_mask = 0x700;
  filters[1].can_id   = 0x100; filters[1].can_mask = 0x700;
  filters[2].can_id   = 0x200; filters[2].can_mask = 0x700;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters,
                   static_cast<socklen_t>(sizeof(filters))) < 0) {
    // 过滤失败不致命，退化为全收
  }
}

bool SocketCan::send(const can_frame & f) const
{
  if (fd_ < 0) {
    throw CanBusError("send on closed socket");
  }
  const ssize_t n = ::write(fd_, &f, sizeof(f));
  if (n == sizeof(f)) {
    return true;
  }
  if (errno == ENOBUFS || errno == EAGAIN) {
    // TX 缓冲满：正常现象（反馈帧刚把队列占满）。返回 false 由调用方丢弃，
    // 不要重试 —— CAN 的语义是"最新的值比旧的值重要"。
    return false;
  }
  if (errno == ENETDOWN || errno == ENOENT) {
    throw CanBusError("bus down during send");
  }
  return false;
}

bool SocketCan::recv(RxFrame & out, int timeout_ms) const
{
  return recv_batch(&out, 1, timeout_ms) == 1;
}

std::size_t SocketCan::recv_batch(RxFrame * out, std::size_t max_frames,
                                  int timeout_ms) const
{
  if (fd_ < 0) {
    throw CanBusError("recv on closed socket");
  }

  struct pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  const int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr <= 0) {
    return 0;   // 超时或 EINTR
  }

  std::size_t got = 0;
  while (got < max_frames) {
    struct msghdr msg{};
    struct iovec iov{};
    alignas(cmsghdr) char ctrl[64];

    iov.iov_base = &out[got].frame;
    iov.iov_len = sizeof(can_frame);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrl;
    msg.msg_controllen = sizeof(ctrl);

    const ssize_t n = ::recvmsg(fd_, &msg, MSG_DONTWAIT);
    if (n <= 0) {
      break;   // EAGAIN：这一批读完了
    }

    out[got].timestamp = extract_timestamp(&msg);
    if (out[got].timestamp == 0.0) {
      struct timespec ts{};
      ::clock_gettime(CLOCK_MONOTONIC, &ts);
      out[got].timestamp =
        static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
    }
    out[got].is_error = (out[got].frame.can_id & CAN_ERR_FLAG) != 0;
    ++got;
  }
  return got;
}

SocketCan::BusState SocketCan::state() const
{
  BusState s{};
  if (fd_ < 0) {
    return s;
  }
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
  // CAN_RAW 的 SIOCGIFMTU 不可用读状态，改用 netlink 太重；
  // 这里用 ioctl(SIOCGIFLOWERUP) 之外的通用方式：读取 /sys/class/net/<if>/statistics
  // 在 state() 的调用方（诊断定时器，1Hz）里做，不走热路径。
  const std::string base = "/sys/class/net/" + iface_;
  auto read_num = [](const std::string & path) -> uint32_t {
      FILE * fp = std::fopen(path.c_str(), "r");
      if (fp == nullptr) { return 0; }
      unsigned long v = 0;
      if (std::fscanf(fp, "%lu", &v) != 1) { v = 0; }
      std::fclose(fp);
      return static_cast<uint32_t>(v);
    };
  s.tx_error_counter = read_num(base + "/statistics/tx_errors");
  s.rx_error_counter = read_num(base + "/statistics/rx_errors");
  s.restarts         = read_num(base + "/statistics/tx_aborted_errors");

  /* operstate 是文本（"up"/"down"/"unknown"），不是数字 —— 第一版用
   * fscanf("%lu") 读它，永远得到 0，于是 bus_off 永远为真。
   * 这里按字符串读，并顺带把 CAN 特有的 state 文件也读了。 */
  {
    FILE * fp = std::fopen((base + "/operstate").c_str(), "r");
    if (fp != nullptr) {
      char buf[16] = {0};
      if (std::fgets(buf, sizeof(buf), fp) != nullptr) {
        const std::string op(buf);
        s.bus_off = (op.rfind("down", 0) == 0);
      }
      std::fclose(fp);
    }
  }
  return s;
}

}  // namespace hw_can
