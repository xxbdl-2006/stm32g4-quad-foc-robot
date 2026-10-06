#!/usr/bin/env bash
# ============================================================================
#  setup_can.sh —— 把 CAN 接口按本项目要求拉起来
#
#  用法：
#     sudo ./scripts/setup_can.sh              # 默认 can0 @ 1Mbps
#     sudo ./scripts/setup_can.sh can1 500000  # 换接口与波特率
#
#  为什么波特率必须显式设置：
#  SocketCAN 不会自己协商波特率，配错了在用户态是"完全收不到数据"，
#  一点错误提示都没有 —— 这是 CAN 调试最常见的第一个坑。
# ============================================================================
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-1000000}"
SAMPLE_POINT="${3:-0.800}"      # 采样点 80%，长线束下的推荐值

if [[ $EUID -ne 0 ]]; then
    echo "需要 root：sudo $0 $*" >&2
    exit 1
fi

if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "接口 $IFACE 不存在。请确认 USB-CAN 适配器已插好，且驱动已加载：" >&2
    echo "  gs_usb（CANable/cantact）：lsmod | grep gs_usb" >&2
    echo "  slcan（串口转 CAN）      ：需先 slcand -o -c -s8 /dev/ttyACM0 $IFACE" >&2
    exit 1
fi

echo "[1/3] 关闭 $IFACE"
ip link set "$IFACE" down

echo "[2/3] 设置 $IFACE 波特率 $BITRATE，采样点 $SAMPLE_POINT"
ip link set "$IFACE" type can bitrate "$BITRATE" sample-point "$SAMPLE_POINT" \
    restart-ms 100

echo "[3/3] 启动 $IFACE"
ip link set "$IFACE" up

sleep 0.3
ip -details -statistics link show "$IFACE" | head -8

echo
echo "完成。终端工具验证："
echo "  candump $IFACE,181:7FF        # 只看主板发的电机反馈帧"
echo "  python3 tools/can_monitor.py -i $IFACE"
echo
echo "提示：若报 'RTNETLINK answers: Device or resource busy'，"
echo "      说明接口已被 SocketCAN 使用中，先 kill 掉占用进程或直接跳过本脚本。"
