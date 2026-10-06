#!/usr/bin/env bash
# ============================================================================
#  flash.sh —— 构建固件并烧录到驱动板
#
#  用法：
#     ./scripts/flash.sh                    # 构建 + 用 ST-Link 烧录
#     ./scripts/flash.sh --build-only        # 只构建
#     ./scripts/flash.sh --dfu               # 走 DFU 模式烧录（无 ST-Link 时）
#
#  烧录前务必确认：电机母线已断电，或至少电机处于可自由转动状态。
#  烧录过程中若驱动板正在驱动电机，复位瞬间的引脚状态不可控。
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FW="$ROOT/firmware"
BUILD="$FW/build"
MODE="${1:-flash}"

TOOLCHAIN="$ROOT/cmake/gcc-arm-none-eabi.cmake"

echo "== 构建固件 =="
cmake -S "$FW" -B "$BUILD" -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD"

echo
arm-none-eabi-size "$BUILD/hwb_mc4g4.elf"

if [[ "$MODE" == "--build-only" ]]; then
    echo
    echo "仅构建完成：$BUILD/hwb_mc4g4.elf / .hex / .bin"
    exit 0
fi

if [[ "$MODE" == "--dfu" ]]; then
    echo "== DFU 烧录 =="
    # 进入 DFU：按住 BOOT0 再复位；或用 1200bps 触碰法
    dfu-util -a 0 -s 0x08000000:leave -D "$BUILD/hwb_mc4g4.bin"
else
    echo "== ST-Link 烧录 =="
    # -rst 让写完自动复位；-v 校验
    st-flash --reset --format ihex write "$BUILD/hwb_mc4g4.hex"
fi

echo
echo "烧录完成。上电后请检查："
echo "  1. 运行灯 1Hz 闪烁（说明主循环在跑，没卡在 HardFault）"
echo "  2. candump can0,001:7FF 能看到 10Hz 心跳帧"
echo "  3. 首次上电必须做电角度标定：ros2 service call /hw/calibrate_motor ..."
