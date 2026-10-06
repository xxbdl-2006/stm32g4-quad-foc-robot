#!/usr/bin/env bash
# ============================================================================
#  build.sh —— 一键构建整个工作空间
#
#  用法：
#     ./scripts/build.sh            # 常规构建
#     ./scripts/build.sh clean      # 清掉 build/install/log 后重建
#     ./scripts/build.sh test       # 构建 + 跑测试
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"
MODE="${1:-build}"

if [[ ! -d /opt/ros/humble ]]; then
    echo "未找到 /opt/ros/humble。若用的是其他 ROS2 发行版，请改这一行。" >&2
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [[ "$MODE" == "clean" ]]; then
    echo "清理 build/ install/ log/ …"
    rm -rf "$WS/build" "$WS/install" "$WS/log"
fi

cd "$WS"
echo "构建中（首次约 3~6 分钟，之后增量 10~30 秒）…"
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers "$(nproc)" \
    --event-handlers console_cohesion+

echo
echo "构建完成。记得每次新开终端都要："
echo "    source /opt/ros/humble/setup.bash"
echo "    source $WS/install/setup.bash"

if [[ "$MODE" == "test" ]]; then
    echo
    echo "运行测试…"
    colcon test --event-handlers console_cohesion+
    colcon test-result --verbose
fi
