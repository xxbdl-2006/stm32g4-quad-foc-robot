#!/usr/bin/env bash
# ============================================================================
#  run_host_test.sh
#  在没有 ROS 的开发机上直接跑 hw_task 的纯逻辑测试。
#
#  这里同时编译 arm_control 的运动学源码 —— 测试要的不是"另一套简化运动学"，
#  而是控制器真正用的那一份。这样"虚实联调"才有意义：如果 D-H 表改了而
#  测试还过，说明改动是自洽的；如果测试挂了，就是真出了问题。
#
#  Eigen：主机上通常没有 Eigen3，用 test/host_shim 下的极简垫片代替
#  （只覆盖 block_tracker 用到的 Vector2d）。arm_control 那边完全不依赖
#  Eigen。在装了 ROS 的机器上不需要这个脚本，直接 colcon test 即可。
# ============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
ARM="$(cd "$ROOT/../arm_control" && pwd)"

CXX="${CXX:-g++}"
OUT="${OUT:-/tmp/hw_task_sequencer_test}"

echo "[1/2] 编译（$($CXX --version | head -1)）"
$CXX -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter \
    -I "$ROOT/include" \
    -I "$ARM/include" \
    -I "$HERE/host_shim" \
    "$HERE/test_sequencer.cpp" \
    "$ROOT/src/task_sequencer.cpp" \
    "$ROOT/src/block_tracker.cpp" \
    "$ROOT/src/table_projection.cpp" \
    "$ARM/src/dh.cpp" \
    "$ARM/src/scara_ik.cpp" \
    "$ARM/src/trajectory.cpp" \
    -o "$OUT"

echo "[2/2] 运行"
"$OUT"
