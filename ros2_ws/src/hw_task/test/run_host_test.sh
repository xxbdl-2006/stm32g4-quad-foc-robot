#!/usr/bin/env bash
# ============================================================================
#  run_host_test.sh
#  在没有 ROS 的开发机上直接跑 hw_task 的纯逻辑测试。
#
#  用 test/host_shim 下的极简 Eigen 垫片代替 Eigen3。
#  在装了 ROS 的机器上不需要这个脚本 —— 直接 colcon test 就行
#  （走 CMakeLists 里 ament_add_test 那条路，用真正的 Eigen3）。
# ============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

CXX="${CXX:-g++}"
OUT="${OUT:-/tmp/hw_task_sequencer_test}"

echo "[1/2] 编译（$($CXX --version | head -1)）"
$CXX -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter \
    -I "$ROOT/include" -I "$HERE/host_shim" \
    "$HERE/test_sequencer.cpp" \
    "$ROOT/src/task_sequencer.cpp" \
    "$ROOT/src/block_tracker.cpp" \
    "$ROOT/src/ground_projection.cpp" \
    -o "$OUT"

echo "[2/2] 运行"
"$OUT"
