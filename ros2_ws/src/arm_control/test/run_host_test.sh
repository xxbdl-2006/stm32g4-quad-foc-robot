#!/usr/bin/env bash
# ============================================================================
#  run_host_test.sh
#  在没有 ROS 的开发机上直接跑 arm_control 的运动学回归测试。
#
#  这个模块刻意不依赖 Eigen、不依赖 rclcpp，所以这里只需要一个 g++：
#  没有任何第三方头文件路径要配，也没有包管理器要跑。这也是当初选择
#  "自己写 4x4 齐次变换"而不是引入 Eigen 的直接好处。
#
#  在装了 ROS 的机器上不需要这个脚本 —— 直接 colcon test 走 CMakeLists
#  里 ament_add_test 那条路即可（同一份源码，两条编译路径）。
# ============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

CXX="${CXX:-g++}"
OUT="${OUT:-/tmp/arm_kinematics_test}"

echo "[1/2] 编译（$($CXX --version | head -1)）"
$CXX -std=c++17 -O1 -Wall -Wextra \
    -I "$ROOT/include" \
    "$HERE/test_arm_kinematics.cpp" \
    "$ROOT/src/dh.cpp" \
    "$ROOT/src/scara_ik.cpp" \
    "$ROOT/src/trajectory.cpp" \
    -o "$OUT"

echo "[2/2] 运行"
"$OUT"
