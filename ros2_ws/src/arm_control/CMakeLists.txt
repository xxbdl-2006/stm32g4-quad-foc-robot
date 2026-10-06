cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hw_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ---------------------------------------------------------------------------
#  运动学核心：D-H 正解、闭式逆解、轨迹规划、电流分配
#
#  刻意做成独立的库，而且**不依赖 rclcpp、不依赖 Eigen**：
#   · 不依赖 rclcpp —— 主机侧回归测试不用起 ROS（test/run_host_test.sh）；
#   · 不依赖 Eigen —— 只用到 4x4 齐次变换，自己写反而更可控，
#     也省掉了"主机上没装 Eigen 就得准备垫片"这件事。
#  这一层的正确性完全靠 test/test_arm_kinematics.cpp 兜住。
# ---------------------------------------------------------------------------
add_library(arm_control_core
  src/dh.cpp
  src/scara_ik.cpp
  src/trajectory.cpp
)
target_include_directories(arm_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
#  节点
# ---------------------------------------------------------------------------
add_executable(arm_control_node src/arm_control_node.cpp)
target_link_libraries(arm_control_node arm_control_core)
ament_target_dependencies(arm_control_node
  rclcpp hw_msgs geometry_msgs sensor_msgs std_srvs)

install(TARGETS arm_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# ---------------------------------------------------------------------------
#  测试
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # 运动学与轨迹规划的逻辑测试。目标机上走这条路径（只需要编译器），
  # 无 ROS 的开发机上走 test/run_host_test.sh（同一份源码）。
  ament_add_gtest(test_arm_kinematics test/test_arm_kinematics.cpp)
  if(TARGET test_arm_kinematics)
    target_link_libraries(test_arm_kinematics arm_control_core)
  endif()
endif()

ament_package()
