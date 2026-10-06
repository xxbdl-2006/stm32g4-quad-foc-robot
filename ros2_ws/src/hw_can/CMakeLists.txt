cmake_minimum_required(VERSION 3.8)
project(hw_can)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hw_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(hw_can_core SHARED
  src/socketcan.cpp
  src/protocol.cpp
)
target_include_directories(hw_can_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hw_can_core PUBLIC
  rclcpp::rclcpp
  ${hw_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
)
# SocketCAN 头文件来自内核 uapi，不需要额外链接库。
# 注意不要 link stdc++fs —— 那是 GCC 8 时代的产物，Ubuntu 22.04 的
# GCC 11 已经把它合进 libstdc++，显式 link 会直接报 "cannot find -lstdc++fs"。

add_executable(can_bridge_node src/can_bridge_node.cpp)
target_include_directories(can_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(can_bridge_node PRIVATE hw_can_core)

# ---- 离线工具：把一段 candump 日志回放成 ROS 话题（无需真实硬件） ----
add_executable(can_replay tools/can_replay.cpp)
target_link_libraries(can_replay PRIVATE hw_can_core)
install(TARGETS can_replay DESTINATION lib/${PROJECT_NAME})

install(TARGETS hw_can_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS can_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(hw_can_core)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
