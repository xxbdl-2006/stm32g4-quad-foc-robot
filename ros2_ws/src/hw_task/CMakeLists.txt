cmake_minimum_required(VERSION 3.8)
project(hw_task)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hw_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
#  纯逻辑核心库
#
#  这一层**刻意不链接 rclcpp**。三个文件放在一起是因为它们共同构成
#  "把像素变成抓取动作"的全部决策逻辑，而它们都可以脱离 ROS 单独测试：
#      task_sequencer   状态机
#      block_tracker    多帧关联与目标排序
#      ground_projection 像素 -> 地面坐标
#  节点只负责把传感器数据喂进来、把动作发出去（见 task_executor_node.cpp）。
# ---------------------------------------------------------------------------
add_library(hw_task_core STATIC
  src/task_sequencer.cpp
  src/block_tracker.cpp
  src/ground_projection.cpp
)
target_include_directories(hw_task_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hw_task_core PUBLIC Eigen3::Eigen)

# ---------------------------------------------------------------------------
#  任务执行节点
# ---------------------------------------------------------------------------
add_executable(task_executor_node src/task_executor_node.cpp)
target_link_libraries(task_executor_node
  hw_task_core
  rclcpp::rclcpp
  ${hw_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

# ---------------------------------------------------------------------------
#  物块识别节点
# ---------------------------------------------------------------------------
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(block_detector_node src/block_detector_node.cpp)
target_link_libraries(block_detector_node
  rclcpp::rclcpp
  ${hw_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  ${OpenCV_LIBS}
)
target_include_directories(block_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})

install(TARGETS task_executor_node block_detector_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS hw_task_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(hw_task_core)
ament_export_targets(export_${PROJECT_NAME})

# ---------------------------------------------------------------------------
#  纯逻辑测试
#
#  用系统 Eigen 编译（目标机上一定有），跑的是与真车完全相同的那份状态机代码，
#  假的只是传感器和底盘运动学。
#
#  没装 ROS 的开发机上想单独跑，可以手工编译（详见 test/test_sequencer.cpp 头部）：
#      g++ -std=c++17 -I include -I test/host_shim test/test_sequencer.cpp \
#          src/task_sequencer.cpp src/block_tracker.cpp src/ground_projection.cpp -o /tmp/t
#  这时会用到 test/host_shim 下的极简 Eigen 垫片（只为不依赖 Eigen3 而存在）。
#  两条路径共用同一份测试源码。
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  add_executable(task_sequencer_test
    test/test_sequencer.cpp
    src/task_sequencer.cpp
    src/block_tracker.cpp
    src/ground_projection.cpp
  )
  target_include_directories(task_sequencer_test PRIVATE include)
  target_link_libraries(task_sequencer_test PRIVATE Eigen3::Eigen)

  ament_add_test(
    task_sequencer_logic
    COMMAND task_sequencer_test
    OUTPUT_FILE task_sequencer_logic.txt
    TIMEOUT 60
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
