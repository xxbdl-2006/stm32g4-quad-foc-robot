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
# 运动学的唯一来源：状态机下达的是末端位姿，要先把"这个位姿可不可达"算出来。
# 测试里也用同一份源码，所以不能是可选依赖。
find_package(arm_control REQUIRED)

# ---------------------------------------------------------------------------
#  纯逻辑核心库
#
#  这一层**刻意不链接 rclcpp**。三个文件放在一起是因为它们共同构成
#  "把像素变成抓取动作"的全部决策逻辑，而它们都可以脱离 ROS 单独测试：
#      task_sequencer    状态机（输出的是末端目标位姿）
#      block_tracker     多帧关联与目标排序
#      table_projection  像素 -> 工作台面坐标
#  节点只负责把传感器数据喂进来、把动作发出去（见 task_executor_node.cpp）。
# ---------------------------------------------------------------------------
add_library(hw_task_core STATIC
  src/task_sequencer.cpp
  src/block_tracker.cpp
  src/table_projection.cpp
)
target_include_directories(hw_task_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hw_task_core PUBLIC Eigen3::Eigen arm_control::arm_control_core)

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
#  用系统 Eigen 编译（目标机上一定有），跑的是与真机完全相同的那份状态机代码，
#  假的只有传感器、气泵与机械臂的关节伺服（机械臂的**运动学是真的** ——
#  仿真器直接调用 arm_control 的 D-H 正逆解，见 test/test_sequencer.cpp）。
#
#  没装 ROS 的开发机上想单独跑，直接执行 test/run_host_test.sh：
#  它会用 test/host_shim 下的极简 Eigen 垫片代替 Eigen3，
#  并一并编译 arm_control 的运动学源码。两条路径共用同一份测试源码。
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  add_executable(task_sequencer_test
    test/test_sequencer.cpp
    src/task_sequencer.cpp
    src/block_tracker.cpp
    src/table_projection.cpp
  )
  target_include_directories(task_sequencer_test PRIVATE include)
  target_link_libraries(task_sequencer_test PRIVATE Eigen3::Eigen arm_control::arm_control_core)

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
