cmake_minimum_required(VERSION 3.22)
project(m5_grasp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)  # 用于时间参数化（TOTG、IterativeSpline等）
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(m5_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# spdlog日志库
find_package(spdlog REQUIRED)

# ========== 日志库（logging） ==========
add_library(m5_grasp_logging_lib
  src/logging/logger.cpp
)
target_link_libraries(m5_grasp_logging_lib
  spdlog::spdlog
)
target_include_directories(m5_grasp_logging_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ========== 工具库（utils） ==========
add_library(m5_grasp_utils_lib
  src/utils/trajectory_planner.cpp
  src/utils/scene_manager.cpp
  src/utils/parameter_manager.cpp
)
ament_target_dependencies(m5_grasp_utils_lib
  rclcpp
  moveit_ros_planning_interface
  moveit_core  # 用于时间参数化
  geometry_msgs
  std_msgs
  tf2_ros
  moveit_msgs
  shape_msgs
  trajectory_msgs
)
target_include_directories(m5_grasp_utils_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ========== MTC库（任务构建和执行） ==========
# 注意：MTC库需要先编译，因为FSM库依赖它的头文件
add_library(m5_grasp_mtc_lib
  src/mtc/task_context.cpp
  src/mtc/task_factory.cpp
  src/mtc/task_runner.cpp
)
ament_target_dependencies(m5_grasp_mtc_lib
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  trajectory_msgs
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  m5_msgs
)
target_link_libraries(m5_grasp_mtc_lib
  m5_grasp_logging_lib
)
target_include_directories(m5_grasp_mtc_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ========== FSM库（状态机） ==========
add_library(m5_grasp_fsm_lib
  src/fsm/grasp_fsm.cpp
)
ament_target_dependencies(m5_grasp_fsm_lib
  rclcpp
  rclcpp_action
  geometry_msgs
  m5_msgs
  moveit_task_constructor_msgs
)
target_link_libraries(m5_grasp_fsm_lib
  m5_grasp_logging_lib
  m5_grasp_mtc_lib
)
target_include_directories(m5_grasp_fsm_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ========== 规划库（planning） ==========
add_library(m5_grasp_planning_lib
  src/planning/kinematics_utils.cpp
  src/planning/mtc_task_creator.cpp
)
ament_target_dependencies(m5_grasp_planning_lib
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2_ros
  moveit_msgs
  trajectory_msgs
  moveit_task_constructor_core
  moveit_task_constructor_msgs
)
target_link_libraries(m5_grasp_planning_lib
  m5_grasp_utils_lib
)
target_include_directories(m5_grasp_planning_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ========== 执行库（execution） ==========
add_library(m5_grasp_execution_lib
  src/execution/gripper_controller.cpp
  src/execution/motion_executor.cpp
  src/execution/cartesian_executor.cpp
)
ament_target_dependencies(m5_grasp_execution_lib
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2_ros
  moveit_msgs
  control_msgs
  trajectory_msgs
  m5_msgs
)
target_link_libraries(m5_grasp_execution_lib
  m5_grasp_utils_lib
  m5_grasp_mtc_lib
)
target_include_directories(m5_grasp_execution_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ========== 可视化库（visualization） ==========
add_library(m5_grasp_visualization_lib
  src/visualization/web_visualizer.cpp
)
ament_target_dependencies(m5_grasp_visualization_lib
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  trajectory_msgs
)
target_link_libraries(m5_grasp_visualization_lib
  m5_grasp_logging_lib
)
target_include_directories(m5_grasp_visualization_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ========== 主节点（模块化版本）==========
add_executable(m5_grasp_node src/m5_grasp.cpp)
ament_target_dependencies(m5_grasp_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2_ros
  moveit_msgs
  control_msgs
  shape_msgs
  trajectory_msgs
  m5_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  moveit_task_constructor_core
  moveit_task_constructor_msgs
)
target_link_libraries(m5_grasp_node
  m5_grasp_logging_lib
  m5_grasp_utils_lib
  m5_grasp_fsm_lib
  m5_grasp_mtc_lib
  m5_grasp_planning_lib
  m5_grasp_execution_lib
  m5_grasp_visualization_lib
)
target_include_directories(m5_grasp_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


# ========== 安装 ==========
# 安装可执行文件
install(TARGETS
  m5_grasp_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装库文件
install(TARGETS
  m5_grasp_logging_lib
  m5_grasp_utils_lib
  m5_grasp_fsm_lib
  m5_grasp_mtc_lib
  m5_grasp_planning_lib
  m5_grasp_execution_lib
  m5_grasp_visualization_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装 config 目录
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
