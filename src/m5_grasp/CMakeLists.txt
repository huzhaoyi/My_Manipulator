cmake_minimum_required(VERSION 3.22)
project(m5_grasp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
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

# 创建轨迹规划库
add_library(trajectory_planner_lib
  src/trajectory_planner.cpp
)
ament_target_dependencies(trajectory_planner_lib
  geometry_msgs
  trajectory_msgs
)
target_include_directories(trajectory_planner_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 添加 m5_grasp_node
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
)
target_link_libraries(m5_grasp_node
  trajectory_planner_lib
)
target_include_directories(m5_grasp_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 安装可执行文件和库
install(TARGETS
  m5_grasp_node
  trajectory_planner_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
