cmake_minimum_required(VERSION 3.22)
project(m5_planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)

# 添加 m5_planning 可执行文件
add_executable(m5_planning src/m5_planning.cpp)
ament_target_dependencies(m5_planning
  rclcpp
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
)

# 安装可执行文件
install(TARGETS
  m5_planning
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
