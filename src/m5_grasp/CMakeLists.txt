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
)

# 安装可执行文件
install(TARGETS
  m5_grasp_node
  DESTINATION lib/${PROJECT_NAME}
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
