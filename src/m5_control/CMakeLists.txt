cmake_minimum_required(VERSION 3.22)
project(m5_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 添加 joint_state_publisher_node
add_executable(joint_state_publisher_node src/joint_state_publisher_node.cpp)
ament_target_dependencies(joint_state_publisher_node
  rclcpp
  sensor_msgs
)

# 安装可执行文件
install(TARGETS
  joint_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
