cmake_minimum_required(VERSION 3.22)
project(m5_configure)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 添加可执行文件
add_executable(m5_planning_node src/m5_planning_node.cpp)
ament_target_dependencies(m5_planning_node
  rclcpp
  moveit_ros_planning_interface
)

# 添加 joint_state_publisher_node
add_executable(joint_state_publisher_node src/joint_state_publisher_node.cpp)
ament_target_dependencies(joint_state_publisher_node
  rclcpp
  sensor_msgs
)

# 添加 demo_moveit
add_executable(demo_moveit src/demo_moveit.cpp)
ament_target_dependencies(demo_moveit
  rclcpp
  moveit_ros_planning_interface
)

# 安装可执行文件
install(TARGETS
  m5_planning_node
  joint_state_publisher_node
  demo_moveit
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 config 和 launch 目录
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

# 安装 scripts 目录（Python脚本）
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# 安装 Python 脚本为可执行文件（使 ros2 run 可用）
install(PROGRAMS
  scripts/publish_target_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装其他文件
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
  install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
