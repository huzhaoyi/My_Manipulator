cmake_minimum_required(VERSION 3.22)
project(m5_moveit_config)

find_package(ament_cmake REQUIRED)

# 安装 config 目录
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.srdf" PATTERN "*.rviz" PATTERN "*.xacro"
)

ament_package()
