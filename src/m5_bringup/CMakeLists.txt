cmake_minimum_required(VERSION 3.22)
project(m5_bringup)

find_package(ament_cmake REQUIRED)

# 安装 launch 目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
