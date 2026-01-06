cmake_minimum_required(VERSION 3.22)
project(m5_tools)

find_package(ament_cmake REQUIRED)

# 安装 scripts 目录
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

ament_package()
