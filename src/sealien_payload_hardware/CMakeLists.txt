cmake_minimum_required(VERSION 3.8)
project(sealien_payload_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 使用nlohmann/json（单头文件库，包含在源码中）

# 包含目录
include_directories(include)

# 硬件接口插件
add_library(sealien_payload_hardware_interface SHARED
  src/sealien_payload_hardware_interface.cpp
)

ament_target_dependencies(sealien_payload_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_msgs
  sensor_msgs
  std_msgs
)

# nlohmann/json是单头文件库，不需要链接

target_include_directories(sealien_payload_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装库和头文件
install(TARGETS sealien_payload_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# 安装插件描述文件
install(FILES sealien_payload_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 注册pluginlib插件
# plugin_category应该是基类所在的包名
pluginlib_export_plugin_description_file(hardware_interface sealien_payload_hardware_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(sealien_payload_hardware_interface)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
