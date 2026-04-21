cmake_minimum_required(VERSION 3.8)
project(xmdog_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# interface 包不应查找 rclpy，也不需其它运行时依赖在这里查找
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDistance.srv"
  "srv/GetPath.srv"
  "srv/GetState.srv"
  "srv/GetTarget.srv"
  "srv/SetVisionMode.srv"
  "msg/VisionState.msg"
  DEPENDENCIES std_msgs
)

# 导出运行时依赖，供其它包使用该接口
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()