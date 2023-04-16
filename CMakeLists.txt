cmake_minimum_required(VERSION 3.8)
project(nhk_video2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pedantic-errors -std=c++20 -fstrict-aliasing -fconcepts-diagnostics-depth=3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(can_plugins2 REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include
  "/home/stew/stew_programming/Library"  #自分の環境でblazeやCRSLibtmp, BLAS系ライブラリを入れたディレクトリのパスに変更すること
)

add_library(omni4 SHARED
  src/omni4_node_register.cpp
)
rclcpp_components_register_nodes(omni4 "NhkVideo2::Omni4Node")
ament_target_dependencies(omni4
  rclcpp
  rclcpp_components
  geometry_msgs
  can_plugins2
)

add_library(test_commander SHARED
  src/test_commander_node_register.cpp
)
rclcpp_components_register_nodes(test_commander "NhkVideo2::TestCommanderNode")
ament_target_dependencies(test_commander
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  can_plugins2
)

add_executable(all_node
  src/all_node.cpp
)
ament_target_dependencies(all_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  can_plugins2
)

install(
  TARGETS omni4 test_commander
  LIBRARY DESTINATION lib
)

install(
  TARGETS all_node
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
