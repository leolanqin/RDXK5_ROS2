cmake_minimum_required(VERSION 3.8)
project(stm32_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(stm32_serial_node src/stm32_serial_node.cpp)
add_executable(test_map_node src/test_map_node.cpp)
add_executable(map_switch_node src/map_switch_node.cpp)

ament_target_dependencies(stm32_serial_node
  rclcpp
  std_msgs
  serial
)

ament_target_dependencies(test_map_node
  rclcpp
  nav_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(map_switch_node
  rclcpp
  std_msgs
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(test_map_node
  yaml-cpp
)

target_include_directories(test_map_node PUBLIC
  include
)

install(TARGETS stm32_serial_node test_map_node map_switch_node DESTINATION lib/${PROJECT_NAME})

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
