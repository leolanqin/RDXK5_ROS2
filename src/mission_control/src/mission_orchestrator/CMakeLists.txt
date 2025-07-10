cmake_minimum_required(VERSION 3.8)
project(mission_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(elevator_interfaces REQUIRED)
find_package(deepseek_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(mission_orchestrator_node
  src/mission_orchestrator_node.cpp
  src/room_location_loader.cpp
)

ament_target_dependencies(mission_orchestrator_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav2_msgs
  rclcpp_action
  yolo_msgs
  deepseek_interfaces
  elevator_interfaces
)


target_include_directories(mission_orchestrator_node PUBLIC
  include
)

target_link_libraries(mission_orchestrator_node
  yaml-cpp
)


install(TARGETS mission_orchestrator_node  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
DESTINATION share/${PROJECT_NAME})

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
