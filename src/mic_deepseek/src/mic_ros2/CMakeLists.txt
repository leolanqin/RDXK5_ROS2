cmake_minimum_required(VERSION 3.8)
project(mic_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

link_directories(
  lib/arm64
)
#ament_index_register_resource("ros1_bridge_foreign_mapping")
install(DIRECTORY launch config feedback_voice
  DESTINATION share/${PROJECT_NAME})

# find dependencies
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(mic_msg REQUIRED)
find_package(turn_on_wheeltec_robot REQUIRED)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(azhsxf_mic src/mic.cpp src/ros2_service.cpp)
ament_target_dependencies(azhsxf_mic rclcpp std_msgs serial mic_msg)
target_link_libraries(azhsxf_mic -ljsoncpp)

install(TARGETS
  azhsxf_mic
  RUNTIME DESTINATION lib/${PROJECT_NAME}    
)

ament_package()
