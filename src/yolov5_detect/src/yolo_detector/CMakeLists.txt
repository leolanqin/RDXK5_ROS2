cmake_minimum_required(VERSION 3.8)
project(yolo_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yolo_msgs REQUIRED)

# 设置RDK_BPU库路径
set(DNN_PATH "/usr/include/dnn")      # BPU头文件路径
set(DNN_LIB_PATH "/usr/lib/")         # BPU库文件路径

add_executable(former_yolo_node src/former_yolo_node.cpp src/bpu_detect.cpp)
add_executable(back_yolo_node src/back_yolo_node.cpp src/bpu_detect.cpp)
target_include_directories(former_yolo_node 
  PUBLIC
    include
    ${DNN_PATH}             # BPU头文件路径
)

target_include_directories(back_yolo_node 
  PUBLIC
    include
    ${DNN_PATH}             # BPU头文件路径
)

target_link_libraries(former_yolo_node 
    dnn 
    pthread            # 线程库
    rt                 # 实时库
    dl                 # 动态链接库
)

target_link_libraries(back_yolo_node 
    dnn 
    pthread            # 线程库
    rt                 # 实时库
    dl                 # 动态链接库
)

ament_target_dependencies(former_yolo_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV yolo_msgs)
ament_target_dependencies(back_yolo_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV yolo_msgs)

install(TARGETS former_yolo_node back_yolo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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
