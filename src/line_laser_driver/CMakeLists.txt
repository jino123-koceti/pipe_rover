cmake_minimum_required(VERSION 3.8)
project(line_laser_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vill_slam_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# C++ node for laser line extraction
add_executable(line_laser_processor_node
  src/line_laser_processor_node.cpp
)
ament_target_dependencies(line_laser_processor_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  vill_slam_msgs
  cv_bridge
  image_transport
)
target_link_libraries(line_laser_processor_node
  ${OpenCV_LIBS}
)

# Install C++ executables
install(TARGETS
  line_laser_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/gpio_control.py
  scripts/laser_sync_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
