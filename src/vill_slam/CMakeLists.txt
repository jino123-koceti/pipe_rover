cmake_minimum_required(VERSION 3.8)
project(vill_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vill_slam_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Try to find GTSAM (optional, will use fallback if not found)
find_package(GTSAM QUIET)
if(GTSAM_FOUND)
  message(STATUS "GTSAM found: ${GTSAM_VERSION}")
  add_definitions(-DUSE_GTSAM)
else()
  message(WARNING "GTSAM not found - using simplified factor graph implementation")
endif()

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

if(GTSAM_FOUND)
  include_directories(${GTSAM_INCLUDE_DIR})
endif()

# Common dependencies
set(COMMON_DEPS
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  vill_slam_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  visualization_msgs
)

# Cylinder Factor library
add_library(cylinder_factor SHARED
  src/cylinder_factor.cpp
)
target_include_directories(cylinder_factor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(GTSAM_FOUND)
  target_link_libraries(cylinder_factor gtsam)
endif()

# Plane Factor library
add_library(plane_factor SHARED
  src/plane_factor.cpp
)
target_include_directories(plane_factor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(GTSAM_FOUND)
  target_link_libraries(plane_factor gtsam)
endif()

# Surface Processor library
add_library(surface_processor SHARED
  src/surface_processor.cpp
)
target_include_directories(surface_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(surface_processor
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# VILL-SLAM Core Node (with loop closure + pose graph)
add_executable(vill_slam_node
  src/vill_slam_node.cpp
)
ament_target_dependencies(vill_slam_node ${COMMON_DEPS})
target_link_libraries(vill_slam_node
  cylinder_factor
  plane_factor
  surface_processor
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${PCL_REGISTRATION_LIBRARIES}
)
if(GTSAM_FOUND)
  target_link_libraries(vill_slam_node gtsam)
endif()

# Dense Mapper Node
add_executable(dense_mapper_node
  src/dense_mapper_node.cpp
)
ament_target_dependencies(dense_mapper_node ${COMMON_DEPS})
target_link_libraries(dense_mapper_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# Install libraries
install(TARGETS
  cylinder_factor
  plane_factor
  surface_processor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  vill_slam_node
  dense_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/environment_detector.py
  scripts/zenoh_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
