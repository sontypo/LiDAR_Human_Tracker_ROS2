cmake_minimum_required(VERSION 3.8)
project(human_tracker_ros2)

# Set CMake policy for find_package
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation filters features io)
find_package(Eigen3 REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedHuman.msg"
  "msg/TrackedHuman.msg"
  "msg/TrackedHumans.msg"
  "msg/GroupState.msg"
  "msg/GroupStates.msg"
  "msg/GroupStateArray.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs visualization_msgs
)

# Create the ament index for the generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add link directories
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create detector library
add_library(detector_lib
  src/detector/lidar_detector.cpp
)
target_link_libraries(detector_lib
  ${PCL_LIBRARIES}
)
ament_target_dependencies(detector_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)
target_link_libraries(detector_lib "${cpp_typesupport_target}")

# Create tracker library
add_library(tracker_lib
  src/tracker/kalman_filter.cpp
  src/tracker/multi_target_tracker.cpp
)
ament_target_dependencies(tracker_lib
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
)
target_link_libraries(tracker_lib "${cpp_typesupport_target}")

# Create group clustering library
add_library(group_cluster_lib
  src/clustering/group_cluster.cpp
)
ament_target_dependencies(group_cluster_lib
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
)
target_link_libraries(group_cluster_lib "${cpp_typesupport_target}")

# Create detector node executable
add_executable(detector_node src/nodes/detector_node.cpp)
target_link_libraries(detector_node 
  detector_lib
  ${PCL_LIBRARIES}
)
ament_target_dependencies(detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)

# Create tracker node executable
add_executable(tracker_node src/nodes/tracker_node.cpp)
target_link_libraries(tracker_node tracker_lib)
ament_target_dependencies(tracker_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

# Create group cluster node executable
add_executable(group_cluster_node src/nodes/group_cluster_node.cpp)
target_link_libraries(group_cluster_node group_cluster_lib)
ament_target_dependencies(group_cluster_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

# Create test pointcloud publisher (for testing colored pointclouds)
add_executable(test_colored_pointcloud src/test/test_colored_pointcloud.cpp)
target_link_libraries(test_colored_pointcloud 
  ${PCL_LIBRARIES}
)
ament_target_dependencies(test_colored_pointcloud
  rclcpp
  sensor_msgs
  pcl_conversions
)

# Install executables
install(TARGETS
  detector_node
  tracker_node
  group_cluster_node
  test_colored_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  detector_lib
  tracker_lib
  group_cluster_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install directories
install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export package dependencies
ament_export_include_directories(include)
ament_export_libraries(detector_lib tracker_lib group_cluster_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)

ament_package()
