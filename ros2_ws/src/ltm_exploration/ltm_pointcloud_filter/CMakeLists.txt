cmake_minimum_required(VERSION 3.5)
project(ltm_pointcloud_filter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(assimp REQUIRED)
find_package(message_filters REQUIRED)
find_package(unitree_go REQUIRED)
find_package(PCL REQUIRED)
include_directories(include)

# Create pointcloud_filter_node executable
add_executable(pointcloud_filter_node 
  src/pointcloud_filter_node.cpp
  src/ground_plane_removal.cpp
  src/robot_cluster_removal.cpp
  src/voxel_grid_filter.cpp
)

target_include_directories(pointcloud_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pointcloud_filter_node
  rclcpp
  ament_index_cpp
  tf2_ros
  tf2
  tf2_eigen
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
  vision_msgs
  geometry_msgs
  pcl_ros
  urdf
  assimp
  message_filters
  unitree_go
)

target_link_libraries(pointcloud_filter_node
  ${PCL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)

# Install executables
install(TARGETS pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install urdf files.
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install mesh files.
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
