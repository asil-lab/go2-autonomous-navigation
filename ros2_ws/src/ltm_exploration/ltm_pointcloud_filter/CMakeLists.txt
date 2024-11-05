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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
include_directories(include)

# Create voxel_grid_filter_node executable
add_executable(voxel_grid_filter_node 
  src/voxel_grid_filter_node.cpp
  src/pointcloud_filter_node.cpp
)

target_include_directories(voxel_grid_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(voxel_grid_filter_node
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
)

target_link_libraries(voxel_grid_filter_node
  ${PCL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)

# Create ground_plane_segmentation_node executable
add_executable(ground_plane_segmentation_node 
  src/ground_plane_segmentation_node.cpp
  src/pointcloud_filter_node.cpp
)

target_include_directories(ground_plane_segmentation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ground_plane_segmentation_node
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
)

target_link_libraries(ground_plane_segmentation_node
  ${PCL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)

# Create statistical_outlier_removal_node executable
add_executable(statistical_outlier_removal_node 
  src/statistical_outlier_removal_node.cpp
  src/pointcloud_filter_node.cpp
)

target_include_directories(statistical_outlier_removal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(statistical_outlier_removal_node
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
)

target_link_libraries(statistical_outlier_removal_node
  ${PC_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)

# Create crop_box_filter_node executable
add_executable(crop_box_filter_node 
  src/crop_box_filter_node.cpp
  src/pointcloud_filter_node.cpp
)

target_include_directories(crop_box_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(crop_box_filter_node
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
)

target_link_libraries(crop_box_filter_node
  ${PCL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)

# Install executables
install(TARGETS
  crop_box_filter_node
  ground_plane_segmentation_node
  statistical_outlier_removal_node
  voxel_grid_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
