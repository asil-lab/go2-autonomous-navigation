cmake_minimum_required(VERSION 3.5)
project(ltm_go2_driver)

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
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(urdf REQUIRED)
find_package(ltm_go2_description REQUIRED)
find_package(ltm_shared_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# Create executable
add_executable(go2_driver_node 
  src/go2_driver_node.cpp 
  src/battery_management_processing.cpp
  src/foot_contact_processing.cpp
  src/gesture_processing.cpp
  src/joint_state_processing.cpp
  src/odom_processing.cpp 
  src/point_cloud_processing.cpp
  src/wireless_controller_processing.cpp
)
target_include_directories(go2_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(go2_driver_node
  rclcpp
  ament_index_cpp
  std_msgs
  geometry_msgs
  sensor_msgs
  ltm_shared_msgs
  unitree_api
  unitree_go
  nlohmann_json
  urdf
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# Install executable
install(TARGETS go2_driver_node
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
