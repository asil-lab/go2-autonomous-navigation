cmake_minimum_required(VERSION 3.5)
project(ltm_navigation_service)

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
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ltm_shared_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
include_directories(include)

# Add the navigation_service_node executable.
add_executable(navigation_service_node
  src/navigation_service_node.cpp
)

target_include_directories(navigation_service_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(navigation_service_node
  rclcpp
  tf2_ros
  geometry_msgs
  std_msgs
  ltm_shared_msgs
  Eigen3
)

# Add the navigation_client_node executable.
add_executable(navigation_client_node
  src/navigation_client_node.cpp
)

target_include_directories(navigation_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(navigation_client_node
  rclcpp
  geometry_msgs
  ltm_shared_msgs
  Eigen3
)

# Install executables.
install(TARGETS 
  navigation_service_node
  navigation_client_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the configuration files.
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Build the test scripts.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
