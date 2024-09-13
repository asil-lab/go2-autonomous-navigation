cmake_minimum_required(VERSION 3.5)
project(ltm_go2_camera)

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

# Additional cmake module path
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(unitree_sdk2 REQUIRED)
include_directories(include)

# Create executable
add_executable(go2_camera_node src/go2_camera_node.cpp)
target_include_directories(go2_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  go2_camera_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  unitree_sdk2
)

#  Install the executable
install(TARGETS go2_camera_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
