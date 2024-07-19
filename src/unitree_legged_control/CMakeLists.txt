cmake_minimum_required(VERSION 3.5)
project(unitree_legged_control)

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
find_package(urdf REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(unitree_legged_msgs REQUIRED)

include_directories(include ${colcon_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})


add_library(unitree_legged_control 
    src/joint_controller.cpp
    src/unitree_joint_control_tool.cpp
)
ament_target_dependencies(unitree_legged_control 
    urdf
    controller_interface
    control_toolbox
    hardware_interface
    pluginlib
    rclcpp
    realtime_tools
    unitree_legged_msgs
)
target_link_libraries(unitree_legged_control ${colcon_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
