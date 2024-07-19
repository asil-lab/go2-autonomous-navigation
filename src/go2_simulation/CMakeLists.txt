cmake_minimum_required(VERSION 3.5)
project(go2_simulation)

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
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${rclcpp_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY
  launch
  models
  plugin
  scripts
  world
DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/spawn_go2.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
target_link_libraries(unitreeFootContactPlugin ${rclcpp_LIBRARIES} ${GAZEBO_LIBRARIES} ${geometry_msgs_LIBRARIES})

add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
target_link_libraries(unitreeDrawForcePlugin ${rclcpp_LIBRARIES} ${GAZEBO_LIBRARIES} ${geometry_msgs_LIBRARIES})

ament_package()
