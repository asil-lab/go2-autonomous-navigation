cmake_minimum_required(VERSION 3.5)
project(ltm_go2_rviz_plugins)

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
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ltm_shared_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/battery_display.cpp
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  rclcpp
  ltm_shared_msgs
  rviz_common::rviz_common
  # rviz_ogre_vendor::rviz_ogre_vendor
  # rviz_rendering::rviz_rendering
  # rviz_default_plugins::rviz_default_plugins
  # std_msgs::std_msgs
  # ltm_shared_msgs::ltm_shared_msgs
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ltm_shared_msgs
  rviz_common
  # rviz_ogre_vendor
  # rviz_rendering
  # rviz_default_plugins
  # std_msgs
  # ltm_shared_msgs
)

install(TARGETS ${PROJECT_NAME}
  # ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  # RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
