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
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(ltm_shared_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Enable Qt MOC compiler
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

qt5_wrap_cpp(MOC_FILES
  include/ltm_go2_rviz_plugins/battery_management_display.hpp
  include/ltm_go2_rviz_plugins/scan_procedure_display.hpp
)

# Create custom RViz library
add_library(ltm_go2_rviz_displays
  src/battery_management_display.cpp
  src/scan_procedure_display.cpp
  ${MOC_FILES}
)

target_include_directories(ltm_go2_rviz_displays PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ltm_go2_rviz_displays
  rclcpp
  pluginlib
  rviz_common
  rviz_default_plugins
  ltm_shared_msgs
)

install(TARGETS ltm_go2_rviz_displays
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
