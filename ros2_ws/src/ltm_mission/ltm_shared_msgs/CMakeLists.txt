cmake_minimum_required(VERSION 3.5)
project(ltm_shared_msgs)

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
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Common messages and services
  "msg/AuxiliarySensorState.msg"
  "msg/BatteryState.msg"
  "msg/FootContact.msg"
  "srv/GetImage.srv"
  "srv/GetPointCloud.srv"
  "srv/NavigateToPose.srv"

  # State-specific messages and services
  "msg/MissionState.msg"
  "srv/CheckWaypoints.srv"
  "srv/CheckDestination.srv"
  "srv/LoadMap.srv"
  "srv/GenerateWaypoints.srv"
  "srv/GetStateTransitionHistory.srv"
  "srv/PerformState.srv"
  "srv/ScanEnvironment.srv"
  "srv/DisplayScanEnvironment.srv"

  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
