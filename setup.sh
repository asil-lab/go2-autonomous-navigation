#!/bin/bash

# Get the directory of the current script
export LTM_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LTM_ROS2_WS="${LTM_DIRECTORY}/ros2_ws"
export LTM_RECORDINGS_DIRECTORY="${LTM_DIRECTORY}/recordings"

# Create maps directory in ltm_mission directory if it doesn't exist
if [ ! -d "${LTM_DIRECTORY}/ros2_ws/src/ltm_mission/maps" ]; then
    mkdir "${LTM_DIRECTORY}/ros2_ws/src/ltm_mission/maps"
fi
export LTM_MAPS_DIRECTORY="${LTM_ROS2_WS}/src/ltm_mission/maps"