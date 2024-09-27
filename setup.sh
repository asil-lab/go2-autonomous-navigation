#!/bin/bash

# Get the directory of the current script
export LTM_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LTM_ROS2_WS="${LTM_DIRECTORY}/ros2_ws"
export LTM_RECORDINGS_DIRECTORY="${LTM_DIRECTORY}/recordings"