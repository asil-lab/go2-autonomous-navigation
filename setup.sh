#!/bin/bash

# Get the directory of the current script
export LTM_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LTM_ROS2_WS="${LTM_DIRECTORY}/ros2_ws"
export LTM_RECORDINGS_DIRECTORY="${LTM_DIRECTORY}/recordings"
export LTM_RECORDINGS_SCAN_DIRECTORY="${LTM_RECORDINGS_DIRECTORY}/scan"
export LTM_RECORDINGS_STREAM_DIRECTORY="${LTM_RECORDINGS_DIRECTORY}/stream"
export LTM_RECORDINGS_AMBIENCE_DIRECTORY="${LTM_RECORDINGS_DIRECTORY}/ambience"


# Create maps directory in ltm_mission directory if it doesn't exist
if [ ! -d "${LTM_DIRECTORY}/ros2_ws/src/ltm_mission/maps" ]; then
    mkdir "${LTM_DIRECTORY}/ros2_ws/src/ltm_mission/maps"
fi
export LTM_MAPS_DIRECTORY="${LTM_ROS2_WS}/src/ltm_mission/maps"

# Create aliases
alias ltm="cd ${HOME}/go2-autonomous-navigation/ros2_ws && source ../setup.sh && source /opt/ros/foxy/setup.bash && source src/unitree_ros2/setup.sh"
alias ltms="ltm && source install/setup.bash"
alias ltmbp="ltm && colcon build --symlink-install --packages-skip cyclonedds"
alias ltmbps="ltm && colcon build --symlink-install --packages-select"
alias ltmbpu="ltm && colcon build --symlink-install --packages-up-to"
alias ltmbc="ltm && rm -rf build/ install/ log/ && colcon build --symlink-install --packages-skip cyclonedds"

# Create aliases on Go2
alias go2_setup="cd ${HOME}/go2-autonomous-navigation/ros2_ws && source /opt/ros/foxy/setup.bash"
alias go2_source="go2_setup && source install/setup.bash"
alias go2_ltmbc="rm -rf build/ install/ log/ && colcon build --symlink-install --packages-skip cyclonedds"