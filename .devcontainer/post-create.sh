#!/bin/bash
set -e

echo "Setting up the development environment..."

# Set the root folder
ROOT=$(dirname "$(dirname "$(readlink -f $0)")")
echo "Root folder: $ROOT"

# Export ROS1 and ROS2
ROS1_INSTALL_PATH=/opt/ros/noetic
ROS2_INSTALL_PATH=/opt/ros/foxy

echo "export ROS1_INSTALL_PATH=$ROS1_INSTALL_PATH" >> ~/.bashrc
echo "export ROS2_INSTALL_PATH=$ROS2_INSTALL_PATH" >> ~/.bashrc

# Setup and configure a ROS1 workspace
source $ROS1_INSTALL_PATH/setup.bash
catkin init --workspace "${ROOT}/ros1_ws" &>/dev/null
catkin config --workspace "${ROOT}/ros1_ws" \
    --extend /opt/ros/noetic \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --
echo "ROS1 workspace setup and configured."

# Configure the ROS2 workspace


# Configure the ROS1 Catkin workspace
# if [ ! -d "${ROOT}/ros1_ws/src" ]; then
#     # Create the ROS1 Catkin workspace and src folder
#     mkdir -p ${ROOT}/ros1_ws/src
#     cd ${ROOT}/ros1_ws

#     # Setup catkin workspace
#     catkin init --workspace "${ROOT}/ros1_ws" &>/dev/null

#     # Configure the workspace
#     catkin config --workspace "${ROOT}/ros1_ws" --extend /opt/ros/noetic \
#         --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# else
#     echo "ROS1 Catkin workspace already exists."
# fi
