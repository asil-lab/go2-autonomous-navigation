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

# Build the ROS1 workspace
echo "Building ROS1 workspace..."
catkin build --workspace "${ROOT}/ros1_ws" &>/dev/null
echo "ROS1 workspace built."

# Configure the ROS2 workspace
# source $ROS2_INSTALL_PATH/setup.bash

# Source both ROS1 Noetic and ROS2 Foxy installations to use ros1_bridge, in this order.
echo "source $ROS1_INSTALL_PATH/setup.bash" >> ~/.bashrc
echo "source $ROS2_INSTALL_PATH/setup.bash" >> ~/.bashrc

# Source aliases
echo "source ${ROOT}/.devcontainer/setup_aliases.sh" >> ~/.bashrc
echo "Aliases sourced. See .devcontainer/setup_aliases.sh for more details."

# Complete
echo "Development environment setup complete."
