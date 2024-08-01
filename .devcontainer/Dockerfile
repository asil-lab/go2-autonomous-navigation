# Base image for ROS2 Foxy and ROS1 Noetic
FROM ubuntu:focal
COPY --from=ros:noetic / /
COPY --from=ros:foxy / /

# # Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=${GetMyUsername}
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install necessary tools
RUN apt-get update && apt-get install -y git openssh-client less iproute2 procps curl lsb-release nano bash-completion iputils-ping
RUN apt-get update && apt-get install -y python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# # Enable Universe, Multiverse, and Restricted
RUN sed -i 's/^# deb/deb/g' /etc/apt/sources.list

# # Install ROS1 Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-desktop-full

# Install ROS1 Noetic dependencies
RUN apt-get update && apt-get install -y python3-catkin-tools python3-osrf-pycommon

# Install ROS1 Noetic unitree_ros package dependencies
RUN apt-get install -y \
    ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-xacro \
    ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz ros-noetic-velodyne-gazebo-plugins

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS2 Foxy dependencies
RUN apt-get update && apt-get install -y python3-argcomplete ros-dev-tools ros-foxy-rclcpp ros-foxy-rviz2
RUN apt-get update && apt-get install -y ros-foxy-sensor-msgs ros-foxy-geometry-msgs ros-foxy-std-msgs ros-foxy-visualization-msgs ros-foxy-vision-msgs
RUN apt-get update && apt-get install -y ros-foxy-tf2 ros-foxy-tf2-ros ros-foxy-tf2-sensor-msgs ros-foxy-tf2-py \
    ros-foxy-tf2-msgs ros-foxy-tf2-eigen ros-foxy-tf2-geometry-msgs ros-foxy-tf2-tools

# Install PCL, Eigen3, and OpenCV
RUN apt-get install -y libpcl-dev libeigen3-dev libopencv-dev
RUN apt-get install -y ros-foxy-pcl-conversions ros-foxy-pcl-msgs ros-foxy-pcl-ros

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
# CMD ["/bin/bash"]

# Set entrypoint
ENTRYPOINT ["/bin/bash"]