FROM ros:noetic
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
RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y git openssh-client less iproute2 procps lsb-release curl wget nano
RUN apt-get install -y python3-pip
RUN apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Tool for repo management
RUN apt-get install -y python3-vcstool
# Tool for catkin packages
RUN apt-get install -y python3-catkin-tools python3-catkin-lint python3-pip
# Tool for installing dependencies
RUN apt-get install -y python3-rosdep
# Miscellaneous libraries/tools
RUN python3 -m pip install osrf-pycommon

# initilize the catkin workspace
RUN catkin init
RUN catkin config --extend /opt/ros/noetic
RUN catkin config -DCMAKE_BUILD_TYPE=Release

# Install unitree_ros dependencies
RUN apt-get install -y \
    ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-xacro \
    ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz

ENV SHELL /bin/bash

# Source aliases and variables
RUN echo -e "source /home/lava-tube-mapping/.devcontainer/setup_aliases.sh" >> ~/.bashrc
# RUN export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/lava-tube-mapping/src/go2_simulation/models

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]