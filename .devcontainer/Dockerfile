FROM ros:foxy
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

# Install ROS Gazebo
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
# RUN apt-get install -y ros-foxy-ros-ign
RUN apt-get install -y ros-foxy-gazebo-ros

# Install ROS 2 dependencies
RUN apt-get install -y ros-foxy-rviz2 ros-foxy-joint-state-publisher-gui ros-foxy-robot-state-publisher

ENV SHELL /bin/bash

# Source aliases
RUN echo -e "source /home/lava-tube-mapping/.devcontainer/setup_aliases.sh" >> ~/.bashrc

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]