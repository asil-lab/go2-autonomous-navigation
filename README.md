# Go2 Autonomous Navigation

This repository is the development environment for implementing an autonomous navigation for the Project Moonshot's Robotic Terrain Mapping using a Unitree Robotics Go2 Edu.

## Prerequisites

### Linux Ubuntu 20.04

This project is developed using **Linux Ubuntu 20.04** in order to run ROS2 Foxy. You can either install it as a dual boot, or use a virtual machine if you are using another OS.

### Git

Install the Git package:

```bash
sudo apt-get update
sudo apt-get -y install git
```

### VSCode

Install the apt repository and signing key from Microsoft's repo:

```bash
sudo apt-get install wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
rm -f packages.microsoft.gpg
```

Then update the package cache and install the VSCode package:

```bash
sudo apt install apt-transport-https
sudo apt update
sudo apt install code
```

### ROS2 Foxy

Please refer to the [official installation guide](https://docs.ros.org/en/foxy/Installation.html) to install ROS2 Foxy.

## How To Configure

If you have fulfilled the prerequisites, clone the repository:

```bash
git clone https://github.com/asil-lab/go2-autonomous-navigation.git
```

### Robot configuration

Instruction taken from [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) and modified. For more information, please refer to the link.

#### 1. Dependencies
```bash
sudo apt update
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl
```

#### 2. Compile cyclone dds
The cyclonedds version of Unitree robot is 0.10.2. To communicate with Unitree robots using ROS2, it is necessary to change the dds implementation. See：https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html

Before compiling cyclonedds, please ensure that ros2 environment has **NOT** been sourced when starting the terminal. Otherwise, it may cause errors in compilation.

If "source/opt/ros/foxy/setup. bash" has been added to the ~/.bashrc file when installing ROS2, it needs to be commented out:

```bash
sudo apt install gedit
sudo gedit ~/.bashrc
``` 

```bash
# source /opt/ros/foxy/setup.bash 
```

Compile cyclone-dds

```bash
cd /path/to/go2-autonomous-navigation/src/unitree_ros2/cyclonedds_ws
colcon build --packages-select cyclonedds #Compile cyclone-dds package
```

#### 3. Compile unitree_go and unitree_api packages
After compiling cyclone-dds, ROS2 dependencies is required for compilation of the unitree_go and unitree_api packages. Therefore, before compiling, it is necessary to source the environment of ROS2.

```bash
source /opt/ros/foxy/setup.bash # source ROS2 environment
colcon build # Compile all packages in the workspace
```

#### 4. Network configuration

Connect Unitree robot and the computer via Wi-Fi with the following credentials. For manual, use Ethernet cable.

- **WLAN:** wed
- **PASSWORD:** sdf

Then, use ifconfig to view the network interface that the robot connected. For example, "enp3s0" in the following figure.
![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/5d22c143-5dad-4964-81f3-55864906a9f0.png)

Next, open the network settings, find the network interface that the robot connected.In IPv4 setting, change the IPv4 mode to manual, set the address to 192.168.123.99, and set the mask to 255.255.255.0. After completion, click apply and wait for the network to reconnect.
![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/721e1660-04dc-42b7-8d6e-14799afe2165.png)

After completing the above configuration, it is recommended to restart the computer before conducting the test.

Ensure that the network of robot is connected correctly, open a terminal and input:  
```bash
cd /path/to/go2-autonomous-navigation/ros2_ws
source /opt/ros/foxy/local_setup.bash && source src/unitree_ros2/setup.sh
ros2 topic list
```
You can see the following topics:
![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/5e45e8ec-9248-47eb-8380-798ed0ef468b.png)

Input ros2 topic echo /sportmodestate，you can see the data of the topic as shown in the following figure：
![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/89214761-6cfb-4b52-bf24-7a5bd9a9806c.png)

### VSCode configuration

You can configure the provided workspace on VSCode to work in the ROS2 workspace and embedded environment, 
simultaneously,by proceeding the following instruction:

1. Open VSCode
2. Open the file `go2-autonomous-navigation.code-workspace`.
3. On the bottom right of the file, press the button "Open Workspace".

Another method to do is via the Command Palette (`CTRL` + `SHIFT` + `P`), select `File: Open Workspace From File`,
and select `go2-autonomous-navigation.code-workspace`.

## Setup and aliases

For this project, aliases are used for quick access to commands. These are all stored in the bash script `setup.sh`.
Please examine the script for further information for each aliases.

To source `setup.sh`, do the following instruction while replacing `/path/to/` to your correct directory:

```bash
cd /path/to/go2-autonomous-navigation
source setup.sh
```

You can also source `setup.sh` at startup by inserting the following lines into `.bashrc`.

```bash
source /path/to/go2-autonomous-navigation/setup.sh
```

## How To Run

### 1. Connect to robot

Before we can run the autonomous navigation, a connection must be established between your PC and the robot via Wi-Fi. Please refer to **Network Configuration** under **Robot Configuration** for information.

### 2. Build abd source

**NOTE:** Make sure that you sourced ROS2 and go2_autonomous_navigation's `setup.sh` beforehand.

```bash
cd /path/to/go2-autonomous-navigation/ros2_ws
colcon build --symlink-install --packages-skip cyclonedds
source install/local_setup.bash
```

or for short using alias.

```bash
ltmbp
ltms # on host PC
# go2_source # on Go2
```

### 3. Launch

```bash
# On first terminal
ros2 launch ltm_stack go2_stack.launch.py # on host PC
# or
ros2 launch ltm_stack go2_stack.launch.py go2:=true # on Go2
ros2 launch ltm_go2_description go2_rviz.launch.py # on host PC

# On second terminal
ros2 launch ltm_stack mission_planner.launch.py

# On third terminal
ros2 launch ltm_state_machine state_machine.launch.py
```

## Issue

If you come across bugs, unintended functions, or have some points of improvement, please refer to the 
[issues](https://github.com/asil-lab/go2-autonomous-navigation/issues), and fill in your remarks.
