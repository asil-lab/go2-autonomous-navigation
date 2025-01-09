# Lava Tube Mapping

This repository is the development environment for Project Moonshot's Rhizome 2.0 Lava Tube Mapping.

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
git clone https://github.com/AlexanderJamesBecoy/lava-tube-mapping.git
```

You can configure the provided workspace on VSCode to work in the ROS2 workspace and embedded environment, 
simultaneously,by proceeding the following instruction:

1. Open VSCode
2. Open the file `lava-tube-mapping.code-workspace`.
3. On the bottom right of the file, press the button "Open Workspace".

Another method to do is via the Command Palette (`CTRL` + `SHIFT` + `P`), select `File: Open Workspace From File`,
and select `lava-tube-mapping.code-workspace`.

## Setup and aliases

For this project, aliases are used for quick access to commands. These are all stored in the bash script `setup.sh`.
Please examine the script for further information for each aliases.

To source `setup.sh`, do the following instruction while replacing `/path/to/` to your correct directory:

```bash
cd /path/to/lava-tube-mapping
source setup.sh
```

You can also source `setup.sh` at startup by inserting the following lines into `.bashrc`.

```bash
source /path/to/lava-tube-mapping/setup.sh
```

## How To Connect To Unitree Go2

### 1. Connect to Wi-Fi

## Issue

If you come across bugs, unintended functions, or have some points of improvement, please refer to the 
[issues](https://github.com/AlexanderJamesBecoy/lava-tube-mapping/issues), and fill in your remarks.
