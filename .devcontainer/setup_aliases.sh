# Source ROS environments
alias noetic='source /opt/ros/noetic/setup.bash && echo "ROS1 sourced."'
alias foxy='source /opt/ros/foxy/setup.bash && echo "ROS2 sourced."'

# Source ROS workspaces
alias ws1='cd /home/lava-tube-mapping/ros1_ws && noetic && source devel/setup.bash && echo "ROS1 workspace sourced."'
alias ws2='cd /home/lava-tube-mapping/ros2_ws && foxy && source install/setup.bash && echo "ROS2 workspace sourced."'

# Connect to the Unitree Go2 robot
# alias connect-go2="export ROS_MASTER_URI=http://\$(ip route show default | \
#     grep -oP 'via \K\d+\.\d+\.\d+').5:11311 ; export ROS_IP=\$(ip route get 8.8.8.8 | \
#     grep -oP '(?<=src )\S+') ; echo 'ROS_MASTER_URI and ROS_IP set to ' ; printenv ROS_MASTER_URI ; printenv ROS_IP"
# alias connect-go2="export ROS_MASTER_URI=http://192.168.1.1:11311 ; export ROS_IP=192.168.1.2; export ROS_DOMAIN_ID=0;"

connect-go2 () {
    auto wlp2s0
    iface wlp2s0 inet static
    address 192.168.123.99
    netmask 255.255.255.0
}