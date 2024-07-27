# Source ROS environments
alias noetic='source /opt/ros/noetic/setup.bash && echo "ROS1 sourced."'
alias foxy='source /opt/ros/foxy/setup.bash && echo "ROS2 sourced."'

# Source ROS workspaces
alias ws1='cd /home/lava-tube-mapping/ros1_ws && noetic && source devel/setup.bash && echo "ROS1 workspace sourced."'
alias ws2='cd /home/lava-tube-mapping/ros2_ws && foxy && source install/setup.bash && echo "ROS2 workspace sourced."'