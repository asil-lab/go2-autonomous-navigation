alias sfox='source /opt/ros/foxy/setup.bash'
alias sws='source /home/lava-tube-mapping/install/setup.bash'

alias rosdep_install='rosdep update --rosdistro foxy && rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy'

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/lava-tube-mapping/src/go2_simulation/models