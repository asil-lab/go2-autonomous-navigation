from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    
    urdf_filepath = os.path.join(get_package_share_directory("go2_description") 
                                 + "/urdf/go2_description.urdf")
    with open(urdf_filepath, 'r') as infp:
        robot_description = infp.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
            arguments=[urdf_filepath],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', os.path.join(get_package_share_directory("march_launch"), "rviz", "izzy.rviz")],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])