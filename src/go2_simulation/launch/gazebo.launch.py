from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    sdf_dir = os.path.join(get_package_share_directory("go2_simulation"), "models", "unitree_go2")
    sdf_filepath = os.path.join(sdf_dir, "unitree_go2.sdf")
    
    world_dir = os.path.join(get_package_share_directory("go2_simulation"), "worlds")
    world_filepath = os.path.join(world_dir, "empty.world")
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui': 'true',
            # 'world': world_filepath,
            'use_sim_time': 'true',
        }.items(),
    )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'unitree_go2',
            '-x', '0',
            '-y', '0',
            '-z', '1',
            # '-P' 'robot_description',
            '-file', sdf_filepath,
            '-package_to_model',
        ],
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
