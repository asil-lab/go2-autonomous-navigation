"""
Project Lava Tube Mapping, Technical University of Delft.
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Load the parameters for every state
    state_parameters = os.path.join(get_package_share_directory('ltm_state_machine'), 
        'config', 'state_machine_params.yaml')
    
    
    # State Machine Node
    state_machine_node = Node(
        package='ltm_state_machine',
        executable='state_machine_node',
        name='state_machine_node',
        output='screen',
        parameters=[state_parameters],
    )

    return LaunchDescription([
        state_machine_node,
    ])