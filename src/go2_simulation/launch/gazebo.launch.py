import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch. event_handlers import OnProcessStart, OnProcessIO

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    xacro_file = os.path.join(get_package_share_directory('go2_description'), 'xacro/', 'robot.xacro')    
    assert os.path.exists(xacro_file), "The robot.xacro doesnt exist in "+str(xacro_file)

    install_dir = get_package_prefix('go2_description')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'


    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    urdf_file = os.path.join(get_package_share_directory('go2_description'), 'urdf', 'go2_description.urdf')
    assert os.path.exists(urdf_file), "The go2_description.urdf doesnt exist in "+str(urdf_file)
    
    # start_steering_control = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_box_car_description, 'launch', 'steering_control.launch.py'),
    #     )
    # ) 

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui': 'true',
            # 'world': world_filepath,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    """ Spawn controllers
    """

    robot_control_config = os.path.join(get_package_share_directory('go2_description'), 'config', 'robot_control.yaml')
    assert os.path.exists(robot_control_config), "The robot_control.yaml doesnt exist in "+str(robot_control_config)
    
    controllers = [Node(
        package='controller_manager',
        executable='spawner.py',
        name=controller + '_spawner',
        namespace='go2_gazebo',
        arguments=[controller, '-c', '/gazebo_ros2_control'],
        output='screen',
    ) for controller in [
        'joint_state_controller',
        'FL_hip_controller',
        'FL_thigh_controller',
        'FL_calf_controller',
        'FR_hip_controller',
        'FR_thigh_controller',
        'FR_calf_controller',
        'RL_hip_controller',
        'RL_thigh_controller',
        'RL_calf_controller',
        'RR_hip_controller',
        'RR_thigh_controller',
        'RR_calf_controller',
    ]]

    return LaunchDescription([
        Node(package='go2_simulation', executable='spawn_go2.py', arguments=[robot_description], output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
            # remappings=[('/joint_states', '/go2_gazebo/joint_states')],
        ),
        gazebo,
        #start_steering_control,
    ] + controllers)

