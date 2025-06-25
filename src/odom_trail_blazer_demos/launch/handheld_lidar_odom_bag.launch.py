#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directories
    pkg_dir = get_package_share_directory('odom_trail_blazer_demos')
    pkg_trailblazer_dir = get_package_share_directory('phasma_description')

    urdf_file = os.path.join(pkg_trailblazer_dir, 'urdf', 'phasma.urdf.xacro')

    # Launch arguments
    bag_file = LaunchConfiguration('bag_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lidar_odom_param_file = LaunchConfiguration('lidar_param_file')

    rviz_config = LaunchConfiguration('rviz_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    # Declare launch arguments
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='~/hilti_handheld/',
        description='Path to the ROS2 bag file to play'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    ) 
    lidar_param_file_arg = DeclareLaunchArgument(
        'lidar_param_file',
        default_value=os.path.join(pkg_dir, 'config', 'handheld_odom_lidar.yaml'),
        description='Path to the lidar odometry parameter file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'config', 'handheld_odom_visualizer.rviz'),
        description='Path to the RViz configuration file'
    )
    
    # Robot description from URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str)
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time}]
    )

    bag_play_w_delay = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, "-d", "5.0"],
        shell=True,
        condition=IfCondition(PythonExpression(["'", bag_file, "'", ' != ""']))
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz))

    lidar_odom_node = Node(package="lid_odom",
                    executable='lid_odom_node',
                    output="screen",
                    #prefix=['xterm -e valgrind --tool=callgrind'],
                    parameters=[lidar_odom_param_file],
                    remappings=[('/points', '/hesai/pandar')])
    
    # Add odom to path node for visualizing the odometry path
    odom_to_path_node = Node(
        package='odom_trail_blazer_demos',
        executable='odom_to_path.py',
        name='odom_to_path_node',
        output='screen',
        parameters=[{
            'odom_topic': '/lid_odom',
            'path_topic': '/lid_odom_path',
            'frame_id': 'lid_odom',
            'max_path_length': 100,
            'use_sim_time': use_sim_time
        }]
    )

    map_to_lidar_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lid_odom'],
        output='screen'
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )


    return LaunchDescription([
        bag_file_arg,
        use_rviz_arg,
        use_sim_time_arg,
        lidar_param_file_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        lidar_odom_node,
        odom_to_path_node,
        bag_play_w_delay,
        map_to_lidar_odom,
        map_to_odom,
        rviz_node
    ]) 