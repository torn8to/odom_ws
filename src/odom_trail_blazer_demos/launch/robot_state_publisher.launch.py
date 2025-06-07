#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directories
    pkg_trailblazer_dir = get_package_share_directory('trailblazer_description')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    #urdf file path 
    urdf_file = os.path.join(pkg_trailblazer_dir, 'urdf', 'trailblazer.urdf.xacro')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # URDF file path
    urdf_file = os.path.join(pkg_trailblazer_dir, 'urdf', 'trailblazer.urdf.xacro')
    
    # Robot description from URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    '''
    # Static transform broadcaster for map->odom_track
    map_to_odom_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'pri_odom'],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )'''
    
    # Fallback direct transform if needed
    base_to_map_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_map_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Create and return launch description
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        #map_to_odom_static_transform,
        base_to_map_static_transform
    ]) 
