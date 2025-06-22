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
    pkg_trailblazer_dir = get_package_share_directory('trailblazer_description')

    urdf_file = os.path.join(pkg_trailblazer_dir, 'urdf', 'trailblazer.urdf.xacro')

    # Launch arguments
    bag_file = LaunchConfiguration('bag_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config = LaunchConfiguration('rviz_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    # Declare launch arguments
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='~/hilti_bag/',
        description='Path to the ROS2 bag file to play'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'config', 'odom_visualization.rviz'),
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

    # Nodes and processes to launch
    # 1. IMU Odometry Node
    imu_odom_node = Node(
        package='imu_odom',
        executable='imu_odom_node',
        name='imu_odom_node',
        output='screen',
        parameters=[{
            'frame_id': 'odom',
            'child_frame_id': 'base_link'
        }]
    )
    
    # 2. ROS2 Bag playback process
    bag_play_w_delay = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, "-d", "5.0"],
        shell=True,
        condition=IfCondition(PythonExpression(["'", bag_file, "'", ' != ""']))
    )
    
    # 3. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    '''
    lid_odom = Node(package="lid_odom",
                    executable='lid_odom_node',
                    ouput="screen")
    '''
    # Static transform broadcaster from map to odom
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )





    # Create and return launch description
    return LaunchDescription([
        bag_file_arg,
        use_rviz_arg,
        use_sim_time_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        imu_odom_node,
        map_to_odom,
        bag_play_w_delay,
        rviz_node
    ])
