#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('odom_trail_blazer_demos')
    pkg_trailblazer_dir = get_package_share_directory('trailblazer_description')
    
    bag_file = LaunchConfiguration('bag_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='~/hilti_bag',
        description='Path to the ROS2 bag file to play'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    urdf_file = os.path.join(pkg_trailblazer_dir, 'urdf', 'trailblazer.urdf.xacro')
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    lidar_odom_node = Node(
        package='lid_odom',
        executable='loose_lidar_imu_nod',
        name='loose_lidar_odom_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'scented_odom_node.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/points_in', '/rslidar_points'),
                    ('/odometry_in', '/track_odometry'),
                    ('/map','/voxel_map')]
    )
    
    odom_to_path_node = Node(
        package='odom_trail_blazer_demos',
        executable='odom_to_path.py',
        name='odom_to_path_node',
        output='screen',
        parameters=[{
            'odom_topic': '/lidar_odometry',
            'path_topic': '/lid_odom_path',
            'frame_id': 'lid_odom',
            'max_path_length': 100,
            'use_sim_time': use_sim_time
        }],
        remappings=[('lid_odom','lidar_odometry')]
        )
    
    map_to_lidar_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_lid_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lid_odom'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_lid_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ROS2 Bag playback with 2-second delay
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file,'-d', '2.0'],
        shell=True,
        condition=IfCondition(PythonExpression(["'", bag_file, "'", ' != ""']))
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'lid_visualization.rviz')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz))
    
    # Create and return launch description
    return LaunchDescription([
        bag_file_arg,
        use_rviz_arg,
        use_sim_time_arg,
        robot_state_publisher,
        lidar_odom_node,
        odom_to_path_node,
        map_to_lidar_odom,
        map_to_odom,
        bag_play,
        rviz_node])
