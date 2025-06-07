#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directories
    pkg_dir = get_package_share_directory('odom_trail_blazer_demos')
    
    # Launch arguments
    bag_file = LaunchConfiguration('bag_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # Include the robot state publisher launch file
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    imu_odom_node = Node(
         package='imu_odom',
         executable='imu_odom_node',
         name='imu_odom_node',
         output='screen',
         parameters=[{
             'frame_id': 'odom_imu',
             'child_frame_id': 'base_link',
             'base_frame_id': 'base_link',
             'imu_frame_id': 'xsens_imu_link',
             'use_sim_time': use_sim_time,
             'publish_tf': True  # Enable TF publishing to update base_link position
         }]
     )
    
    # Track Odometry to TF broadcaster - subscribes to /track_odometry and publishes TF
    '''
    track_odom_tf_node = Node(
        package='odom_trail_blazer_demos',
        executable='odom_to_tf_node.py',
        name='track_odom_tf_node',
        output='screen',
        parameters=[{
            'odom_topic': '/track_odometry',
            'frame_id': 'odom_track',
            'child_frame_id': 'base_link',
            'use_sim_time': use_sim_time
        }]
    )'''
    
    # 2. ROS2 Bag playback process
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, '--clock'],
        shell=True,
        condition=IfCondition(PythonExpression(["'", bag_file, "'", ' != ""']))
    )
    
    # Create RViz configuration file if it doesn't exist
    rviz_config_path = os.path.join(pkg_dir, 'config')
    os.makedirs(rviz_config_path, exist_ok=True)
    rviz_config_file = os.path.join(rviz_config_path, 'odom.rviz')
    
    # 3. RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(use_rviz)
    )
    
    # Create and return launch description
    return LaunchDescription([
        bag_file_arg,
        use_rviz_arg,
        use_sim_time_arg,
        robot_state_publisher,
        imu_odom_node,
        #track_odom_tf_node,  # Use track odometry instead of IMU odometry
        bag_play,
        rviz_node
    ]) 
