# Odometry Workspace for Trailblazer Robot
This workspace contains ROS 2 packages for odometry using the Trailblazer robot platform from the 2023 Hilti SLAM Challenge.

Is not setup to work with the handheld device also used for data collection in the challenge

## lib dependencies
requires oneapi TBB, eigen, and sophus outside of ros2

## TODO
- [x] IMU ODOM
- [ ] lid_odom working in real time (broken)
- [ ] fuse sensor_data
- [ ] create_depth image_dataset

-- parameter_optimization to fix_speed up

## visualizations
z axis is pinned
----------------------------------------
![Odometry Visualization](media/output.gif)

## Packages

### imu_odom
- IMU-based odometry package
- Provides odometry estimation via inertial measurement unit data


### lid_odom
- LiDAR-based odometry package


### trailblazer_description_ros2
- URDF description files for the Trailblazer robot platform
- Provides visualization and robot state publishing capabilities
- Includes a tool for converting MultiCal extrinsics to URDF format
-modified to support ros2 gz non-classic

### odom_trail_blazer_demos
- Demo packages for running odometry with the Trailblazer robot
- Launch files for:
  - Playing back ROS bags from the Hilti SLAM Challenge
  - Visualizing odometry results
  - Publishing robot state

### kindr_ros
- ROS interface for the Kindr library
- Provides message definitions and conversions for kinematic and dynamic quantities


## 2023 Hilti SLAM Challenge Dataset

The workspace is designed to work with the 2023 Hilti SLAM Challenge dataset. The dataset features:
- Data collected from the Trailblazer robot platform and the handheld device only supporting 
- ROS bag files containing sensor data including:
  - LiDAR point clouds from RoboSense LiDAR (`/rslidar_points` topic)
  - IMU measurements ()
  - Ground truth poses (where available)
- Challenging environments for SLAM evaluation
- Designed to test the robustness and accuracy of SLAM algorithms
- does not publish tf tree

The launch files in the `odom_trail_blazer_demos` package are configured to play back these bag files and run the odometry algorithms on the dataset. The default path for the bag files is set to `~/hilti_bag/`.
For more information about the Hilti SLAM Challenge, visit the [official Hilti Research GitHub repository](https://github.com/Hilti-Research/trailblazer_description).
