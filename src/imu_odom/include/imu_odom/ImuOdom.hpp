#ifndef IMU_ODOM_HPP_
#define IMU_ODOM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include "imu_odom/tf2_kindr.hpp"
#include <rclcpp/time.hpp>
#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>
#include <list>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>


namespace imu_odom
{

class ImuOdom : public rclcpp::Node
{
public:
  explicit ImuOdom(const rclcpp::NodeOptions & options);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publishOdomMessage(const rclcpp::Time& current_time);

private:
  void integrateImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Type definitions for kindr
  using Vector3 = kindr::VectorTypeless3D;
  using AngularVelocity = kindr::AngularVelocity3D;
  using Position = kindr::Position3D;
  using Rotation = kindr::RotationQuaternion<double>;
  using Transform = kindr::HomTransformMatrixD;

  // Subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Buffer for IMU messages
  std::list<sensor_msgs::msg::Imu> imu_buffer_;
  size_t imu_buffer_size_{100};
  
  // State variables
  rclcpp::Time last_time_;
  bool has_first_message_{false};
  bool has_orientation_{false};

  Vector3 gravity_bias_{0.0, 0.0, -9.81};

  Vector3 linear_acceleration_bias_{0.0, 0.0, 0.0};
  AngularVelocity angular_velocity_bias_{0.0, 0.0, 0.0};

  Vector3 linear_velocity_{0.0, 0.0, 0.0};
  AngularVelocity rotation_velocity_{0.0, 0.0, 0.0};

  // Frame IDs
  std::string frame_id_{"odom"};
  std::string child_frame_id_{"base_link"};
  
  // Current state
  Transform transform_;
};

}  // namespace imu_odom

#endif  // IMU_ODOM_HPP_ 