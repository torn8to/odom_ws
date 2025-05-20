#ifndef IMU_ODOM_HPP_
#define IMU_ODOM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/time.hpp>
#include <Eigen/Dense>
#include <sophus/so3.hpp>


namespace imu_odom
{

class ImuOdom : public rclcpp::Node
{
public:
  explicit ImuOdom(const rclcpp::NodeOptions & options);

private:
  // Subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Callback for IMU messages
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // State variables
  double pos_x_{0.0}, pos_y_{0.0}, pos_z_{0.0};
  double vel_x_{0.0}, vel_y_{0.0}, vel_z_{0.0};
  double orientation_x_{0.0}, orientation_y_{0.0}, orientation_z_{0.0}, orientation_w_{1.0};
  rclcpp::Time last_time_;
  bool first_message_{true};

  // Frame IDs
  std::string frame_id_{"odom"};
  std::string child_frame_id_{"base_link"};
};

}  // namespace imu_odom

#endif  // IMU_ODOM_HPP_ 