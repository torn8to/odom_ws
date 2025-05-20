#include "imu_odom/ImuOdom.hpp"

#include <memory>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>

namespace imu_odom
{

ImuOdom::ImuOdom(const rclcpp::NodeOptions & options)
: Node("imu_odom", options)
{
  // Initialize subscribers
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", 10,
    std::bind(&ImuOdom::imuCallback, this, std::placeholders::_1));

  // Initialize publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // Initialize transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Declare parameters
  declare_parameter("frame_id", frame_id_);
  declare_parameter("child_frame_id", child_frame_id_);

  // Get parameters
  frame_id_ = get_parameter("frame_id").as_string();
  child_frame_id_ = get_parameter("child_frame_id").as_string();

  RCLCPP_INFO(get_logger(), "IMU Odometry node initialized");
}

void ImuOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  rclcpp::Time current_time = msg->header.stamp;
  
  if (first_message_) {
    last_time_ = current_time;
    first_message_ = false;
    return;
  }

  // Calculate time difference
  double dt = (current_time - last_time_).nanoseconds() * 1e-9;  // Convert nanoseconds to seconds
  last_time_ = current_time;

  // Update orientation from IMU quaternion
  orientation_x_ = msg->orientation.x;
  orientation_y_ = msg->orientation.y;
  orientation_z_ = msg->orientation.z;
  orientation_w_ = msg->orientation.w;

  // Integrate linear acceleration to get velocity
  // Note: This is a simple integration method. For better results, you might want to
  // implement a more sophisticated algorithm that accounts for gravity and bias
  vel_x_ += msg->linear_acceleration.x * dt;
  vel_y_ += msg->linear_acceleration.y * dt;
  vel_z_ += msg->linear_acceleration.z * dt;

  // Integrate velocity to get position
  pos_x_ += vel_x_ * dt;
  pos_y_ += vel_y_ * dt;
  pos_z_ += vel_z_ * dt;

  // Create and publish odometry message
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = frame_id_;
  odom_msg.child_frame_id = child_frame_id_;

  // Set position
  odom_msg.pose.pose.position.x = pos_x_;
  odom_msg.pose.pose.position.y = pos_y_;
  odom_msg.pose.pose.position.z = pos_z_;

  // Set orientation
  odom_msg.pose.pose.orientation = msg->orientation;

  // Set velocity
  odom_msg.twist.twist.linear.x = vel_x_;
  odom_msg.twist.twist.linear.y = vel_y_;
  odom_msg.twist.twist.linear.z = vel_z_;
  odom_msg.twist.twist.angular = msg->angular_velocity;

  // Publish odometry message
  odom_pub_->publish(odom_msg);

  // Broadcast transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = current_time;
  transform.header.frame_id = frame_id_;
  transform.child_frame_id = child_frame_id_;

  transform.transform.translation.x = pos_x_;
  transform.transform.translation.y = pos_y_;
  transform.transform.translation.z = pos_z_;

  transform.transform.rotation = msg->orientation;

  tf_broadcaster_->sendTransform(transform);
}

}  // namespace imu_odom

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(imu_odom::ImuOdom) 