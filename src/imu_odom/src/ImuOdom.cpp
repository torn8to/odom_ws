#include "imu_odom/ImuOdom.hpp"
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

void ImuOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
  // assumes that orientation is not provided in the IMU msg
  imu_buffer_.push_back(*msg);
  if (imu_buffer_.size() > imu_buffer_size_) {
    imu_buffer_.pop_front();
  }
  integrateImu(msg);
  publishOdomMessage(msg->header.stamp);
}

void ImuOdom::integrateImu(const sensor_msgs::msg::Imu::SharedPtr msg){
  rclcpp::Time current_time = msg->header.stamp;
  
  if (has_first_message_) {
    last_time_ = current_time;
    has_first_message_ = false;
    return;
  }
  Vector3 linear_acceleration;
  AngularVelocity angular_velocity;
  tf2::fromMsg(msg->linear_acceleration, linear_acceleration);
  tf2::fromMsg(msg->angular_velocity, angular_velocity);
  // Calculate time difference
  double dt = (current_time - last_time_).nanoseconds() * 1e-9;  // Convert nanoseconds to seconds
  const AngularVelocity processed_angular_velocity = angular_velocity
                                              - angular_velocity_bias_;
  // Calculate delta rotation from angular velocity
  const AngularVelocity delta_angle = dt *(processed_angular_velocity + rotation_velocity_);
  const Rotation delta_half_rotation = Rotation::exponentialMap(delta_angle.toImplementation()/2.0);
  
  transform_.getRotation() = transform_.getRotation() * delta_half_rotation;
  const Vector3 linear_velocity_delta = dt * (linear_acceleration
             - linear_acceleration_bias_ +
             transform_.getRotation().invert().rotate(gravity_bias_));

  transform_.getPosition() = transform_.getPosition() + 
            static_cast<Position>(transform_.getRotation().rotate(
            linear_velocity_ + (0.5 * linear_velocity_delta)));

  linear_velocity_ = linear_velocity_ + linear_velocity_delta;

  transform_.getRotation() = transform_.getRotation() * delta_half_rotation;

  last_time_ = current_time;
}

void ImuOdom::publishOdomMessage(const rclcpp::Time& current_time) {
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = frame_id_;
  odom_msg.child_frame_id = child_frame_id_;

  // Convert transform position to odometry message
  const auto& position = transform_.getPosition();
  odom_msg.pose.pose.position.x = position.x();
  odom_msg.pose.pose.position.y = position.y();
  odom_msg.pose.pose.position.z = position.z();

  // Convert transform rotation to odometry message
  kindr_ros::convertToRosGeometryMsg(transform_, odom_msg.pose.pose);
  kindr_ros::convertToRosGeometryMsg(linear_velocity_,odom_msg.twist.twist.linear);
  kindr_ros::convertToRosGeometryMsg(rotation_velocity_,odom_msg.twist.twist.angular);
  // Set velocities
  // Publish odometry message
  odom_pub_->publish(odom_msg);
  // Broadcast transform
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = current_time;
  tf_msg.header.frame_id = frame_id_;
  tf_msg.child_frame_id = child_frame_id_;

  kindr_ros::convertToRosGeometryMsg(transform_, tf_msg.transform);
  tf_broadcaster_->sendTransform(tf_msg);
}


}  // namespace imu_odom

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(imu_odom::ImuOdom) 
