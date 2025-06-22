#define USE_MATH_DEFINES
#include "imu_odom/ImuOdom.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>


#define ROT90 M_PI/2

// Add explicit implementation for the fromMsg conversion function


namespace tf2 {
  void fromMsg(const geometry_msgs::msg::Transform& msg, Sophus::SE3d& tf_mat) {
    tf_mat = Sophus::SE3d(
      Eigen::Quaterniond(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z),
      Eigen::Vector3d(msg.translation.x, msg.translation.y, msg.translation.z)
    );
  }
}

namespace imu_odom
{
ImuOdom::ImuOdom()
: Node("imu_odom")
, last_time_(0,0,RCL_ROS_TIME)
{

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    std::bind(&ImuOdom::imuCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_imu", 10);


  set_bias_service_ = create_service<std_srvs::srv::Trigger>(
    "set_imu_bias", 
    std::bind(&ImuOdom::setBiasCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Declare parameterss
  declare_parameter("publish_transform",true);
  declare_parameter("frame_id", frame_id_);
  declare_parameter("child_frame_id", child_frame_id_);
  declare_parameter("imu_frame_id", imu_frame_id_);
  declare_parameter("x_acceleration_bias", -0.23);
  declare_parameter("y_acceleration_bias", -0.13);
  declare_parameter("z_acceleration_bias", 0.10);
  declare_parameter("x_angular_velocity_bias", 0.00);
  declare_parameter("y_angular_velocity_bias", 0.00);
  declare_parameter("z_angular_velocity_bias", 0.00);
  declare_parameter("flip_x_axis",false);// parameters to align the imu frame if imu frame tf frame rules differ from imu
  declare_parameter("flip_y_axis",false);
  declare_parameter("flip_z_axis",false);
  declare_parameter("pin_z_axis",false);

  pin_z = get_parameter("pin_z_axis").as_bool();

  frame_id_ = get_parameter("frame_id").as_string();
  child_frame_id_ = get_parameter("child_frame_id").as_string();
  imu_frame_id_ = get_parameter("imu_frame_id").as_string();

  linear_acceleration_bias_ = Eigen::Vector3d(
    get_parameter("x_acceleration_bias").as_double(),
    get_parameter("y_acceleration_bias").as_double(),
    get_parameter("z_acceleration_bias").as_double());
  angular_velocity_bias_ = Eigen::Vector3d(
    get_parameter("x_angular_velocity_bias").as_double(),
    get_parameter("y_angular_velocity_bias").as_double(),
    get_parameter("z_angular_velocity_bias").as_double());

  renormalize_imu_orientation_ = Sophus::SO3d();
  imu_base_link_transform_  = Sophus::SE3d();
  transform_ = Sophus::SE3d();
  RCLCPP_INFO(get_logger(), "IMU Odometry node initialized");
}

void ImuOdom::integrateImu(const sensor_msgs::msg::Imu::SharedPtr msg){
  rclcpp::Time current_time = msg->header.stamp;
  if(!imu_transform_acquired){
    try {
      geometry_msgs::msg::TransformStamped imu_transform = geometry_msgs::msg::TransformStamped();
      Sophus::SE3d transform;
      imu_transform = tf_buffer_->lookupTransform(
        imu_frame_id_,
        child_frame_id_,
        tf2::TimePointZero, 
        tf2::durationFromSec(0.5));
      tf2::fromMsg(imu_transform.transform, transform);
      renormalize_imu_orientation_ = transform.so3();
      RCLCPP_INFO(get_logger(), "Successfully got IMU transform from TF tree");
      imu_transform_acquired = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Could not get IMU transform: %s", ex.what());
      return;
    }
  }
  RCLCPP_INFO(get_logger(), "Integrating IMU message at time: %f", current_time.seconds());
  if (!has_first_message_) {
    RCLCPP_INFO(get_logger(), "Retrieved_first_message");
    last_time_ = current_time;
    has_first_message_ = true;
    return;
  }
  
  // Convert IMU message to Eigen vectors
  Eigen::Vector3d linear_acceleration(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);

  Eigen::Vector3d imu_angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

 //orienation_corrections 
  Eigen::Vector3d orientation_corrected_acceleration = (renormalize_imu_orientation_.inverse()
                                                        * linear_acceleration) + gravity_bias_ 
                                                        - linear_acceleration_bias_;
  //orientation_corrected_acceleration = orientation_corrected_acceleration.array() * frame_flip.array();
  Eigen::Vector3d processed_angular_velocity = (renormalize_imu_orientation_.inverse() * imu_angular_velocity) - angular_velocity_bias_;
  double dt = (current_time - last_time_).seconds(); 
  Eigen::Vector3d delta_angle = dt * (processed_angular_velocity + angular_velocity_)/2.0;
  angular_velocity_ = processed_angular_velocity;
  Sophus::SO3d delta_half_rotation = Sophus::SO3d::exp(delta_angle / 2.0);
  transform_.so3() = delta_half_rotation * transform_.so3() ;
  Eigen::Vector3d linear_velocity_delta = dt * (orientation_corrected_acceleration);
  transform_.translation() = (transform_.translation() + (transform_.rotationMatrix() * (dt * (linear_velocity_ + linear_velocity_delta/2.0))));
  linear_velocity_ += linear_velocity_delta;
  transform_.so3() = delta_half_rotation * transform_.so3() ;
  last_time_ = current_time;
  publishOdomMessage(current_time);
  publishTransform(current_time);
}

void ImuOdom::publishOdomMessage(const rclcpp::Time& current_time) {
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = frame_id_;
  odom_msg.child_frame_id = child_frame_id_;
  
  odom_msg.pose.pose.position.x = transform_.translation().x();
  odom_msg.pose.pose.position.y = transform_.translation().y();
  odom_msg.pose.pose.position.z = !this->pin_z ? transform_.translation().z(): 0;
  
  odom_msg.pose.covariance.fill(0.0);
  odom_msg.pose.covariance[0] = pose_covariance_;
  odom_msg.pose.covariance[7] = pose_covariance_;
  odom_msg.pose.covariance[14] = pose_covariance_;
  odom_msg.pose.covariance[21] = orientation_covariance_;
  odom_msg.pose.covariance[28] = orientation_covariance_;
  odom_msg.pose.covariance[35] = orientation_covariance_;
  
  // Get the rotation matrix
  Eigen::Matrix3d rot_matrix = transform_.rotationMatrix();
  
  // Create a quaternion directly from the rotation matrix
  Eigen::Quaterniond quat(rot_matrix);
  quat.normalize();
  
  // Set the orientation in the odometry message
  odom_msg.pose.pose.orientation.x = quat.x();
  odom_msg.pose.pose.orientation.y = quat.y();
  odom_msg.pose.pose.orientation.z = quat.z();
  odom_msg.pose.pose.orientation.w = quat.w();
  
  odom_msg.twist.twist.linear.x = linear_velocity_.x();
  odom_msg.twist.twist.linear.y = linear_velocity_.y();
  odom_msg.twist.twist.linear.z = !this->pin_z ? linear_velocity_.z() : 0;
  
  odom_msg.twist.twist.angular.x = angular_velocity_.x();
  odom_msg.twist.twist.angular.y = angular_velocity_.y();
  odom_msg.twist.twist.angular.z = angular_velocity_.z();
  
  odom_msg.twist.covariance.fill(0.0);
  odom_msg.twist.covariance[0] = pose_covariance_;
  odom_msg.twist.covariance[7] = pose_covariance_;
  odom_msg.twist.covariance[14] = pose_covariance_;
  odom_msg.twist.covariance[21] = orientation_covariance_;
  odom_msg.twist.covariance[28] = orientation_covariance_;
  odom_msg.twist.covariance[35] = orientation_covariance_;
  odom_pub_->publish(std::move(odom_msg));
}

void ImuOdom::publishTransform(const rclcpp::Time& current_time) {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time;
  tf_msg.header.frame_id = frame_id_;
  tf_msg.child_frame_id = child_frame_id_;

  // Set translation
  tf_msg.transform.translation.x = transform_.translation().x();
  tf_msg.transform.translation.y = transform_.translation().y();
  tf_msg.transform.translation.z = !this->pin_z ? transform_.translation().z() : 0;

  // Get the quaternion directly from the transform's rotation part
  // This ensures we get the proper orientation representation
  Eigen::Quaterniond quat = transform_.so3().unit_quaternion();
  
  // Ensure the quaternion is normalized
  if (std::abs(quat.norm() - 1.0) > 1e-6) {
    quat.normalize();
    RCLCPP_WARN(get_logger(), "TF quaternion was not normalized, normalized it: w=%.4f, x=%.4f, y=%.4f, z=%.4f", 
                quat.w(), quat.x(), quat.y(), quat.z());
  }
  
  // Set rotation
  tf_msg.transform.rotation.x = quat.x();
  tf_msg.transform.rotation.y = quat.y();
  tf_msg.transform.rotation.z = quat.z();
  tf_msg.transform.rotation.w = quat.w();
  
  tf_broadcaster_->sendTransform(tf_msg);
}

void ImuOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Store the message in the buffer
  if (imu_buffer_.size() >= imu_buffer_size_) {
    imu_buffer_.pop_front();
  }
  imu_buffer_.push_back(*msg);
  
  // Process the IMU data
  integrateImu(msg);
}
}  // namespace imu_odom

