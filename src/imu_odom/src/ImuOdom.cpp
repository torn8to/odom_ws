#include "imu_odom/ImuOdom.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

// Add explicit implementation for the fromMsg conversion function



namespace tf2 {
  void fromMsg(const geometry_msgs::msg::Transform& msg, Sophus::SE3d tf_mat) {
    tf_mat = Sophus::SE3d(
      Eigen::Quaterniond(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z),
      Eigen::Translation3d(msg.translation.x, msg.translation.y, msg.translation.z)
    );
  }
}

namespace imu_odom
{

ImuOdom::ImuOdom(const rclcpp::NodeOptions & options)
: Node("imu_odom", options)
, last_time_(0,0,RCL_ROS_TIME)
{

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize subscribers
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    std::bind(&ImuOdom::imuCallback, this, std::placeholders::_1));

  // Initialize publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_imu", 10);

  // Initialize transform broadcaster

  // Initialize service
  set_bias_service_ = create_service<std_srvs::srv::Trigger>(
    "set_imu_bias", 
    std::bind(&ImuOdom::setBiasCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Declare parameterss
  declare_parameter("publish_transform",true);
  declare_parameter("frame_id", frame_id_);
  declare_parameter("child_frame_id", child_frame_id_);
  declare_parameter("base_frame_id", base_frame_id_);
  declare_parameter("imu_frame_id", imu_frame_id_);
  declare_parameter("x_acceleration_bias", 0.00);
  declare_parameter("y_acceleration_bias", 0.00);
  declare_parameter("z_acceleration_bias", 0.00);
  declare_parameter("x_rotation_velocity_bias", 0.00);
  declare_parameter("y_rotational_velocity_bias", 0.00);
  declare_parameter("z_rotational_velocity_bias", 0.00);

  frame_id_ = get_parameter("frame_id").as_string();
  child_frame_id_ = get_parameter("child_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();
  imu_frame_id_ = get_parameter("imu_frame_id").as_string();
  linear_acceleration_bias_ = Eigen::Vector3d(
    get_parameter("x_acceleration_bias").as_double(),
    get_parameter("y_acceleration_bias").as_double(),
    get_parameter("z_acceleration_bias").as_double()
  );
  angular_velocity_vias_ = Eigen::Vector3d(
    get_parameter("x_angular_velocity_bias").as_double();
    get_parameter("y_angular_velocity_bias").as_double();
    get_parameter("z_angular_velocity_bias").as_double();
  );

  try {
    geometry_msgs::msg::TransformStamped imu_transform;
    imu_transform = tf_buffer_->lookupTransform(
      imu_frame_id_,
      base_frame_id_,
      tf2::TimePointZero, 
      tf2::durationFromSec(0.5));
    tf2::fromMsg(imu_transform.transform, transform_);
    
    RCLCPP_INFO(get_logger(), "Successfully got IMU transform from TF tree");
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Could not get IMU transform: %s", ex.what());
  }
  imu_base_link_transform_ = transform_.inverse();
  RCLCPP_INFO(get_logger(), "gravity_compensation: %f %f %f",  gravity_bias_.x(), gravity_bias_.y(), gravity_bias_.z());
  transform_ = Sophus::SE3d();
  RCLCPP_INFO(get_logger(), "IMU Odometry node initialized");
}

bool ImuOdom::setBias() {
  if (imu_buffer_.size() < 10) {
    RCLCPP_ERROR(get_logger(), "Not enough IMU samples to set bias (need at least 10, got %ld)", imu_buffer_.size());
    return false;
  }

  // Calculate average of angular velocity and linear acceleration from buffer
  Vector3 avg_angular_velocity(0, 0, 0);
  Vector3 avg_linear_acceleration(0, 0, 0);
  
  for (const auto& imu_msg : imu_buffer_) {
    avg_angular_velocity += Vector3(
      imu_msg.angular_velocity.x,
      imu_msg.angular_velocity.y,
      imu_msg.angular_velocity.z
    );
    
    avg_linear_acceleration += Vector3(
      imu_msg.linear_acceleration.x,
      imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z
    );
  }

  
  
  avg_angular_velocity /= imu_buffer_.size();
  avg_linear_acceleration /= imu_buffer_.size();
  
  // Set biases (assuming the robot is stationary when calibrating)
  angular_velocity_bias_ = avg_angular_velocity;
  // For gravity, we need to account for the IMU orientation
  // Assuming gravity is approximately 9.81 m/s² downward in z-direction
  Vector3 expected_gravity(0, 0.0, 9.81);
  RCLCPP_INFO(get_logger(), "linear_acceleration_bias: %f %f %f",  linear_acceleration_bias_.x(), linear_acceleration_bias_.y(), linear_acceleration_bias_.z());
  // Reset odometry
  linear_velocity_ = Vector3(0, 0, 0);
  angular_velocity_ = Vector3(0, 0, 0);
  
  RCLCPP_INFO(get_logger(), "IMU bias set: angular_velocity_bias=[%f, %f, %f], linear_acceleration_bias=[%f, %f, %f]",
              angular_velocity_bias_.x(), angular_velocity_bias_.y(), angular_velocity_bias_.z(),
              linear_acceleration_bias_.x(), linear_acceleration_bias_.y(), linear_acceleration_bias_.z());
  return true;
}

void ImuOdom::setBiasCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
{
  (void)request; // Unused
  bool success = setBias();
  response->success = success;
  
  if (success) {
    response->message = "IMU bias calibration successful";
  } else {
    response->message = "IMU bias calibration failed";
  }
}

void ImuOdom::integrateImu(const sensor_msgs::msg::Imu::SharedPtr msg){
  rclcpp::Time current_time = msg->header.stamp;
  RCLCPP_INFO(get_logger(), "Integrating IMU message at time: %f", current_time.seconds());
  if (!has_first_message_) {
  RCLCPP_INFO(get_logger(), "Retrieved_first_message");
    last_time_ = current_time;
    has_first_message_ = true;
    return;
  }
  
  // Convert IMU message to Eigen vectors
  Vector3 linear_acceleration(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z
  );
  
  Vector3 angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Vector3 orientation_corrected_acceleration = (linear_acceleration * transform.so3().inverse().transpose());
  RCLCPP_INFO(get_logger(), "orientation_corrected_acceleration: %f %f %f", 
              orientation_corrected_acceleration.x(), 
              orientation_corrected_acceleration.y(), 
              orientation_corrected_acceleration.z());

  double dt = (current_time - last_time_).seconds(); 
  Vector3 processed_angular_velocity = angular_velocity - angular_velocity_bias_;
  Vector3 delta_angle = dt * (processed_angular_velocity) / 2.0;
  Sophus::SE3d delta_half_rotation = Rotation::exp(delta_angle / 2.0);
  transform_.setRotationMatrix(transform_.rotationMatrix() * delta_half_rotation.matrix());
  Vector3 linear_velocity_delta = dt * (linear_acceleration -
                         (transform_.so3() * gravity_bias_)-
                         linear_acceleration_bias_);
  auto new_acceleration_term = transform_.so3().inverse() * gravity_bias_;
  RCLCPP_INFO(get_logger(), "gravity_compensation: %f %f %f",  new_acceleration_term.x(), new_acceleration_term.y(), new_acceleration_term.z());

  transform_.translation() = (transform_.translation() + (transform_.rotationMatrix() * (dt * (linear_velocity_ + linear_velocity_delta/2.0))));
  linear_velocity_ += linear_velocity_delta;
  transform_.setRotationMatrix(transform_.rotationMatrix() * delta_half_rotation.matrix());
  // Update timestamp
  last_time_ = current_time;
  
  RCLCPP_INFO(get_logger(), "current pose: %f %f %f %f %f %f", 
              transform_.translation().x(), transform_.translation().y(), transform_.translation().z(), transform_.angleX(), transform_.angleY(), transform_.angleZ());

  publishOdomMessage(current_time);
}

void ImuOdom::publishOdomMessage(const rclcpp::Time& current_time) {
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = frame_id_;
  odom_msg.child_frame_id = child_frame_id_;
  odom_msg.pose.pose.position.x = transform_.translation().x();
  odom_msg.pose.pose.position.y = transform_.translation().y();
  odom_msg.pose.pose.position.z = transform_.translation().z();
  odom_msg.pose.covariance.fill(0.0);
  odom_msg.pose.covariance[0] = pose_covariance_;
  odom_msg.pose.covariance[7] = pose_covariance_;
  odom_msg.pose.covariance[14] = pose_covariance_;
  odom_msg.pose.covariance[21] = orientation_covariance_;
  odom_msg.pose.covariance[28] = orientation_covariance_;
  odom_msg.pose.covariance[35] = orientation_covariance_;
  odom_msg.pose.pose.orientation.x = transform_.unit_quaternion().x();
  odom_msg.pose.pose.orientation.y = transform_.unit_quaternion().y();
  odom_msg.pose.pose.orientation.z = transform_.unit_quaternion().z();
  odom_msg.pose.pose.orientation.w = transform_.unit_quaternion().w();
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
  tf_msg.transform.translation.z = transform_.translation().z();

  // Set rotation
  tf_msg.transform.rotation.x = transform_.unit_quaternion().x();
  tf_msg.transform.rotation.y = transform_.unit_quaternion().y();
  tf_msg.transform.rotation.z = transform_.unit_quaternion().z();
  tf_msg.transform.rotation.w = transform_.unit_quaternion().w();
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

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_odom::ImuOdom>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(imu_odom::ImuOdom) 
