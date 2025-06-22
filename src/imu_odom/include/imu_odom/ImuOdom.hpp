#ifndef IMU_ODOM_HPP_
#define IMU_ODOM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include <rclcpp/time.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <list>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
#include <std_srvs/srv/trigger.hpp>


namespace imu_odom
{
class ImuOdom : public rclcpp::Node
{
public:


  /**
   * @brief Constructor for the ImuOdom node
   * @param options Node options for ROS2 node initialization
   */
  explicit ImuOdom();

  /**
   * @brief Callback function for processing IMU messages
   * @param msg Shared pointer to the received IMU message
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief Publishes the current odometry message updates to tf tree with the frame_id and child_frame_id
   * @param current_time Current timestamp for the odometry message
   */
  void publishOdomMessage(const rclcpp::Time& current_time);

  /**
   * @brief Publishes the current odometry message updates to tf tree with the frame_id and child_frame_id
   * @param current_time Current timestamp for the odometry message
   */
  void publishTransform(const rclcpp::Time& current_time);

private:
  /**
   * @brief Integrates IMU data to update the current pose and integrate over the
   * @param msg Shared pointer to the received IMU message
   */
  void integrateImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief Sets the biases for the IMU sensor based on current readings
   * @return True if bias setting was successful
   */
  bool setBias();

  /**
   * @brief Service callback to set IMU biases
   * @param request Service request (unused)
   * @param response Service response with success status
   */
  void setBiasCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Type definitions using Eigen and Sophus

  // Subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // TF2 transform lookup
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Buffer for IMU messagesa
  std::list<sensor_msgs::msg::Imu> imu_buffer_;
  size_t imu_buffer_size_{100};
  
  // State variables
  rclcpp::Time last_time_;
  bool has_first_message_{false};
  bool has_orientation_{false};
  Eigen::Vector3d gravity_bias_{0, 0.0, -9.81};
  Eigen::Vector3d linear_acceleration_bias_{0.0, 0.0, 0.0};
  Eigen::Vector3d angular_velocity_bias_{0.0, 0.0, 0.0};
  Eigen::Vector3d linear_velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d angular_velocity_{0.0, 0.0, 0.0};

  double pose_covariance_{0.1};
  double orientation_covariance_{0.1};

  bool pin_z = false;
  bool imu_transform_acquired{false};
  Sophus::SE3d imu_base_link_transform_;
  Sophus::SO3d renormalize_imu_orientation_;
  Sophus::SE3d transform_; 

  std::string frame_id_{"map"};
  std::string child_frame_id_{"odom_imu"};
  std::string imu_frame_id_{"xsens_imu_link"};
  std::string base_frame_id_{"base_link"};
  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_bias_service_;
};

}  // namespace imu_odom

#endif  // IMU_ODOM_HPP_
