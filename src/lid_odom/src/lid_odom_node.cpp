#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer>

#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/geometry>
#include <sophus/se3.hpp>

#include "Pipeline.hpp"
#include "Convert.hpp"

namespace lid_odom {

class LidarOdometryNode : public rclcpp::Node {
public:
  explicit LidarOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lidar_odometry_mapping", options),
    buffer_(this->get_clock())
  {
    // Declare parameters
    declare_parameter("max_distance", 30.0);
    declare_parameter("voxel_factor", 100.0);
    declare_parameter("voxel_resolution_alpha", 1.5);
    declare_parameter("voxel_resolution_beta", 0.5);
    declare_parameter("max_points_per_voxel", 27);
    declare_parameter("frame_id", "lid_odom");
    declare_parameter("child_frame_id", "base_link");
    declare_parameter("lidar_link","lidar_link");
    declare_parameter("imu_integration_enabled", false);
    
    // Get parameters
    cloud::PipelineConfig config;
    config.max_distance = get_parameter("max_distance").as_double();
    config.voxel_factor = get_parameter("voxel_factor").as_double();
    config.voxel_resolution_alpha = get_parameter("voxel_resolution_alpha").as_double();
    config.voxel_resolution_beta = get_parameter("voxel_resolution_beta").as_double();
    config.max_points_per_voxel = get_parameter("max_points_per_voxel").as_int();
    config.imu_integration_enabled = get_parameter("imu_integration_enabled").as_bool();
    
    frame_id_ = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();
    
    // Initialize pipeline
    pipeline_ = std::make_unique<cloud::Pipeline>(config);
    
    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_listener = std::make_unique<tf2_ros::TransformListener>(buffer_);
    
    // Create subscribers
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", 10, 
      std::bind(&LidarOdometryNode::pointCloudCallback, this, std::placeholders::_1));
    
    // Create publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("voxel_map", 10);

    
    RCLCPP_INFO(get_logger(), "Lidar Odometry Node initialized");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received point cloud with %u points", 
                msg->width * msg->height);
    
    // Convert PointCloud2 to vector of Eigen::Vector3d
    std::vector<Eigen::Vector3d> points = cloud::convertMsgToCloud(msg);
    std::vector<Eigen::Vector3d> lidar_transformed_points = transform_point_cloud_data(points);
    
    // Get current pose
    Sophus::SE3d current_pose = pipeline_->position();
    
    // Update odometry
    Sophus::SE3d updated_pose = pipeline_->odometryUpdate(points, current_pose);
    
    // Add points to map
    pipeline_->addToMap(points);
    
    // Publish odometry
    publishOdometry(msg->header.stamp, updated_pose);
  }

  void publishMap()
  {
    // Get map points from pipeline
    std::vector<Eigen::Vector3d> map_points = pipeline_->getMap();
    
    // Convert to PointCloud2 message
    auto map_msg = cloud::convertCloudToMsg(map_points);
    
    // Set header information
    map_msg.header.stamp = now();
    map_msg.header.frame_id = frame_id_;
    
    // Publish map
    map_pub_->publish(map_msg);
  }

  std::vector<Eigen::Vector3d> transform_point_cloud_data(std::vector<Eigen::Vector3d> &points){
    std::vector<Eigen::Vector3d> transformed_points;
    transformed_points.reserve(points.size());
    for(Eigen::Vector3d const &point: points){
      transformed_points.emplace_back(point);
    }
    return transformed_points;

  }
  
  
  void publishOdometry(const rclcpp::Time & timestamp, const Sophus::SE3d & pose)
  {
    // Create and publish odometry message
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = timestamp;
    odom_msg->header.frame_id = frame_id_;
    odom_msg->child_frame_id = child_frame_id_;
    
    // Set position
    odom_msg->pose.pose.position.x = pose.translation().x();
    odom_msg->pose.pose.position.y = pose.translation().y();
    odom_msg->pose.pose.position.z = pose.translation().z();
    
    // Convert rotation to quaternion
    Eigen::Quaterniond quat(pose.rotationMatrix());
    odom_msg->pose.pose.orientation.w = quat.w();
    odom_msg->pose.pose.orientation.x = quat.x();
    odom_msg->pose.pose.orientation.y = quat.y();
    odom_msg->pose.pose.orientation.z = quat.z();
    
    // Publish odometry message
    odom_pub_->publish(std::move(odom_msg));
    
    // Publish transform
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = timestamp;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = child_frame_id;
    
    transform_stamped.transform.translation.x = pose.translation().x();
    transform_stamped.transform.translation.y = pose.translation().y();
    transform_stamped.transform.translation.z = pose.translation().z();
    
    transform_stamped.transform.rotation.w = quat.w();
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    
    tf_broadcaster_->sendTransform(transform_stamped);
  }
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  
  // Transform broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // Transform Lookup
  tf2_ros::Buffer buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Pipeline for LiDAR odometry
  std::unique_ptr<cloud::Pipeline> pipeline_;
  
  // Frame IDs
  std::string frame_id_;
  std::string child_frame_id_;
  std::string lidar_link_;
};

} // namespace lid_odom

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lid_odom::LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
