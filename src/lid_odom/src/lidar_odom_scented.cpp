#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include "Convert.hpp"
#include "Pipeline.hpp"
#include "tf2_sophus.hpp"
#include "PointUtils.hpp"

#include <vector>
#include <tuple>
#include <mutex>


class LidarOdomScented : public rclcpp::Node
{
public:
    LidarOdomScented() : Node("lidar_odom_scented")
    {
        declare_parameter("max_distance", 30.0);
        declare_parameter("voxel_factor", 100.0);
        declare_parameter("voxel_resolution_alpha", 1.5);
        declare_parameter("voxel_resolution_beta", 0.5);
        declare_parameter("max_points_per_voxel", 27);
        declare_parameter("position_covariance", 0.1);
        declare_parameter("orientation_covariance", 0.1);
        declare_parameter("initial_threshold", 2.0);
        declare_parameter("min_motion_threshod", 0.1);
        declare_parameter("base_frame", "base_link");
        declare_parameter("odom_frame", "lid_odom");
        declare_parameter("lidar_frame", "rslidar");
        declare_parameter("publish_tf", true);


        base_frame_ = get_parameter("base_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        lidar_frame_ = get_parameter("lidar_frame").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();

        position_covariance_ = get_parameter("position_covariance").as_double();
        position_covariance_ = get_parameter("orientation_covariance").as_double();

        config_.max_distance = get_parameter("max_distance").as_double();
        config_.voxel_factor = get_parameter("voxel_factor").as_double();
        config_.voxel_resolution_alpha = get_parameter("voxel_resolution_alpha").as_double();
        config_.voxel_resolution_beta = get_parameter("voxel_resolution_beta").as_double();
        config_.max_points_per_voxel = get_parameter("max_points_per_voxel").as_int();

        lidar_pose_acquired_ = false; 
        
        pipeline_ = std::make_unique<cloud::Pipeline>(config_);

        odom_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        pcl_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("LidarOdometrty", 10);
        cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("Map", 10);

        cloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", 10, 
            std::bind(&LidarOdomScented::cloudCallback, this, std::placeholders::_1));
        
        odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10,
            std::bind(&LidarOdomScented::odomCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        RCLCPP_INFO(get_logger(), "LidarOdomScented node has been initialized");
    }

private:
    // Subscriber callbacks
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        RCLCPP_INFO(get_logger(), "Received point cloud with %d points", 
                   msg->width * msg->height);

        if(!lidar_pose_acquired_){
            RCLCPP_WARN(get_logger(), "Lidar pose not acquired trying to reacquire");
            try{
                geometry_msgs::msg::TransformStamped transform_stamped = buffer_->lookupTransform(
                                                        lidar_frame_,
                                                        base_frame_,
                                                        tf2::TimePointZero);
                lidar_pose_rel_to_base_ = tf2::transformToSophus(transform_stamped);
                lidar_pose_acquired_ = true;
            }
            catch(const std::exception & e){
                RCLCPP_ERROR(get_logger(), "Failed to lookup transform from %s to %s: %s", 
                base_frame_.c_str(), lidar_frame_.c_str(), e.what());\
                return;
            }
        }// end of if
        std::vector<Eigen::Vector3d> cloud_raw;
        std::vector<double> timestamps;
        cloud_raw = cloud::convertMsgToCloud(msg);
        timestamps = cloud::extractTimestampsFromCloudMsg(msg);
        std::vector<Eigen::Vector3d> cloud_transformed = cloud::transformPoints(cloud_raw,
                                                                    lidar_pose_rel_to_base_);
        std::vector<Eigen::Vector3d> &cloud_processed = cloud_transformed;
        Sophus::SE3d best_guess_odometry;
        odom_queue_mutex.lock();
        best_guess_odometry = interweaved_pose;
        odom_queue_mutex.unlock();
        auto odometry_update  = pipeline_->odometryUpdate(cloud_processed, best_guess_odometry, false);
        Sophus::SE3d new_pose = std::get<0>(odometry_update);
        odom_queue_mutex.lock();
        interweaved_pose = new_pose;
        odom_queue_mutex.unlock();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received odometry message");
        auto data_tuple = cloud::extractOdometryData(msg);
        Sophus::SE3d new_pose = std::get<0>(data_tuple);
        odom_queue_mutex.lock();
        odom_pose_differential = new_pose * last_odometry_pose.inverse();
        last_odometry_pose = new_pose;
        interweaved_pose = interweaved_pose * odom_pose_differential;
        odom_queue_mutex.unlock();
    }

    void publishOdometry(){
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
    
        Eigen::Vector3d position = interweaved_pose.translation();
        Eigen::Quaterniond orientation = interweaved_pose.unit_quaternion();
    
        odom_msg.pose.pose.position.x = position.x();
        odom_msg.pose.pose.position.y = position.y();
        odom_msg.pose.pose.position.z = position.z();
    
        odom_msg.pose.pose.orientation.x = orientation.x();
        odom_msg.pose.pose.orientation.y = orientation.y();
        odom_msg.pose.pose.orientation.z = orientation.z();
        odom_msg.pose.pose.orientation.w = orientation.w();
        
        odom_msg.pose.covariance[0] = position_covariance_;   // x position variance
        odom_msg.pose.covariance[7] = position_covariance_;   // y position variance
        odom_msg.pose.covariance[14] = position_covariance_;  // z position variance
        odom_msg.pose.covariance[21] = orientation_covariance_; // x orientation variance
        odom_msg.pose.covariance[28] = orientation_covariance_; // y orientation variance
        odom_msg.pose.covariance[35] = orientation_covariance_; // z orientation variance
        
        odom_msg.twist.covariance.fill(0.0);
        odom_msg.twist.covariance[0] = position_covariance_;   // x linear velocity variance
        odom_msg.twist.covariance[7] = position_covariance_;   // y linear velocity variance
        odom_msg.twist.covariance[14] = position_covariance_;  // z linear velocity variance
        odom_msg.twist.covariance[21] = orientation_covariance_; // x angular velocity variance
        odom_msg.twist.covariance[28] = orientation_covariance_; // y angular velocity variance
        odom_msg.twist.covariance[35] = orientation_covariance_; // z angular velocity variance
    
        odom_queue_mutex.lock();
        Sophus::SE3d pose_diff = odom_pose_differential;
        odom_queue_mutex.unlock();
    
        Eigen::Vector3d linear_velocity = pose_diff.translation();
        Eigen::Vector3d angular_velocity = pose_diff.so3().log();
        odom_msg.twist.twist.linear.x = linear_velocity.x();
        odom_msg.twist.twist.linear.y = linear_velocity.y();
        odom_msg.twist.twist.linear.z = linear_velocity.z();
        odom_msg.twist.twist.angular.x = angular_velocity.x();
        odom_msg.twist.twist.angular.y = angular_velocity.y();
        odom_msg.twist.twist.angular.z = angular_velocity.z();
    
        odom_publisher_->publish(odom_msg);
    
        if (publish_tf_) {
            publishTF(std::make_shared<nav_msgs::msg::Odometry>(odom_msg));
        }
    }

    void publishTF(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = msg->header;
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_frame_;
        
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(transform);
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
                      std::vector<Eigen::Vector3d> &cloud){
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.frame_id = odom_frame_;
        cloud_msg->header.stamp = this->now();
        cloud::convertCloudToMsg(cloud, cloud_msg);
        publisher->publish(*cloud_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_; 
    rclcpp::CallbackGroup::SharedPtr pcl_callback_group_; 

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> buffer_;


    std::tuple<Sophus::SE3d, Eigen::Vector3d, Eigen::Vector3d, double> odometry_reading;

    std::mutex odom_queue_mutex;
    Sophus::SE3d odom_pose_differential;
    Sophus::SE3d interweaved_pose;
    Sophus::SE3d last_odometry_pose;

    bool lidar_pose_acquired_;
    Sophus::SE3d  lidar_pose_rel_to_base_;

    cloud::PipelineConfig config_;
    std::unique_ptr<cloud::Pipeline> pipeline_;

    double last_processsed_framestamp;
    
    double position_covariance_;
    double orientation_covariance_;

    std::string base_frame_;
    std::string odom_frame_;
    std::string lidar_frame_;
    bool publish_tf_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarOdomScented>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}