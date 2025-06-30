#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include "lid_odom/Convert.hpp"
#include "lid_odom/Pipeline.hpp"
#include "lid_odom/tf2_sophus.hpp"
#include "lid_odom/PointUtils.hpp"

#include <vector>
#include <tuple>
#include <mutex>
#include <list>


class LidarOdomScented : public rclcpp::Node
{
public:
    LidarOdomScented() : Node("lidar_odom_scented"), last_imu_time_(0,0,RCL_ROS_TIME)
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
        declare_parameter("imu_frame", "imu_link");

        declare_parameter("publish_tf", true);
        declare_parameter("x_acceleration_bias", 0.0);
        declare_parameter("y_acceleration_bias", 0.0);
        declare_parameter("z_acceleration_bias", 0.0);
        declare_parameter("x_angular_velocity_bias", 0.0);
        declare_parameter("y_angular_velocity_bias", 0.0);
        declare_parameter("z_angular_velocity_bias", 0.0);
        declare_parameter("imu_buffer_size", 100);


        base_frame_ = get_parameter("base_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        lidar_frame_ = get_parameter("lidar_frame").as_string();
        imu_frame_ = get_parameter("imu_frame").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();
        pin_z_ = get_parameter("pin_z_axis").as_bool();
        imu_buffer_size_ = get_parameter("imu_buffer_size").as_int();

        position_covariance_ = get_parameter("position_covariance").as_double();
        orientation_covariance_ = get_parameter("orientation_covariance").as_double();

        linear_acceleration_bias_ = Eigen::Vector3d(
            get_parameter("x_acceleration_bias").as_double(),
            get_parameter("y_acceleration_bias").as_double(),
            get_parameter("z_acceleration_bias").as_double());
        
        angular_velocity_bias_ = Eigen::Vector3d(
            get_parameter("x_angular_velocity_bias").as_double(),
            get_parameter("y_angular_velocity_bias").as_double(),
            get_parameter("z_angular_velocity_bias").as_double());

        config_.max_distance = get_parameter("max_distance").as_double();
        config_.voxel_factor = get_parameter("voxel_factor").as_double();
        config_.voxel_resolution_alpha = get_parameter("voxel_resolution_alpha").as_double();
        config_.voxel_resolution_beta = get_parameter("voxel_resolution_beta").as_double();
        config_.max_points_per_voxel = get_parameter("max_points_per_voxel").as_int();

        lidar_pose_acquired_ = false;
        imu_transform_acquired_ = false;
        has_first_imu_message_ = false;
        
        pipeline_ = std::make_unique<cloud::Pipeline>(config_);

        odom_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        pcl_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/lidar_odometry", 10);
        cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map", 10);

        cloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_in", 10, 
            std::bind(&LidarOdomScented::cloudCallback, this, std::placeholders::_1));
        
        odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry_in", 10,
            std::bind(&LidarOdomScented::odomCallback, this, std::placeholders::_1));

        imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&LidarOdomScented::imuCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        
        // Initialize transforms and vectors
        imu_pose_rel_to_base_ = Sophus::SE3d();
        renormalize_imu_orientation_ = Sophus::SO3d();
        imu_transform_ = Sophus::SE3d();
        gravity_bias_ = Eigen::Vector3d(0, 0.0, -9.81);
        linear_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        angular_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        
        RCLCPP_INFO(get_logger(), "LidarOdomScented node has been initialized");
    }

private:
    // IMU integration methods
    void integrateImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp;
        
        if(!imu_transform_acquired_) {
            try {
                geometry_msgs::msg::TransformStamped imu_transform = geometry_msgs::msg::TransformStamped();
                imu_transform = buffer_->lookupTransform(
                    imu_frame_,
                    base_frame_,
                    tf2::TimePointZero, 
                    tf2::durationFromSec(0.5));
                tf2::fromMsg(imu_transform.transform, imu_pose_rel_to_base_);
                RCLCPP_INFO(get_logger(), "Successfully got IMU transform from TF tree");
                RCLCPP_INFO(get_logger(), "Transform frame: %s -> %s", imu_frame_.c_str(), base_frame_.c_str());
                
                // Print transform position and quaternion
                RCLCPP_INFO(get_logger(), "Transform position: x=%.3f, y=%.3f, z=%.3f", 
                    imu_transform.transform.translation.x,
                    imu_transform.transform.translation.y,
                    imu_transform.transform.translation.z);
                RCLCPP_INFO(get_logger(), "Transform quaternion: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                    imu_transform.transform.rotation.x,
                    imu_transform.transform.rotation.y,
                    imu_transform.transform.rotation.z,
                    imu_transform.transform.rotation.w);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(get_logger(), "Could not get IMU transform: %s", ex.what());
                return;
            }
        }
        
        if (!has_first_imu_message_) {
            RCLCPP_INFO(get_logger(), "Retrieved first IMU message");
            last_imu_time_ = current_time;
            has_first_imu_message_ = true;
            return;
        }

        Sophus::SE3d current_imu_pose = current_odometry_position  * imu_pose_rel_to_base_;
        
        // Convert IMU message to Eigen vectors
        Eigen::Vector3d linear_acceleration(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        Eigen::Vector3d imu_angular_velocity(
            msg->angular_velocity.x, 
            msg->angular_velocity.y, 
            msg->angular_velocity.z);

        Eigen::Vector3d processed_acceleration = (linear_acceleration + current_imu_pose.so3().inverse() * gravity_bias_) - linear_acceleration_bias_;
        Eigen::Vector3d processed_angular_velocity = imu_pose_rel_to_base_.so3().inverse() * (imu_angular_velocity - angular_velocity_bias_);
        double dt = (current_time - last_imu_time_).seconds(); 
        Eigen::Vector3d delta_angle = dt * (processed_angular_velocity + angular_velocity_) / 2.0;
        angular_velocity_ = processed_angular_velocity;

        Sophus::SO3d delta_half_rotation = Sophus::SO3d::exp(delta_angle / 2.0);

        imu_transform_.so3() = imu_transform_.so3() * delta_half_rotation;
        Eigen::Vector3d linear_velocity_delta = dt * (orientation_corrected_acceleration);
        imu_transform_.translation() = (imu_transform_.translation() + 
                                    (imu_transform_.rotationMatrix() * (dt * (linear_velocity_ + linear_velocity_delta/2.0))));
        linear_velocity_ += linear_velocity_delta;
        imu_transform_.so3() = imu_transform_.so3() * delta_half_rotation;
        
        last_imu_time_ = current_time;
        
        // Update the interweaved pose with IMU data
        imu_queue_mutex_.lock();
        interweaved_pose = imu_transform_;
        imu_queue_mutex_.unlock();
        
        publishOdometry();
    }

    // Subscriber callbacks
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Store the message in the buffer
        if (imu_buffer_.size() >= imu_buffer_size_) {
            imu_buffer_.pop_front();
        }
        imu_buffer_.push_back(*msg);
        // Process the IMU data
        integrateImu(msg);
    }
    
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
                RCLCPP_INFO(get_logger(), "Lidar pose relative to base: Position: [%.3f, %.3f, %.3f], Quaternion: [%.3f, %.3f, %.3f, %.3f]", 
                           lidar_pose_rel_to_base_.translation().x(), lidar_pose_rel_to_base_.translation().y(), lidar_pose_rel_to_base_.translation().z(),
                           lidar_pose_rel_to_base_.unit_quaternion().x(), lidar_pose_rel_to_base_.unit_quaternion().y(), 
                           lidar_pose_rel_to_base_.unit_quaternion().z(), lidar_pose_rel_to_base_.unit_quaternion().w());
            }
            catch(const std::exception & e){
                RCLCPP_ERROR(get_logger(), "Failed to lookup transform from %s to %s: %s", 
                base_frame_.c_str(), lidar_frame_.c_str(), e.what());
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
        
        // Use combined IMU and wheel odometry for best guess
        imu_queue_mutex_.lock();
        best_guess_odometry = interweaved_pose;
        imu_queue_mutex_.unlock();
        
        auto odometry_update  = pipeline_->odometryUpdate(cloud_processed, best_guess_odometry, false);
        Sophus::SE3d new_pose = std::get<0>(odometry_update);
        
        imu_queue_mutex_.lock();
        interweaved_pose = new_pose;
        RCLCPP_INFO(get_logger(), "Interweaved pose - Position: [%.3f, %.3f, %.3f], Quaternion: [%.3f, %.3f, %.3f, %.3f]", 
                   interweaved_pose.translation().x(), interweaved_pose.translation().y(), interweaved_pose.translation().z(),
                   interweaved_pose.unit_quaternion().x(), interweaved_pose.unit_quaternion().y(), 
                   interweaved_pose.unit_quaternion().z(), interweaved_pose.unit_quaternion().w());
        imu_queue_mutex_.unlock();
        
        publishOdometry();
        publishCloud(cloud_publisher_, pipeline_->getMap());
    }

    void imuCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received odometry message");
        auto data_tuple = cloud::extractOdometryData(msg);
        Sophus::SE3d new_pose = std::get<0>(data_tuple);
        
        odom_queue_mutex.lock();
        odom_pose_differential = new_pose * last_odometry_pose.inverse();
        last_odometry_pose = new_pose;
        
        // Combine with IMU data in the interweaved pose
        imu_queue_mutex_.lock();
        interweaved_pose = interweaved_pose * odom_pose_differential;
        RCLCPP_INFO(get_logger(), "Interweaved pose - Position: [%.3f, %.3f, %.3f], Quaternion: [%.3f, %.3f, %.3f, %.3f]", 
                   interweaved_pose.translation().x(), interweaved_pose.translation().y(), interweaved_pose.translation().z(),
                   interweaved_pose.unit_quaternion().x(), interweaved_pose.unit_quaternion().y(), 
                   interweaved_pose.unit_quaternion().z(), interweaved_pose.unit_quaternion().w());
        imu_queue_mutex_.unlock();
        
        odom_queue_mutex.unlock();
        publishOdometry();
    }

    void publishOdometry(){
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
    
        imu_queue_mutex_.lock();
        Eigen::Vector3d position = interweaved_pose.translation();
        Eigen::Quaterniond orientation = interweaved_pose.unit_quaternion();
        imu_queue_mutex_.unlock();
    
        odom_msg.pose.pose.position.x = position.x();
        odom_msg.pose.pose.position.y = position.y();
        odom_msg.pose.pose.position.z = pin_z_ ? 0.0 : position.z();
    
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
    
        // Get velocity from IMU
        imu_queue_mutex_.lock();
        Eigen::Vector3d linear_vel = linear_velocity_;
        Eigen::Vector3d angular_vel = angular_velocity_;
        imu_queue_mutex_.unlock();
        
        odom_msg.twist.twist.linear.x = linear_vel.x();
        odom_msg.twist.twist.linear.y = linear_vel.y();
        odom_msg.twist.twist.linear.z = pin_z_ ? 0.0 : linear_vel.z();
        odom_msg.twist.twist.angular.x = angular_vel.x();
        odom_msg.twist.twist.angular.y = angular_vel.y();
        odom_msg.twist.twist.angular.z = angular_vel.z();
    
        odom_publisher_->publish(std::move(odom_msg));
    
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
                      const std::vector<Eigen::Vector3d> &cloud){
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.frame_id = odom_frame_;
        cloud_msg->header.stamp = this->now();
        cloud::convertCloudToMsg(cloud, cloud_msg);
        publisher->publish(*cloud_msg);
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    
    // Callback groups
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_; 
    rclcpp::CallbackGroup::SharedPtr pcl_callback_group_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> buffer_;

    // IMU data
    std::list<sensor_msgs::msg::Imu> imu_buffer_;
    size_t imu_buffer_size_{100};
    rclcpp::Time last_imu_time_;
    bool has_first_imu_message_{false};
    bool imu_transform_acquired_{false};
    std::mutex imu_queue_mutex_;
    
    Eigen::Vector3d gravity_bias_{0, 0.0, -9.81};
    Eigen::Vector3d linear_acceleration_bias_{0.0, 0.0, 0.0};
    Eigen::Vector3d angular_velocity_bias_{0.0, 0.0, 0.0};
    Eigen::Vector3d linear_velocity_{0.0, 0.0, 0.0};
    Eigen::Vector3d angular_velocity_{0.0, 0.0, 0.0};
    Sophus::SE3d imu_pose_rel_to_base_;
    Sophus::SE3d imu_transform_;

    std::tuple<Sophus::SE3d, Eigen::Vector3d, Eigen::Vector3d, double> odometry_reading;
    std::mutex pose_mutex;
    Sophus::SE3d odom_pose_differential;
    Sophus::SE3d interweaved_pose;
    Sophus::SE3d last_odometry_pose;

    bool lidar_pose_acquired_ =  false;
    Sophus::SE3d lidar_pose_rel_to_base_;

    cloud::PipelineConfig config_;
    std::unique_ptr<cloud::Pipeline> pipeline_;

    double last_processsed_framestamp;
    
    double position_covariance_;
    double orientation_covariance_;

    std::string base_frame_;
    std::string odom_frame_;
    std::string lidar_frame_;
    std::string imu_frame_;
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