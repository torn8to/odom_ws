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

#include <vector>
#include <tuple>
#include <mutex>

class LidarOdomScented : public rclcpp::Node
{
public:
    LidarOdomScented() : Node("lidar_odom_scented")
    {
        declare_parameter("base_frame", "base_link");
        declare_parameter("odom_frame", "odom");
        declare_parameter("lidar_frame", "lidar");
        declare_parameter("publish_tf", true);
        declare_parameter("odometry_feedback")
        
        base_frame_ = get_parameter("base_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        lidar_frame_ = get_parameter("lidar_frame").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();

        odom_callback_group_ = create_callbackgroup(rclcpp::CallbackGroupType::MutuallyExclusive);
        point_callback_group_ = create_callbackgroup(rclcpp::CallbackGroupType::MutuallyExclusive);

        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("LidarOdometrty", 10);
        cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("Map", 10);

        cloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", 10, 
            std::bind(&LidarOdomScented::cloudCallback, this, std::placeholders::_1));
        
        odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10,
            std::bind(&LidarOdomScented::odomCallback, this, std::placeholders::_1));

        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(), "LidarOdomScented node has been initialized");
    }

private:
    // Subscriber callbacks
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received point cloud with %d points", 
                   msg->width * msg->height);
        std::vector<Eigen::Vector3d> cloud_raw;
        std::vector<Eigen::Vector3d> timestamps;
        cloud_raw = cloud::convertMsgToCloud(msg);
        timestamps = cloud::extractTimestampsFromCloudMsg(msg);
        odom_queue_mutex.lock();
        //TODO: get diff between last and curren_position

        odom_queue_mutex.unlock();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received odometry message");
        auto data_tuple = extractOdometryData(msg);
        Sophus::SE3d pose = std::get<0>(data_tuple);
        odom_queue_mutex.lock();
        odom_pose_differential = pose * last_pose_odometry.inverse();
        last_odometry_pose = pose_differential * last_odometry_pose.inv();
        odom_queue_mutex.unlock();
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

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_; 
    rclcpp::CallbackGroup::SharedPtr pcl_callback_group_; 

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::tuple<Sophus::SE3d, Eigen::Vector3d, Eigen::Vector3d, double> odometry_reading;

    Sophus::SE3d odom_pose_differential;
    Sophus::SE3d point_cloud_pose;
    Sophus::SE3d last_odometry_pose;

    int odometry_differential_queue_length = 200;
    std::mutex odom_queue_mutex;

    double last_processsed_framestamp;
    
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
