#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "imu_odom/ImuOdom.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<imu_odom::ImuOdom>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 