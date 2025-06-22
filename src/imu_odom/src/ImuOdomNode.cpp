#include "imu_odom/ImuOdom.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imu_odom::ImuOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}