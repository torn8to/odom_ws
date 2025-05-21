#include "imu_odom/ImuOdom.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_odom::ImuOdom>());
  rclcpp::shutdown();
  return 0;
}