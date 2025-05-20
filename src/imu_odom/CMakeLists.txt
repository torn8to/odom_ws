cmake_minimum_required(VERSION 3.16)
project(imu_odom VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find Eigen3
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# Find system-installed Sophus
find_package(Sophus REQUIRED)

add_library(imu_odometry SHARED
  src/ImuOdom.cpp
)

target_compile_features(imu_odometry PUBLIC cxx_std_20)
target_include_directories(imu_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu_odometry
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  geometry_msgs
)

# Link Eigen3 and Sophus
target_link_libraries(imu_odometry
  Eigen3::Eigen
  Sophus::Sophus
)

rclcpp_components_register_nodes(imu_odometry "imu_odom::ImuOdom")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS imu_odometry
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
