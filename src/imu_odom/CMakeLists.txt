cmake_minimum_required(VERSION 3.16)
project(imu_odom VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-g -Wno-unused-variable)

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
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(std_srvs REQUIRED)

# Create the library
add_library(${PROJECT_NAME}_lib SHARED
  src/ImuOdom.cpp
)

target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_eigen
  geometry_msgs
  std_srvs
)

target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
  Sophus::Sophus
)

# Register the component
rclcpp_components_register_nodes(${PROJECT_NAME}_lib "imu_odom::ImuOdom")

# Create the executable
add_executable(${PROJECT_NAME}_node src/ImuOdomNode.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_eigen
  geometry_msgs
  std_srvs
  Eigen3
  Sophus
)

ament_package()
