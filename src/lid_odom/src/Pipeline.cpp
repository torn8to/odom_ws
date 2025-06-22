#include "Pipeline.hpp"
#include "VoxelUtils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace cloud {

Pipeline::Pipeline(const PipelineConfig &config)
  : registration_(config.num_iterations, config.max_points_per_voxel, config.convergence, config.num_threads),
    current_position_(Sophus::SE3d()),
    voxel_factor_(config.voxel_factor),
    max_distance_(config.max_distance),
    voxel_resolution_alpha_(config.voxel_resolution_alpha),
    voxel_resolution_beta_(config.voxel_resolution_beta),
    imu_integration_enabled_(config.imu_integration_enabled),
    max_points_per_voxel_(config.max_points_per_voxel),
    voxel_map_(cloud::VoxelMap((config.max_distance/config.voxel_factor) * config.voxel_resolution_alpha,
    config.max_distance_ ,config.max_points_per_voxel)),
{
  // Any additional initialization if needed
}

Pipeline::~Pipeline() {
  // Cleanup if needed
}


std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>> Pipeline::odometryUpdate(std::vector<Eigen::Vector3d> &cloud){
  // Create registration object for point cloud alignment down 
  // sample via odom and downsample via mapping to create
  // run alignment with odometry downsampling and update via 
  if(voxel_map_.empty()){
    std::vector<Eigen::Vector3d> cloud_voxel_mapping = voxelDownsample(cloud,
   max_distance_ / voxel_factor_ * voxel_resolution_alpha_,1);
    voxel_map_.addPoints(cloud_voxel_mapping);

  return std::make_tuple(position(), cloud_voxel_mapping);
  }
  std::vector<Eigen::Vector3d> cloud_voxel_odom = voxelDownsample(cloud,
   max_distance_ / voxel_factor_ * voxel_resolution_beta_, max_distance_,1);

  std::vector<Eigen::Vector3d> cloud_voxel_mapping = voxelDownsample(cloud,
   max_distance_ / voxel_factor_ * voxel_resolution_alpha_, max_distance_
    1);

  Sophus::SE3d new_position = registration_.alignPointsToMap(
    cloud_voxel_odom,
    voxel_map_,
    position(),
    0.1,
    1.0);

  Sophus::SE3d pose_diff = new_position * current_position_.inverse();
  RCLCPP_INFO(rclcpp::get_logger("pipeline"), "Position differential: translation=[%f, %f, %f]",
    pose_diff.translation().x(), pose_diff.translation().y(), pose_diff.translation().z());
  std::vector<Eigen::Vector3d> cloud_voxel_mapping_transformed = voxel_map_.transform_cloud(cloud_voxel_mapping, new_position);
  voxel_map_.addPoints(cloud_voxel_mapping_transformed);
  updatePosition(new_position);
  return std::make_tuple(new_position, cloud_voxel_mapping);
}


  std::vector<Eigen::Vector3d> Pipeline::removePointsFarFromLocation(const std::vector<Eigen::Vector3d> &cloud,
                                                                     const Eigen::Vector3d &point){
    std::vector<Eigen::Vector3d> pruned_points;
    pruned_points.reserve(cloud.size());
    for(Eigen::Vector3d cloud_point: cloud){
      if((cloud_point - point).squaredNorm() <= max_distance_ * max_distance_){
        pruned_points.push_back(cloud_point);
      }
    }
    return pruned_points;
  }

void Pipeline::addToMap(const std::vector<Eigen::Vector3d> &points) {
  // Add points to the global map
  voxel_map_.addPoints(points);
}


Sophus::SE3d Pipeline::position() const {
  return current_position_;
}

void Pipeline::updatePosition(const Sophus::SE3d &transformation_matrix) {
  current_position_ = transformation_matrix;
}

} // namespace cloud
