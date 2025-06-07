#include "Pipeline.hpp"
#include "VoxelUtils.hpp"

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
    voxel_map_(cloud::VoxelMap((config.max_distance/config.voxel_factor) * config.voxel_resolution_alpha, config.max_points_per_voxel)),
    voxel_odom_(cloud::VoxelMap((config.max_distance/config.voxel_factor) * config.voxel_resolution_beta, config.max_points_per_voxel))
{
  // Any additional initialization if needed
}

Pipeline::~Pipeline() {
  // Cleanup if needed
}

void Pipeline::update(std::vector<Eigen::Vector3d> &cloud) {
  // Store the current cloud as the last measurement
  std::vector<Eigen::Vector3d> cloud_voxel_odom = voxelDownsample(cloud, max_distance_ / voxel_factor_ * voxel_resolution_beta_, 1);
  std::vector<Eigen::Vector3d> cloud_voxel_mapping = voxelDownsample(cloud, max_distance_ / voxel_factor_ * voxel_resolution_alpha_, 1);
  // Add points to the map
  

  addToMap(cloud);
}

Sophus::SE3d Pipeline::odometryUpdate(std::vector<Eigen::Vector3d> &update_cloud, Sophus::SE3d &update_pose) {
  // Create registration object for point cloud alignment
  std::vector<Eigen::Vector3d> update_cloud_voxel = voxelDownsample(update_cloud, 
                             max_distance_ / voxel_factor_ * voxel_resolution_beta_, 
                             3);

  
  // Clear previous odometry voxel map
  voxel_odom_.clear();
  
  // Add current cloud to odometry voxel map
  voxel_odom_.addPoints(update_cloud);
  
  // Perform point cloud registration to align the new cloud with the map
  Sophus::SE3d result = registration_.alignPointsToMap(
    update_cloud,
    voxel_map_,
    update_pose,
    (max_distance_ / voxel_factor_) * 2.0, // Max correspondence distance
    1.0  // Kernel scale
  );
  
  // Update the current position
  current_position_ = result;
  
  return result;
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
