#include "Pipeline.hpp"
#include "VoxelUtils.hpp"
#include <sstream>

namespace cloud {

Pipeline::Pipeline(const PipelineConfig &config)
  : registration_(config.num_iterations,  config.convergence, config.num_threads),
    current_position_(Sophus::SE3d()),
    voxel_factor_(config.voxel_factor),
    max_distance_(config.max_distance),
    voxel_resolution_alpha_(config.voxel_resolution_alpha),
    voxel_resolution_beta_(config.voxel_resolution_beta),
    imu_integration_enabled_(config.imu_integration_enabled),
    max_points_per_voxel_(config.max_points_per_voxel),
    odom_voxel_downsample_(config.odom_downsample),
    voxel_map_((config.max_distance/config.voxel_factor) * config.voxel_resolution_alpha,
    config.max_distance ,
    config.max_points_per_voxel),
    threshold(config.initial_threshold, config.min_motion_threshold, config.max_distance){}

Pipeline::~Pipeline() {
  // Cleanup if needed
}


std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>> Pipeline::odometryUpdate(std::vector<Eigen::Vector3d> &cloud){
  // Create registration object for point cloud alignment down 
  // sample via odom and downsample via mapping to create
  // run alignment with odometry downsampling and update via 
  std::vector<Eigen::Vector3d> cloud_voxel_odom;
if (odom_voxel_downsample_){
  cloud_voxel_odom = voxelDownsample(cloud, max_distance_ / voxel_factor_ * voxel_resolution_beta_);
}
else{
  cloud_voxel_odom = cloud;
}

  std::vector<Eigen::Vector3d> cloud_voxel_mapping = voxelDownsample(cloud,max_distance_ / voxel_factor_ * voxel_resolution_alpha_);
   


  auto initial_guess = pose_diff_ * position();
  const double sigma = threshold.computeThreshold();

  Sophus::SE3d new_position = registration_.alignPointsToMap(
    cloud_voxel_odom,
    voxel_map_,
    position(),//initial_guess,
    3.0 * sigma,
    sigma);

  pose_diff_ = initial_guess.inverse() * new_position;
  
  threshold.updateModelDeviation(pose_diff_);
  // Convert matrices to strings for logging
  std::stringstream pose_diff_ss;
  pose_diff_ss << pose_diff_.matrix();
  
  std::stringstream new_pos_ss;
  new_pos_ss << new_position.matrix();

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

void Pipeline::updatePosition(const Sophus::SE3d transformation_matrix) {
  current_position_ = transformation_matrix;
}

} // namespace cloud
