#pragma once

#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "VoxelMap.hpp"
#include "Registration.hpp"
#include "Threshold.hpp"


namespace cloud {

struct PipelineConfig {
  double max_distance = 30.0;
  double voxel_factor = 100;
  double voxel_resolution_alpha = 1.5;
  double voxel_resolution_beta = 0.5; // beta is recommended to be smaller for better odom updates
  bool imu_integration_enabled = false;
  int max_points_per_voxel = 27;
  int num_threads = 8;
  int num_iterations = 500;
  double convergence = 1e-4;
  bool odom_downsample = true;
  double initial_threshold = 1.0;
  double min_motion_threshold =  0.1;
};

class Pipeline {
public:
  /**
   * @brief Constructor for the Pipeline class
   * @param config Configuration parameters for the pipeline
   */
  explicit Pipeline(const PipelineConfig &config = PipelineConfig());
  
  /**
   * @brief Destructor for the Pipeline class
   */
  ~Pipeline();

  /**
   * @brief Performs an odometry update using the existing pose
   *
   * This method performs an odometry update using the existing pose and the new cloud.
   * It first performs a voxelization of the new cloud and then performs a nearest neighbor search
   * to find the closest points in the voxel map.
   *
   * @param update_cloud The new cloud to update odometry
   * @param update_pose The current pose of the robot
   * @return The updated pose of the robot
   */
  std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>> odometryUpdate(std::vector<Eigen::Vector3d> &cloud);


  /**
   * @brief Adds points to the map
   * @param points The points to add to the map
   */
  void addToMap(const std::vector<Eigen::Vector3d> &points);

  /**
   * @brief prunes points further then the max_distnace stored in the cloud 
   * @param points a point cloud
   * @param pouint the point relative to which points are being removed
   */
  std::vector<Eigen::Vector3d> removePointsFarFromLocation(const std::vector<Eigen::Vector3d> &cloud,
                                                            const Eigen::Vector3d &point);
  /**
   * @brief Gets the map points
   * @return Vector of points in the map
   */
  std::vector<Eigen::Vector3d> getMap(){
    return voxel_map_.cloud();
  }

  /**
   * @brief Removes points from a cloud that are beyond the maximum distance from origin
   * 
   * This method filters out points from the input cloud that have a Euclidean distance
   * from the origin (0,0,0) greater than the maximum distance threshold. Points are
   * kept if their norm (distance from origin) is less than max_distance_.
   * 
   * @param cloud The input point cloud to filter
   * @return A filtered point cloud containing only points within the maximum distance
   */

std::vector<Eigen::Vector3d> removeFarPoints(std::vector<Eigen::Vector3d> &cloud){
  std::vector<Eigen::Vector3d> pruned_cloud;
  pruned_cloud.reserve(cloud.size());
  std::for_each(cloud.begin(),cloud.end(),
  [&](const auto point){
    if(point.norm()<max_distance_){pruned_cloud.push_back(point);}});
  return pruned_cloud;
  }

 inline bool mapEmpty(){
    return voxel_map_.empty();
  }

  /**
   * @brief Gets the current position transformation
   * @return The current position transformation matrix
   */
  Sophus::SE3d position() const;

  /**
   * @brief Sets the current position
   * @param transformation_matrix The transformation matrix to set as current position
   */
  void updatePosition(const Sophus::SE3d transformation_matrix);

private:
  Registration registration_;
  Sophus::SE3d current_position_;
  Sophus::SE3d pose_diff_;
  AdaptiveThreshold threshold;
  double voxel_factor_;
  double max_distance_;
  double voxel_resolution_alpha_; // voxel resolution for the alpha layer
  double voxel_resolution_beta_; // voxel resolution for the beta layer
  bool imu_integration_enabled_;
  int max_points_per_voxel_;
  bool odom_voxel_downsample_;
  cloud::VoxelMap voxel_map_;
};

} // namespace cloud
