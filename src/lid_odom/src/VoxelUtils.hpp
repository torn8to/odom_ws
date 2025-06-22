#pragma once

#include <vector>
#include <Eigen/Core>
#include "VoxelMap.hpp"


namespace cloud{
/**
 * @brief Voxel downsample the point to the voxel resolution
 * @param cloud The point cloud to downsample
 * @param voxel_size The size of the voxel
 * @param max_points_per_voxel The maximum number of points per voxel
 * @return The downsampled point cloud
 */
std::vector<Eigen::Vector3d> voxelDownsample(std::vector<Eigen::Vector3d> &cloud, double voxel_size,double max_distance, int max_points_per_voxel) {
    cloud::VoxelMap voxel_filter(voxel_size, max_distance, max_points_per_voxel);
    std::vector<Eigen::Vector3d> pruned_cloud =  voxel_filter.removeFarPoints(cloud);
    voxel_filter.addPoints(pruned_cloud);
    return voxel_filter.cloud();
}


  //TODO: velocity compensation on point cloud
  // propogate last update delta to cover unskew points takes in 
  //std::vector<Eigen::Vector3d> deSkewCloud(geometry_msgs::msg::PointCloud2, std::tuple<Eigen::Vector3d,Eigen::Vector3d> velocity,)


} // namespace cloud