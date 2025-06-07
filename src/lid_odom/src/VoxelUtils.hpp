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
std::vector<Eigen::Vector3d> voxelDownsample(std::vector<Eigen::Vector3d> &cloud, double voxel_size, int max_points_per_voxel) {
    cloud::VoxelMap voxel_filter(voxel_size, max_points_per_voxel);
    voxel_filter.addPoints(cloud);
    return voxel_filter.cloud();
}

} // namespace cloud