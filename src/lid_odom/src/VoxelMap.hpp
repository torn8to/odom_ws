#pragma once

#include <vector>
#include <tuple>
#include <unordered_map>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "PointToVoxel.hpp"


namespace cloud {
class VoxelMap {
  public:
    explicit VoxelMap(double voxel_resolution, double max_range, int max_points_per_voxel)
    : voxel_resolution_(voxel_resolution),
      max_range_(max_range),
      max_points_per_voxel_(max_points_per_voxel) {map_.reserve(999983);}

    inline void clear() { map_.clear(); }
    inline bool empty() const { return map_.empty(); }
    void addPoints(const std::vector<Eigen::Vector3d> &points);
    std::vector<Eigen::Vector3d> removeFarPoints(const std::vector<Eigen::Vector3d> &cloud);
    std::vector<Eigen::Vector3d> transform_cloud(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &transform);
    std::vector<Eigen::Vector3d> cloud() const;
    std::tuple<Eigen::Vector3d, double> firstNearestNeighborQuery(const Eigen::Vector3d &point) const;
    void removePointsFarFromOrigin(Eigen::Vector3d &origin);
  private:
    double voxel_resolution_;
    double max_range_;
    int max_points_per_voxel_;
    std::unordered_map<cloud::Voxel, std::vector<Eigen::Vector3d>> map_;
  };
}
