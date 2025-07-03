#pragma once

#include <vector>
#include <tuple>
#include <unordered_map>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "PointToVoxel.hpp"
#include "PointUtils.hpp"
#include "LRUCache.hpp"

namespace cloud {
class VoxelMap {
  public:
    explicit VoxelMap(double voxel_resolution, double max_range, int max_points_per_voxel)
    : voxel_resolution_(voxel_resolution),
      max_range_(max_range),
      max_points_per_voxel_(max_points_per_voxel) {
        map_.reserve(999983);
    }
    inline void clear() { map_.clear(); }
    inline bool empty() const { return map_.empty(); }
    inline size_t size() const { return map_.size(); }
    void addPoints(const std::vector<Eigen::Vector3d> &points);
    std::vector<Eigen::Vector3d> removeFarPoints(const std::vector<Eigen::Vector3d> &cloud);
    std::vector<Eigen::Vector3d> cloud() const;
    std::tuple<Eigen::Vector3d, double> firstNearestNeighborQuery(const Eigen::Vector3d &point) const;
    void removePointsFarFromOrigin(const Eigen::Vector3d &origin);
    inline void resetfrequencyCounter(){
      frequencyCache_.reset();
    }

    inline void pruneBasedOnFrequencyCounter(int threshold){
      auto it_cache = frequencyCache_.lookup_.begin();
      for(auto it = map_.begin(); it != map_.end(); ++it, ++it_cache){
        Eigen::Vector3i voxel =  it->first;
        if(frequencyCache_.lookup_.find(voxel) != frequencyCache_.lookup_.end() &&
         frequencyCache_.lookup_[voxel] > threshold){
          map_.erase(it);
          frequencyCache_.lookup_.erase(voxel);
        }
      }
    }

  private:
    double voxel_resolution_;
    double max_range_;
    int max_points_per_voxel_;
    cloud::FrequencyCache frequencyCache_;
    std::unordered_map<cloud::Voxel, std::vector<Eigen::Vector3d>> map_;
  };
}
