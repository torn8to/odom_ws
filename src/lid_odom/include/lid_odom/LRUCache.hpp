#pragma once
#include <unordered_map>
#include <Eigen/Core>

#include "PointToVoxel.hpp"


using Voxel = Eigen::Vector3i;
namespace cloud{
struct FrequencyCache{
  public:
    FrequencyCache(){
        lookup_.reserve(999283);
    }

    inline uint getCounter() const {
        return counter;
    };

    void reset(){
      counter = 0;
      empty();
    }

    inline void empty(){
      lookup_.clear();
      lookup_.reserve(999283);
    }

    inline void update(Voxel voxel) const {
      if(lookup_.find(voxel) != lookup_.end()){
        lookup_[voxel]= lookup_[voxel]+1;
      }
      lookup_.insert({voxel, 0});
    }
    
    mutable std::unordered_map<Voxel,int> lookup_;
    mutable uint counter = 0;

};


} // NAMESPACE cloud