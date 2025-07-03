#pragma once

#include <algorithim>
#include <vector>
#include <eigen/Core>
#include "../PlaneRansac.hpp"

namespace cloud{

    std::vector<Eigen::Vector3d> removePointsbyPlane(std::vector<Eigen::Vector3d> &points,
     Eigen::Vector4d plane_params,
     double threshold){
      std::vector<Eigen::Vector3d> pruned_points;
      pruned_points.reserve(points.size);
      std::foreach(points.begin(),points.end(),
        [&](auto &pt){
          double distance_point
          if(!std::abs(distance_point) > threshold){
            pruned_points.push_back(pt)
           }
        });
      return pruned_points;
    }

} //NAMESPACE CLOUD
