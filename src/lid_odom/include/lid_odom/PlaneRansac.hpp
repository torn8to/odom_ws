#include <vector>
#include <random>
#include <tuple>
#include <iterator>
#include <cmath>

#include <eigen/Core>
#include <sophus/SE3d.hpp>

namespace cloud{
    inline Eigen::Vector4d PlaneRansac(std::vector<Eigen::Vector3d> points,
     double thresh = 0.05,
     int minPoints = 100,
     int maxIterations = 1000){
        uint num_points = static_cast<int>(points.size());
        Eigen::Vector4d best_equation;
        uint bestInliers = 0;
        std::vector<Eigen::Vector3d> sampled;
        sampled.reserve(3);
        for(int i = 0; i < maxIterations; i++){
            std::sample(points.begin(), points.end(), std::back_inserter(sampled),
             3, std::mt19937 {std::random_device{}()});
            Eigen::Vector3d vecA = sampled[1] - sampled[0];
            Eigen::Vector3d vecB = sampled[2] - sampled[0];
            Eigen::Vector3d crossAB = vecA.cross(vecB);
            Eigen::Vector3d crossABNorm = crossAB / crossAB.norm();
            double k = -crossABNorm.dot(sampled[1]);

            Eigen::Vector4d plane_params;
            plane_params << crossABNorm, k;
            std::vector<Eigen::Vector3d> inliers;
            inliers.reserve(points.size());
            double distance_point;
            for (const auto& pt : points){

                //this is taken from  https://mathworld.wolfram.com/Point-PlaneDistance.html
                //literally no idea how this works this math is beyond me
                distance_point = (plane_params[0]*pt[0] + plane_params[1]*pt[1] +
                                    plane_params[2]*pt[2]+plane_params[3])/std::sqrt(
                                        plane_params[0]*plane_params[0]+
                                        plane_params[1]*plane_params[1]+
                                        plane_params[2]*plane_params[2]);
                if (std::abs(distance_point) < thresh){
                    inliers.emplace_back(pt);
                }
            }
            if(inliers.size() > bestInliers){
                best_equation = plane_params;
                bestInliers = static_cast<uint>(inliers.size());
            }
            sampled.clear()
            sampled.resize(3)
        }
        return best_equation;
    }

    inline std::vector<uint> inlierIndicesPlane(const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector4d& plane_params, double thresh = 0.05){
        std::vector<uint> inliers;
        inliers.reserve(points.size());
        double distance_point;
        for(uint i = 0; i < points.size(); i++){
            const auto& pt = points[i];
            distance_point = (plane_params[0]*pt[0] + plane_params[1]*pt[1] +
                           plane_params[2]*pt[2]+plane_params[3])/std::sqrt(
                               plane_params[0]*plane_params[0]+
                               plane_params[1]*plane_params[1]+
                               plane_params[2]*plane_params[2]);
            if (std::abs(distance_point) < thresh){
                inliers.emplace_back(i);
            }
        }
        return inliers;
    }

} // NAMESPACE cloud