#include <cassert>
#include <vector>
#include <algorithm>
#include <sophus/se3.hpp>
#include <eigen/Core>

namespace cloud{
/**
 * @brief Performs motion compensation on a point cloud using timestamps and transformation
 * 
 * This function compensates for motion during the LiDAR scan by applying a transformation
 * to each point based on its timestamp relative to the scan start time. The transformation
 * is interpolated using the time difference between the current point and the first point.
 * 
 * @param cloud The input point cloud to be motion compensated
 * @param cloud_timestamps Vector of timestamps corresponding to each point in the cloud
 * @param diff_transformation The differential transformation to apply (typically velocity * dt)
 * @return std::vector<Eigen::Vector3d> The motion-compensated point cloud
 * @note The cloud and timestamps vectors must have the same size
 * @note The diff_transformation should represent the motion between consecutive timestamps
 */

std::vector<Eigen::Vector3d>  motionCompensation(const std::vector<Eigen::Vector3d> &cloud,
                                         std::vector<double> &cloud_timestamps,
                                         const Sophus::SE3d &diff_transformation){
  assert(cloud.size() == cloud_timestamps.size() && "Cloud and timestamps must have the same size");
  std::vector<Eigen::Vector3d> compensated_cloud;
  compensated_cloud.reserve(cloud.size());

  auto cloud_it = cloud.begin();
  auto timestamps_it = cloud.timestamps.begin();
  double last_time = cloud_timestamps.front();
  for(; cloud_it != cloud.end(); ++cloud_it, ++timestamps_it){
    double time = *timestamp_it;
    Eigen::Vector3d point = *cloud_it;
    compensated_cloud.push_back((diff_transformation*(time-last_time)) * point);
  }
  return compensated_cloud;
}

} // namespace cloud