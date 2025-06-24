#include "lidar_mapping_pcl/simple_icp.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace lidar_mapping_pcl
{

SimpleICP::SimpleICP()
: leaf_size_(0.1), fitness_score_(0.0)
{
  // Default initialization
  initialize();
}

SimpleICP::~SimpleICP()
{
}

void SimpleICP::initialize(
  int max_iterations,
  double transformation_epsilon,
  double max_correspondence_distance,
  double leaf_size)
{
  // Set ICP parameters
  icp_.setMaximumIterations(max_iterations);
  icp_.setTransformationEpsilon(transformation_epsilon);
  icp_.setMaxCorrespondenceDistance(max_correspondence_distance);
  icp_.setEuclideanFitnessEpsilon(1e-6);  // Additional convergence criteria

  // Store leaf size for point cloud downsampling
  leaf_size_ = leaf_size;
  voxel_grid_phase1.setLeafSize(leaf_size,leaf_size, leaf_size) ;
  voxel_grid_phase2.setLeafSize(leaf_size/2.00, leaf_size/2.0, leaf_size/2.0);

  RCLCPP_INFO(
    logger_,
    "ICP initialized with: max_iter=%d, epsilon=%.8f, max_dist=%.2f, leaf_size=%.3f",
    max_iterations, transformation_epsilon, max_correspondence_distance, leaf_size);
}

Eigen::Matrix4f SimpleICP::align(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & target)
{
  if (source->empty() || target->empty()) {
    RCLCPP_WARN(logger_, "Empty point cloud provided for ICP alignment");
    return Eigen::Matrix4f::Identity();
  }

  // Create processed copies of the input clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_processed = 
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*source);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_processed = 
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*target);

  // Remove NaN points
  removeNaNPoints(source_processed);
  removeNaNPoints(target_processed);

  // Check if clouds still have points after NaN removal
  if (source_processed->empty() || target_processed->empty()) {
    RCLCPP_WARN(logger_, "Point cloud empty after NaN filtering");
    return Eigen::Matrix4f::Identity();
  }

  // Downsample if leaf_size > 0
  if (leaf_size_ > 0.0) {
    source_processed = downsampleCloudPhase2(source_processed, leaf_size_);
    target_processed = downsampleCloudPhase2(target_processed, leaf_size_);
  }

  RCLCPP_INFO(
    logger_, "Aligning point clouds: source=%ld points, target=%ld points",
    source_processed->size(), target_processed->size());

  // Set the input clouds
  icp_.setInputSource(source_processed);
  icp_.setInputTarget(target_processed);

  // Perform alignment
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
  icp_.align(*aligned);

  // Store fitness score
  fitness_score_ = icp_.getFitnessScore();

  // Check convergence
  if (icp_.hasConverged()) {
    RCLCPP_INFO(
      logger_, "ICP converged with fitness score: %.6f", fitness_score_);
  } else {
    RCLCPP_WARN(
      logger_, "ICP did not converge! Fitness score: %.6f", fitness_score_);
  }

  // Return transformation matrix
  return icp_.getFinalTransformation();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SimpleICP::convertFromROS(
  const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);
  return cloud;
}

void SimpleICP::setInitialAlignment(const Eigen::Matrix4f & initial_guess)
{
  icp_.setInitialTransformation(initial_guess);
  RCLCPP_INFO(logger_, "Set initial alignment transformation");
}

double SimpleICP::getFitnessScore() const
{
  return fitness_score_;
}

void SimpleICP::removeNaNPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SimpleICP::downsampleCloudPhase1(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_phase1.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid_phase1.setInputCloud(cloud);
  voxel_grid_phase1.filter(*cloud_filtered);
  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SimpleICP::downsampleCloudPhase2(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  
  voxel_grid_phase2.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid_phase2.setInputCloud(cloud);
  voxel_grid_phase2.filter(*cloud_filtered);
  
  return cloud_filtered;
}

}  // namespace lidar_mapping_pcl 