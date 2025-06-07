#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <Eigen/Core>

namespace cloud {
  std::vector<Eigen::Vector3d> convertMsgToCloud(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    std::vector<Eigen::Vector3d> cloud;
    cloud.reserve(cloud_msg->height * cloud_msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> x_iter(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iter(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iter(*cloud_msg, "z");

    for(; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
      cloud.push_back(Eigen::Vector3d(
        static_cast<double>(*x_iter),
        static_cast<double>(*y_iter),
        static_cast<double>(*z_iter)
      ));
    }
    return cloud;
  }

  /**
   * @brief Creates an XYZ point cloud message
   * @param points The point cloud data to convert to a message
   * @return PointCloud2 message containing the points
   */
  sensor_msgs::msg::PointCloud2 convertCloudToMsg(const std::vector<Eigen::Vector3d> &points) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.height = 1;
    msg.width = points.size();
    msg.fields.resize(3);
    msg.point_step = 12; //3*4
    msg.row_step = msg.point_step * msg.width;
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.data.resize(msg.row_step);

    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    sensor_msgs::PointCloud2Iterator<float> x_iter(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iter(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iter(msg, "z");

    for(size_t i = 0; i < points.size(); ++i, ++x_iter, ++y_iter, ++z_iter) {
      *x_iter = static_cast<float>(points[i].x());
      *y_iter = static_cast<float>(points[i].y());
      *z_iter = static_cast<float>(points[i].z());
    }
    
    return msg;
  }
} // namespace cloud
