#ifndef FILTER_UTILS_H

#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

Eigen::Matrix4d odomToTransform(const nav_msgs::msg::Odometry& odom);

sensor_msgs::msg::PointCloud2 createPointCloud2(const std::vector<Eigen::Vector3d>& points, const std::vector<double>& reflects, const std::vector<double>& times,
    const std_msgs::msg::Header& header);

#endif // FILTER_UTILS_H