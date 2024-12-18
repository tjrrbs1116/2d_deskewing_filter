
#include "filter_utils.hpp"

Eigen::Matrix4d odomToTransform(const nav_msgs::msg::Odometry& odom)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    // Translation
    transform(0, 3) = odom.pose.pose.position.x;
    transform(1, 3) = odom.pose.pose.position.y;
    transform(2, 3) = odom.pose.pose.position.z;

    // Rotation
    Eigen::Quaterniond orientation(
        odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z);
    transform.block<3, 3>(0, 0) = orientation.toRotationMatrix();

    return transform;
}

sensor_msgs::msg::PointCloud2 createPointCloud2(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<double>& reflects,
    const std::vector<double>& times,
    const std_msgs::msg::Header& header)
{
    // Check that all input vectors have the same size
    if (points.size() != reflects.size() || points.size() != times.size())
    {
        throw std::runtime_error("Points, reflects, and times must have the same size");
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;

    // Set up the PointCloud2 header
    cloud_msg.header = header;
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;

    // Define the fields
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(
        5,  // Number of fields
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
        "time", 1, sensor_msgs::msg::PointField::FLOAT64);
    modifier.resize(points.size());

    // Create iterators
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<double> iter_time(cloud_msg, "time");

    // Fill the data
    for (size_t i = 0; i < points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_time)
    {
        *iter_x = static_cast<float>(points[i].x());
        *iter_y = static_cast<float>(points[i].y());
        *iter_z = static_cast<float>(points[i].z());
        *iter_intensity = static_cast<float>(reflects[i]);  // Cast to float for intensity
        *iter_time = times[i];  // Keep time as double
    }

    return cloud_msg;
}