#ifndef BASIC_PCL_SUBSCRIBER_H
#define BASIC_PCL_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudNode : public rclcpp::Node
{
public:
    PointCloudNode();

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
#endif  // BASIC_PCL_SUBSCRIBER_H

