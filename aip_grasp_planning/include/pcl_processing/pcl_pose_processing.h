#ifndef PCL_POSE_PROCESSING_H
#define PCL_POSE_PROCESSING_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>

class PointCloudProcessingNode : public rclcpp::Node
{
public:
    PointCloudProcessingNode();
    void processPointCloud(const std::shared_ptr<grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Request> request, std::shared_ptr<geometry_msgs::srv::Pose::Response> response);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointsToPointCloud(const std::vector<pcl::PointXYZ>& maskedPoints);
    pcl::ModelCoefficients::Ptr extractSurfacePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractInlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr inliers);

    rclcpp::Service<geometry_msgs::srv::PoseArrayToPose>::SharedPtr service_;
};

#endif // PCL_POSE_PROCESSING_Hl
