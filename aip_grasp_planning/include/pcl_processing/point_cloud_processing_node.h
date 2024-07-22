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
#include <aip_grasp_planning_interfaces/srv/grasp_object_surface_normal.hpp>
#include <pcl/io/pcd_io.h>

class PointCloudProcessingNode : public rclcpp::Node
{
public:
    PointCloudProcessingNode();
    void processPointCloud(const std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Request> request, std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Response> response);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointsToPointCloud(const std::vector<geometry_msgs::msg::Point>& maskedPoints);
    pcl::ModelCoefficients::Ptr extractSurfacePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractInlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr inliers);

    rclcpp::Service<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal>::SharedPtr service;

    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
};

#endif // PCL_POSE_PROCESSING_Hl
