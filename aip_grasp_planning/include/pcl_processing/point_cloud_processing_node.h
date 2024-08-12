#ifndef PCL_POSE_PROCESSING_H
#define PCL_POSE_PROCESSING_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <pcl/filters/project_inliers.h>
#include <geometry_msgs/msg/transform.hpp>
#include <aip_grasp_planning_interfaces/srv/grasp_object_surface_normal.hpp>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudProcessingNode : public rclcpp::Node
{
public:
    PointCloudProcessingNode();
    void processPointCloud(const std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Request> request, std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Response> response);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr concatinatedCloud;
    void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointsToPointCloud(const std::vector<geometry_msgs::msg::Point>& maskedPoints);
    pcl::ModelCoefficients::Ptr extractSurfacePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& surfacePlane);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractInlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr inliers);
    void publish_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::ModelCoefficients::Ptr coefficients);

    rclcpp::Service<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal>::SharedPtr service;

    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

};

#endif // PCL_POSE_PROCESSING_Hl
