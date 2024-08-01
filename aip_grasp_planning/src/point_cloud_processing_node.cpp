#include <pcl_processing/point_cloud_processing_node.h>


    PointCloudProcessingNode::PointCloudProcessingNode(): Node("grasp_object_surface_normal")
    {
        // Create the service
        this->service = this->create_service<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal>("grasp_object_surface_normal", std::bind(&PointCloudProcessingNode::processPointCloud, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create the publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("grasp_object_surface_point_cloud", 10);    
    }

    void PointCloudProcessingNode::publish_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Convert the PCL point cloud to a ROS 2 PointCloud2 message
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*cloud, point_cloud_msg);

        // Fill the header
        point_cloud_msg.header.stamp = this->now();
        point_cloud_msg.header.frame_id = "camera";

        // Publish the point cloud
        publisher_->publish(point_cloud_msg);
    }

    void PointCloudProcessingNode::processPointCloud(const std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Request> request, std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Response> response)
    {
        // Process the point cloud and generate the pose
        geometry_msgs::msg::Pose pose;
        

        // Extract the point cloud from the request
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud = this->transformPointsToPointCloud(request->masked_points);
        filterCloud(cloud, 0.01);
        pcl::PointCloud<pcl::PointXYZ>::Ptr surfacePlane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr planeCoefficients = extractSurfacePlane(cloud, surfacePlane);
        this->publish_point_cloud(surfacePlane);
        // Save the point cloud as a PCD file
        //
        // pcl::io::savePCDFileASCII("./src/aip_grasp_planning/cloud.pcd", *cloud);
        // pcl::ModelCoefficients::Ptr coefficients = this->extractSurfacePlane(cloud);

        // RCLCPP_INFO(this->get_logger(), "Point cloud saved as PCD file");
        pcl::PointXYZ averagePosition(0.0, 0.0, 0.0);
        for (const auto& point : surfacePlane->points)
        {
            averagePosition.x += point.x;
            averagePosition.y += point.y;
            averagePosition.z += point.z;
        }
        int numPoints = surfacePlane->size();
        averagePosition.x /= numPoints;
        averagePosition.y /= numPoints;
        averagePosition.z /= numPoints;

        // Set the pose
        pose.position.x = averagePosition.x;
        pose.position.y = averagePosition.y;
        pose.position.z = averagePosition.z;
        pose.orientation.x = 0.0;
        pose.orientation.y = 1.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0;
        // pose.position.x = planeCoefficients->values[0];
        // pose.position.y = planeCoefficients->values[1];
        // pose.position.z = planeCoefficients->values[2];
        // Set the response
        response->surface_normal_to_grasp = pose;
    }

    // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr result_publisher_;


void PointCloudProcessingNode::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size)
{
        pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
        voxelFilter.setInputCloud(cloud);
        voxelFilter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxelFilter.filter(*cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessingNode::transformPointsToPointCloud(const std::vector<geometry_msgs::msg::Point>& maskedPoints)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (const auto& point : maskedPoints)
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        cloud->points.push_back(pcl_point);
    }

    // Set the size of the point cloud
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

pcl::ModelCoefficients::Ptr PointCloudProcessingNode::extractSurfacePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& surfacePlane)
{
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // Maybe set this to SACMODEL_PERPENDICULAR_PLANE to set the normal vector of the plane
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*surfacePlane);
    return coefficients;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessingNode::extractInlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr inliers)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*inlierCloud);

    return inlierCloud;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasp object surface normal processing node started");
    rclcpp::spin(std::make_shared<PointCloudProcessingNode>());
    rclcpp::shutdown();
    return 0;
}