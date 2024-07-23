#include <pcl_processing/point_cloud_processing_node.h>


    PointCloudProcessingNode::PointCloudProcessingNode(): Node("grasp_object_surface_normal")
    {
        // Create the service
        this->service = this->create_service<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal>("grasp_object_surface_normal", std::bind(&PointCloudProcessingNode::processPointCloud, this, std::placeholders::_1, std::placeholders::_2));
    }

    void PointCloudProcessingNode::processPointCloud(const std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Request> request, std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Response> response)
    {
        // Process the point cloud and generate the pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        // Extract the point cloud from the request
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud = this->transformPointsToPointCloud(request->masked_points);
        
        // Save the point cloud as a PCD file
        //
        pcl::io::savePCDFileASCII("./src/aip_grasp_planning/cloud.pcd", *cloud);
        // pcl::ModelCoefficients::Ptr coefficients = this->extractSurfacePlane(cloud);

        RCLCPP_INFO(this->get_logger(), "Point cloud saved as PCD file");


        // Set the response
        response->surface_normal_to_grasp = pose;
    }

    // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr result_publisher_;


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

pcl::ModelCoefficients::Ptr PointCloudProcessingNode::extractSurfacePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
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