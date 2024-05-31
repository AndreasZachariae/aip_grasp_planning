#include <pcl_processing/pcl_pose_processing.h>


    PointCloudProcessingNode::PointCloudProcessingNode(): Node("grasp_object_surface_normal")
    {
        // Create the service
        service_ = this->create_service<grasp_planning_interfaces::srv::GraspObjectSurfaceNormal>(
            "grasp_object_surface_normal",
            std::bind(&PointCloudProcessingNode::processPointCloud, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    void PointCloudProcessingNode::processPointCloud(const std::shared_ptr<grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Request> request, std::shared_ptr<geometry_msgs::srv::Pose::Response> response)
    {
        // Process the point cloud and generate the pose
        geometry_msgs::msg::Pose pose;

        // Extract the point cloud from the request
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud = this->transformPointsToPointCloud(request->maskedPoints);

        // Set the response
        response->pose = pose;
    }

    rclcpp::Service<geometry_msgs::srv::PoseArrayToPose>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr result_publisher_;


pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointsToPointCloud(const std::vector<pcl::PointXYZ>& maskedPoints)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = maskedPoints.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->maskedPoints.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < maskedPoints.size(); ++i)
    {
        cloud->points[i].x = maskedPoints[i].x;
        cloud->points[i].y = maskedPoints[i].y;
        cloud->points[i].z = maskedPoints[i].z;
    }

    return cloud;
}

pcl::ModelCoefficients::Ptr extractSurfacePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
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

pcl::PointCloud<pcl::PointXYZ>::Ptr extractInlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr inliers)
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
    auto node = std::make_shared<PointCloudProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}