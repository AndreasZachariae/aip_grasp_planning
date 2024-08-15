#include <pcl_processing/point_cloud_processing_node.h>


    PointCloudProcessingNode::PointCloudProcessingNode(): Node("grasp_object_surface_normal"), concatinatedCloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
        // Create the service
        this->service = this->create_service<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal>("grasp_object_surface_normal", std::bind(&PointCloudProcessingNode::processPointCloud, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create the publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("grasp_object_surface_point_cloud", 10);    
    }

    void PointCloudProcessingNode::publish_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Convert the PCL point cloud to a ROS 2 PointCloud2 message
        concatinatedCloud->points.insert(concatinatedCloud->points.end(), cloud->points.begin(), cloud->points.end());
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*concatinatedCloud, point_cloud_msg);

        // Fill the header
        point_cloud_msg.header.stamp = this->now();
        point_cloud_msg.header.frame_id = "camera";

        // Publish the point cloud
        publisher_->publish(point_cloud_msg);
    }

    void PointCloudProcessingNode::processPointCloud(const std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Request> request, std::shared_ptr<aip_grasp_planning_interfaces::srv::GraspObjectSurfaceNormal::Response> response)
    {
        if (request->reset_viz)
            concatinatedCloud->points.clear();
        // Process the point cloud and generate the pose
        geometry_msgs::msg::Pose pose;
        

        // Extract the point cloud from the request
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud = this->transformPointsToPointCloud(request->masked_points);
        filterCloud(cloud, 0.01);
        pcl::PointCloud<pcl::PointXYZ>::Ptr surfacePlane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr planeCoefficients = extractSurfacePlane(cloud, surfacePlane);
        pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud = projectCloud(cloud, planeCoefficients);
        filterCloud(projectedCloud, 0.01);

        this->publish_point_cloud(projectedCloud);
        // Save the point cloud as a PCD file
        //
        // pcl::io::savePCDFileASCII("./src/aip_grasp_planning/cloud.pcd", *cloud);
        // pcl::ModelCoefficients::Ptr coefficients = this->extractSurfacePlane(cloud);

        // RCLCPP_INFO(this->get_logger(), "Point cloud saved as PCD file");
        pcl::PointXYZ medianPosition(0.0, 0.0, 0.0);
        for (const auto& point : projectedCloud->points)
        {
            std::vector<double> x_values;
            std::vector<double> y_values;
            std::vector<double> z_values;

            for (const auto& point : projectedCloud->points)
            {
                x_values.push_back(point.x);
                y_values.push_back(point.y);
                z_values.push_back(point.z);
            }

            std::sort(x_values.begin(), x_values.end());
            std::sort(y_values.begin(), y_values.end());
            std::sort(z_values.begin(), z_values.end());

            double median_x = x_values[x_values.size() / 2];
            double median_y = y_values[y_values.size() / 2];
            double median_z = z_values[z_values.size() / 2];

            medianPosition.x = median_x;
            medianPosition.y = median_y;
            medianPosition.z = median_z;


        }


        // Set the pose
        pose.position.x = medianPosition.x;
        pose.position.y = medianPosition.y;
        pose.position.z = medianPosition.z;

        // Calculate the quaternion between the normal vector of the plane and a normal vector that only points in the z direction
        Eigen::Vector3f planeNormal(planeCoefficients->values[0], planeCoefficients->values[1], planeCoefficients->values[2]);
        Eigen::Vector3f zDirection(0.0, 0.0, 1.0);
        Eigen::Quaternionf quaternion;
        quaternion.setFromTwoVectors(planeNormal, zDirection);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quaternion: %f %f %f %f", quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

        // Set the orientation of the pose
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();

        // Set the response
        response->surface_normal_to_grasp = pose;
    }

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessingNode::projectCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::ModelCoefficients::Ptr coefficients)
{
    // project the points onto the plane
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>); 
    proj.filter(*projectedCloud);
    return projectedCloud;
}

void PointCloudProcessingNode::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size)
{
        // Use a voxel grid filter to downsample the point cloud with a leaf size 
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