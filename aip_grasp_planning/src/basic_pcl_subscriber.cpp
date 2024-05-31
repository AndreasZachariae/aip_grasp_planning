#include <pcl_processing/basic_pcl_subscriber.h>

PointCloudNode::PointCloudNode()
    : Node("point_cloud_node")
{
    RCLCPP_INFO(this->get_logger(), "Creating PointCloud subscriber ");
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/stereo/points2", 10, std::bind(&PointCloudNode::topic_callback, this, std::placeholders::_1));
}

void PointCloudNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    RCLCPP_INFO(this->get_logger(), "Converting to PCD and saving...");

    pcl::io::savePCDFileASCII("./pcl_recordings/bigbox2.pcd", cloud);
    RCLCPP_INFO(this->get_logger(), "Saved %d data points to bigbox2.pcd.", (int)cloud.points.size());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudNode>());
    rclcpp::shutdown();
    return 0;
}