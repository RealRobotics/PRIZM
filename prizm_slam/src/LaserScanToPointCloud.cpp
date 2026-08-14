#include "prizm_slam/LaserScanToPointCloud.hpp"

LaserScanToPointCloud::LaserScanToPointCloud() : Node("ScanToPC") {}

LaserScanToPointCloud::LaserScanToPointCloud(std::string& nodeName,
                                             rclcpp::NodeOptions& options)
    : Node(nodeName, options) {
    // declare parameters here
    this->declare_parameter("laser_scan_topic", "/scan");
    this->declare_parameter("point_cloud_topic", "/point_cloud");

    laser_topic = this->get_parameter("laser_scan_topic").as_string();
    point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();

    scanSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic, 10,
        std::bind(&LaserScanToPointCloud::scanCallback_, this,
                  std::placeholders::_1));

    pointCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic, 10);
}

void LaserScanToPointCloud::scanCallback_(
    const sensor_msgs::msg::LaserScan& scan_in) {
    projector_.projectLaser(scan_in, pointCloud_);
    pointCloud_.header.stamp =
        scan_in.header.stamp +
        rclcpp::Duration::from_seconds(scan_in.ranges.size() *
                                        scan_in.time_increment);
    pointCloudPub_->publish(pointCloud_);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::string nodeName = "scan_to_pc";
    auto scanToPC = std::make_shared<LaserScanToPointCloud>(nodeName, options);

    rclcpp::spin(scanToPC);
    rclcpp::shutdown();

    return 0;
}