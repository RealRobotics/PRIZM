#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LaserScanToPointCloud : public rclcpp::Node {
private:
    std::string nodeName_, laser_topic, point_cloud_topic;
    sensor_msgs::msg::PointCloud2 pointCloud_;
    laser_geometry::LaserProjection projector_;

public:
    LaserScanToPointCloud();
    LaserScanToPointCloud(std::string& nodeName, rclcpp::NodeOptions& options);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub_;

    void scanCallback_(const sensor_msgs::msg::LaserScan& scan_in);
};

int main(int argc, char* argv[]);