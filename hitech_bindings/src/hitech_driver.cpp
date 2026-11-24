#include <rclcpp/rclcpp.hpp>

#include "hitech_bindings/hitech_bindings.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::string nodeName = "hitech_driver";

    auto driver = std::make_shared<HitechBindings>(nodeName, options);

    rclcpp::spin(driver);

    rclcpp::shutdown();

    return 0;
}
