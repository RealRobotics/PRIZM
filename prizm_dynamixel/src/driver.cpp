#include <rclcpp/rclcpp.hpp>
#include "prizm_dynamixel/dynamixelDriver.hpp"


int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::string nodeName = "driver";

    auto driver = std::make_shared<DynamixelDriver>(nodeName, options);

    rclcpp::spin(driver);
    rclcpp::shutdown();

    return 0;
}