#include <prizm_dynamixel/dynamixelServoDriver.hpp>

DynamixelServoDriver::DynamixelServoDriver() : Node("servo_driver") {
    // get parameters in
    // 1. how many motors
    // 2. what address we are targeting

    // this->declare_parameter("num_servos", )
    // this->declare_parameters("servo_driver", );
    // const std::map<std::string, std::pair<,
    // rcl_interfaces::msg::ParameterDescriptor>> &parameters>

    // std::map<std::string, std::pair<ParameterT, rcl
    // std::pair<rclcpp::Parameter, rcl_interfaces::msg::ParameterDescriptor> parameters;
    // parameters.first = rclcpp::PARAMETER_INTEGER;
    rcl_interfaces::msg::ParameterDescriptor numServosDesc_, addressParamDesc_;
    numServosDesc_.description = "Number of servos in config";
    numServosDesc_.type = rclcpp::PARAMETER_INTEGER;
    addressParamDesc_.description = "COM Port for comps";
    addressParamDesc_.type = rclcpp::PARAMETER_STRING;

    auto addressParamPair = paramPair(rclcpp::PARAMETER_STRING, addressParamDesc_);
    auto numServosParamPair = paramPair(rclcpp::PARAMETER_INTEGER, numServosDesc_);

     std::map<paramPair,rclcpp::Parameter> params_map = {
       {addressParamPair, addresParam_},
       {numServosParamPair, numServosParam_}
     };
    this->declare_parameters("servo_driver", params_map);

    this->InitServos();
}

void DynamixelServoDriver::InitServos() {
    // set up packethandler
    // set up porthandler
    // set up bulkreader
    // setup bulkwriter
}