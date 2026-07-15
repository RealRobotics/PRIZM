#include <prizm_dynamixel/dynamixelServoDriver.hpp>

DynamixelServoDriver::DynamixelServoDriver() : Node("servo_driver") {
    // get parameters in
    // 1. int how many motors
    // 2. what address we are targeting
    // 3. int servo max
    // 4. int servo min

    this->declare_parameter("device_addr", "/dev/ttyUSB0");
    address = this->get_parameter("device_addr").as_string();

    auto parameter_map = std::map<std::string, int>{
        {"num_servos", 1},
        {"servo_min", 0},
        {"servo_max", 360},
    };
    this->declare_parameters(paramNamespace_, parameter_map);
    numServos_ =
        this->get_parameter(paramNamespace_ + "." + "num_servos").as_int();
    servoMin_ =
        this->get_parameter(paramNamespace_ + "." + "servo_min").as_int();
    servoMax_ =
        this->get_parameter(paramNamespace_ + "." + "servo_max").as_int();


}

void DynamixelServoDriver::InitServos() {
    // set up packethandler
    // set up porthandler
    // set up bulkreader
    // setup bulkwriter
}


#ifndef BUILD_TESTING

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("servo_driver"), "Started node");

    auto servo_driver = std::make_shared<DynamixelServoDriver>();


    rclcpp::spin(servo_driver);

    rclcpp::shutdown();


    return 0;
}

#endif